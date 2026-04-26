/*
 * usart2.h — QBOX wrapper for the full-featured Usart peripheral model.
 *
 * ── Architecture ─────────────────────────────────────────────────────────────
 *
 *   CPU ──TLM──► target_socket (Usart2) ──► Usart::bus (core model)
 *
 *   TX: TBUF write intercepted in b_transport → backend_socket.enqueue(byte)
 *       Usart core still receives the write to update internal state/IRQs.
 *
 *   RX: backend_socket receives bytes → rx_inject() loads them directly into
 *       the Usart core's RBUF (bypasses baud-rate serial framing).
 *
 *   IRQ: Four IRQ outputs (tbir, tir, rir, eir) combined → single irq output.
 *       A sticky STATUS register (offset 0x20) records which IRQs fired.
 *
 * ── STATUS register (offset 0x20) ────────────────────────────────────────────
 *
 *   Bit  Mask         Description
 *   0    STATUS_TBIR  TX buffer empty IRQ fired (TBUF freed into TSR)
 *   1    STATUS_TIR   TX complete IRQ fired (all bits shifted out)
 *   2    STATUS_RIR   RX data ready IRQ fired (RBUF has valid data)
 *   3    STATUS_EIR   Error IRQ fired (overrun, framing, or parity)
 *
 *   Read: returns current sticky status.
 *   Write: write-1-to-clear (W1C) — firmware clears bits after handling.
 *
 * ── STATUS update strategy ────────────────────────────────────────────────────
 *
 *   In QBOX's multithread-unconstrained model, QEMU calls b_transport from its
 *   own OS thread.  sc_signal::write() inside b_transport schedules a delta-
 *   cycle update, but the SystemC scheduler thread does not run until QEMU
 *   yields at a quantum boundary.  This means irq_method() (sensitive to
 *   sc_signals) would not fire during a tight QEMU STATUS-polling loop.
 *
 *   To avoid this, STATUS bits are updated SYNCHRONOUSLY within b_transport
 *   by reading the Usart core's IRQ assertion timestamps directly (via
 *   is_tbir_asserted() etc.), bypassing the sc_signal path entirely.
 *   update_status_from_core() is called:
 *     • After every TBUF write (catches TBIR)
 *     • Inside rx_receive after rx_inject (catches RIR / EIR)
 *     • In the STATUS read handler after m_usart.sync() (catches TIR)
 *
 *   irq_method() is still registered and still drives the irq output port
 *   for use with a GIC, but it no longer owns the STATUS register.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <queue>
#include <unistd.h>   /* write() for async-signal-safe tracing */
#include <systemc>
#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
#include "tlm_utils/simple_initiator_socket.h"
#include <cci_configuration>

#include <ports/initiator-signal-socket.h>
#include <ports/biflow-socket.h>
#include <async_event.h>
#include <module_factory_registery.h>
#include <scp/report.h>
#include <tlm_sockets_buswidth.h>

#include "Usart.h"
#include "Usart_types.h"
#include "Usart_regdefs.h"

class Usart2 : public sc_core::sc_module
{
    SCP_LOGGER();

public:
    /* TLM register interface — name must match Lua "target_socket = {}" */
    tlm_utils::simple_target_socket<Usart2, DEFAULT_TLM_BUSWIDTH> socket;

    /* Character-stream / bridge backend */
    gs::biflow_socket<Usart2> backend_socket;

    /* Combined interrupt output (SC_ZERO_OR_MORE_BOUND — optional GIC) */
    InitiatorSignalSocket<bool> irq;

    SC_HAS_PROCESS(Usart2);

    Usart2(sc_core::sc_module_name name)
        : sc_module(name)
        , socket("target_socket")
        , backend_socket("backend_socket")
        , irq("irq")
        , m_usart("usart_core")
        , m_sig_clk("sig_clk")
        , m_sig_rst("sig_rst")
        , m_sig_txd("sig_txd")
        , m_sig_rxd("sig_rxd")
        , m_sig_tbir("sig_tbir")
        , m_sig_tir("sig_tir")
        , m_sig_rir("sig_rir")
        , m_sig_eir("sig_eir")
        , m_bus_fwd("bus_fwd")
        , m_clk_hz("clk_hz", 100000000ULL,
                   "Peripheral clock frequency in Hz (default 100 MHz)")
        , m_irq_state(false)
        , m_status(0u)
        , m_tbuf_write_count(0u)
        , m_tbir_fired_count(0u)
        , m_tir_fired_count(0u)
        , m_rx_accept_count(0u)
        , m_rir_fired_count(0u)
        , m_rx_overrun_count(0u)
        , m_eir_fired_count(0u)
        , m_sig_txd_line("sig_txd_line")
        , m_sig_rxd_line("sig_rxd_line")
        , m_sig_irq_out("sig_irq_out")
        , m_trace_tbir("trace_tbir")
        , m_trace_tir("trace_tir")
        , m_trace_rir("trace_rir")
        , m_trace_eir("trace_eir")
        , m_trace_irq("trace_irq")
        , m_sig_txd_wave("sig_txd_wave")
        , m_sig_rxd_wave("sig_rxd_wave")
        , m_txd_wave_cnt(0u)
        , m_rxd_wave_cnt(0u)
        , m_pending_trace_irqs(0u)
        , m_vcd_file("vcd_file", "",
                     "VCD output file basename; empty = disabled")
        , m_tf(nullptr)
    {
        SCP_TRACE(()) << "Usart2 constructor";

        m_bus_fwd.bind(m_usart.bus);

        m_usart.usartClkIn.bind(m_sig_clk);
        m_usart.rst.bind(m_sig_rst);
        m_usart.txd.bind(m_sig_txd);
        m_usart.rxd.bind(m_sig_rxd);
        m_usart.tbir.bind(m_sig_tbir);
        m_usart.tir.bind(m_sig_tir);
        m_usart.rir.bind(m_sig_rir);
        m_usart.eir.bind(m_sig_eir);

        socket.register_b_transport(this, &Usart2::b_transport);
        backend_socket.register_b_transport(this, &Usart2::rx_receive);
        /* do NOT call can_receive_set() here — sockets not yet bound */

        /* irq_method: fires when m_irq_event is notified by update_status_from_core.
         * m_irq_event is a gs::async_event with async_attach_suspending() active,
         * which keeps SystemC's m_has_suspending_channels=true.  This ensures the
         * SC kernel's async_suspend() blocks on the semaphore rather than returning
         * immediately, so QEMU's async_request_update() always wakes SC before the
         * simulation is declared deadlocked.  Without this, can_receive_any() leaves
         * all biflow sockets detached, SC sees no suspending channels, and the
         * suspend() call returns instantly — creating a race where SC terminates
         * before processing the pending IRQ notification.                        */
        SC_METHOD(irq_method);
        sensitive << m_irq_event;
        dont_initialize();

        /* irq_pulse_method: second half of the GIC pulse — drives HIGH.       */
        SC_METHOD(irq_pulse_method);
        sensitive << m_irq_pulse_event;
        dont_initialize();

        /* irq_deassert_method: fires when the Usart core's IRQ sc_signals
         * change.  When all four sources are deasserted (pulses expired via
         * updateIrqPulses inside advance()), drives irq LOW.  This ensures
         * the GIC never sees irq permanently HIGH, which would cause infinite
         * re-delivery if the GIC is in level-sensitive mode.                  */
        SC_METHOD(irq_deassert_method);
        sensitive << m_sig_tbir << m_sig_tir << m_sig_rir << m_sig_eir;
        dont_initialize();

        /* txd_enqueue_method: woken by m_sig_txd_wave (written from QEMU thread
         * via SC_MANY_WRITERS async path) when a TBUF byte is queued.  Starts
         * the txd bit-state-machine if it is idle.                          */
        SC_METHOD(txd_enqueue_method);
        sensitive << m_sig_txd_wave;
        dont_initialize();

        /* txd_wave_method: bit-level state machine for TXD waveform + TIR pulse.
         * States: 0=START, 1-8=DATA bits, 9=STOP, 10=TIR_assert, 11=TIR_deassert.
         * Each state drives m_sig_txd_line and schedules the next state via
         * m_txd_wave_ev.notify(baud_period).  No next_trigger(), no SC_THREAD. */
        SC_METHOD(txd_wave_method);
        sensitive << m_txd_wave_ev;
        dont_initialize();

        /* rxd_enqueue_method / rxd_wave_method: same pattern for the RX pin.  */
        SC_METHOD(rxd_enqueue_method);
        sensitive << m_sig_rxd_wave;
        dont_initialize();

        SC_METHOD(rxd_wave_method);
        sensitive << m_rxd_wave_ev;
        dont_initialize();

        /* Expire SC_METHODs: deassert trace pulse signals after 2 clk periods.
         * Notified by irq_method (tbir/rir/eir) and irq_pulse_method (irq). */
        SC_METHOD(tbir_expire_method); sensitive << m_tbir_expire_ev; dont_initialize();
        SC_METHOD(rir_expire_method);  sensitive << m_rir_expire_ev;  dont_initialize();
        SC_METHOD(eir_expire_method);  sensitive << m_eir_expire_ev;  dont_initialize();
        SC_METHOD(irq_expire_method);  sensitive << m_irq_expire_ev;  dont_initialize();
    }

    void start_of_simulation() override
    {
        /* Use can_receive_any() (INFINITE_VALUE) rather than can_receive_set(N).
         *
         * can_receive_set(N) sends ABSOLUTE_VALUE to the peer, which calls
         * async_attach_suspending() on the peer's m_send_event.  When QEMU's
         * b_transport thread later delivers bytes (via force_send chain) and
         * the peer calls enqueue() → m_send_event.notify(), the notify may
         * try to acquire the SystemC kernel lock while the SystemC thread holds
         * it — ABBA deadlock.
         *
         * can_receive_any() sends INFINITE_VALUE, sets m_infinite=true on the
         * peer and does NOT call async_attach_suspending(), making it safe for
         * any thread to enqueue bytes on the peer socket. */
        backend_socket.can_receive_any();
        m_sig_rst.write(false);

        uint64_t hz = m_clk_hz.get_value();
        if (hz > 0u) {
            double period_s = 1.0 / static_cast<double>(hz);
            m_sig_clk.write(sc_core::sc_time(period_s, sc_core::SC_SEC));
        } else {
            SCP_WARN(()) << "clk_hz is 0 — Usart core clock not configured";
        }

        std::string vcd = m_vcd_file.get_value();
        if (!vcd.empty()) {
            m_tf = sc_core::sc_create_vcd_trace_file(vcd.c_str());
            if (m_tf) {
                /* Seed idle high levels before trace file records t=0. */
                m_sig_txd_line.write(true);
                m_sig_rxd_line.write(true);
                std::string n = name();
                sc_core::sc_trace(m_tf, m_sig_txd_line, n + ".txd");
                sc_core::sc_trace(m_tf, m_sig_rxd_line, n + ".rxd");
                sc_core::sc_trace(m_tf, m_trace_tbir,   n + ".tbir");
                sc_core::sc_trace(m_tf, m_trace_tir,    n + ".tir");
                sc_core::sc_trace(m_tf, m_trace_rir,    n + ".rir");
                sc_core::sc_trace(m_tf, m_trace_eir,    n + ".eir");
                sc_core::sc_trace(m_tf, m_trace_irq,    n + ".irq");
            }
        }
    }

    void end_of_simulation() override
    {
        if (m_tf) sc_core::sc_close_vcd_trace_file(m_tf);
    }

private:
    /* ── Compile-time constants ─────────────────────────────────────────── */
    static constexpr int      RX_FIFO_DEPTH        = 16;
    static constexpr uint64_t USART2_STATUS_OFFSET = 0x20u;

    /* STATUS register bit masks (sticky, W1C) */
    static constexpr uint32_t STATUS_TBIR = (1u << 0);
    static constexpr uint32_t STATUS_TIR  = (1u << 1);
    static constexpr uint32_t STATUS_RIR  = (1u << 2);
    static constexpr uint32_t STATUS_EIR  = (1u << 3);

    /* ── Sub-modules and sockets ────────────────────────────────────────── */
    Usart m_usart;
    tlm_utils::simple_initiator_socket<Usart2, DEFAULT_TLM_BUSWIDTH> m_bus_fwd;

    /* ── Internal signals ───────────────────────────────────────────────── */
    sc_core::sc_signal<sc_core::sc_time> m_sig_clk;
    sc_core::sc_signal<bool>             m_sig_rst;
    sc_core::sc_signal<USART_TxRx_Tlm>  m_sig_txd;
    sc_core::sc_signal<USART_TxRx_Tlm>  m_sig_rxd;
    /* SC_MANY_WRITERS: the Usart core drives these from two process contexts
     * (sendall SC_METHOD via rx_inject, and jobs_handler SC_THREAD via sync/
     * advance). SC_ONE_WRITER (default) would raise E115; SC_MANY_WRITERS
     * suppresses that check while still delivering value-change events. */
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_tbir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_tir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_rir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_eir;

    /* ── CCI parameters ─────────────────────────────────────────────────── */
    cci::cci_param<uint64_t> m_clk_hz;

    /* ── GIC IRQ async wakeup event ─────────────────────────────────────── */
    /* m_irq_event uses gs::async_event (start_attached=true by default) so that
     * async_attach_suspending() is called during construction, keeping
     * m_has_suspending_channels=true for the lifetime of simulation.  This
     * ensures the SC kernel's async_suspend() blocks on its semaphore rather
     * than returning immediately; QEMU's async_request_update() (posted from
     * update_status_from_core via any thread) unblocks SC reliably, eliminating
     * the race where SC terminates before processing the pending IRQ update. */
    gs::async_event m_irq_event;

    /* ── GIC IRQ output state ───────────────────────────────────────────── */
    bool m_irq_state;

    /* ── STATUS register (sticky; updated directly from IRQ assert times) ─ */
    uint32_t m_status;

    /* Per-event counters for reliable edge detection in same-quantum VP mode.
     *
     * In QBOX multithread-unconstrained, all b_transport calls within one
     * quantum share the same sc_time_stamp().  advance() returns early when
     * now <= m_last_advance_time, so the Usart core's 2-period IRQ pulses
     * never expire.  is_tbir_asserted() stays true indefinitely, making
     * prev-based rising-edge detection permanently blind after the first fire.
     *
     * Solution: count TBUF writes vs. TBIR fires.  One TBIR per write,
     * one RIR per successful rx_inject, one EIR per overrun — regardless
     * of whether the baud clock has advanced.
     *
     * TIR (TX-frame complete) still uses prev-based detection because it
     * requires stepTx() to run via advance().  In same-quantum mode TIR
     * will not fire; Test 5 is expected to fail in that scenario.          */
    unsigned int m_tbuf_write_count;   /* incremented per TBUF write in b_transport  */
    unsigned int m_tbir_fired_count;   /* matched to m_tbuf_write_count when TBIR set */
    unsigned int m_tir_fired_count;    /* matched to get_tir_fire_count() when TIR set */
    unsigned int m_rx_accept_count;    /* incremented per successful rx_inject call   */
    unsigned int m_rir_fired_count;    /* matched to m_rx_accept_count when RIR set   */
    unsigned int m_rx_overrun_count;   /* incremented per failed rx_inject (overrun)  */
    unsigned int m_eir_fired_count;    /* matched to m_rx_overrun_count when EIR set  */

    /* Event for the second half of the GIC LOW→HIGH pulse: irq_method() drives
     * LOW and notifies this event; irq_pulse_method() drives HIGH next delta. */
    sc_core::sc_event m_irq_pulse_event;

    /* ── update_status_from_core ─────────────────────────────────────────
     * Poll the Usart core's IRQ assert timestamps and set sticky STATUS bits
     * on rising edges.  Must be called after any operation that may fire an
     * IRQ (TBUF write, rx_inject, sync).  Safe to call from QEMU's thread
     * since it only reads/writes module-private data — no SystemC scheduler
     * interaction required.
     * ──────────────────────────────────────────────────────────────────── */
    void update_status_from_core()
    {
        /* No GIC connection — skip entirely.  USART_C (console) has irq.size()==0;
         * generating m_sig_irq_seq events there creates unnecessary SC-scheduler
         * churn on every put_char() call, which in practice causes the SC thread
         * to thrash while QEMU is waiting for GIC interrupt delivery. */
        if (irq.size() == 0u) return;

        bool new_irq = false;

        /* ── TBIR ─────────────────────────────────────────────────────────────
         * Fire once per TBUF write.  We do NOT check is_tbir_asserted() here.
         *
         * In same-quantum VP execution, sc_time_stamp() is constant across all
         * b_transport calls, so advance() returns early every time and the
         * Usart core's 2-period IRQ pulse never expires.  is_tbir_asserted()
         * therefore stays true indefinitely after the first TBUF write.  Using
         * it as a rising-edge signal would leave m_irq_tbir_prev permanently
         * stuck at true after the first interrupt, silencing all subsequent
         * TBIR events.
         *
         * The write-count mismatch is sufficient: the Usart core's assertIrq
         * for TBIR is always called (either by loadTSR immediately if TSR is
         * idle, or by stepTx later — but in either case update_status_from_core
         * is called right after the TBUF b_transport, and the count guarantees
         * exactly one STATUS_TBIR set per write regardless of timing.          */
        if (m_tbuf_write_count > m_tbir_fired_count) {
            m_status |= STATUS_TBIR;
            m_tbir_fired_count = m_tbuf_write_count;
            new_irq = true;
            m_pending_trace_irqs.fetch_or(STATUS_TBIR, std::memory_order_relaxed);
            SCP_INFO(()) << "TBIR fired (tbuf_write_count=" << m_tbuf_write_count << ")";
        }

        /* ── TIR ──────────────────────────────────────────────────────────────
         * TX-frame complete — fires from stepTx() via advance().
         *
         * Prev-based detection (is_tir_asserted()) cannot be used here because
         * updateIrqPulses() runs at the end of advance() BEFORE this function
         * is called.  When the elapsed simulation time is large (e.g. 10 ms
         * after a WFI gap), updateIrqPulses sees (now - assert_time) >> 2 clk
         * and immediately de-asserts TIR, so is_tir_asserted() always returns
         * false by the time we check it.
         *
         * m_tir_fire_count is incremented by stepTx() at the exact moment TIR
         * is asserted and is never touched by updateIrqPulses(), so it reliably
         * survives the pulse-expiry window.                                     */
        {
            unsigned int tir_count = m_usart.get_tir_fire_count();
            if (tir_count > m_tir_fired_count) {
                m_status |= STATUS_TIR;
                m_tir_fired_count = tir_count;
                new_irq = true;
                SCP_INFO(()) << "TIR  fired (tir_fire_count=" << tir_count << ")";
            }
        }

        /* ── RIR ──────────────────────────────────────────────────────────────
         * Fire once per successful rx_inject.  Same-quantum issue as TBIR:
         * is_rir_asserted() stays true after the first injection; the accept-
         * count prevents re-firing for the same injection event.              */
        if (m_usart.is_rir_asserted() && m_rx_accept_count > m_rir_fired_count) {
            m_status |= STATUS_RIR;
            m_rir_fired_count = m_rx_accept_count;
            new_irq = true;
            m_pending_trace_irqs.fetch_or(STATUS_RIR, std::memory_order_relaxed);
            SCP_INFO(()) << "RIR  fired (rx_accept_count=" << m_rx_accept_count << ")";
        }

        /* ── EIR ──────────────────────────────────────────────────────────────
         * Fire once per overrun (failed rx_inject when RBUF full).            */
        if (m_usart.is_eir_asserted() && m_rx_overrun_count > m_eir_fired_count) {
            m_status |= STATUS_EIR;
            m_eir_fired_count = m_rx_overrun_count;
            new_irq = true;
            m_pending_trace_irqs.fetch_or(STATUS_EIR, std::memory_order_relaxed);
            SCP_INFO(()) << "EIR  fired (rx_overrun_count=" << m_rx_overrun_count << ")";
        }

        if (new_irq) {
            m_irq_event.notify(sc_core::SC_ZERO_TIME);
        }
    }

    /* ── b_transport ─────────────────────────────────────────────────────
     * Handles:
     *   offset 0x20: STATUS register (read=sticky bits, write=W1C)
     *   offset 0x04: TBUF write — intercept byte → backend, forward to core
     *   all others:  forward directly to Usart core
     * ──────────────────────────────────────────────────────────────────── */
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay)
    {
        uint64_t offset = trans.get_address();

        /* ── STATUS register ────────────────────────────────────────────── */
        if (offset == USART2_STATUS_OFFSET) {
            /* Do NOT call m_usart.sync()/advance() here.
             *
             * In QBOX multithread-unconstrained, STATUS reads are called from
             * jobs_handler (QEMU's b_transport goes through RunOnSysc).  If
             * m_usart.sync() → advance(sc_time_stamp()) is called here and the
             * simulation time has advanced far ahead of the last advance() call
             * (because SystemC jumped forward while QEMU ran between quanta),
             * advance() would loop over potentially millions of baud-clock ticks,
             * causing a multi-second stall for each STATUS read in the polling
             * loop — effectively a hang.
             *
             * All STATUS bits are already kept up-to-date synchronously:
             *   TBIR / TIR : set in update_status_from_core() after every
             *                TBUF write (the TBUF write forwards to the core
             *                via m_bus_fwd which calls advance internally).
             *   RIR / EIR  : set in rx_receive() via update_status_from_core()
             *                immediately when the biflow byte arrives.
             *
             * The STATUS register is sticky (W1C) so no update is missed.
             * For Test 5 (TIR), TIR fires during the TBUF write's advance()
             * call and is captured there; no additional advance is needed here.
             */

            if (trans.get_command() == tlm::TLM_READ_COMMAND) {
                std::memcpy(trans.get_data_ptr(), &m_status, sizeof(m_status));
                trans.set_dmi_allowed(false);
                trans.set_response_status(tlm::TLM_OK_RESPONSE);
            } else {
                uint32_t wval = 0u;
                std::memcpy(&wval, trans.get_data_ptr(), sizeof(wval));
                m_status &= ~wval; /* write-1-to-clear */
                trans.set_response_status(tlm::TLM_OK_RESPONSE);
            }
            return;
        }

        /* ── RBUF read: return data directly without calling advance() ─────
         *
         * Forwarding RBUF reads to m_bus_fwd triggers advance(sc_time_stamp())
         * in the Usart core, which replays every baud-clock tick since the last
         * advance() call.  After a test that let simulation time run (e.g. test 1
         * whose sc_stop shows ~1.9 SC_SEC elapsed), this can mean 190 million
         * 100 MHz ticks — seconds of wall-clock stall.
         *
         * get_rbuf_raw() / clear_rbuf() read and clear m_rbuf directly without
         * calling advance(), matching exactly what m_bus_fwd would return once
         * the core state is up to date.  rx_inject() already wrote the byte into
         * m_rbuf and set m_rbuf_full when the byte arrived, so the value is
         * always fresh.                                                          */
        if (trans.get_command() == tlm::TLM_READ_COMMAND &&
            offset == USART_RBUF_OFFSET)
        {
            uint32_t data = m_usart.get_rbuf_raw();
            std::memcpy(trans.get_data_ptr(), &data, sizeof(data));
            m_usart.clear_rbuf();
            trans.set_dmi_allowed(false);
            trans.set_response_status(tlm::TLM_OK_RESPONSE);
            return;
        }

        /* ── TBUF write: send byte to biflow backend ────────────────────── */
        if (trans.get_command() == tlm::TLM_WRITE_COMMAND &&
            offset == USART_TBUF_OFFSET)
        {
            const uint8_t* ptr = trans.get_data_ptr();
            unsigned       len = trans.get_data_length();
            uint32_t       val = 0u;
            switch (len) {
            case 1: val = *ptr;                                     break;
            case 2: val = *reinterpret_cast<const uint16_t*>(ptr); break;
            case 4: val = *reinterpret_cast<const uint32_t*>(ptr); break;
            default: break;
            }
            /* Send byte to biflow backend synchronously via force_send.
             *
             * biflow_socket::enqueue() posts to an internal queue and notifies
             * an async_event.  In QBOX's multithread-unconstrained model the
             * QEMU thread calls b_transport (and therefore this code) while
             * holding no SystemC locks, but async_event::notify() calls
             * async_request_update() which—in some SystemC 3.0.2 builds—
             * acquires the kernel update mutex.  That mutex may already be
             * held by the SystemC scheduler thread, causing a deadlock.
             *
             * force_send() calls output_socket->b_transport() directly without
             * going through the async queue, so it is safe to call from any
             * thread context.  The chain:
             *   usart2_a → SerialBridge.recv_from_a → socket_b.enqueue
             *   → (async) → usart2_b.rx_receive → rx_inject → RIR
             * remains asynchronous for the B side (socket_b is in a
             * SystemC context), so no further deadlock arises there. */
            {
                uint8_t byte = static_cast<uint8_t>(val & 0xFFu);
                tlm::tlm_generic_payload fwd_txn;
                fwd_txn.set_command(tlm::TLM_WRITE_COMMAND);
                fwd_txn.set_data_ptr(&byte);
                fwd_txn.set_data_length(1);
                fwd_txn.set_streaming_width(1);
                fwd_txn.set_byte_enable_ptr(nullptr);
                fwd_txn.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
                backend_socket.force_send(fwd_txn);
            /* Count this write so update_status_from_core() fires exactly one
             * TBIR per TBUF write, independent of whether advance() has run. */
            ++m_tbuf_write_count;
            if (m_tf) {
                {
                    std::lock_guard<std::mutex> lk(m_txd_wave_mtx);
                    m_txd_wave_queue.push(static_cast<uint8_t>(val & 0xFFu));
                }
                m_sig_txd_wave.write(++m_txd_wave_cnt);
            }
            }
        }

        /* ── Forward all other offsets to Usart core ───────────────────── */
        trans.set_address(offset + m_usart.base_addr.get_value());
        m_bus_fwd->b_transport(trans, delay);
        trans.set_address(offset);

        /* ── Capture TBIR (and any other IRQs) that fired during the above */
        update_status_from_core();
    }

    /* ── rx_receive ──────────────────────────────────────────────────────
     * Called when the biflow backend (bridge or stdio) delivers bytes.
     * Inject each byte directly into the Usart core's RBUF.
     * STATUS[RIR] / STATUS[EIR] are set here via update_status_from_core().
     * ──────────────────────────────────────────────────────────────────── */
    void rx_receive(tlm::tlm_generic_payload& txn, sc_core::sc_time& /*t*/)
    {
        uint8_t* ptr = txn.get_data_ptr();
        unsigned len = txn.get_data_length();

        for (unsigned i = 0u; i < len; ++i) {
            if (m_usart.rx_inject(ptr[i])) {
                /* Accepted: byte stored in RBUF, RIR asserted by core. */
                ++m_rx_accept_count;
                if (m_tf) {
                    {
                        std::lock_guard<std::mutex> lk(m_rxd_wave_mtx);
                        m_rxd_wave_queue.push(ptr[i]);
                    }
                    m_sig_rxd_wave.write(++m_rxd_wave_cnt);
                }
            } else {
                /* Overrun: RBUF full, EIR asserted by core (if OEN=1). */
                ++m_rx_overrun_count;
                SCP_WARN(()) << "rx_receive: overrun — byte 0x"
                             << std::hex << static_cast<unsigned>(ptr[i])
                             << " dropped (EIR fires if OEN=1)";
            }
            update_status_from_core();
        }
        /* No can_receive_more() — backend_socket uses can_receive_any() (infinite
         * credits), so calling can_receive_more() would send DELTA_CHANGE on a
         * socket with m_infinite=true, tripping sc_assert(m_infinite == false). */
    }

    /* ── irq_method ──────────────────────────────────────────────────────
     * Fires when m_sig_irq_seq changes — i.e., whenever update_status_from_core()
     * detects a new IRQ rising edge.  Generates a GIC LOW→HIGH pulse:
     *   • This delta: drive irq LOW (ensures falling edge before next rising)
     *   • Next delta: irq_pulse_method drives irq HIGH (rising edge for GIC)
     *
     * Always doing LOW→HIGH rather than conditional writes means the GIC sees
     * a fresh rising edge for every IRQ event regardless of current irq level.
     * ──────────────────────────────────────────────────────────────────── */
    void irq_method()
    {
        if (irq.size() > 0) irq->write(false);
        m_sig_irq_out.write(false);
        m_irq_state = false;
        m_irq_pulse_event.notify(sc_core::SC_ZERO_TIME);

        if (m_tf) {
            uint32_t mask = m_pending_trace_irqs.exchange(0u,
                                std::memory_order_acquire);
            sc_core::sc_time two_clk = 2u * m_sig_clk.read();
            if (two_clk != sc_core::SC_ZERO_TIME) {
                if (mask & STATUS_TBIR) {
                    m_trace_tbir.write(true);
                    m_tbir_expire_ev.notify(two_clk);
                }
                if (mask & STATUS_RIR) {
                    m_trace_rir.write(true);
                    m_rir_expire_ev.notify(two_clk);
                }
                if (mask & STATUS_EIR) {
                    m_trace_eir.write(true);
                    m_eir_expire_ev.notify(two_clk);
                }
            }
        }
    }

    /* ── irq_pulse_method ────────────────────────────────────────────────
     * Second half of the LOW→HIGH GIC pulse: drive irq HIGH so the GIC
     * latches the rising edge.
     * ──────────────────────────────────────────────────────────────────── */
    void irq_pulse_method()
    {
        m_irq_state = true;
        if (irq.size() > 0) {
            irq->write(true);
        }
        m_sig_irq_out.write(true);
        if (m_tf) {
            m_trace_irq.write(true);
            sc_core::sc_time two_clk = 2u * m_sig_clk.read();
            if (two_clk != sc_core::SC_ZERO_TIME)
                m_irq_expire_ev.notify(two_clk);
        }
    }

    /* ── irq_deassert_method ─────────────────────────────────────────────
     * Fires when any of the four IRQ sc_signals change (driven by the Usart
     * core's updateIrqPulses / assertIrq).  When all sources are deasserted,
     * drives irq LOW so the GIC does not continuously re-deliver the interrupt
     * in level-sensitive mode and so the next event gets a proper rising edge.
     *
     * The coalescing race (write(false)+write(true) in the same b_transport)
     * is NOT a problem here: if a new pulse fires simultaneously with an
     * old pulse deassert, the net sc_signal value is HIGH (true), so
     * `asserted` is true and we do NOT deassert.  The new interrupt is handled
     * by irq_method (via m_sig_irq_seq).
     * ──────────────────────────────────────────────────────────────────── */
    void irq_deassert_method()
    {
        bool asserted = m_sig_tbir.read() || m_sig_tir.read() ||
                        m_sig_rir.read()  || m_sig_eir.read();
        if (!asserted && m_irq_state) {
            m_irq_state = false;
            if (irq.size() > 0) irq->write(false);
            m_sig_irq_out.write(false);
        }
    }
    /* ── IRQ pulse expire methods ───────────────────────────────────────────
     * Each SC_METHOD fires after a timed sc_event::notify() from irq_method
     * (tbir/rir/eir) or irq_pulse_method (irq), 2 f_PERIPH clock periods
     * after the pulse was asserted.  No next_trigger(); no SC_THREAD.
     * ──────────────────────────────────────────────────────────────────── */
    void tbir_expire_method() { m_trace_tbir.write(false); }
    void rir_expire_method()  { m_trace_rir.write(false);  }
    void eir_expire_method()  { m_trace_eir.write(false);  }
    void irq_expire_method()  { m_trace_irq.write(false);  }

    /* ── TXD bit-level state machine ────────────────────────────────────────
     *
     * txd_enqueue_method: woken (via SC_MANY_WRITERS async path) when b_transport
     * queues a new byte.  If the state machine is idle, dequeues the byte and
     * fires m_txd_wave_ev immediately so txd_wave_method starts bit 0.
     *
     * txd_wave_method: one call per bit period.  Drives m_sig_txd_line for the
     * current state, advances the state counter, and schedules the next firing
     * via m_txd_wave_ev.notify(baud_period).  After the STOP bit it drives a
     * 2-clock-period TIR pulse, then checks the queue for the next byte.
     *
     * m_txd_wave_state:  -1=idle  0=START  1..8=DATA  9=STOP  10=TIR_on  11=TIR_off
     * ──────────────────────────────────────────────────────────────────── */
    void txd_enqueue_method()
    {
        if (m_txd_wave_state != -1) return;  // busy — byte stays in queue
        uint8_t byte;
        {
            std::lock_guard<std::mutex> lk(m_txd_wave_mtx);
            if (m_txd_wave_queue.empty()) return;
            byte = m_txd_wave_queue.front();
            m_txd_wave_queue.pop();
        }
        m_txd_wave_byte  = byte;
        m_txd_wave_state = 0;
        m_txd_wave_ev.notify(sc_core::SC_ZERO_TIME);
    }

    void txd_wave_method()
    {
        sc_core::sc_time bp = m_usart.get_baud_period();
        sc_core::sc_time cp = m_sig_clk.read();
        if (bp == sc_core::SC_ZERO_TIME) { m_txd_wave_state = -1; return; }

        switch (m_txd_wave_state) {
        case 0:                              // START bit
            m_sig_txd_line.write(false);
            m_txd_wave_state = 1;
            m_txd_wave_ev.notify(bp);
            break;
        case 1: case 2: case 3: case 4:
        case 5: case 6: case 7: case 8:     // DATA bits D0..D7
            m_sig_txd_line.write(
                (m_txd_wave_byte >> (m_txd_wave_state - 1)) & 1u);
            ++m_txd_wave_state;
            m_txd_wave_ev.notify(bp);
            break;
        case 9:                              // STOP bit
            m_sig_txd_line.write(true);
            m_txd_wave_state = (m_tf && cp != sc_core::SC_ZERO_TIME) ? 10 : 11;
            m_txd_wave_ev.notify(bp);
            break;
        case 10:                             // TIR assert (after STOP bit elapses)
            m_trace_tir.write(true);
            m_txd_wave_state = 11;
            m_txd_wave_ev.notify(2u * cp);
            break;
        default:                             // state 11: TIR deassert, check queue
            m_trace_tir.write(false);
            m_txd_wave_state = -1;
            {   // pick up next queued byte immediately
                std::lock_guard<std::mutex> lk(m_txd_wave_mtx);
                if (!m_txd_wave_queue.empty()) {
                    m_txd_wave_byte = m_txd_wave_queue.front();
                    m_txd_wave_queue.pop();
                    m_txd_wave_state = 0;
                    m_txd_wave_ev.notify(sc_core::SC_ZERO_TIME);
                }
            }
            break;
        }
    }

    /* ── RXD bit-level state machine (same structure, no TIR) ───────────── */
    void rxd_enqueue_method()
    {
        if (m_rxd_wave_state != -1) return;
        uint8_t byte;
        {
            std::lock_guard<std::mutex> lk(m_rxd_wave_mtx);
            if (m_rxd_wave_queue.empty()) return;
            byte = m_rxd_wave_queue.front();
            m_rxd_wave_queue.pop();
        }
        m_rxd_wave_byte  = byte;
        m_rxd_wave_state = 0;
        m_rxd_wave_ev.notify(sc_core::SC_ZERO_TIME);
    }

    void rxd_wave_method()
    {
        sc_core::sc_time bp = m_usart.get_baud_period();
        if (bp == sc_core::SC_ZERO_TIME) { m_rxd_wave_state = -1; return; }

        switch (m_rxd_wave_state) {
        case 0:
            m_sig_rxd_line.write(false);
            m_rxd_wave_state = 1;
            m_rxd_wave_ev.notify(bp);
            break;
        case 1: case 2: case 3: case 4:
        case 5: case 6: case 7: case 8:
            m_sig_rxd_line.write(
                (m_rxd_wave_byte >> (m_rxd_wave_state - 1)) & 1u);
            ++m_rxd_wave_state;
            m_rxd_wave_ev.notify(bp);
            break;
        case 9:
            m_sig_rxd_line.write(true);
            m_rxd_wave_state = -1;
            {
                std::lock_guard<std::mutex> lk(m_rxd_wave_mtx);
                if (!m_rxd_wave_queue.empty()) {
                    m_rxd_wave_byte = m_rxd_wave_queue.front();
                    m_rxd_wave_queue.pop();
                    m_rxd_wave_state = 0;
                    m_rxd_wave_ev.notify(bp);  // back-to-back: gap = STOP period
                }
            }
            break;
        default:
            m_rxd_wave_state = -1;
            break;
        }
    }

    /* ── VCD trace signals and file ─────────────────────────────────────── */
    sc_core::sc_signal<bool>    m_sig_txd_line; ///< TXD physical line (driven by state machine)
    sc_core::sc_signal<bool>    m_sig_rxd_line; ///< RXD physical line (driven by state machine)
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_irq_out; ///< mirror of irq output

    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_tbir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_tir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_rir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_eir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_irq;

    sc_core::sc_event m_tbir_expire_ev;
    sc_core::sc_event m_rir_expire_ev;
    sc_core::sc_event m_eir_expire_ev;
    sc_core::sc_event m_irq_expire_ev;

    /* Wake-up signals for the txd/rxd state machines.  Written from QEMU
     * thread (txd) or SC biflow thread (rxd) via SC_MANY_WRITERS async path. */
    sc_core::sc_signal<unsigned int, sc_core::SC_MANY_WRITERS> m_sig_txd_wave;
    sc_core::sc_signal<unsigned int, sc_core::SC_MANY_WRITERS> m_sig_rxd_wave;
    std::atomic<unsigned int> m_txd_wave_cnt;
    std::atomic<unsigned int> m_rxd_wave_cnt;

    /* State machines for txd/rxd waveform generation. */
    std::queue<uint8_t> m_txd_wave_queue;
    std::mutex          m_txd_wave_mtx;
    int                 m_txd_wave_state{-1};   ///< -1=idle, 0=START, 1-8=DATA, 9=STOP, 10-11=TIR
    uint8_t             m_txd_wave_byte{0};
    sc_core::sc_event   m_txd_wave_ev;

    std::queue<uint8_t> m_rxd_wave_queue;
    std::mutex          m_rxd_wave_mtx;
    int                 m_rxd_wave_state{-1};   ///< -1=idle, 0=START, 1-8=DATA, 9=STOP
    uint8_t             m_rxd_wave_byte{0};
    sc_core::sc_event   m_rxd_wave_ev;

    /* Pending trace IRQ mask — ORed from QEMU thread, consumed in irq_method. */
    std::atomic<uint32_t> m_pending_trace_irqs;

    cci::cci_param<std::string> m_vcd_file; ///< VCD basename; empty = disabled
    sc_core::sc_trace_file*     m_tf;       ///< null when not tracing
};

extern "C" void module_register();
