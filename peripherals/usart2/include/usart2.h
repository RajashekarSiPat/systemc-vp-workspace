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

#include <cstdint>
#include <cstring>
#include <systemc>
#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
#include "tlm_utils/simple_initiator_socket.h"
#include <cci_configuration>

#include <ports/initiator-signal-socket.h>
#include <ports/biflow-socket.h>
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
        , m_sig_irq_seq("sig_irq_seq")
        , m_irq_state(false)
        , m_status(0u)
        , m_irq_tbir_prev(false)
        , m_irq_tir_prev(false)
        , m_irq_rir_prev(false)
        , m_irq_eir_prev(false)
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

        /* irq_method: fires when a new IRQ event is detected (m_sig_irq_seq
         * incremented by update_status_from_core).  Generates the first half
         * of the GIC LOW→HIGH pulse: drives LOW, then schedules irq_pulse_method
         * via m_irq_pulse_event to drive HIGH in the next delta cycle.
         * The counter ensures a genuine signal value-change on every new event,
         * avoiding the false/true coalescing issue of sc_signal<bool>.        */
        SC_METHOD(irq_method);
        sensitive << m_sig_irq_seq;
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

    /* ── GIC pulse sequencer ────────────────────────────────────────────── */
    /* m_sig_irq_seq is written by update_status_from_core() (QEMU thread) each
     * time a new IRQ event is detected.  It is monotonically increasing so every
     * write produces a genuine signal value-change, which reliably triggers
     * irq_method() in the SystemC thread regardless of whether the previous
     * pulse's deassert and the new pulse's assert coalesced in the async queue.
     *
     * Using sc_signal<bool> for GIC edge generation was broken because:
     *   advance()  writes  m_sig_tbir.write(false)  (deassert old pulse)
     *   loadTSR()  writes  m_sig_tbir.write(true)   (assert new pulse)
     * Both happen in the same b_transport call from QEMU's thread.  SystemC's
     * async_request_update() batches them: last write wins (true), the pending
     * value equals the current value (true), so NO value-change event fires and
     * irq_method() never runs.  The sequence-number signal avoids this by always
     * changing value on a new IRQ event.                                      */
    sc_core::sc_signal<unsigned int, sc_core::SC_MANY_WRITERS> m_sig_irq_seq;

    /* ── GIC IRQ output state ───────────────────────────────────────────── */
    bool m_irq_state;

    /* ── STATUS register (sticky; updated directly from IRQ assert times) ─ */
    uint32_t m_status;

    /* Previous IRQ assertion states — used for rising-edge detection in
     * update_status_from_core().  Track Usart::is_*_asserted() (timestamp-based),
     * not sc_signal values. */
    bool m_irq_tbir_prev;
    bool m_irq_tir_prev;
    bool m_irq_rir_prev;
    bool m_irq_eir_prev;

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
        bool tbir = m_usart.is_tbir_asserted();
        bool tir  = m_usart.is_tir_asserted();
        bool rir  = m_usart.is_rir_asserted();
        bool eir  = m_usart.is_eir_asserted();

        bool new_irq = false;

        if (tbir && !m_irq_tbir_prev) {
            m_status |= STATUS_TBIR;
            new_irq = true;
            SCP_INFO(()) << "TBIR fired at " << sc_core::sc_time_stamp()
                         << "  (TX buffer freed into TSR)";
        }
        if (tir && !m_irq_tir_prev) {
            m_status |= STATUS_TIR;
            new_irq = true;
            SCP_INFO(()) << "TIR  fired at " << sc_core::sc_time_stamp()
                         << "  (TX frame complete)";
        }
        if (rir && !m_irq_rir_prev) {
            m_status |= STATUS_RIR;
            new_irq = true;
            SCP_INFO(()) << "RIR  fired at " << sc_core::sc_time_stamp()
                         << "  (RX data ready in RBUF)";
        }
        if (eir && !m_irq_eir_prev) {
            m_status |= STATUS_EIR;
            new_irq = true;
            SCP_INFO(()) << "EIR  fired at " << sc_core::sc_time_stamp()
                         << "  (error: OE/FE/PE)";
        }

        m_irq_tbir_prev = tbir;
        m_irq_tir_prev  = tir;
        m_irq_rir_prev  = rir;
        m_irq_eir_prev  = eir;

        if (new_irq) {
            /* Increment the GIC pulse sequencer.  m_sig_irq_seq.read() returns
             * the current committed value (not the pending async value), so
             * even if this is called multiple times before a delta fires, each
             * call writes the same +1 value — guaranteed ≠ current → the signal
             * change event always fires exactly once per batch, waking irq_method
             * to generate one GIC LOW→HIGH pulse per new IRQ event group.      */
            m_sig_irq_seq.write(m_sig_irq_seq.read() + 1u);
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
            if (!m_usart.rx_inject(ptr[i])) {
                SCP_WARN(()) << "rx_receive: overrun — byte 0x"
                             << std::hex << static_cast<unsigned>(ptr[i])
                             << " dropped (EIR fires if OEN=1)";
            }
            /* Capture RIR (accepted) or EIR (overrun, OEN=1) */
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
        m_irq_state = false;
        m_irq_pulse_event.notify(sc_core::SC_ZERO_TIME);
    }

    /* ── irq_pulse_method ────────────────────────────────────────────────
     * Second half of the LOW→HIGH GIC pulse: drive irq HIGH so the GIC
     * latches the rising edge.
     * ──────────────────────────────────────────────────────────────────── */
    void irq_pulse_method()
    {
        m_irq_state = true;
        if (irq.size() > 0) irq->write(true);
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
        }
    }
};

extern "C" void module_register();
