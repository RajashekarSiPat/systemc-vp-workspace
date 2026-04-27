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
 *   RX: backend_socket receives bytes → deferred via rxd_pending_q → rx_inject()
 *       after one full frame duration, to model serial receive time.
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
 * ── TX waveform model ────────────────────────────────────────────────────────
 *
 *   When a byte is written to TBUF, its complete frame is presented upfront:
 *   the full frame duration  =  (1 + data_bits + parity + stop_bits) × baud_period
 *   is computed once and a single SC event is scheduled for frame completion.
 *   m_sig_txd_line is driven LOW at frame-start and HIGH at frame-end.
 *   No per-bit state machine; no periodic serial-clock scheduling.
 *
 * ── RX waveform / injection model ────────────────────────────────────────────
 *
 *   Bytes arriving from the biflow backend are held in rxd_pending_q.
 *   A deferred SC event fires after one frame_duration; only then is the byte
 *   injected into the Usart core's RBUF and RIR asserted.  This models the
 *   time the serial line is occupied without simulating individual bit samples.
 *   m_sig_rxd_line is driven LOW at frame-start and HIGH at frame-end.
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
 *     • Inside rx_done_method after rx_inject (catches RIR / EIR)
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
        , m_txd_busy(false)
        , m_rxd_busy(false)
        , m_rxd_frame_byte(0u)
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

        SC_METHOD(irq_method);
        sensitive << m_irq_event;
        dont_initialize();

        SC_METHOD(irq_pulse_method);
        sensitive << m_irq_pulse_event;
        dont_initialize();

        SC_METHOD(irq_deassert_method);
        sensitive << m_sig_tbir << m_sig_tir << m_sig_rir << m_sig_eir;
        dont_initialize();

        /* tx_start_method: woken (via gs::async_event) when b_transport queues
         * a byte for TX waveform tracing.  Computes the full frame duration once
         * and schedules m_txd_done_ev — no per-bit stepping.               */
        SC_METHOD(tx_start_method);
        sensitive << m_txd_start_ev;
        dont_initialize();

        /* tx_done_method: fires after one frame_duration.  Drives txd_line idle
         * and chains the next queued byte if any.                           */
        SC_METHOD(tx_done_method);
        sensitive << m_txd_done_ev;
        dont_initialize();

        /* rx_done_method: fires after one frame_duration from byte arrival.
         * Performs the deferred rx_inject into the Usart core's RBUF and
         * asserts RIR / EIR via update_status_from_core().                  */
        SC_METHOD(rx_done_method);
        sensitive << m_rxd_done_ev;
        dont_initialize();

        SC_METHOD(tbir_expire_method); sensitive << m_tbir_expire_ev; dont_initialize();
        SC_METHOD(tir_expire_method);  sensitive << m_tir_expire_ev;  dont_initialize();
        SC_METHOD(rir_expire_method);  sensitive << m_rir_expire_ev;  dont_initialize();
        SC_METHOD(eir_expire_method);  sensitive << m_eir_expire_ev;  dont_initialize();
        SC_METHOD(irq_expire_method);  sensitive << m_irq_expire_ev;  dont_initialize();
    }

    void start_of_simulation() override
    {
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
                m_sig_txd_line.write(true);
                m_sig_rxd_line.write(true);
                std::string n = name();
                sc_core::sc_trace(m_tf, m_sig_txd_line, n + ".txd");
                sc_core::sc_trace(m_tf, m_sig_rxd_line, n + ".rxd");
                sc_core::sc_trace(m_tf, m_sig_txd,      n + ".txd_tlm");
                sc_core::sc_trace(m_tf, m_sig_rxd,      n + ".rxd_tlm");
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
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_tbir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_tir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_rir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_eir;

    /* ── CCI parameters ─────────────────────────────────────────────────── */
    cci::cci_param<uint64_t> m_clk_hz;

    /* ── GIC IRQ async wakeup event ─────────────────────────────────────── */
    gs::async_event m_irq_event;

    /* ── GIC IRQ output state ───────────────────────────────────────────── */
    bool m_irq_state;

    /* ── STATUS register (sticky; updated directly from IRQ assert times) ─ */
    uint32_t m_status;

    /* ── Per-event counters for reliable edge detection ─────────────────── */
    unsigned int m_tbuf_write_count;
    unsigned int m_tbir_fired_count;
    unsigned int m_tir_fired_count;
    unsigned int m_rx_accept_count;
    unsigned int m_rir_fired_count;
    unsigned int m_rx_overrun_count;
    unsigned int m_eir_fired_count;

    sc_core::sc_event m_irq_pulse_event;

    /* ── update_status_from_core ──────────────────────────────────────────*/
    void update_status_from_core()
    {
        if (irq.size() == 0u) return;

        bool new_irq = false;

        if (m_tbuf_write_count > m_tbir_fired_count) {
            m_status |= STATUS_TBIR;
            m_tbir_fired_count = m_tbuf_write_count;
            new_irq = true;
            m_pending_trace_irqs.fetch_or(STATUS_TBIR, std::memory_order_relaxed);
            SCP_INFO(()) << "TBIR fired (tbuf_write_count=" << m_tbuf_write_count << ")";
        }

        {
            unsigned int tir_count = m_usart.get_tir_fire_count();
            if (tir_count > m_tir_fired_count) {
                m_status |= STATUS_TIR;
                m_tir_fired_count = tir_count;
                new_irq = true;
                m_pending_trace_irqs.fetch_or(STATUS_TIR, std::memory_order_relaxed);
                SCP_INFO(()) << "TIR  fired (tir_fire_count=" << tir_count << ")";
            }
        }

        if (m_usart.is_rir_asserted() && m_rx_accept_count > m_rir_fired_count) {
            m_status |= STATUS_RIR;
            m_rir_fired_count = m_rx_accept_count;
            new_irq = true;
            m_pending_trace_irqs.fetch_or(STATUS_RIR, std::memory_order_relaxed);
            SCP_INFO(()) << "RIR  fired (rx_accept_count=" << m_rx_accept_count << ")";
        }

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

    /* ── b_transport ─────────────────────────────────────────────────────── */
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay)
    {
        uint64_t offset = trans.get_address();

        /* ── STATUS register ────────────────────────────────────────────── */
        if (offset == USART2_STATUS_OFFSET) {
            if (trans.get_command() == tlm::TLM_READ_COMMAND) {
                std::memcpy(trans.get_data_ptr(), &m_status, sizeof(m_status));
                trans.set_dmi_allowed(false);
                trans.set_response_status(tlm::TLM_OK_RESPONSE);
            } else {
                uint32_t wval = 0u;
                std::memcpy(&wval, trans.get_data_ptr(), sizeof(wval));
                m_status &= ~wval;
                trans.set_response_status(tlm::TLM_OK_RESPONSE);
            }
            return;
        }

        /* ── RBUF read: bypass advance() ────────────────────────────────── */
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

            uint8_t byte = static_cast<uint8_t>(val & 0xFFu);
            {
                tlm::tlm_generic_payload fwd_txn;
                fwd_txn.set_command(tlm::TLM_WRITE_COMMAND);
                fwd_txn.set_data_ptr(&byte);
                fwd_txn.set_data_length(1);
                fwd_txn.set_streaming_width(1);
                fwd_txn.set_byte_enable_ptr(nullptr);
                fwd_txn.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
                backend_socket.force_send(fwd_txn);
            }
            ++m_tbuf_write_count;

            /* Queue byte for TX waveform tracing and wake the SC-side
             * tx_start_method via async_event (safe from QEMU thread). */
            if (m_tf) {
                {
                    std::lock_guard<std::mutex> lk(m_txd_mtx);
                    m_txd_queue.push(byte);
                }
                m_txd_start_ev.notify(sc_core::SC_ZERO_TIME);
            }
        }

        /* ── Forward all other offsets to Usart core ───────────────────── */
        trans.set_address(offset + m_usart.base_addr.get_value());
        m_bus_fwd->b_transport(trans, delay);
        trans.set_address(offset);

        update_status_from_core();
    }

    /* ── rx_receive ──────────────────────────────────────────────────────────
     * Called when the biflow backend delivers bytes.  Bytes are held in
     * rxd_pending_q; the first byte immediately starts a frame timer
     * (rxd_try_start).  Actual RBUF injection happens in rx_done_method after
     * one frame_duration, modelling the time the serial line is occupied.
     * ──────────────────────────────────────────────────────────────────────── */
    void rx_receive(tlm::tlm_generic_payload& txn, sc_core::sc_time& /*t*/)
    {
        uint8_t* ptr = txn.get_data_ptr();
        unsigned len = txn.get_data_length();
        for (unsigned i = 0u; i < len; ++i)
            m_rxd_pending_q.push(ptr[i]);
        rxd_try_start();
    }

    /* ── irq_method ─────────────────────────────────────────────────────── */
    /* Step 1 of the two-delta IRQ pulse.
     * Drives irq LOW unconditionally — even if it was already LOW — so that
     * the subsequent write(true) in irq_pulse_method always produces a
     * genuine LOW→HIGH rising edge visible to the GIC.  Without this split,
     * write(false)+write(true) in the same SC process coalesce to a no-op
     * when irq is already HIGH (sc_signal last-write-wins within one delta),
     * and the GIC misses the edge after consecutive interrupts.            */
    void irq_method()
    {
        if (irq.size() > 0)
            irq->write(false);          /* force LOW  (delta D)             */
        m_sig_irq_out.write(false);
        m_irq_state = false;
        m_irq_pulse_event.notify(sc_core::SC_ZERO_TIME); /* HIGH in delta D+1 */
    }

    /* ── irq_pulse_method ───────────────────────────────────────────────── */
    /* Step 2: drive HIGH, producing the guaranteed rising edge for the GIC. */
    void irq_pulse_method()
    {
        if (irq.size() > 0)
            irq->write(true);           /* LOW→HIGH edge (delta D+1)        */
        m_sig_irq_out.write(true);
        m_irq_state = true;

        if (m_tf) {
            uint32_t mask = m_pending_trace_irqs.exchange(0u,
                                std::memory_order_acquire);
            sc_core::sc_time two_clk = 2u * m_sig_clk.read();
            if (two_clk != sc_core::SC_ZERO_TIME) {
                if (mask & STATUS_TBIR) {
                    m_trace_tbir.write(true);
                    m_tbir_expire_ev.notify(two_clk);
                }
                if (mask & STATUS_TIR) {
                    m_trace_tir.write(true);
                    m_tir_expire_ev.notify(two_clk);
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
            m_trace_irq.write(true);
            if (two_clk != sc_core::SC_ZERO_TIME)
                m_irq_expire_ev.notify(two_clk);
        }
    }

    /* ── irq_deassert_method ────────────────────────────────────────────── */
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

    void tbir_expire_method() { m_trace_tbir.write(false); }
    void tir_expire_method()  { m_trace_tir.write(false);  }
    void rir_expire_method()  { m_trace_rir.write(false);  }
    void eir_expire_method()  { m_trace_eir.write(false);  }
    void irq_expire_method()  { m_trace_irq.write(false);  }

    /* ── TX frame-level waveform ─────────────────────────────────────────────
     *
     * tx_start_method: pops the next byte from m_txd_queue, drives txd_line
     * LOW (frame in progress), and schedules m_txd_done_ev after exactly one
     * frame_duration.  The full frame content (data + parity + framing) is
     * already known at this point — nothing changes mid-frame.
     *
     * tx_done_method: drives txd_line HIGH (idle) and chains the next queued
     * byte if any, maintaining back-to-back frame timing with no gap.
     * ──────────────────────────────────────────────────────────────────────── */
    void tx_start_method() { txd_try_start(); }

    void tx_done_method()
    {
        m_sig_txd_line.write(true);   // back to idle
        m_txd_busy = false;
        txd_try_start();              // chain next byte if queued
    }

    void txd_try_start()
    {
        if (m_txd_busy) return;
        {
            std::lock_guard<std::mutex> lk(m_txd_mtx);
            if (m_txd_queue.empty()) return;
            m_txd_queue.pop();        // timing only — data already in Usart core
        }
        sc_core::sc_time fd = m_usart.get_frame_duration();
        if (fd == sc_core::SC_ZERO_TIME) return; // baud not yet configured
        m_txd_busy = true;
        m_sig_txd_line.write(false);             // LOW = frame occupying line
        m_txd_done_ev.notify(fd);
    }

    /* ── RX deferred injection ───────────────────────────────────────────────
     *
     * rxd_try_start: if no frame is in-flight, pops the head of rxd_pending_q,
     * drives rxd_line LOW, and schedules m_rxd_done_ev after one frame_duration.
     * If baud is not yet configured, injects immediately (no timing available).
     *
     * rx_done_method: performs the actual rx_inject into the Usart RBUF and
     * calls update_status_from_core() to assert RIR / EIR, then chains the
     * next pending byte.
     * ──────────────────────────────────────────────────────────────────────── */
    void rx_done_method()
    {
        if (m_tf) m_sig_rxd_line.write(true);    // back to idle
        m_rxd_busy = false;
        rxd_inject_frame_byte();
        rxd_try_start();
    }

    void rxd_try_start()
    {
        while (!m_rxd_busy && !m_rxd_pending_q.empty()) {
            m_rxd_frame_byte = m_rxd_pending_q.front();
            m_rxd_pending_q.pop();
            sc_core::sc_time fd = m_usart.get_frame_duration();
            if (fd != sc_core::SC_ZERO_TIME) {
                m_rxd_busy = true;
                if (m_tf) m_sig_rxd_line.write(false); // LOW = frame in progress
                m_rxd_done_ev.notify(fd);
                return;
            }
            /* Baud not yet configured — inject immediately without timing. */
            rxd_inject_frame_byte();
        }
    }

    void rxd_inject_frame_byte()
    {
        if (m_usart.rx_inject(m_rxd_frame_byte)) {
            ++m_rx_accept_count;
        } else {
            ++m_rx_overrun_count;
            SCP_WARN(()) << "rx overrun — byte 0x"
                         << std::hex << static_cast<unsigned>(m_rxd_frame_byte)
                         << " dropped (EIR fires if OEN=1)";
        }
        update_status_from_core();
    }

    /* ── VCD trace signals ──────────────────────────────────────────────── */
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_txd_line; ///< LOW=frame in progress, HIGH=idle
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_rxd_line; ///< LOW=frame in progress, HIGH=idle
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_sig_irq_out;

    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_tbir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_tir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_rir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_eir;
    sc_core::sc_signal<bool, sc_core::SC_MANY_WRITERS> m_trace_irq;

    sc_core::sc_event m_tbir_expire_ev;
    sc_core::sc_event m_tir_expire_ev;
    sc_core::sc_event m_rir_expire_ev;
    sc_core::sc_event m_eir_expire_ev;
    sc_core::sc_event m_irq_expire_ev;

    /* ── TX frame-level state ───────────────────────────────────────────── */
    gs::async_event          m_txd_start_ev; ///< async: wakes SC from QEMU thread
    sc_core::sc_event        m_txd_done_ev;  ///< fires after frame_duration
    std::queue<uint8_t>      m_txd_queue;    ///< bytes awaiting waveform tracing
    std::mutex               m_txd_mtx;      ///< protects m_txd_queue
    bool                     m_txd_busy;     ///< true while a TX frame timer runs

    /* ── RX deferred-injection state ────────────────────────────────────── */
    sc_core::sc_event        m_rxd_done_ev;      ///< fires after frame_duration
    std::queue<uint8_t>      m_rxd_pending_q;    ///< bytes awaiting injection
    uint8_t                  m_rxd_frame_byte;   ///< byte held for current frame
    bool                     m_rxd_busy;         ///< true while RX frame timer runs

    /* ── Pending trace IRQ mask ─────────────────────────────────────────── */
    std::atomic<uint32_t> m_pending_trace_irqs;

    cci::cci_param<std::string> m_vcd_file;
    sc_core::sc_trace_file*     m_tf;
};

extern "C" void module_register();
