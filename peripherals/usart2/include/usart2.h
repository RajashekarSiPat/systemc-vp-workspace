/*
 * usart2.h — QBOX wrapper for the full-featured Usart peripheral model.
 *
 * Integrates the 4-file USART model (Usart.h / Usart.cc / Usart_types.h /
 * Usart_regdefs.h) from peripherals/usart2/ into the GreenSocs QBOX
 * ModuleFactory + biflow_socket framework.
 *
 * ── Architecture ─────────────────────────────────────────────────────────────
 *
 *   CPU ──TLM──► target_socket (Usart2) ──► Usart::bus (core model)
 *
 *   TX: TBUF write intercepted in b_transport → backend_socket.enqueue(byte)
 *       Usart core still receives the write to update internal state.
 *
 *   IRQ: Usart {tbir,tir,rir,eir} signals combined → single irq output.
 *
 * ── Integration notes ────────────────────────────────────────────────────────
 *
 *   • TBUF writes are intercepted and sent directly to the char backend.
 *     This gives correct byte-level console output without timing-accurate
 *     baud simulation.  The core b_transport is still called to keep its
 *     internal state (advance, BG counter, IRQ pulses) consistent.
 *
 *   • Usart::b_transport subtracts base_addr from the transaction address.
 *     QBOX's router delivers the offset (not the full physical address), so
 *     b_transport temporarily adds m_usart.base_addr before forwarding and
 *     restores it after.
 *
 *   • RX injection (char backend → Usart RBUF) is stubbed.  The bit-level
 *     rxd pin model requires scheduling serial frames; for now incoming bytes
 *     are discarded and flow-control credit is returned.
 *
 *   • Clock: driven by CCI param clk_hz (default 100 MHz) written to
 *     m_sig_clk in start_of_simulation().
 *
 *   • Reset: held de-asserted (m_sig_rst = false) throughout simulation.
 */

#pragma once

#include <cstdint>
#include <systemc>
#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
#include <cci_configuration>

#include <ports/initiator-signal-socket.h>
#include <ports/biflow-socket.h>
#include <module_factory_registery.h>
#include <scp/report.h>
#include <tlm_sockets_buswidth.h>

#include "tlm_utils/simple_initiator_socket.h"
#include "Usart.h"
#include "Usart_types.h"
#include "Usart_regdefs.h"

class Usart2 : public sc_core::sc_module
{
    SCP_LOGGER();

public:
    /* TLM register interface — name must match Lua "target_socket = {}" */
    tlm_utils::simple_target_socket<Usart2, DEFAULT_TLM_BUSWIDTH> socket;

    /* Character-stream backend (char_backend_stdio) */
    gs::biflow_socket<Usart2> backend_socket;

    /* Combined interrupt output */
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
        , m_clk_hz("clk_hz", 100000000ULL, "Peripheral clock frequency in Hz (default 100 MHz)")
        , m_irq_state(false)
    {
        SCP_TRACE(()) << "Usart2 constructor";

        /* Bind internal initiator socket to Usart core's bus target socket.
         * This satisfies SystemC's "all ports must be bound" check and lets
         * us forward TLM transactions through the proper socket path. */
        m_bus_fwd.bind(m_usart.bus);

        /* Bind all Usart core ports to internal signals */
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
        /* NOTE: do NOT call backend_socket.can_receive_set() here.
         * The biflow control initiator is unbound at construction time.
         * Call it in start_of_simulation() instead (Rule 1). */

        SC_METHOD(irq_method);
        sensitive << m_sig_tbir << m_sig_tir << m_sig_rir << m_sig_eir;
        dont_initialize();
    }

    void start_of_simulation() override
    {
        /* Sockets now bound: safe to configure flow control */
        backend_socket.can_receive_set(RX_FIFO_DEPTH);

        /* Hold reset de-asserted throughout simulation */
        m_sig_rst.write(false);

        /* Drive the clock period into the Usart core */
        uint64_t hz = m_clk_hz.get_value();
        if (hz > 0u) {
            double period_s = 1.0 / static_cast<double>(hz);
            m_sig_clk.write(sc_core::sc_time(period_s, sc_core::SC_SEC));
        } else {
            SCP_WARN(()) << "clk_hz is 0 — Usart core clock not configured";
        }
    }

private:
    static constexpr int RX_FIFO_DEPTH = 16;

    Usart                                 m_usart;
    tlm_utils::simple_initiator_socket<Usart2, DEFAULT_TLM_BUSWIDTH> m_bus_fwd;

    sc_core::sc_signal<sc_core::sc_time>  m_sig_clk;
    sc_core::sc_signal<bool>              m_sig_rst;
    sc_core::sc_signal<USART_TxRx_Tlm>   m_sig_txd;
    sc_core::sc_signal<USART_TxRx_Tlm>   m_sig_rxd;
    sc_core::sc_signal<bool>              m_sig_tbir;
    sc_core::sc_signal<bool>              m_sig_tir;
    sc_core::sc_signal<bool>              m_sig_rir;
    sc_core::sc_signal<bool>              m_sig_eir;

    cci::cci_param<uint64_t>              m_clk_hz;
    bool                                  m_irq_state;

    /* ------------------------------------------------------------------ */
    /* b_transport — intercept TBUF writes, forward all to Usart core     */
    /* ------------------------------------------------------------------ */
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay)
    {
        /* Router delivers offset-from-base, not the full physical address */
        uint64_t offset = trans.get_address();

        /* Intercept TBUF writes: forward byte directly to the char backend.
         * This gives immediate console output without relying on the baud-rate
         * serial simulation (which requires many clock ticks per bit).       */
        if (trans.get_command() == tlm::TLM_WRITE_COMMAND &&
            offset == USART_TBUF_OFFSET)
        {
            const uint8_t* ptr = trans.get_data_ptr();
            unsigned       len = trans.get_data_length();
            uint32_t       val = 0u;
            switch (len) {
            case 1: val = *ptr; break;
            case 2: val = *reinterpret_cast<const uint16_t*>(ptr); break;
            case 4: val = *reinterpret_cast<const uint32_t*>(ptr); break;
            default: break;
            }
            backend_socket.enqueue(static_cast<uint8_t>(val & 0xFFu));
        }

        /* Forward to Usart core so advance(), BG counter, TBUF/TSR state,
         * and IRQ pulse timestamps all stay consistent.
         * Usart::b_transport computes: offset = addr - base_addr
         * We receive the offset; restore the full address before forwarding. */
        trans.set_address(offset + m_usart.base_addr.get_value());
        m_bus_fwd->b_transport(trans, delay);
        trans.set_address(offset); /* restore for the router */
    }

    /* ------------------------------------------------------------------ */
    /* rx_receive — stub: incoming bytes from the char backend            */
    /* ------------------------------------------------------------------ */
    void rx_receive(tlm::tlm_generic_payload& txn, sc_core::sc_time& /*t*/)
    {
        /* TODO: inject bytes into the Usart core via the rxd pin interface.
         * Requires scheduling serial frames at baud-rate intervals.
         * For now, discard and restore flow-control credit.               */
        backend_socket.can_receive_more(txn.get_data_length());
    }

    /* ------------------------------------------------------------------ */
    /* irq_method — combine four interrupt outputs onto one line          */
    /* ------------------------------------------------------------------ */
    void irq_method()
    {
        bool asserted = m_sig_tbir.read() || m_sig_tir.read() ||
                        m_sig_rir.read()  || m_sig_eir.read();
        if (asserted != m_irq_state) {
            m_irq_state = asserted;
            /* InitiatorSignalSocket is SC_ZERO_OR_MORE_BOUND; only drive it
             * if connected to a GIC/interrupt controller in the platform. */
            if (irq.size() > 0)
                irq->write(m_irq_state);
        }
    }
};

extern "C" void module_register();
