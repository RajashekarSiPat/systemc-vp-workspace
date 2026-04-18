/*
 * Usart.h — Simple USART SystemC/TLM peripheral for QBOX virtual platforms.
 *
 * Register map (word-aligned, 32-bit accesses):
 *   Offset  Register  Description
 *   0x00    SR        Status Register
 *                       [7] TXE  — Transmit Data Register Empty (always 1 in sim)
 *                       [5] RXNE — Receive Data Register Not Empty
 *   0x04    DR        Data Register (write = TX, read = RX clears RXNE)
 *   0x08    BRR       Baud Rate Register (ignored in simulation)
 *   0x0C    CR1       Control Register 1
 *                       [13] UE   — USART Enable
 *                       [ 5] RXNEIE — RXNE Interrupt Enable
 *                       [ 3] TE   — Transmitter Enable
 *                       [ 2] RE   — Receiver Enable
 *
 * Baremetal minimal init sequence:
 *   USART_CR1 = CR1_UE | CR1_TE;      // enable USART + TX
 *   while (!(USART_SR & SR_TXE));     // wait TX ready (always immediate)
 *   USART_DR = 'H';                   // send character
 */

#pragma once

#include <cstdint>
#include <queue>

#include <systemc>
#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"

#include <ports/initiator-signal-socket.h>
#include <async_event.h>
#include <ports/biflow-socket.h>
#include <module_factory_registery.h>
#include <scp/report.h>
#include <tlm_sockets_buswidth.h>

/* ---- Register offsets ---- */
#define USART_SR_OFFSET  0x00u
#define USART_DR_OFFSET  0x04u
#define USART_BRR_OFFSET 0x08u
#define USART_CR1_OFFSET 0x0Cu

/* ---- SR bits ---- */
#define USART_SR_TXE   (1u << 7)
#define USART_SR_RXNE  (1u << 5)

/* ---- CR1 bits ---- */
#define USART_CR1_UE     (1u << 13)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_TE     (1u << 3)
#define USART_CR1_RE     (1u << 2)

class Usart : public sc_core::sc_module
{
    SCP_LOGGER();

public:
    /* Memory-mapped register interface (target socket) */
    tlm_utils::simple_target_socket<Usart, DEFAULT_TLM_BUSWIDTH> socket;

    /* Character-stream backend (connects to char_backend_stdio, socket, …) */
    gs::biflow_socket<Usart> backend_socket;

    /* Interrupt line: asserted when RXNE && RXNEIE */
    InitiatorSignalSocket<bool> irq;

    SC_HAS_PROCESS(Usart);

    Usart(sc_core::sc_module_name name)
        : sc_module(name)
        , socket("target_socket")
        , backend_socket("backend_socket")
        , irq("irq")
        , m_cr1(0)
        , m_brr(0)
        , m_irq_state(false)
    {
        SCP_TRACE(()) << "Usart constructor";

        socket.register_b_transport(this, &Usart::b_transport);
        backend_socket.register_b_transport(this, &Usart::rx_receive);
        /* NOTE: do NOT call backend_socket.can_receive_set() here.
         * The biflow_socket's control initiator is unbound at construction
         * time; calling it would throw and abort module creation.
         * Use start_of_simulation() instead. */

        SC_METHOD(update_irq);
        sensitive << m_irq_event;
        dont_initialize();
    }

    void start_of_simulation() override
    {
        /* Now all sockets are bound — tell the backend we can buffer
         * up to RX_FIFO_DEPTH bytes before back-pressure kicks in. */
        backend_socket.can_receive_set(RX_FIFO_DEPTH);
    }

private:
    static constexpr int RX_FIFO_DEPTH = 16;

    uint32_t          m_cr1;
    uint32_t          m_brr;
    std::queue<uint8_t> m_rx_fifo;
    bool              m_irq_state;
    sc_core::sc_event m_irq_event;

    /* ------------------------------------------------------------------ */
    /* TLM b_transport — register read/write                               */
    /* ------------------------------------------------------------------ */
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay)
    {
        uint64_t     offset = trans.get_address();
        unsigned int len    = trans.get_data_length();
        uint8_t*     ptr    = trans.get_data_ptr();

        trans.set_dmi_allowed(false);
        trans.set_response_status(tlm::TLM_OK_RESPONSE);

        if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
            uint32_t val = read_payload(ptr, len);
            reg_write(offset, val);
        } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
            uint32_t val = reg_read(offset);
            write_payload(ptr, len, val);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Register access                                                     */
    /* ------------------------------------------------------------------ */
    uint32_t reg_read(uint64_t offset)
    {
        switch (offset) {
        case USART_SR_OFFSET: {
            uint32_t sr = USART_SR_TXE; /* TX always ready in simulation */
            if (!m_rx_fifo.empty())
                sr |= USART_SR_RXNE;
            return sr;
        }
        case USART_DR_OFFSET: {
            if (m_rx_fifo.empty())
                return 0;
            uint8_t ch = m_rx_fifo.front();
            m_rx_fifo.pop();
            backend_socket.can_receive_more(1);
            m_irq_event.notify(sc_core::SC_ZERO_TIME);
            return ch;
        }
        case USART_BRR_OFFSET:
            return m_brr;
        case USART_CR1_OFFSET:
            return m_cr1;
        default:
            SCP_WARN(()) << "Read from unknown USART offset 0x" << std::hex << offset;
            return 0;
        }
    }

    void reg_write(uint64_t offset, uint32_t val)
    {
        switch (offset) {
        case USART_DR_OFFSET: {
            uint8_t ch = static_cast<uint8_t>(val & 0xFF);
            backend_socket.enqueue(ch);
            break;
        }
        case USART_BRR_OFFSET:
            m_brr = val;
            break;
        case USART_CR1_OFFSET:
            m_cr1 = val;
            m_irq_event.notify(sc_core::SC_ZERO_TIME);
            break;
        case USART_SR_OFFSET:
            /* Writes to SR are ignored (read-only in hardware) */
            break;
        default:
            SCP_WARN(()) << "Write to unknown USART offset 0x" << std::hex << offset;
            break;
        }
    }

    /* ------------------------------------------------------------------ */
    /* Backend receive: byte(s) arriving from the char backend             */
    /* ------------------------------------------------------------------ */
    void rx_receive(tlm::tlm_generic_payload& txn, sc_core::sc_time& t)
    {
        uint8_t* data = txn.get_data_ptr();
        for (unsigned i = 0; i < txn.get_streaming_width(); ++i) {
            if (static_cast<int>(m_rx_fifo.size()) < RX_FIFO_DEPTH)
                m_rx_fifo.push(data[i]);
        }
        m_irq_event.notify(sc_core::SC_ZERO_TIME);
    }

    /* ------------------------------------------------------------------ */
    /* IRQ update (runs as SC_METHOD, zero-time after any state change)    */
    /* ------------------------------------------------------------------ */
    void update_irq()
    {
        bool assert_irq = (m_cr1 & USART_CR1_RXNEIE) && !m_rx_fifo.empty();
        if (assert_irq != m_irq_state) {
            m_irq_state = assert_irq;
            irq->write(m_irq_state);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Helpers: extract/deposit a value from/into a byte buffer            */
    /* ------------------------------------------------------------------ */
    static uint32_t read_payload(const uint8_t* ptr, unsigned len)
    {
        switch (len) {
        case 1: return *ptr;
        case 2: return *reinterpret_cast<const uint16_t*>(ptr);
        case 4: return *reinterpret_cast<const uint32_t*>(ptr);
        default: return 0;
        }
    }

    static void write_payload(uint8_t* ptr, unsigned len, uint32_t val)
    {
        switch (len) {
        case 1: *ptr                                     = static_cast<uint8_t>(val);  break;
        case 2: *reinterpret_cast<uint16_t*>(ptr)        = static_cast<uint16_t>(val); break;
        case 4: *reinterpret_cast<uint32_t*>(ptr)        = val;                        break;
        default: break;
        }
    }
};

extern "C" void module_register();
