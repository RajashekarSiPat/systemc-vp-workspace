/*
 * i2c_stm32.h — STM32 I2C TLM peripheral (RM0008 Chapter 26) for QBOX.
 *
 * ── Overview ──────────────────────────────────────────────────────────────────
 *
 *   Models master-mode operation of the STM32 I2C controller with a virtual
 *   slave device at a configurable 7-bit address backed by 256-byte RAM.
 *
 *   Two interrupt outputs match the STM32 hardware split:
 *     ev_irq — I2C_EV: SB, ADDR, TxE, RxNE
 *     er_irq — I2C_ER: AF (acknowledge failure), BERR, ARLO, OVR
 *
 * ── Register map ──────────────────────────────────────────────────────────────
 *
 *   0x00  CR1    PE(0), START(8), STOP(9), ACK(10), SWRST(15)
 *   0x04  CR2    FREQ[5:0], ITERREN(8), ITEVTEN(9), ITBUFEN(10)
 *   0x08  OAR1   own address (not modeled in detail)
 *   0x0C  OAR2   dual address (not modeled)
 *   0x10  DR     data register
 *   0x14  SR1    SB(0),ADDR(1),BTF(2),STOPF(4),RxNE(6),TxE(7),AF(10),...
 *   0x18  SR2    MSL(0),BUSY(1),TRA(2) — reading clears ADDR in SR1
 *   0x1C  CCR    clock control (stored, not used for timing in TLM model)
 *   0x20  TRISE  rise time (stored, not used)
 *
 * ── Synchronisation mechanism ─────────────────────────────────────────────────
 *
 *   Uses a generation counter (m_irq_gen) instead of a bool flag.
 *   wait_for_irq_delivery() captures m_irq_gen before the write, then
 *   waits for it to change (any irq_pulse_method increments it).
 *
 *   Compared to the bool approach: no "stolen-wake" race — rapid-fire
 *   interrupts each get their own generation increment, so every caller
 *   of wait_for_irq_delivery() waits for exactly its own delivery.
 *
 *   m_irq_sync_timeouts counts timeout-path occurrences for diagnostics.
 *
 * ── State machine ─────────────────────────────────────────────────────────────
 *
 *   IDLE → START(SB) → ADDR_ACK(ADDR)/AF(ER) → TX_DATA(TxE per byte)
 *                                             → RX_DATA(RxNE per byte)
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <thread>
#include <systemc>
#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
#include <cci_configuration>

#include <ports/initiator-signal-socket.h>
#include <async_event.h>
#include <module_factory_registery.h>
#include <scp/report.h>
#include <tlm_sockets_buswidth.h>

class I2cStm32 : public sc_core::sc_module
{
    SCP_LOGGER();

public:
    tlm_utils::simple_target_socket<I2cStm32, DEFAULT_TLM_BUSWIDTH> socket{"target_socket"};

    /* STM32-style split: EV interrupt and ER interrupt on separate lines */
    InitiatorSignalSocket<bool> ev_irq{"ev_irq"};
    InitiatorSignalSocket<bool> er_irq{"er_irq"};

    SC_HAS_PROCESS(I2cStm32);

    I2cStm32(sc_core::sc_module_name name)
        : sc_module(name)
        , m_slave_addr("slave_addr", 0x50u, "Virtual slave 7-bit address")
        , m_nack_addr ("nack_addr",  0xFFu, "Force NACK on this address (0xFF = disabled)")
        , m_cr1(0u), m_cr2(0u), m_oar1(0u), m_oar2(0u)
        , m_dr_rx(0u), m_sr1(0u), m_sr2(0u), m_ccr(0u), m_trise(0u)
        , m_state(State::IDLE)
        , m_rx_mode(false)
        , m_slave_ptr(0u)
        , m_irq_gen(0u)
        , m_irq_sync_timeouts(0u)
    {
        for (int i = 0; i < 256; ++i)
            m_slave_mem[i] = static_cast<uint8_t>(0xA0u + static_cast<uint8_t>(i));

        socket.register_b_transport(this, &I2cStm32::b_transport);

        SC_METHOD(ev_irq_method);  sensitive << m_ev_event;       dont_initialize();
        SC_METHOD(ev_pulse_method);sensitive << m_ev_pulse_event;  dont_initialize();
        SC_METHOD(er_irq_method);  sensitive << m_er_event;       dont_initialize();
        SC_METHOD(er_pulse_method);sensitive << m_er_pulse_event;  dont_initialize();
    }

private:
    /* ── Register bit constants ─────────────────────────────────────────────── */
    static constexpr uint32_t CR1_PE    = (1u << 0);
    static constexpr uint32_t CR1_START = (1u << 8);
    static constexpr uint32_t CR1_STOP  = (1u << 9);
    static constexpr uint32_t CR1_ACK   = (1u << 10);
    static constexpr uint32_t CR1_SWRST = (1u << 15);

    static constexpr uint32_t CR2_ITERREN = (1u << 8);
    static constexpr uint32_t CR2_ITEVTEN = (1u << 9);
    static constexpr uint32_t CR2_ITBUFEN = (1u << 10);

    static constexpr uint32_t SR1_SB    = (1u << 0);
    static constexpr uint32_t SR1_ADDR  = (1u << 1);
    static constexpr uint32_t SR1_BTF   = (1u << 2);
    static constexpr uint32_t SR1_STOPF = (1u << 4);
    static constexpr uint32_t SR1_RxNE  = (1u << 6);
    static constexpr uint32_t SR1_TxE   = (1u << 7);
    static constexpr uint32_t SR1_AF    = (1u << 10);

    /* Error bits cleared on SR1 read or by writing 0 */
    static constexpr uint32_t SR1_ERR_MASK =
        (1u << 8) | (1u << 9) | (1u << 10) | (1u << 11) | (1u << 12);

    static constexpr uint32_t SR2_MSL  = (1u << 0);
    static constexpr uint32_t SR2_BUSY = (1u << 1);
    static constexpr uint32_t SR2_TRA  = (1u << 2);

    /* Register offsets */
    static constexpr uint64_t OFF_CR1   = 0x00u;
    static constexpr uint64_t OFF_CR2   = 0x04u;
    static constexpr uint64_t OFF_OAR1  = 0x08u;
    static constexpr uint64_t OFF_OAR2  = 0x0Cu;
    static constexpr uint64_t OFF_DR    = 0x10u;
    static constexpr uint64_t OFF_SR1   = 0x14u;
    static constexpr uint64_t OFF_SR2   = 0x18u;
    static constexpr uint64_t OFF_CCR   = 0x1Cu;
    static constexpr uint64_t OFF_TRISE = 0x20u;

    /* ── State machine ──────────────────────────────────────────────────────── */
    enum class State { IDLE, STARTED, TX_DATA, RX_DATA };

    /* ── CCI parameters ─────────────────────────────────────────────────────── */
    cci::cci_param<uint32_t> m_slave_addr;
    cci::cci_param<uint32_t> m_nack_addr;

    /* ── Registers ──────────────────────────────────────────────────────────── */
    uint32_t m_cr1, m_cr2, m_oar1, m_oar2;
    uint32_t m_dr_rx;  /* RX holding register */
    uint32_t m_sr1, m_sr2, m_ccr, m_trise;

    /* ── State ──────────────────────────────────────────────────────────────── */
    State   m_state;
    bool    m_rx_mode;
    uint8_t m_slave_ptr;
    uint8_t m_slave_mem[256];

    /* ── Generation-counter sync ────────────────────────────────────────────── */
    std::mutex              m_irq_sync_mtx;
    std::condition_variable m_irq_sync_cv;
    uint32_t                m_irq_gen;
    std::atomic<uint32_t>   m_irq_sync_timeouts;

    /* ── Async events (wakes SC from vCPU thread) ───────────────────────────── */
    gs::async_event m_ev_event{true};
    gs::async_event m_er_event{true};

    /* ── Two-delta pulse events (SC scheduler only) ─────────────────────────── */
    sc_core::sc_event m_ev_pulse_event;
    sc_core::sc_event m_er_pulse_event;

    /* ── Helpers ────────────────────────────────────────────────────────────── */

    void do_reset()
    {
        m_cr1 = m_cr2 = m_oar1 = m_oar2 = 0u;
        m_dr_rx = m_sr1 = m_sr2 = 0u;
        m_state = State::IDLE;
        m_rx_mode = false;
        m_slave_ptr = 0u;
        SCP_INFO(()) << "I2cStm32: reset";
    }

    /* Call after scheduling an EV or ER event to ensure the GIC edge is
     * delivered before b_transport returns.  Uses a generation counter so
     * rapid-fire interrupts never wake the wrong waiter.                   */
    void wait_for_irq_delivery()
    {
        if (ev_irq.size() == 0u && er_irq.size() == 0u) return;
        std::unique_lock<std::mutex> lk(m_irq_sync_mtx);
        const uint32_t before = m_irq_gen;
        const bool fired = m_irq_sync_cv.wait_for(
            lk, std::chrono::milliseconds(5),
            [this, before]{ return m_irq_gen != before; });
        if (!fired) {
            ++m_irq_sync_timeouts;
            SCP_WARN(()) << "I2cStm32: IRQ delivery timeout #"
                         << m_irq_sync_timeouts.load();
        }
    }

    void notify_ev_after_mmio_return()
    {
        /* Read-side RxNE IRQs must not wake QEMU until QBOX has committed the
         * MMIO load result back into the guest register.  Notify from a host
         * thread so async_event posts the SC event after b_transport returns. */
        std::thread([this] {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            m_ev_event.notify(sc_core::SC_ZERO_TIME);
        }).detach();
    }

    /* ── b_transport ────────────────────────────────────────────────────────── */
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& /*delay*/)
    {
        const uint64_t off    = trans.get_address();
        const bool     is_rd  = (trans.get_command() == tlm::TLM_READ_COMMAND);
        uint8_t* const ptr    = trans.get_data_ptr();

        trans.set_dmi_allowed(false);
        trans.set_response_status(tlm::TLM_OK_RESPONSE);

        if (is_rd) {
            uint32_t val = 0u;
            switch (off) {
            case OFF_CR1:   val = m_cr1;  break;
            case OFF_CR2:   val = m_cr2;  break;
            case OFF_OAR1:  val = m_oar1; break;
            case OFF_OAR2:  val = m_oar2; break;
            case OFF_CCR:   val = m_ccr;  break;
            case OFF_TRISE: val = m_trise; break;

            case OFF_SR1:
                val = m_sr1;
                m_sr1 &= ~SR1_ERR_MASK;   /* error bits cleared on read */
                break;

            case OFF_SR2:
                val = m_sr2;
                /* Second step of ADDR-clear sequence: reading SR2 clears ADDR */
                if (m_sr1 & SR1_ADDR) {
                    m_sr1 &= ~SR1_ADDR;
                    if (m_rx_mode && m_state == State::RX_DATA) {
                        /* Load first RxNE byte and fire the interrupt.
                         * Do NOT call wait_for_irq_delivery() here — for reads,
                         * QBOX delivers the MMIO result after b_transport returns.
                         * Calling wait_for_irq_delivery() inside a read b_transport
                         * causes cpu_interrupt() to wake the QEMU thread before
                         * QBOX commits the TLM buffer, corrupting the load result.
                         * The firmware's i2c_wait_n + WFI loop handles the async
                         * interrupt delivery correctly.                           */
                        m_dr_rx = m_slave_mem[m_slave_ptr % 256u];
                        ++m_slave_ptr;
                        m_sr1  |= SR1_RxNE;
                        notify_ev_after_mmio_return();
                    }
                }
                break;

            case OFF_DR:
                val     = m_dr_rx & 0xFFu;
                m_sr1  &= ~SR1_RxNE;
                m_dr_rx = m_slave_mem[m_slave_ptr++ % 256u];
                m_sr1  |= SR1_RxNE;
                notify_ev_after_mmio_return();
                break;

            default:
                SCP_WARN(()) << "I2cStm32: read from unknown offset 0x" << std::hex << off;
                break;
            }
            if (ptr) std::memcpy(ptr, &val, sizeof(val));

        } else {
            uint32_t wval = 0u;
            if (ptr) std::memcpy(&wval, ptr, sizeof(wval));

            switch (off) {
            case OFF_CR1:   handle_cr1_write(wval);   break;
            case OFF_CR2:   m_cr2 = wval & 0x3FFFu;  break;
            case OFF_OAR1:  m_oar1 = wval;            break;
            case OFF_OAR2:  m_oar2 = wval;            break;
            case OFF_CCR:   m_ccr  = wval;            break;
            case OFF_TRISE: m_trise = wval;           break;
            case OFF_DR:    handle_dr_write(wval);    break;
            case OFF_SR1:
                /* Write-0-to-clear for error bits */
                m_sr1 &= (wval | ~SR1_ERR_MASK);
                break;
            default:
                SCP_WARN(()) << "I2cStm32: write to unknown offset 0x" << std::hex << off;
                break;
            }
        }
    }

    void handle_cr1_write(uint32_t val)
    {
        if (val & CR1_SWRST) { do_reset(); return; }

        if ((m_cr1 & CR1_PE) && !(val & CR1_PE)) { do_reset(); return; }

        /* Latch CR1 without the self-clearing bits */
        m_cr1 = val & ~(CR1_START | CR1_STOP);

        if (val & CR1_STOP) {
            m_sr1   = 0u;
            m_sr2   = 0u;
            m_state = State::IDLE;
            return;  /* no interrupt on master STOP */
        }

        if ((val & CR1_START) && (val & CR1_PE)) {
            m_slave_ptr = 0u;  /* reset slave pointer for each new transaction */
            m_sr1   = SR1_SB;
            m_sr2   = SR2_MSL | SR2_BUSY;
            m_state = State::STARTED;
            m_ev_event.notify(sc_core::SC_ZERO_TIME);
            wait_for_irq_delivery();
        }
    }

    void handle_dr_write(uint32_t val)
    {
        const uint8_t byte = static_cast<uint8_t>(val & 0xFFu);

        if (m_state == State::STARTED) {
            /* Address byte: bits[7:1] = 7-bit addr, bit[0] = R/W */
            const uint8_t addr = byte >> 1u;
            m_rx_mode          = (byte & 1u) != 0u;

            const uint32_t slave = m_slave_addr.get_value() & 0x7Fu;
            const uint32_t nack  = m_nack_addr.get_value()  & 0x7Fu;

            if (addr == slave && addr != nack) {
                m_sr1   = SR1_ADDR;
                m_sr2   = SR2_MSL | SR2_BUSY | (m_rx_mode ? 0u : SR2_TRA);
                m_state = m_rx_mode ? State::RX_DATA : State::TX_DATA;
                m_ev_event.notify(sc_core::SC_ZERO_TIME);
                wait_for_irq_delivery();
            } else {
                m_sr1   = SR1_AF;
                m_state = State::IDLE;
                m_er_event.notify(sc_core::SC_ZERO_TIME);
                wait_for_irq_delivery();
            }

        } else if (m_state == State::TX_DATA) {
            /* TLM slave absorbs the byte; read memory stays immutable */
            m_sr1 = (m_sr1 & ~SR1_TxE) | SR1_TxE;
            m_ev_event.notify(sc_core::SC_ZERO_TIME);
            wait_for_irq_delivery();
        } else {
            SCP_WARN(()) << "I2cStm32: DR write in unexpected state";
        }
    }

    /* ── SC-thread IRQ delivery (two-delta pulse) ───────────────────────────── */

    void ev_irq_method()
    {
        if (ev_irq.size() > 0u) ev_irq->write(false);
        m_ev_pulse_event.notify(sc_core::SC_ZERO_TIME);
    }

    void ev_pulse_method()
    {
        if (ev_irq.size() > 0u) ev_irq->write(true);
        std::lock_guard<std::mutex> lk(m_irq_sync_mtx);
        ++m_irq_gen;
        m_irq_sync_cv.notify_all();
    }

    void er_irq_method()
    {
        if (er_irq.size() > 0u) er_irq->write(false);
        m_er_pulse_event.notify(sc_core::SC_ZERO_TIME);
    }

    void er_pulse_method()
    {
        if (er_irq.size() > 0u) er_irq->write(true);
        std::lock_guard<std::mutex> lk(m_irq_sync_mtx);
        ++m_irq_gen;
        m_irq_sync_cv.notify_all();
    }
};

extern "C" void module_register();
