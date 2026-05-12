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
 *   0x1C  CCR    clock control (used to derive frame-level byte time)
 *   0x20  TRISE  rise time (stored)
 *
 * ── IRQ and transfer timing ───────────────────────────────────────────────────
 *
 *   Events are delivered asynchronously to the GIC using two-delta pulses.
 *   Immediate register events are delayed by 1 ns of simulated time so the
 *   QEMU MMIO path can return before the interrupt is observed.  Data transfer
 *   events use one scheduled SystemC event per I2C byte frame.
 *
 * ── State machine ─────────────────────────────────────────────────────────────
 *
 *   IDLE → START(SB) → ADDR_ACK(ADDR)/AF(ER) → TX_DATA(TxE+BTF per byte)
 *                                             → RX_DATA(RxNE/OVR per byte)
 *
 *   Transfer timing is frame-level: one SystemC event per I2C byte frame
 *   (8 data bits + ACK/NACK).  The model does not sample bits or toggle
 *   SCL/SDA.
 */

#pragma once

#include <cstdint>
#include <cstring>
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
        , m_dr_tx(0u), m_dr_rx(0u), m_sr1(0u), m_sr2(0u), m_ccr(0u), m_trise(0u)
        , m_state(State::IDLE)
        , m_rx_mode(false)
        , m_rx_frame_pending(false)
        , m_tx_frame_pending(false)
        , m_slave_ptr(0u)
    {
        for (int i = 0; i < 256; ++i)
            m_slave_mem[i] = static_cast<uint8_t>(0xA0u + static_cast<uint8_t>(i));

        socket.register_b_transport(this, &I2cStm32::b_transport);

        SC_METHOD(ev_irq_method);  sensitive << m_ev_event;       dont_initialize();
        SC_METHOD(ev_pulse_method);sensitive << m_ev_pulse_event;  dont_initialize();
        SC_METHOD(er_irq_method);  sensitive << m_er_event;       dont_initialize();
        SC_METHOD(er_pulse_method);sensitive << m_er_pulse_event;  dont_initialize();
        SC_METHOD(rx_arm_method); sensitive << m_rx_arm_event; dont_initialize();
        SC_METHOD(rx_frame_done_method); sensitive << m_rx_frame_done_event; dont_initialize();
        SC_METHOD(tx_frame_done_method); sensitive << m_tx_frame_done_event; dont_initialize();
    }

private:
    /* ── Register bit constants ─────────────────────────────────────────────── */
    static constexpr uint32_t CR1_PE    = (1u << 0);
    static constexpr uint32_t CR1_SMBUS = (1u << 1);
    static constexpr uint32_t CR1_ENPEC = (1u << 5);
    static constexpr uint32_t CR1_ENGC  = (1u << 6);
    static constexpr uint32_t CR1_NOSTRETCH = (1u << 7);
    static constexpr uint32_t CR1_START = (1u << 8);
    static constexpr uint32_t CR1_STOP  = (1u << 9);
    static constexpr uint32_t CR1_ACK   = (1u << 10);
    static constexpr uint32_t CR1_POS   = (1u << 11);
    static constexpr uint32_t CR1_PEC   = (1u << 12);
    static constexpr uint32_t CR1_ALERT = (1u << 13);
    static constexpr uint32_t CR1_SWRST = (1u << 15);
    static constexpr uint32_t CR1_RW_MASK =
        CR1_PE | CR1_SMBUS | CR1_ENPEC | CR1_ENGC | CR1_NOSTRETCH |
        CR1_ACK | CR1_POS | CR1_PEC | CR1_ALERT;

    static constexpr uint32_t CR2_FREQ_MASK = 0x3Fu;
    static constexpr uint32_t CR2_ITERREN = (1u << 8);
    static constexpr uint32_t CR2_ITEVTEN = (1u << 9);
    static constexpr uint32_t CR2_ITBUFEN = (1u << 10);
    static constexpr uint32_t CR2_DMAEN   = (1u << 11);
    static constexpr uint32_t CR2_LAST    = (1u << 12);
    static constexpr uint32_t CR2_RW_MASK =
        CR2_FREQ_MASK | CR2_ITERREN | CR2_ITEVTEN | CR2_ITBUFEN |
        CR2_DMAEN | CR2_LAST;

    static constexpr uint32_t SR1_SB    = (1u << 0);
    static constexpr uint32_t SR1_ADDR  = (1u << 1);
    static constexpr uint32_t SR1_BTF   = (1u << 2);
    static constexpr uint32_t SR1_STOPF = (1u << 4);
    static constexpr uint32_t SR1_RxNE  = (1u << 6);
    static constexpr uint32_t SR1_TxE   = (1u << 7);
    static constexpr uint32_t SR1_BERR  = (1u << 8);
    static constexpr uint32_t SR1_ARLO  = (1u << 9);
    static constexpr uint32_t SR1_AF    = (1u << 10);
    static constexpr uint32_t SR1_OVR   = (1u << 11);
    static constexpr uint32_t SR1_PECERR = (1u << 12);
    static constexpr uint32_t SR1_TIMEOUT = (1u << 14);
    static constexpr uint32_t SR1_SMBALERT = (1u << 15);

    /* Error bits cleared on SR1 read or by writing 0 */
    static constexpr uint32_t SR1_ERR_MASK =
        SR1_BERR | SR1_ARLO | SR1_AF | SR1_OVR | SR1_PECERR |
        SR1_TIMEOUT | SR1_SMBALERT;

    static constexpr uint32_t SR1_RO_MASK =
        SR1_SB | SR1_ADDR | SR1_BTF | SR1_STOPF | SR1_RxNE | SR1_TxE |
        SR1_ERR_MASK;

    static constexpr uint32_t SR2_MSL  = (1u << 0);
    static constexpr uint32_t SR2_BUSY = (1u << 1);
    static constexpr uint32_t SR2_TRA  = (1u << 2);
    static constexpr uint32_t SR2_GENCALL = (1u << 4);
    static constexpr uint32_t SR2_SMBDEFAULT = (1u << 5);
    static constexpr uint32_t SR2_SMBHOST = (1u << 6);
    static constexpr uint32_t SR2_DUALF = (1u << 7);

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
    uint32_t m_dr_tx;  /* TX holding register */
    uint32_t m_dr_rx;  /* RX holding register */
    uint32_t m_sr1, m_sr2, m_ccr, m_trise;

    /* ── State ──────────────────────────────────────────────────────────────── */
    State   m_state;
    bool    m_rx_mode;
    bool    m_rx_frame_pending;
    bool    m_tx_frame_pending;
    uint8_t m_slave_ptr;
    uint8_t m_slave_mem[256];

    /* ── Async events (wakes SC from vCPU thread) ───────────────────────────── */
    gs::async_event m_ev_event{true};
    gs::async_event m_er_event{true};

    /* ── Two-delta pulse events (SC scheduler only) ─────────────────────────── */
    sc_core::sc_event m_ev_pulse_event;
    sc_core::sc_event m_er_pulse_event;
    sc_core::sc_event m_rx_arm_event;
    sc_core::sc_event m_rx_frame_done_event;
    sc_core::sc_event m_tx_frame_done_event;

    /* ── Helpers ────────────────────────────────────────────────────────────── */

    void do_reset()
    {
        m_cr1 = m_cr2 = m_oar1 = m_oar2 = 0u;
        m_dr_tx = m_dr_rx = m_sr1 = m_sr2 = 0u;
        m_ccr = m_trise = 0u;
        m_state = State::IDLE;
        m_rx_mode = false;
        m_rx_frame_pending = false;
        m_tx_frame_pending = false;
        m_slave_ptr = 0u;
        m_rx_arm_event.cancel();
        m_rx_frame_done_event.cancel();
        m_tx_frame_done_event.cancel();
        SCP_INFO(()) << "I2cStm32: reset";
    }

    sc_core::sc_time byte_frame_time() const
    {
        uint32_t freq_mhz = m_cr2 & CR2_FREQ_MASK;
        if (freq_mhz < 2u) freq_mhz = 2u;

        uint32_t ccr = m_ccr & 0x0FFFu;
        if (ccr == 0u) ccr = 1u;

        const bool fast = (m_ccr & (1u << 15)) != 0u;
        const bool duty16_9 = (m_ccr & (1u << 14)) != 0u;
        uint64_t scl_period_ns;
        if (fast) {
            const uint64_t mul = duty16_9 ? 25u : 3u;
            scl_period_ns = (mul * static_cast<uint64_t>(ccr) * 1000u) / freq_mhz;
        } else {
            scl_period_ns = (2u * static_cast<uint64_t>(ccr) * 1000u) / freq_mhz;
        }
        if (scl_period_ns == 0u) scl_period_ns = 1u;
        return sc_core::sc_time(9u * scl_period_ns, sc_core::SC_NS);
    }

    bool ev_irq_enabled(uint32_t bits) const
    {
        if ((m_cr2 & CR2_ITEVTEN) == 0u) return false;
        const uint32_t buf_bits = SR1_TxE | SR1_RxNE;
        if ((bits & buf_bits) && (m_cr2 & CR2_ITBUFEN) == 0u)
            bits &= ~buf_bits;
        return bits != 0u;
    }

    bool er_irq_enabled(uint32_t bits) const
    {
        return bits != 0u && (m_cr2 & CR2_ITERREN) != 0u;
    }

    void set_ev_flags(uint32_t bits, bool sync_delivery)
    {
        (void)sync_delivery;
        m_sr1 |= bits;
        if (ev_irq_enabled(bits)) {
            m_ev_event.notify(sc_core::sc_time(1, sc_core::SC_NS));
        }
    }

    void set_er_flags(uint32_t bits, bool sync_delivery)
    {
        (void)sync_delivery;
        m_sr1 |= bits;
        if (er_irq_enabled(bits)) {
            m_er_event.notify(sc_core::sc_time(1, sc_core::SC_NS));
        }
    }

    void clear_bus()
    {
        m_sr1 &= ~(SR1_SB | SR1_ADDR | SR1_BTF | SR1_RxNE | SR1_TxE);
        m_sr2 &= ~(SR2_MSL | SR2_BUSY | SR2_TRA);
        m_state = State::IDLE;
        m_rx_mode = false;
        m_rx_frame_pending = false;
        m_tx_frame_pending = false;
        m_rx_arm_event.cancel();
        m_rx_frame_done_event.cancel();
        m_tx_frame_done_event.cancel();
    }

    void schedule_rx_frame()
    {
        if (m_state != State::RX_DATA || m_rx_frame_pending) return;
        if ((m_cr1 & (CR1_PE | CR1_ACK)) != (CR1_PE | CR1_ACK)) return;
        m_rx_frame_pending = true;
        m_rx_frame_done_event.notify(byte_frame_time());
    }

    void schedule_tx_frame()
    {
        if (m_state != State::TX_DATA || m_tx_frame_pending) return;
        m_tx_frame_pending = true;
        m_tx_frame_done_event.notify(byte_frame_time());
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
            case OFF_DR: {
                val = m_dr_rx & 0xFFu;
                m_sr1 &= ~(SR1_RxNE | SR1_BTF);
                if (m_rx_mode && m_state == State::RX_DATA)
                    m_rx_arm_event.notify(sc_core::SC_ZERO_TIME);
                break;
            }
            case OFF_CCR:   val = m_ccr;  break;
            case OFF_TRISE: val = m_trise; break;

            case OFF_SR1:
                val = m_sr1;
                break;

            case OFF_SR2:
                val = m_sr2;
                /* Second step of ADDR-clear sequence: reading SR2 clears ADDR */
                if (m_sr1 & SR1_ADDR) {
                    m_sr1 &= ~SR1_ADDR;
                    if (m_rx_mode && m_state == State::RX_DATA)
                        m_rx_arm_event.notify(sc_core::SC_ZERO_TIME);
                }
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
            case OFF_CR2:   m_cr2 = wval & CR2_RW_MASK; break;
            case OFF_OAR1:  m_oar1 = wval & 0x03FFu;    break;
            case OFF_OAR2:  m_oar2 = wval & 0x00FFu;    break;
            case OFF_CCR:   m_ccr  = wval & 0xCFFFu;    break;
            case OFF_TRISE: m_trise = wval & 0x3Fu;      break;
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
        m_cr1 = val & CR1_RW_MASK;

        if (val & CR1_STOP) {
            clear_bus();
            return;  /* no interrupt on master STOP */
        }

        if ((val & CR1_START) && (val & CR1_PE)) {
            m_slave_ptr = 0u;  /* reset slave pointer for each new transaction */
            m_sr1  &= ~(SR1_ADDR | SR1_BTF | SR1_RxNE | SR1_TxE);
            m_sr2   = SR2_MSL | SR2_BUSY;
            m_state = State::STARTED;
            m_rx_mode = false;
            set_ev_flags(SR1_SB, true);
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
                m_sr1   = (m_sr1 & ~(SR1_SB | SR1_TxE | SR1_RxNE | SR1_BTF)) | SR1_ADDR;
                m_sr2   = SR2_MSL | SR2_BUSY | (m_rx_mode ? 0u : SR2_TRA);
                m_state = m_rx_mode ? State::RX_DATA : State::TX_DATA;
                set_ev_flags(SR1_ADDR, true);
            } else {
                m_sr1   = (m_sr1 & ~SR1_SB) | SR1_AF;
                m_state = State::IDLE;
                m_sr2  &= ~(SR2_MSL | SR2_BUSY | SR2_TRA);
                set_er_flags(SR1_AF, true);
            }

        } else if (m_state == State::TX_DATA) {
            m_dr_tx = byte;
            m_sr1 &= ~(SR1_TxE | SR1_BTF);
            schedule_tx_frame();
        } else {
            SCP_WARN(()) << "I2cStm32: DR write in unexpected state";
        }
    }

    void rx_arm_method()
    {
        schedule_rx_frame();
    }

    void rx_frame_done_method()
    {
        m_rx_frame_pending = false;
        if (m_state != State::RX_DATA || (m_cr1 & CR1_PE) == 0u) return;

        if (m_sr1 & SR1_RxNE) {
            set_er_flags(SR1_OVR, false);
            return;
        }

        m_dr_rx = m_slave_mem[m_slave_ptr++ % 256u];
        set_ev_flags(SR1_RxNE, false);
    }

    void tx_frame_done_method()
    {
        m_tx_frame_pending = false;
        if (m_state != State::TX_DATA || (m_cr1 & CR1_PE) == 0u) return;

        (void)m_dr_tx;
        set_ev_flags(SR1_TxE | SR1_BTF, false);
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
    }

    void er_irq_method()
    {
        if (er_irq.size() > 0u) er_irq->write(false);
        m_er_pulse_event.notify(sc_core::SC_ZERO_TIME);
    }

    void er_pulse_method()
    {
        if (er_irq.size() > 0u) er_irq->write(true);
    }
};

extern "C" void module_register();
