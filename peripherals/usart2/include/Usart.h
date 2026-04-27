// =============================================================================
// Usart.h
// -----------------------------------------------------------------------------
// QBox USART peripheral — class declaration.
//
// ── Execution model: TLM frame-level ─────────────────────────────────────────
//
//  TX: When a byte is written to TBUF, the complete frame is fully known
//  at that instant (data + parity + framing bits determined by CON).  There is
//  nothing to "discover" bit by bit, so the model does NOT simulate individual
//  bit periods.  Instead:
//    1. loadTSR() fires TBIR immediately (TBUF slot freed).
//    2. loadTSR() schedules m_tx_done_ev after one frame_duration.
//    3. txDoneMethod() fires TIR, clears m_tsr_busy, chains the next byte.
//
//  RX: Bytes arrive via rx_inject() (called from Usart2 after one frame_duration
//  has elapsed).  rx_inject() writes directly to RBUF and fires RIR/EIR.
//  The stepRx() pin-sampling path is not used in the Usart2 VP context.
//
// ── IRQ pulse de-assertion ────────────────────────────────────────────────────
//
//  assertIrq() drives the output HIGH and schedules m_irq_deassert_ev after
//  2 * m_clk_period.  irqDeassertMethod() calls updateIrqPulses() to drive
//  the output LOW once the 2-cycle pulse window expires.
//
// ── Baud-rate helpers ─────────────────────────────────────────────────────────
//
//  get_baud_period()   — baud_period from CON/BG state
//  get_frame_duration()— baud_period × (1+data+parity+stop) per frame
//  Both return SC_ZERO_TIME if clock not configured or R=0.
//
// ── Process list ─────────────────────────────────────────────────────────────
//
//  rstMethod()          — sensitive to rst.pos()
//  usartClkInMethod()   — sensitive to usartClkIn
//  txDoneMethod()       — sensitive to m_tx_done_ev (one full frame after TBUF load)
//  irqDeassertMethod()  — sensitive to m_irq_deassert_ev (2 clk after assertIrq)
//
// =============================================================================
#ifndef USART_H
#define USART_H

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <cci_configuration>

#include "Usart_types.h"
#include "Usart_regdefs.h"

SC_MODULE(Usart)
{
    // =========================================================================
    // Public ports and sockets
    // =========================================================================

    tlm_utils::simple_target_socket<Usart, 32> bus;

    sc_core::sc_in<sc_core::sc_time>    usartClkIn;
    sc_core::sc_in<bool>                rst;

    sc_core::sc_out<USART_TxRx_Tlm>    txd;
    sc_core::sc_inout<USART_TxRx_Tlm>  rxd;

    sc_core::sc_out<bool> tbir;
    sc_core::sc_out<bool> tir;
    sc_core::sc_out<bool> rir;
    sc_core::sc_out<bool> eir;

    // =========================================================================
    // CCI parameters
    // =========================================================================
    cci::cci_param<uint64_t> base_addr;
    cci::cci_param<int>      debug_level;

    // =========================================================================
    // Constructor / Destructor
    // =========================================================================
    SC_CTOR(Usart);
    ~Usart();

    // =========================================================================
    // TLM callbacks
    // =========================================================================
    void         b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay);
    unsigned int transport_dbg(tlm::tlm_generic_payload &trans);
    bool         get_direct_mem_ptr(tlm::tlm_generic_payload &trans,
                                    tlm::tlm_dmi            &dmi_data);

    // =========================================================================
    // Direct RX injection — bypasses pin-level RX state machine.
    // Called by Usart2 after one frame_duration has elapsed (deferred inject).
    // Returns true if accepted, false on overrun.
    // =========================================================================
    bool rx_inject(uint8_t byte);

    // Advance IRQ pulse state to now (call before reading interrupt status
    // from outside the module).
    void sync() { advance(sc_core::sc_time_stamp()); }

    // =========================================================================
    // IRQ state accessors
    // =========================================================================
    bool is_tbir_asserted() const { return m_tbir_assert_time != sc_core::SC_ZERO_TIME; }
    bool is_tir_asserted()  const { return m_tir_assert_time  != sc_core::SC_ZERO_TIME; }
    bool is_rir_asserted()  const { return m_rir_assert_time  != sc_core::SC_ZERO_TIME; }
    bool is_eir_asserted()  const { return m_eir_assert_time  != sc_core::SC_ZERO_TIME; }

    // Monotone counter: incremented each time TIR fires.
    // Never cleared by updateIrqPulses(), so Usart2 can detect TIR even after
    // the 2-cycle pulse has expired.
    unsigned int get_tir_fire_count() const { return m_tir_fire_count; }

    sc_core::sc_time get_baud_period()    const;
    sc_core::sc_time get_frame_duration() const;

    // =========================================================================
    // Direct RBUF accessors — do NOT call advance(), safe from any thread.
    // =========================================================================
    uint32_t get_rbuf_raw() const { return m_rbuf.raw; }
    void     clear_rbuf()         { m_rbuf_full = false; m_rbuf.raw = 0u; }

private:
    // =========================================================================
    // Register bank
    // =========================================================================
    CON_reg  m_con;
    TBUF_reg m_tbuf_reg;
    RBUF_reg m_rbuf;
    BG_reg   m_bg_reload;
    uint16_t m_bg_counter;  // not ticked; preserved for BG read-back
    FDR_reg  m_fdr;

    // =========================================================================
    // TX path
    // =========================================================================
    bool     m_tbuf_full;
    uint32_t m_tbuf_data;
    bool     m_tsr_busy;
    uint32_t m_tsr;         // loaded from TBUF; written to txd at frame start

    // =========================================================================
    // RX path
    // =========================================================================
    bool m_rbuf_full;

    // =========================================================================
    // Timing
    // =========================================================================
    sc_core::sc_time m_last_advance_time;
    sc_core::sc_time m_clk_period;

    // =========================================================================
    // IRQ pulse tracking
    // =========================================================================
    sc_core::sc_time m_tbir_assert_time;
    sc_core::sc_time m_tir_assert_time;
    sc_core::sc_time m_rir_assert_time;
    sc_core::sc_time m_eir_assert_time;
    unsigned int     m_tir_fire_count;

    // =========================================================================
    // Events
    // =========================================================================
    sc_core::sc_event m_tx_done_ev;       // fired after one frame_duration
    sc_core::sc_event m_irq_deassert_ev;  // fired after 2 * m_clk_period

    // =========================================================================
    // SC_METHOD processes
    // =========================================================================
    void usartClkInMethod();
    void rstMethod();
    void txDoneMethod();
    void irqDeassertMethod();

    // =========================================================================
    // Private helpers
    // =========================================================================
    void advance(sc_core::sc_time now);
    void updateIrqPulses(sc_core::sc_time now);
    void assertIrq(sc_core::sc_time      &assert_time,
                   sc_core::sc_out<bool> &port,
                   const char            *irq_name);

    uint32_t regRead(uint32_t offset);
    void     regWrite(uint32_t offset, uint32_t value);
    void     loadTSR();
    void     transferRSR(bool oe, bool fe, bool pe);
    static bool computeEvenParity(uint32_t data, uint8_t bits);

    Usart(const Usart &)            = delete;
    Usart &operator=(const Usart &) = delete;
};

#endif // USART_H
