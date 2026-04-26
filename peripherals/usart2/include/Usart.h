// =============================================================================
// Usart.h
// -----------------------------------------------------------------------------
// QBox USART peripheral — class declaration.
//
// Compliant with:
//   IEEE 1666-2011  (SystemC 2.3)
//   OSCI TLM-2.0
//   CCI-400 (cci::cci_param)
//   USART Peripheral Specification Rev 1.0 (§1–§6)
//
// ── Clock model ───────────────────────────────────────────────────────────────
//
//  sc_in<sc_time> usartClkIn  carries the peripheral clock *period* as a value.
//  It does NOT toggle.  It changes only on startup or dynamic reconfig.
//
//  Constraints satisfied:
//    • NO SC_THREAD anywhere.
//    • NO next_trigger() anywhere.
//    • NO toggling Boolean clock port.
//
// ── Execution model ───────────────────────────────────────────────────────────
//
//  Because there is no periodic edge to trigger SC_METHODs, all time-driven
//  work is performed *lazily*: every entry point into the module computes how
//  many baud ticks have elapsed since the last entry and fast-forwards the
//  entire hardware state by that count before doing anything else.
//
//  Entry points that call advance():
//    1. usartClkInMethod()  — fires when usartClkIn value changes
//    2. rstMethod()         — fires on rst posedge
//    3. b_transport()       — called by CPU for register reads/writes
//    4. transport_dbg()     — called by debugger (advance but no side-effects)
//
//  advance(now) does in one call:
//    a. Compute elapsed_ticks = floor((now - m_last_advance_time) / bit_period)
//       where bit_period = f_PRE_period × (BG_reload+1)   (full baud cycle)
//    b. For each elapsed tick, run one step of:
//         • Prescaler / sigma-delta → BG counter decrement
//         • BG underflow → post m_baud_tick; reload BG
//         • TX state machine step (on m_baud_tick)
//         • RX state machine step (on m_baud_tick)
//         • IRQ counter decrement
//    c. Update m_last_advance_time.
//
//  Because advance() posts m_baud_tick at SC_ZERO_TIME, txBitMethod() and
//  rxBitMethod() (sensitive to m_baud_tick) run in the same delta-cycle and
//  see the correct state.
//
// ── Baud-rate generation (§4.4, §6.2, §6.3) ──────────────────────────────────
//
//  advanceBaudGen(n_clk_ticks) is called inside advance() and runs the
//  hardware chain for n_clk_ticks f_PERIPH cycles:
//
//    Sync  (M=0) : f_PRE tick every cycle  (prescaler bypassed)
//    FDE=0, BRS=0: f_PRE tick every 2 cycles  (÷2)
//    FDE=0, BRS=1: f_PRE tick every 3 cycles  (÷3)
//    FDE=1       : sigma-delta accumulator; carry = f_PRE tick
//
//  On each f_PRE tick, m_bg_counter is decremented.
//  On BG underflow: m_baud_tick_pending++ and m_bg_counter reloaded.
//
//  BG register read: returns m_bg_counter (kept accurate by advance()).
//
// ── TX / RX state machines (§4.2, §4.3) ──────────────────────────────────────
//
//  txBitMethod()  — sensitive to m_baud_tick; one baud tick per invocation.
//  rxBitMethod()  — sensitive to m_baud_tick; one baud tick per invocation.
//  Both count m_{tx,rx}_sub_tick 0..15 (async) or 0..3 (sync) for oversampling.
//
// ── IRQ pulses (§4.7) ─────────────────────────────────────────────────────────
//
//  Each IRQ output is asserted for exactly 2 f_PERIPH clock periods.
//  The assertion timestamp is stored in m_{tbir,tir,rir,eir}_assert_time.
//  On every advance() call, if (now - assert_time) >= 2 * clk_period the
//  output is de-asserted.  This is a pure timestamp comparison — no counter,
//  no periodic process.
//
// ── Process list (ALL SC_METHOD, NO SC_THREAD, NO next_trigger) ───────────────
//
//  rstMethod()        — sensitive to rst.pos()
//  usartClkInMethod() — sensitive to usartClkIn
//  txBitMethod()      — sensitive to m_baud_tick
//  rxBitMethod()      — sensitive to m_baud_tick
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

// =============================================================================
SC_MODULE(Usart)
{
    // =========================================================================
    // Public ports and sockets
    // =========================================================================

    /// TLM-2.0 32-bit target socket for CPU register access.
    tlm_utils::simple_target_socket<Usart, 32> bus;

    /// Peripheral clock — carries the clock *period* as an sc_time value.
    /// Does NOT toggle.  SC_METHOD usartClkInMethod fires only when this
    /// value changes (startup, dynamic reconfig).
    sc_core::sc_in<sc_core::sc_time> usartClkIn;

    /// Active-high reset.
    sc_core::sc_in<bool> rst;

    /// Transmit data pin.
    ///   Async: serial data out (LSB first); idle = mark (logic 1).
    ///   Sync (Mode 0): clock from ÷4 sampler; idle = high.
    sc_core::sc_out<USART_TxRx_Tlm> txd;

    /// Receive / bidirectional data pin.
    ///   Async: serial data in; idle = mark (logic 1).
    ///   Sync:  CON.REN=0 → output (TX); CON.REN=1 → input (RX).
    sc_core::sc_inout<USART_TxRx_Tlm> rxd;

    // ── Interrupt outputs (§4.7) ──────────────────────────────────────────────
    sc_core::sc_out<bool> tbir; ///< TX buffer empty — 2 × f_PERIPH pulse
    sc_core::sc_out<bool> tir;  ///< TX complete    — 2 × f_PERIPH pulse
    sc_core::sc_out<bool> rir;  ///< RX data ready  — 2 × f_PERIPH pulse
    sc_core::sc_out<bool> eir;  ///< Error          — 2 × f_PERIPH pulse

    // =========================================================================
    // CCI parameters
    // =========================================================================
    cci::cci_param<uint64_t> base_addr;   ///< Peripheral base address (default 0x60000000)
    cci::cci_param<int>      debug_level; ///< 0=silent 1=regs 2=baud 3=sm 4=irq

    // =========================================================================
    // Constructor / Destructor
    // =========================================================================
    SC_CTOR(Usart);
    ~Usart();

    // =========================================================================
    // TLM callbacks
    // =========================================================================
    void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay);
    unsigned int transport_dbg(tlm::tlm_generic_payload &trans);
    bool get_direct_mem_ptr(tlm::tlm_generic_payload &trans,
                            tlm::tlm_dmi            &dmi_data);

    // =========================================================================
    // Direct RX injection — bypasses the serial RX state machine.
    // Used by the QBOX wrapper to bridge two USART instances without
    // requiring baud-rate accurate serial framing.
    // Returns true if the byte was accepted, false on overrun.
    // =========================================================================
    bool rx_inject(uint8_t byte);

    // Advance peripheral state to the current simulation time.
    // Call before reading interrupt status from outside the module so that
    // baud-timed interrupts (TIR, RIR) are up to date.
    void sync() { advance(sc_core::sc_time_stamp()); }

    // =========================================================================
    // IRQ assertion-state accessors
    // =========================================================================
    // In QBOX's multithreaded model, sc_signal updates are deferred to the
    // next delta cycle (the SystemC scheduler thread), which does not run
    // while QEMU is executing b_transport callbacks.  Reading the assert
    // timestamp directly (SC_ZERO_TIME = de-asserted) gives the correct state
    // without requiring a delta-cycle round-trip.
    bool is_tbir_asserted() const { return m_tbir_assert_time != sc_core::SC_ZERO_TIME; }
    bool is_tir_asserted()  const { return m_tir_assert_time  != sc_core::SC_ZERO_TIME; }
    bool is_rir_asserted()  const { return m_rir_assert_time  != sc_core::SC_ZERO_TIME; }
    bool is_eir_asserted()  const { return m_eir_assert_time  != sc_core::SC_ZERO_TIME; }

    // Monotone counter incremented each time stepTx() asserts TIR.
    // Unlike m_tir_assert_time, this is never cleared by updateIrqPulses(),
    // so Usart2 can reliably detect TIR even when the 2-cycle pulse has expired
    // before update_status_from_core() gets to run.
    unsigned int get_tir_fire_count() const { return m_tir_fire_count; }

    // Returns the full bit period based on current CON/BG register state:
    //   bit_period = clk_period × prescaler × (BG_reload+1) × oversampling
    // Prescaler: 1 (sync Mode 0), 3 (async BRS=1), 2 otherwise.
    // Oversampling: 4 (sync), 16 (async).
    // Returns SC_ZERO_TIME if clock not configured or R=0.
    sc_core::sc_time get_baud_period() const;

    // =========================================================================
    // Direct RBUF accessors — do NOT call advance(), safe from any thread.
    // Used by Usart2::b_transport to serve RBUF reads without triggering the
    // lazy-advance engine (which replays all elapsed baud ticks and is slow
    // when called after a long simulation-time gap between b_transport calls).
    // =========================================================================
    uint32_t get_rbuf_raw()  const { return m_rbuf.raw; }
    void     clear_rbuf()          { m_rbuf_full = false; m_rbuf.raw = 0u; }

private:
    // =========================================================================
    // Register bank
    // =========================================================================
    CON_reg  m_con;        ///< 0x00 CON
    TBUF_reg m_tbuf_reg;   ///< 0x04 TBUF latch
    RBUF_reg m_rbuf;       ///< 0x08 RBUF
    BG_reg   m_bg_reload;  ///< 0x0C BG reload register
    uint16_t m_bg_counter; ///< 0x0C BG live downcounter
    FDR_reg  m_fdr;        ///< 0x10 FDR

    // =========================================================================
    // TX path
    // =========================================================================
    bool     m_tbuf_full;
    uint32_t m_tbuf_data;
    bool     m_tsr_busy;
    uint32_t m_tsr;
    bool     m_tsr_parity;
    int      m_tx_sub_tick; ///< 0..15 async, 0..3 sync

    enum class TxState : uint8_t { IDLE, START, DATA, PARITY, STOP1, STOP2 };
    TxState m_tx_state;
    int     m_tx_bit_idx;

    // =========================================================================
    // RX path
    // =========================================================================
    bool     m_rbuf_full;
    bool     m_rsr_busy;
    uint32_t m_rsr;
    bool     m_rx_par_error;
    bool     m_rx_rcvd_par; ///< raw received parity bit, for RBUF packing
    int      m_rx_sub_tick; ///< 0..15 async, 0..3 sync

    enum class RxState : uint8_t { IDLE, START_CHK, DATA, PARITY, STOP };
    RxState m_rx_state;
    int     m_rx_bit_idx;
    int     m_rx_stop_cnt;

    // =========================================================================
    // Baud-rate generation — lazy / timestamp-based
    // =========================================================================
    //
    // All integer counters below are maintained by advanceBaudGen(), which is
    // called from advance() with the number of f_PERIPH clock ticks elapsed
    // since the last call.  No next_trigger, no toggling clock.

    uint32_t m_pre_counter; ///< Integer prescaler countdown  (FDE=0)
    uint32_t m_fdr_accum;   ///< Sigma-delta accumulator      (FDE=1)

    // =========================================================================
    // Lazy advance machinery
    // =========================================================================

    /// sc_time of the last advance() call.  Initialised to SC_ZERO_TIME.
    sc_core::sc_time m_last_advance_time;

    /// Cached f_PERIPH period (updated when usartClkIn changes).
    sc_core::sc_time m_clk_period;

    /// Advance the entire peripheral state to sc_time_stamp() (or a given time).
    /// Computes elapsed f_PERIPH ticks, steps the baud-gen chain, and applies
    /// all baud ticks to the TX / RX state machines and IRQ counters.
    /// Posts m_baud_tick for each BG underflow encountered.
    void advance(sc_core::sc_time now);

    /// Run the prescaler / sigma-delta / BG chain for exactly one f_PERIPH tick.
    /// Returns true if a BG underflow (= baud tick) occurred this tick.
    bool stepBaudGen();

    /// Advance TX state machine by one baud sub-tick (called per baud tick).
    void stepTx();

    /// Advance RX state machine by one baud sub-tick (called per baud tick).
    void stepRx();

    /// Check and de-assert any IRQ whose 2-cycle pulse has expired.
    void updateIrqPulses(sc_core::sc_time now);

    // =========================================================================
    // IRQ pulse tracking — timestamp-based, no periodic counter
    // =========================================================================
    // Each output is de-asserted when (now - assert_time) >= 2 * clk_period.
    // SC_ZERO_TIME means "not currently asserted".

    sc_core::sc_time m_tbir_assert_time;
    sc_core::sc_time m_tir_assert_time;
    sc_core::sc_time m_rir_assert_time;
    sc_core::sc_time m_eir_assert_time;
    unsigned int     m_tir_fire_count;  ///< incremented by stepTx on every TIR assert

    // =========================================================================
    // Internal event — posted at SC_ZERO_TIME on each BG underflow
    // =========================================================================
    sc_core::sc_event m_baud_tick;

    // =========================================================================
    // SC_METHOD process declarations
    // =========================================================================

    /// Fires when usartClkIn value changes; updates m_clk_period and calls advance().
    void usartClkInMethod();

    /// Fires on rst posedge; resets all state.
    void rstMethod();

    /// TX state machine step — sensitive to m_baud_tick.
    void txBitMethod();

    /// RX state machine step — sensitive to m_baud_tick.
    void rxBitMethod();

    // =========================================================================
    // Private helpers
    // =========================================================================
    uint32_t regRead(uint32_t offset);
    void     regWrite(uint32_t offset, uint32_t value);
    void     loadTSR();
    void     transferRSR(bool oe, bool fe, bool pe);
    static bool computeEvenParity(uint32_t data, uint8_t bits);

    /// Assert one IRQ output for 2 f_PERIPH periods.
    void assertIrq(sc_core::sc_time        &assert_time,
                   sc_core::sc_out<bool>   &port,
                   const char              *irq_name);

    Usart(const Usart &)            = delete;
    Usart &operator=(const Usart &) = delete;
};

#endif // USART_H
