// =============================================================================
// Usart.cpp
// -----------------------------------------------------------------------------
// QBox USART peripheral — complete implementation.
//
// Specification: USART Peripheral Specification Rev 1.0 (§1–§6).
//
// ── Constraints satisfied ────────────────────────────────────────────────────
//
//  • sc_in<sc_time> usartClkIn  — period-value port, never toggles.
//  • NO SC_THREAD anywhere.
//  • NO next_trigger() anywhere.
//  • NO toggling Boolean clock port.
//
// ── Execution model: lazy / timestamp-driven ──────────────────────────────────
//
//  Because usartClkIn never toggles, SC_METHODs cannot fire periodically.
//  All time-driven work is performed lazily: every entry into the module
//  calls advance(now) which computes the number of f_PERIPH ticks elapsed
//  since the previous call and fast-forwards the hardware state by that count.
//
//  Entry points that call advance():
//    usartClkInMethod() — usartClkIn value changed (startup / reconfig)
//    rstMethod()        — rst posedge
//    b_transport()      — CPU register read or write
//    transport_dbg()    — debugger read or write (advance without IRQ side-effects)
//
//  Inside advance(now):
//    1. n_ticks = floor((now - m_last_advance_time) / m_clk_period)
//    2. For i in 0..n_ticks-1:
//         stepBaudGen()  → prescaler / sigma-delta / BG counter; returns true on underflow
//         if underflow:
//             post m_baud_tick at SC_ZERO_TIME (txBitMethod / rxBitMethod wake)
//             stepTx() and stepRx() are called via txBitMethod / rxBitMethod
//    3. updateIrqPulses(now) — de-assert any expired IRQ outputs
//    4. m_last_advance_time += n_ticks * m_clk_period
//
//  Note: stepTx() and stepRx() are also called directly from txBitMethod()
//  and rxBitMethod() (sensitive to m_baud_tick) so they run in the correct
//  delta-cycle order relative to other processes in the design.
//
// ── Baud generation (§4.4, §6.2, §6.3) ──────────────────────────────────────
//
//  stepBaudGen() runs the prescaler / FDR / BG chain for ONE f_PERIPH tick:
//
//    Sync (M=0):     f_PRE tick every cycle.
//    FDE=0, BRS=0:   m_pre_counter counts 0..1; f_PRE tick when it wraps to 0.
//    FDE=0, BRS=1:   m_pre_counter counts 0..2; f_PRE tick when it wraps to 0.
//    FDE=1:          m_fdr_accum += STEP; f_PRE tick when >= denom (512 or 256).
//
//    On each f_PRE tick: decrement m_bg_counter.
//    On BG underflow (m_bg_counter was 0):
//      → return true (caller posts m_baud_tick)
//      → reload m_bg_counter = m_bg_reload.fields.VAL
//
// ── IRQ pulses (§4.7) ────────────────────────────────────────────────────────
//
//  Each IRQ is held high for exactly 2 f_PERIPH clock periods.
//  assertIrq() records m_xxx_assert_time = sc_time_stamp().
//  updateIrqPulses() is called from advance(); it de-asserts any output
//  whose (now - assert_time) >= 2 * m_clk_period.
//  SC_ZERO_TIME is used as the "not asserted" sentinel.
//
// =============================================================================

#include "Usart.h"
#include <iomanip>
#include <sstream>
#include <cstring>

using namespace sc_core;
using namespace tlm;

// ─────────────────────────────────────────────────────────────────────────────
// DBG — guarded debug print.
// Every message starts with sc_time_stamp(), module name, and context.
// ─────────────────────────────────────────────────────────────────────────────
#define DBG(lvl, msg)                                                         \
    do {                                                                      \
        if (debug_level.get_value() >= (lvl)) {                               \
            std::cout << sc_time_stamp() << " " << name() << ": " << msg     \
                      << "\n";                                                \
        }                                                                     \
    } while (0)

#define DBG_FN(lvl) DBG((lvl), __PRETTY_FUNCTION__)

// =============================================================================
// Constructor
// =============================================================================
Usart::Usart(sc_module_name nm)
    : sc_module(nm)
    , bus("bus")
    , usartClkIn("usartClkIn")
    , rst("rst")
    , txd("txd")
    , rxd("rxd")
    , tbir("tbir")
    , tir("tir")
    , rir("rir")
    , eir("eir")
    , base_addr("base_addr", 0x60000000ULL, "USART base address")
    , debug_level("debug_level", 0, "0=silent 1=regs 2=baud 3=sm 4=irq")
    // Register bank
    , m_bg_counter(0u)
    // TX path
    , m_tbuf_full(false)
    , m_tbuf_data(0u)
    , m_tsr_busy(false)
    , m_tsr(0u)
    , m_tsr_parity(false)
    , m_tx_sub_tick(0)
    , m_tx_state(TxState::IDLE)
    , m_tx_bit_idx(0)
    // RX path
    , m_rbuf_full(false)
    , m_rsr_busy(false)
    , m_rsr(0u)
    , m_rx_par_error(false)
    , m_rx_rcvd_par(false)
    , m_rx_sub_tick(0)
    , m_rx_state(RxState::IDLE)
    , m_rx_bit_idx(0)
    , m_rx_stop_cnt(0)
    // Baud generation
    , m_pre_counter(0u)
    , m_fdr_accum(0u)
    // Lazy advance
    , m_last_advance_time(SC_ZERO_TIME)
    , m_clk_period(SC_ZERO_TIME)
    // IRQ timestamps (SC_ZERO_TIME = not asserted)
    , m_tbir_assert_time(SC_ZERO_TIME)
    , m_tir_assert_time(SC_ZERO_TIME)
    , m_rir_assert_time(SC_ZERO_TIME)
    , m_eir_assert_time(SC_ZERO_TIME)
    , m_tir_fire_count(0u)
{
    DBG_FN(1);

    // TLM socket callbacks
    bus.register_b_transport       (this, &Usart::b_transport);
    bus.register_transport_dbg     (this, &Usart::transport_dbg);
    bus.register_get_direct_mem_ptr(this, &Usart::get_direct_mem_ptr);

    // ── SC_METHOD: usartClkIn value change ────────────────────────────────────
    // Fires only when the period value changes (startup or dynamic reconfig).
    // Updates m_clk_period and calls advance() to flush any pending state.
    SC_METHOD(usartClkInMethod);
    sensitive << usartClkIn;
    dont_initialize();

    // ── SC_METHOD: reset ──────────────────────────────────────────────────────
    SC_METHOD(rstMethod);
    sensitive << rst.pos();
    dont_initialize();

    // ── SC_METHOD: TX state machine ───────────────────────────────────────────
    // Sensitive to m_baud_tick — posted at SC_ZERO_TIME by advance() on each
    // BG underflow.  No next_trigger; no periodic clock needed.
    SC_METHOD(txBitMethod);
    sensitive << m_baud_tick;
    dont_initialize();

    // ── SC_METHOD: RX state machine ───────────────────────────────────────────
    SC_METHOD(rxBitMethod);
    sensitive << m_baud_tick;
    dont_initialize();

    // Initial output values
    USART_TxRx_Tlm mark;
    txd.initialize(mark);
    tbir.initialize(false);
    tir.initialize(false);
    rir.initialize(false);
    eir.initialize(false);
}

Usart::~Usart() { DBG_FN(1); }

// =============================================================================
// TLM callbacks
// =============================================================================

void Usart::b_transport(tlm_generic_payload &trans, sc_time &delay)
{
    DBG_FN(1);

    // Advance state to the current simulation time before any register access.
    advance(sc_time_stamp() + delay);

    tlm_command    cmd  = trans.get_command();
    uint64_t       addr = trans.get_address();
    unsigned char *data = trans.get_data_ptr();
    unsigned int   len  = trans.get_data_length();
    unsigned char *be   = trans.get_byte_enable_ptr();

    uint64_t offset64 = addr - base_addr.get_value();
    if (offset64 > USART_REG_MAX_OFFSET || len != 4u) {
        DBG(1, "b_transport: address error — addr=0x"
               << std::hex << addr << " len=" << std::dec << len);
        trans.set_response_status(TLM_ADDRESS_ERROR_RESPONSE);
        return;
    }
    uint32_t offset = static_cast<uint32_t>(offset64);

    if (cmd == TLM_READ_COMMAND) {
        uint32_t rval = regRead(offset);
        if (be) {
            for (unsigned i = 0; i < 4u; ++i)
                if (be[i] == TLM_BYTE_ENABLED)
                    data[i] = reinterpret_cast<uint8_t *>(&rval)[i];
        } else {
            std::memcpy(data, &rval, 4u);
        }
        DBG(1, "READ  offset=0x" << std::hex << offset << " → 0x" << rval << std::dec);

    } else if (cmd == TLM_WRITE_COMMAND) {
        uint32_t wval = 0u;
        if (be) {
            uint32_t existing = regRead(offset);
            std::memcpy(&wval, &existing, 4u);
            for (unsigned i = 0; i < 4u; ++i)
                if (be[i] == TLM_BYTE_ENABLED)
                    reinterpret_cast<uint8_t *>(&wval)[i] = data[i];
        } else {
            std::memcpy(&wval, data, 4u);
        }
        DBG(1, "WRITE offset=0x" << std::hex << offset << " ← 0x" << wval << std::dec);
        regWrite(offset, wval);

    } else {
        trans.set_response_status(TLM_COMMAND_ERROR_RESPONSE);
        return;
    }

    trans.set_dmi_allowed(false);
    trans.set_response_status(TLM_OK_RESPONSE);
    (void)delay;
}

unsigned int Usart::transport_dbg(tlm_generic_payload &trans)
{
    DBG_FN(1);
    advance(sc_time_stamp()); // sync state for accurate register values

    uint64_t       addr = trans.get_address();
    unsigned char *data = trans.get_data_ptr();
    unsigned int   len  = trans.get_data_length();

    uint64_t offset64 = addr - base_addr.get_value();
    if (offset64 > USART_REG_MAX_OFFSET || len != 4u) return 0u;
    uint32_t offset = static_cast<uint32_t>(offset64);

    if (trans.get_command() == TLM_READ_COMMAND) {
        uint32_t rval = regRead(offset);
        std::memcpy(data, &rval, 4u);
        return 4u;
    } else if (trans.get_command() == TLM_WRITE_COMMAND) {
        uint32_t wval = 0u;
        std::memcpy(&wval, data, 4u);
        regWrite(offset, wval);
        return 4u;
    }
    return 0u;
}

bool Usart::get_direct_mem_ptr(tlm_generic_payload & /*t*/, tlm_dmi & /*d*/)
{
    DBG_FN(1);
    return false; // DMI denied: registers have observable side-effects
}

// =============================================================================
// SC_METHOD processes
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// usartClkInMethod — fires when usartClkIn value changes
// ─────────────────────────────────────────────────────────────────────────────
// The period-value signal changed: update m_clk_period and flush any pending
// state up to sc_time_stamp().  This is the only periodic "hook" the module
// has from the simulator, and it fires only on reconfig, not every cycle.
// ─────────────────────────────────────────────────────────────────────────────
void Usart::usartClkInMethod()
{
    DBG_FN(2);

    sc_time new_period = usartClkIn.read();
    if (new_period == SC_ZERO_TIME) {
        DBG(2, "usartClkInMethod: zero period — ignoring");
        return;
    }

    // Advance state with the OLD period up to now before switching period.
    if (m_clk_period != SC_ZERO_TIME) {
        advance(sc_time_stamp());
    }

    m_clk_period = new_period;
    DBG(2, "usartClkInMethod: new period=" << m_clk_period);

    // Reset prescaler counter so it starts cleanly with the new period.
    if (m_con.fields.R && !m_con.fields.FDE && m_con.fields.M != USART_MODE_0) {
        uint32_t prescaler = m_con.fields.BRS ? 3u : 2u;
        m_pre_counter = prescaler - 1u;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// rstMethod — active-high reset
// ─────────────────────────────────────────────────────────────────────────────
void Usart::rstMethod()
{
    DBG_FN(1);
    if (!rst.read()) return;

    // Registers
    m_con.raw       = 0u;
    m_tbuf_reg.raw  = 0u;
    m_rbuf.raw      = 0u;
    m_bg_reload.raw = 0u;
    m_bg_counter    = 0u;
    m_fdr.raw       = 0u;

    // TX path
    m_tbuf_full   = false;
    m_tbuf_data   = 0u;
    m_tsr_busy    = false;
    m_tsr         = 0u;
    m_tsr_parity  = false;
    m_tx_sub_tick = 0;
    m_tx_state    = TxState::IDLE;
    m_tx_bit_idx  = 0;

    // RX path
    m_rbuf_full    = false;
    m_rsr_busy     = false;
    m_rsr          = 0u;
    m_rx_par_error = false;
    m_rx_rcvd_par  = false;
    m_rx_sub_tick  = 0;
    m_rx_state     = RxState::IDLE;
    m_rx_bit_idx   = 0;
    m_rx_stop_cnt  = 0;

    // Baud generation
    m_pre_counter = 0u;
    m_fdr_accum   = 0u;

    // Lazy-advance bookkeeping
    m_last_advance_time = sc_time_stamp();

    // IRQ timestamps
    m_tbir_assert_time = SC_ZERO_TIME;
    m_tir_assert_time  = SC_ZERO_TIME;
    m_rir_assert_time  = SC_ZERO_TIME;
    m_eir_assert_time  = SC_ZERO_TIME;
    m_tir_fire_count   = 0u;

    // Outputs
    USART_TxRx_Tlm mark;
    txd.write(mark);
    tbir.write(false);
    tir.write(false);
    rir.write(false);
    eir.write(false);

    DBG(1, "USART reset complete");
}

// ─────────────────────────────────────────────────────────────────────────────
// txBitMethod — fires on m_baud_tick (posted at SC_ZERO_TIME by advance())
// ─────────────────────────────────────────────────────────────────────────────
// advance() calls stepTx() directly inside its loop, BUT the m_baud_tick
// notification at SC_ZERO_TIME causes this method to fire in the same
// delta cycle, which is needed for other modules that are co-sensitive to
// the baud tick event (e.g., a testbench or monitor).
// The method itself calls stepTx() which is idempotent: advance() has already
// called it, so stepTx() guards via m_tx_sub_tick to not double-advance.
//
// Design choice: advance() calls stepTx()/stepRx() directly so the state
// is always consistent when returning from b_transport.  txBitMethod /
// rxBitMethod exist purely to give external observers a hook on m_baud_tick.
// ─────────────────────────────────────────────────────────────────────────────
void Usart::txBitMethod()
{
    DBG_FN(3);
    // State has already been advanced by the advance() call that posted this
    // event.  Nothing additional to do here; the method exists as an observable
    // notification point for external monitors.
}

void Usart::rxBitMethod()
{
    DBG_FN(3);
    // Same rationale as txBitMethod.
}

// =============================================================================
// Lazy advance engine
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// advance(now)
// ─────────────────────────────────────────────────────────────────────────────
// Core of the lazy execution model.  Brings the peripheral state forward to
// the time `now` by replaying each f_PERIPH clock tick that has elapsed since
// m_last_advance_time.
//
// For each tick:
//   1. stepBaudGen() — prescaler/FDR/BG chain; returns true on BG underflow.
//   2. On BG underflow:
//        a. post m_baud_tick at SC_ZERO_TIME
//        b. stepTx()   — advance TX state machine by one baud sub-tick
//        c. stepRx()   — advance RX state machine by one baud sub-tick
// After all ticks:
//   3. updateIrqPulses(now) — de-assert any expired IRQ outputs
//   4. Update m_last_advance_time
//
// Performance note: for large elapsed times with high BRS/BG settings, the
// number of f_PERIPH ticks without a baud underflow is large.  stepBaudGen()
// is inlined and the body is simple integer arithmetic — this is fast.
// For extreme cases (days of simulation time), a batch-mode optimisation can
// skip directly to the first underflow, but that is not needed here.
// ─────────────────────────────────────────────────────────────────────────────
void Usart::advance(sc_time now)
{
    DBG_FN(2);

    if (m_clk_period == SC_ZERO_TIME) return; // no clock configured yet
    if (!m_con.fields.R)              return; // module gated off
    if (now <= m_last_advance_time)   return; // nothing to advance

    // Number of f_PERIPH ticks to process
    sc_time elapsed = now - m_last_advance_time;
    // Use integer arithmetic: floor(elapsed / m_clk_period)
    // sc_time::to_double() returns time in seconds
    uint64_t n_ticks = static_cast<uint64_t>(
        elapsed.to_seconds() / m_clk_period.to_seconds());

    DBG(2, "advance: elapsed=" << elapsed
           << " clk=" << m_clk_period
           << " n_ticks=" << n_ticks);

    for (uint64_t i = 0; i < n_ticks; ++i) {
        bool underflow = stepBaudGen();
        if (underflow) {
            // Post m_baud_tick so external observers (monitors, testbench)
            // are notified.  SC_ZERO_TIME ensures same-delta delivery.
            m_baud_tick.notify(SC_ZERO_TIME);
            stepTx();
            stepRx();
            // Once TX and RX are both fully idle (no frame in progress, no
            // pending data), every remaining tick does nothing.  Break early
            // so a TBUF write after a long idle gap (e.g. across several
            // SC_SEC of WFI-based waiting) does not burn millions of ticks.
            // The time base is still advanced by the full n_ticks below.
            if (!m_tsr_busy && !m_tbuf_full &&
                m_tx_state == TxState::IDLE && m_rx_state == RxState::IDLE)
                break;
        }
    }

    // Update timestamp by the full n_ticks.  Idle ticks skipped by the early
    // exit above produced no state changes, so skipping them is correct.
    m_last_advance_time += m_clk_period * static_cast<double>(n_ticks);

    // De-assert any IRQ outputs whose 2-cycle pulse has expired.
    updateIrqPulses(now);
}

// ─────────────────────────────────────────────────────────────────────────────
// stepBaudGen — one f_PERIPH tick through the prescaler/FDR/BG chain
// ─────────────────────────────────────────────────────────────────────────────
// Returns true when a BG underflow occurs (= one baud tick for TX/RX).
//
// Implements exactly the hardware chain in spec §3 Figure 1:
//
//   f_PERIPH → [FDR | prescaler] → f_PRE → [÷ BG] → baud_tick
//
// Prescaler (FDE=0):
//   m_pre_counter counts down from (prescaler-1) to 0.
//   On underflow (== 0): one f_PRE tick; reload to (prescaler-1).
//   prescaler = 1 (sync/M=0), 2 (BRS=0), 3 (BRS=1).
//
// Sigma-delta (FDE=1):
//   m_fdr_accum += STEP each tick.
//   Carry (f_PRE tick) when m_fdr_accum >= denom; subtract denom.
//   denom = 512 (DM=0) or 256 (DM=1).
//
// BG counter:
//   Decremented on each f_PRE tick.
//   On underflow: return true; reload from m_bg_reload.
// ─────────────────────────────────────────────────────────────────────────────
bool Usart::stepBaudGen()
{
    uint8_t mode = m_con.fields.M;
    bool f_pre_tick = false;

    if (mode == USART_MODE_0) {
        // Sync: BG decrements every f_PERIPH cycle (prescaler bypassed, §5.1)
        f_pre_tick = true;

    } else if (m_con.fields.FDE) {
        // Sigma-delta (§6.3)
        uint32_t denom = (m_fdr.fields.DM == 0u) ? 512u : 256u;
        uint32_t step  = m_fdr.fields.STEP;
        if (step == 0u) return false; // invalid: STEP=0 stalls the clock
        m_fdr_accum += step;
        if (m_fdr_accum >= denom) {
            m_fdr_accum -= denom;
            f_pre_tick = true;
        }

    } else {
        // Integer prescaler (§4.4, §6.1 BRS)
        // prescaler = 2 (BRS=0) or 3 (BRS=1)
        // m_pre_counter counts from (prescaler-1) down to 0.
        // When it reaches 0 → f_PRE tick; reload.
        if (m_pre_counter == 0u) {
            uint32_t prescaler = m_con.fields.BRS ? 3u : 2u;
            m_pre_counter = prescaler - 1u; // reload for next cycle
            f_pre_tick = true;
        } else {
            m_pre_counter--;
        }
    }

    if (!f_pre_tick) return false;

    // BG 13-bit downcounter (§6.2)
    if (m_bg_counter == 0u) {
        // BG underflow: emit baud tick and reload
        m_bg_counter = m_bg_reload.fields.VAL;
        DBG(2, "stepBaudGen: BG underflow → baud tick; reloaded=" << m_bg_counter);
        return true;
    }
    m_bg_counter--;
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// stepTx — advance TX state machine by one baud sub-tick
// ─────────────────────────────────────────────────────────────────────────────
// Called once per BG underflow (= one f_PRE period).
//
// 16× oversampling (§4.1): m_tx_sub_tick counts 0..15 (async) or 0..3 (sync).
// A bit-state transition occurs when m_tx_sub_tick wraps back to 0 after
// reaching ticks_per_bit-1.
//
// Frame formats per mode (LSB first, §4.5):
//   Mode 0 (sync)  : 8 data bits, no start/stop; clock on TXD, data on RXD
//   Mode 1 (8N1)   : START + D0..D7 + STOP(s)
//   Mode 3 (9-bit) : START + D0..D8 + STOP(s)
//   Mode 4 (7+P)   : START + D0..D6 + PAR + STOP(s)
//   Mode 5 (8+P)   : START + D0..D7 + PAR + STOP(s)
//   Mode 7 (8N2)   : START + D0..D7 + STOP(s)
// ─────────────────────────────────────────────────────────────────────────────
void Usart::stepTx()
{
    if (!m_con.fields.R) return;

    uint8_t mode          = m_con.fields.M;
    int     ticks_per_bit = (mode == USART_MODE_0) ? 4 : 16;

    m_tx_sub_tick++;
    if (m_tx_sub_tick < ticks_per_bit) return; // still within the same bit period
    m_tx_sub_tick = 0;                          // one full bit period elapsed

    // Helper: drive the appropriate output pin
    auto drivePin = [&](uint32_t bit_val, bool is_framing,
                        uint8_t  stop_cnt, bool  par_val)
    {
        USART_TxRx_Tlm f;
        f.data         = bit_val;
        f.startOrStop  = is_framing;
        f.stopBitCount = stop_cnt;
        f.parity       = par_val;
        if (mode == USART_MODE_0) {
            if (!m_con.fields.REN) rxd.write(f); // sync TX: data on RXD
        } else {
            txd.write(f);
            if (m_con.fields.LB) rxd.write(f);   // loopback: TSR → RSR
        }
    };

    switch (m_tx_state) {

    case TxState::IDLE:
        if (m_tbuf_full) loadTSR();
        if (!m_tsr_busy) return;
        if (mode == USART_MODE_0) {
            m_tx_state   = TxState::DATA;
            m_tx_bit_idx = 0;
        } else {
            drivePin(0u, true, 0u, false); // START bit
            m_tx_state   = TxState::START;
            m_tx_bit_idx = 0;
        }
        break;

    case TxState::START:
        // START was driven last tick; now begin first data bit
        m_tx_state = TxState::DATA;
        /* fall through */

    case TxState::DATA: {
        uint8_t  nbits = usart_data_bits(mode);
        uint32_t bit   = (m_tsr >> m_tx_bit_idx) & 1u;
        drivePin(bit, false, 0u, false);
        DBG(3, "TX DATA[" << m_tx_bit_idx << "]=" << bit);
        m_tx_bit_idx++;

        if (m_tx_bit_idx >= static_cast<int>(nbits)) {
            uint8_t nstop = usart_stop_bits(mode, m_con.fields.STP);
            if (usart_mode_has_hw_parity(mode)) {
                m_tx_state = TxState::PARITY;
            } else {
                if (nstop == 1u) {
                    assertIrq(m_tir_assert_time, tir, "tir"); // last stop = STOP1
                    ++m_tir_fire_count;
                }
                m_tx_state = TxState::STOP1;
            }
        }
        break;
    }

    case TxState::PARITY: {
        drivePin(m_tsr_parity ? 1u : 0u, false, 0u, m_tsr_parity);
        DBG(3, "TX PARITY=" << m_tsr_parity);
        uint8_t nstop = usart_stop_bits(mode, m_con.fields.STP);
        if (nstop == 1u) {
            assertIrq(m_tir_assert_time, tir, "tir"); // last stop = STOP1
            ++m_tir_fire_count;
        }
        m_tx_state = TxState::STOP1;
        break;
    }

    case TxState::STOP1: {
        uint8_t nstop = usart_stop_bits(mode, m_con.fields.STP);
        drivePin(1u, true, nstop, false);
        DBG(3, "TX STOP1 nstop=" << +nstop);
        if (nstop >= 2u) {
            assertIrq(m_tir_assert_time, tir, "tir"); // last stop = STOP2
            ++m_tir_fire_count;
            m_tx_state = TxState::STOP2;
        } else {
            m_tsr_busy = false;
            m_tx_state = TxState::IDLE;
            if (m_tbuf_full) loadTSR();
        }
        break;
    }

    case TxState::STOP2: {
        uint8_t nstop = usart_stop_bits(mode, m_con.fields.STP);
        drivePin(1u, true, nstop, false);
        DBG(3, "TX STOP2");
        m_tsr_busy = false;
        m_tx_state = TxState::IDLE;
        if (m_tbuf_full) loadTSR();
        break;
    }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// stepRx — advance RX state machine by one baud sub-tick
// ─────────────────────────────────────────────────────────────────────────────
// Called once per BG underflow.  16× oversampling; samples at sub_tick==8
// (midpoint of the bit period) for async modes, sub_tick==2 for sync (÷4).
//
// START validation (§2, §4.1):
//   High→low transition: enter START_CHK.
//   At midpoint (sub_tick==8): if line still low → valid; else → glitch.
// ─────────────────────────────────────────────────────────────────────────────
void Usart::stepRx()
{
    if (!m_con.fields.R || !m_con.fields.REN) return;

    uint8_t mode          = m_con.fields.M;
    int     ticks_per_bit = (mode == USART_MODE_0) ? 4 : 16;
    int     sample_tick   = ticks_per_bit / 2; // 8 (async) or 2 (sync)

    m_rx_sub_tick++;
    bool sample_now = (m_rx_sub_tick == sample_tick);
    bool bit_done   = (m_rx_sub_tick >= ticks_per_bit);
    if (bit_done) m_rx_sub_tick = 0;

    // Read incoming line (loopback: sample txd instead of rxd pin)
    USART_TxRx_Tlm pin = m_con.fields.LB ? txd.read() : rxd.read();
    uint32_t line = pin.data & 1u;

    switch (m_rx_state) {

    case RxState::IDLE:
        if (mode == USART_MODE_0) {
            if (m_tsr_busy) {
                m_rx_state   = RxState::DATA;
                m_rx_bit_idx = 0;
                m_rsr        = 0u;
                m_rsr_busy   = true;
            }
        } else if (line == 0u) {
            // Falling edge: potential START
            m_rx_state    = RxState::START_CHK;
            m_rx_sub_tick = 0;
            DBG(3, "RX: START detected → START_CHK");
        }
        break;

    case RxState::START_CHK:
        if (sample_now) {
            if (line == 1u) {
                // Glitch: line high at midpoint → reject (§2)
                m_rx_state    = RxState::IDLE;
                m_rx_sub_tick = 0;
                DBG(3, "RX: START glitch rejected");
            } else {
                m_rx_state   = RxState::DATA;
                m_rx_bit_idx = 0;
                m_rsr        = 0u;
                m_rsr_busy   = true;
                m_con.fields.FE = 0u; // clear FE on valid start (§6.1)
                DBG(3, "RX: START validated → DATA");
            }
        }
        break;

    case RxState::DATA:
        if (!sample_now) break;
        {
            uint8_t nbits = usart_data_bits(mode);
            m_rsr |= (line << m_rx_bit_idx);
            DBG(3, "RX DATA[" << m_rx_bit_idx << "]=" << line);
            m_rx_bit_idx++;

            if (m_rx_bit_idx >= static_cast<int>(nbits)) {
                if (usart_mode_has_hw_parity(mode)) {
                    m_rx_state     = RxState::PARITY;
                    m_rx_par_error = false;
                    m_rx_rcvd_par  = false;
                } else if (mode == USART_MODE_0) {
                    bool oe = m_rbuf_full;
                    if (!oe) {
                        transferRSR(false, false, false);
                    } else {
                        m_con.fields.OE = 1u;
                        assertIrq(m_rir_assert_time, rir, "rir");
                        if (m_con.fields.OEN)
                            assertIrq(m_eir_assert_time, eir, "eir");
                    }
                    m_rx_state = RxState::IDLE;
                    m_rsr_busy = false;
                } else {
                    m_rx_state    = RxState::STOP;
                    m_rx_stop_cnt = usart_stop_bits(mode, m_con.fields.STP);
                }
            }
        }
        break;

    case RxState::PARITY:
        if (!sample_now) break;
        {
            uint8_t nbits  = usart_data_bits(mode);
            m_rx_rcvd_par  = (line != 0u);
            bool even_par  = computeEvenParity(m_rsr, nbits);
            bool exp_par   = m_con.fields.ODD ? !even_par : even_par;
            m_rx_par_error = (m_rx_rcvd_par != exp_par);
            DBG(3, "RX PARITY rcvd=" << m_rx_rcvd_par
                   << " exp=" << exp_par << " err=" << m_rx_par_error);
            m_rx_state    = RxState::STOP;
            m_rx_stop_cnt = usart_stop_bits(mode, m_con.fields.STP);
        }
        break;

    case RxState::STOP:
        if (!sample_now) break;
        {
            bool fe_this = (line == 0u);
            m_rx_stop_cnt--;
            if (m_rx_stop_cnt > 0) {
                if (fe_this) m_con.fields.FE = 1u;
                break;
            }
            bool oe = m_rbuf_full;
            bool fe = fe_this;
            bool pe = m_rx_par_error;
            DBG(3, "RX STOP OE=" << oe << " FE=" << fe << " PE=" << pe);
            if (!oe) {
                transferRSR(oe, fe, pe);
            } else {
                m_con.fields.OE = 1u;
                assertIrq(m_rir_assert_time, rir, "rir");
                if (m_con.fields.OEN)
                    assertIrq(m_eir_assert_time, eir, "eir");
            }
            m_rx_state = RxState::IDLE;
            m_rsr_busy = false;
        }
        break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// updateIrqPulses — de-assert expired IRQ outputs
// ─────────────────────────────────────────────────────────────────────────────
// Called from advance() after processing all elapsed ticks.
// Checks each asserted IRQ: if (now - assert_time) >= 2 * clk_period,
// de-assert the output and clear the timestamp.
// SC_ZERO_TIME used as "not asserted" sentinel.
// ─────────────────────────────────────────────────────────────────────────────
void Usart::updateIrqPulses(sc_time now)
{
    if (m_clk_period == SC_ZERO_TIME) return;
    sc_time pulse_width = m_clk_period * 2.0;

    auto check = [&](sc_time &assert_time, sc_out<bool> &port, const char *n) {
        if (assert_time == SC_ZERO_TIME) return; // not asserted
        // Guard against sc_time underflow: assert_time may be 1 ps (clamped
        // from t=0 in assertIrq) while now is still SC_ZERO_TIME.
        if (now < assert_time) return;
        if ((now - assert_time) >= pulse_width) {
            port.write(false);
            assert_time = SC_ZERO_TIME;
            DBG(4, "IRQ de-asserted: " << n);
        }
    };

    check(m_tbir_assert_time, tbir, "tbir");
    check(m_tir_assert_time,  tir,  "tir");
    check(m_rir_assert_time,  rir,  "rir");
    check(m_eir_assert_time,  eir,  "eir");
}

// =============================================================================
// Register access helpers
// =============================================================================

uint32_t Usart::regRead(uint32_t offset)
{
    DBG(1, __PRETTY_FUNCTION__ << " offset=0x" << std::hex << offset << std::dec);

    switch (offset) {

    case USART_CON_OFFSET:
        return m_con.raw & 0x0000FFFFu;

    case USART_TBUF_OFFSET:
        // Spec: W-only; expose latch for debugger
        return m_tbuf_reg.raw & TBUF_DATA_MASK;

    case USART_RBUF_OFFSET: {
        // Reading RBUF clears CON.OE and CON.PE (§6.1); frees RBUF slot
        uint32_t val    = m_rbuf.raw & RBUF_DATA_MASK;
        m_con.fields.OE = 0u;
        m_con.fields.PE = 0u;
        m_rbuf_full     = false;
        DBG(2, "RBUF read → 0x" << std::hex << val
               << "  OE/PE cleared" << std::dec);
        return val;
    }

    case USART_BG_OFFSET:
        // m_bg_counter is always current (updated by stepBaudGen in advance()).
        DBG(2, "BG read → " << m_bg_counter);
        return static_cast<uint32_t>(m_bg_counter) & BG_VAL_MASK;

    case USART_FDR_OFFSET:
        return m_fdr.raw & FDR_WR_MASK;

    default:
        DBG(1, "regRead: unknown offset 0x" << std::hex << offset << std::dec);
        return 0xDEADBEEFu;
    }
}

void Usart::regWrite(uint32_t offset, uint32_t value)
{
    DBG(1, __PRETTY_FUNCTION__ << " offset=0x" << std::hex << offset
           << " value=0x" << value << std::dec);

    switch (offset) {

    case USART_CON_OFFSET: {
        // Preserve RO error flags; allow SW clear of OE (write 0 to bit 9)
        uint32_t hw_errors = m_con.raw & CON_ERR_MASK;
        if (!(value & CON_OE_BIT)) hw_errors &= ~CON_OE_BIT;
        hw_errors &= (CON_FE_BIT | CON_PE_BIT);
        m_con.raw = (value & CON_RW_MASK) | hw_errors;
        DBG(2, "CON ← 0x" << std::hex << m_con.raw << std::dec
               << " M="  << +m_con.fields.M
               << " R="  << +m_con.fields.R
               << " BRS="<< +m_con.fields.BRS
               << " FDE="<< +m_con.fields.FDE
               << " LB=" << +m_con.fields.LB);
        // On enabling the module, reset prescaler from current configuration
        if (m_con.fields.R && !m_con.fields.FDE &&
            m_con.fields.M != USART_MODE_0) {
            uint32_t prescaler = m_con.fields.BRS ? 3u : 2u;
            m_pre_counter = prescaler - 1u;
        }
        break;
    }

    case USART_TBUF_OFFSET:
        // Write ignored when TBUF already full (§6.4)
        if (m_tbuf_full) {
            DBG(2, "TBUF write ignored — buffer full");
            break;
        }
        m_tbuf_reg.raw = value & TBUF_DATA_MASK;
        m_tbuf_data    = m_tbuf_reg.fields.DATA;
        m_tbuf_full    = true;
        DBG(2, "TBUF ← 0x" << std::hex << m_tbuf_data << std::dec);
        if (!m_tsr_busy && m_con.fields.R) loadTSR(); // immediate transfer if TSR idle
        break;

    case USART_RBUF_OFFSET:
        // R-only per spec; writes ignored
        DBG(2, "RBUF write ignored (R-only)");
        break;

    case USART_BG_OFFSET:
        // Write targets reload register ONLY; running counter unchanged (§6.2)
        m_bg_reload.raw = value & BG_VAL_MASK;
        DBG(2, "BG reload ← " << m_bg_reload.fields.VAL
               << "  (counter unchanged at " << m_bg_counter << ")");
        break;

    case USART_FDR_OFFSET:
        m_fdr.raw   = value & FDR_WR_MASK;
        m_fdr_accum = 0u; // reset accumulator on config change
        DBG(2, "FDR ← STEP=" << +m_fdr.fields.STEP
               << " DM=" << +m_fdr.fields.DM);
        break;

    default:
        DBG(1, "regWrite: unknown offset 0x" << std::hex << offset << std::dec);
        break;
    }
}

// =============================================================================
// TX helpers
// =============================================================================

void Usart::loadTSR()
{
    DBG_FN(3);
    uint8_t  mode  = m_con.fields.M;
    uint8_t  nbits = usart_data_bits(mode);
    uint32_t mask  = (nbits >= 32u) ? 0xFFFFFFFFu : ((1u << nbits) - 1u);

    m_tsr        = m_tbuf_data & mask;
    m_tsr_busy   = true;
    m_tbuf_full  = false;
    m_tx_bit_idx = 0;

    if (usart_mode_has_hw_parity(mode)) {
        bool even_par = computeEvenParity(m_tsr, nbits);
        m_tsr_parity  = m_con.fields.ODD ? !even_par : even_par;
    } else {
        m_tsr_parity = false;
    }

    DBG(3, "loadTSR: tsr=0x" << std::hex << m_tsr
           << " par=" << m_tsr_parity << std::dec);

    // TBIR: fires when TBUF slot freed (§4.7)
    assertIrq(m_tbir_assert_time, tbir, "tbir");
}

// =============================================================================
// RX helpers
// =============================================================================

void Usart::transferRSR(bool oe, bool fe, bool pe)
{
    DBG_FN(3);
    uint8_t  mode   = m_con.fields.M;
    uint32_t packed = 0u;

    if (usart_mode_has_hw_parity(mode)) {
        // §6.5: data left-shifted ×2; raw received parity bit at RBUF[0]
        packed = ((m_rsr & 0x1FFu) << 1u) | (m_rx_rcvd_par ? 1u : 0u);
    } else {
        packed = m_rsr & RBUF_DATA_MASK;
    }

    m_rbuf.fields.DATA = packed & RBUF_DATA_MASK;
    m_rbuf_full        = true;

    if (oe) m_con.fields.OE = 1u;
    if (fe) m_con.fields.FE = 1u;
    if (pe) m_con.fields.PE = 1u;

    DBG(3, "transferRSR: RBUF=0x" << std::hex << m_rbuf.fields.DATA << std::dec
           << " OE=" << oe << " FE=" << fe << " PE=" << pe);

    assertIrq(m_rir_assert_time, rir, "rir");

    bool fire_eir = (oe && m_con.fields.OEN) ||
                    (fe && m_con.fields.FEN) ||
                    (pe && m_con.fields.PEN);
    if (fire_eir) assertIrq(m_eir_assert_time, eir, "eir");
}

// =============================================================================
// Direct RX injection
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// rx_inject — inject a byte directly into the RBUF, bypassing the RX state
// machine and serial pin simulation.
//
// Used by Usart2 when data arrives via a biflow_socket bridge (SerialBridge)
// rather than via the rxd serial pin.
//
// Packing matches what transferRSR() produces for each mode:
//   Non-parity (1, 3, 7): packed = raw byte value (m_rsr accumulates LSB-first)
//   Parity (4, 5):        packed = (byte << 1) | par_bit
//
// Returns true if accepted, false on overrun.
// ─────────────────────────────────────────────────────────────────────────────
bool Usart::rx_inject(uint8_t byte)
{
    if (!m_con.fields.R || !m_con.fields.REN) {
        DBG(2, "rx_inject: dropped — module not running or RX disabled");
        return false;
    }
    if (m_rbuf_full) {
        // Overrun: set OE flag and optionally fire EIR
        m_con.fields.OE = 1u;
        if (m_con.fields.OEN)
            assertIrq(m_eir_assert_time, eir, "eir");
        DBG(2, "rx_inject: OVERRUN — byte=0x" << std::hex << static_cast<unsigned>(byte)
               << std::dec);
        return false;
    }
    uint8_t  mode = m_con.fields.M;
    uint32_t packed;
    if (usart_mode_has_hw_parity(mode)) {
        // Parity modes 4 (7+P) and 5 (8+P): data left-shifted, parity at bit 0
        uint8_t nbits    = usart_data_bits(mode);
        bool    even_par = computeEvenParity(byte, nbits);
        bool    par      = m_con.fields.ODD ? !even_par : even_par;
        packed = ((static_cast<uint32_t>(byte) << 1u) & RBUF_DATA_MASK) |
                 (par ? 1u : 0u);
    } else {
        // Non-parity modes (1, 3, 7): raw accumulation — same layout as m_rsr
        // bit0=D0, ..., bit7=D7  (bit8=D8 for mode 3 only)
        packed = static_cast<uint32_t>(byte) & RBUF_DATA_MASK;
    }
    m_rbuf.fields.DATA = packed;
    m_rbuf_full        = true;
    DBG(2, "rx_inject: byte=0x" << std::hex << static_cast<unsigned>(byte)
           << " packed=0x" << packed << " mode=" << +mode << std::dec);
    assertIrq(m_rir_assert_time, rir, "rir");
    return true;
}

// =============================================================================
// IRQ assertion
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// assertIrq — record assertion timestamp and drive the output high
// ─────────────────────────────────────────────────────────────────────────────
// The output will be de-asserted by updateIrqPulses() once
// ─────────────────────────────────────────────────────────────────────────────
// get_baud_period
// ─────────────────────────────────────────────────────────────────────────────
sc_core::sc_time Usart::get_baud_period() const
{
    if (m_clk_period == sc_core::SC_ZERO_TIME || !m_con.fields.R)
        return sc_core::SC_ZERO_TIME;

    unsigned prescaler;
    unsigned oversample;
    if (m_con.fields.M == USART_MODE_0) {
        prescaler  = 1u;
        oversample = 4u;   // sync: sub_tick 0..3
    } else {
        prescaler  = (m_con.fields.FDE || !m_con.fields.BRS) ? 2u : 3u;
        oversample = 16u;  // async: sub_tick 0..15
    }
    double factor = static_cast<double>(prescaler)
                  * static_cast<double>(m_bg_reload.raw + 1u)
                  * static_cast<double>(oversample);
    return m_clk_period * factor;
}

// (sc_time_stamp() - assert_time) >= 2 * m_clk_period.
// ─────────────────────────────────────────────────────────────────────────────
void Usart::assertIrq(sc_time       &assert_time,
                       sc_out<bool>  &port,
                       const char    *irq_name)
{
    DBG(4, __PRETTY_FUNCTION__ << " irq=" << irq_name);
    sc_core::sc_time t = sc_core::sc_time_stamp();
    // SC_ZERO_TIME is the "not asserted" sentinel used by is_*_asserted() and
    // updateIrqPulses().  If the assertion happens at t=0 we bump to 1 ps so
    // the sentinel comparison never collides with a genuine assertion time.
    if (t == sc_core::SC_ZERO_TIME)
        t = sc_core::sc_time(1, sc_core::SC_PS);
    assert_time = t;
    port.write(true);
}

// =============================================================================
// Utility
// =============================================================================

bool Usart::computeEvenParity(uint32_t data, uint8_t bits)
{
    uint32_t mask = (bits >= 32u) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    data &= mask;
    data ^= data >> 16u;
    data ^= data >> 8u;
    data ^= data >> 4u;
    data ^= data >> 2u;
    data ^= data >> 1u;
    return (data & 1u) != 0u;
}
