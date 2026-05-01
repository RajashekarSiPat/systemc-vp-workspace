// =============================================================================
// Usart.cc
// -----------------------------------------------------------------------------
// USART peripheral — TLM frame-level implementation.
//
// TX model (frame-level, not bit-by-bit):
//   TBUF write → loadTSR():
//     • TBIR asserted immediately (TBUF slot freed)
//     • Complete frame written to txd port once (data known at load time)
//     • m_tx_done_ev scheduled after get_frame_duration()
//   m_tx_done_ev → txDoneMethod():
//     • TIR asserted
//     • m_tsr_busy cleared
//     • Chains next byte if m_tbuf_full
//
// RX model:
//   rx_inject() called by Usart2 after one frame_duration has elapsed.
//   Writes directly to RBUF, asserts RIR/EIR.  No pin sampling.
//
// IRQ pulses:
//   assertIrq() drives output HIGH and schedules m_irq_deassert_ev.
//   irqDeassertMethod() calls updateIrqPulses() to lower expired outputs.
//
// advance() is kept as a lightweight call that updates m_last_advance_time
// and calls updateIrqPulses().  It no longer loops over baud ticks.
// =============================================================================

#include "Usart.h"
#include <iomanip>
#include <sstream>
#include <cstring>

using namespace sc_core;
using namespace tlm;

#define DBG(lvl, msg)                                                        \
    do {                                                                     \
        if (debug_level.get_value() >= (lvl)) {                              \
            std::cout << sc_time_stamp() << " " << name() << ": " << msg    \
                      << "\n";                                               \
        }                                                                    \
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
    , m_bg_counter(0u)
    , m_tbuf_full(false)
    , m_tbuf_data(0u)
    , m_tsr_busy(false)
    , m_tsr(0u)
    , m_rbuf_full(false)
    , m_last_advance_time(SC_ZERO_TIME)
    , m_clk_period(SC_ZERO_TIME)
    , m_tbir_assert_time(SC_ZERO_TIME)
    , m_tir_assert_time(SC_ZERO_TIME)
    , m_rir_assert_time(SC_ZERO_TIME)
    , m_eir_assert_time(SC_ZERO_TIME)
    , m_tir_fire_count(0u)
{
    DBG_FN(1);

    bus.register_b_transport       (this, &Usart::b_transport);
    bus.register_transport_dbg     (this, &Usart::transport_dbg);
    bus.register_get_direct_mem_ptr(this, &Usart::get_direct_mem_ptr);

    SC_METHOD(usartClkInMethod);
    sensitive << usartClkIn;
    dont_initialize();

    SC_METHOD(rstMethod);
    sensitive << rst.pos();
    dont_initialize();

    // Fires after one full frame_duration from loadTSR()
    SC_METHOD(txDoneMethod);
    sensitive << m_tx_done_ev;
    dont_initialize();

    // Fires 2 * m_clk_period after assertIrq() to de-assert the IRQ pulse
    SC_METHOD(irqDeassertMethod);
    sensitive << m_irq_deassert_ev;
    dont_initialize();

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
    advance(sc_time_stamp() + delay);

    tlm_command    cmd  = trans.get_command();
    uint64_t       addr = trans.get_address();
    unsigned char *data = trans.get_data_ptr();
    unsigned int   len  = trans.get_data_length();
    unsigned char *be   = trans.get_byte_enable_ptr();

    uint64_t offset64 = addr - base_addr.get_value();
    if (offset64 > USART_REG_MAX_OFFSET || len != 4u) {
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
    advance(sc_time_stamp());

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
    return false;
}

// =============================================================================
// SC_METHOD processes
// =============================================================================

void Usart::usartClkInMethod()
{
    DBG_FN(2);
    sc_time new_period = usartClkIn.read();
    if (new_period == SC_ZERO_TIME) return;
    if (m_clk_period != SC_ZERO_TIME)
        advance(sc_time_stamp());
    m_clk_period = new_period;
    DBG(2, "usartClkInMethod: new period=" << m_clk_period);
}

void Usart::rstMethod()
{
    DBG_FN(1);
    if (!rst.read()) return;

    m_con.raw       = 0u;
    m_tbuf_reg.raw  = 0u;
    m_rbuf.raw      = 0u;
    m_bg_reload.raw = 0u;
    m_bg_counter    = 0u;
    m_fdr.raw       = 0u;

    m_tbuf_full  = false;
    m_tbuf_data  = 0u;
    m_tsr_busy   = false;
    m_tsr        = 0u;
    m_rbuf_full  = false;

    m_last_advance_time = sc_time_stamp();

    m_tbir_assert_time = SC_ZERO_TIME;
    m_tir_assert_time  = SC_ZERO_TIME;
    m_rir_assert_time  = SC_ZERO_TIME;
    m_eir_assert_time  = SC_ZERO_TIME;
    m_tir_fire_count   = 0u;

    m_tx_done_ev.cancel();
    m_irq_deassert_ev.cancel();

    USART_TxRx_Tlm mark;
    txd.write(mark);
    tbir.write(false);
    tir.write(false);
    rir.write(false);
    eir.write(false);

    DBG(1, "USART reset complete");
}

// ─────────────────────────────────────────────────────────────────────────────
// txDoneMethod — fires after one frame_duration from loadTSR()
//
// The entire serial frame has now "completed" on the line.  Fire TIR to
// notify the CPU that the transmitter is free, then chain the next byte if
// TBUF was filled while the previous frame was in progress.
// ─────────────────────────────────────────────────────────────────────────────
void Usart::txDoneMethod()
{
    DBG_FN(3);
    if (!m_con.fields.R) return;

    assertIrq(m_tir_assert_time, tir, "tir");
    ++m_tir_fire_count;

    m_tsr_busy = false;

    if (m_tbuf_full)
        loadTSR();
}

// ─────────────────────────────────────────────────────────────────────────────
// irqDeassertMethod — fires in the next delta cycle after assertIrq()
//
// Drives all active IRQ outputs LOW immediately (bypasses the 2-cycle
// pulse-width check used by updateIrqPulses).  assertIrq() now schedules
// the deassert at SC_ZERO_TIME so the de-assertion happens within the same
// SC-time epoch as the assertion — this avoids requiring the SC scheduler
// to advance real simulated time (which is gated by the QEMU thread's local
// time and can stall in multithread-unconstrained mode while the QEMU thread
// is blocked in sleep_for()).  The advance() / updateIrqPulses() path still
// handles late de-assertions if the event fires after the time has moved.
// ─────────────────────────────────────────────────────────────────────────────
void Usart::irqDeassertMethod()
{
    DBG_FN(4);
    auto deassert = [](sc_core::sc_time &atime, sc_out<bool> &port) {
        if (atime != sc_core::SC_ZERO_TIME) {
            port.write(false);
            atime = sc_core::SC_ZERO_TIME;
        }
    };
    deassert(m_tbir_assert_time, tbir);
    deassert(m_tir_assert_time,  tir);
    deassert(m_rir_assert_time,  rir);
    deassert(m_eir_assert_time,  eir);
}

// =============================================================================
// advance — lightweight timestamp update + IRQ pulse check
//
// No baud-tick loop: TX is driven by m_tx_done_ev, RX by rx_inject().
// Called from b_transport / transport_dbg before every register access so
// that updateIrqPulses() sees the current time and can de-assert stale pulses
// if irqDeassertMethod() was delayed by simulation scheduling.
// =============================================================================
void Usart::advance(sc_time now)
{
    if (m_clk_period == SC_ZERO_TIME) return;
    if (!m_con.fields.R)              return;
    if (now <= m_last_advance_time)   return;
    m_last_advance_time = now;
    updateIrqPulses(now);
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
        return m_tbuf_reg.raw & TBUF_DATA_MASK;

    case USART_RBUF_OFFSET: {
        uint32_t val    = m_rbuf.raw & RBUF_DATA_MASK;
        m_con.fields.OE = 0u;
        m_con.fields.PE = 0u;
        m_rbuf_full     = false;
        DBG(2, "RBUF read → 0x" << std::hex << val << "  OE/PE cleared" << std::dec);
        return val;
    }

    case USART_BG_OFFSET:
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
        uint32_t hw_errors = m_con.raw & CON_ERR_MASK;
        if (!(value & CON_OE_BIT)) hw_errors &= ~CON_OE_BIT;
        hw_errors &= (CON_FE_BIT | CON_PE_BIT);
        m_con.raw = (value & CON_RW_MASK) | hw_errors;
        DBG(2, "CON ← 0x" << std::hex << m_con.raw << std::dec
               << " M=" << +m_con.fields.M << " R=" << +m_con.fields.R);
        break;
    }

    case USART_TBUF_OFFSET:
        if (m_tbuf_full) {
            DBG(2, "TBUF write ignored — buffer full");
            break;
        }
        m_tbuf_reg.raw = value & TBUF_DATA_MASK;
        m_tbuf_data    = m_tbuf_reg.fields.DATA;
        m_tbuf_full    = true;
        DBG(2, "TBUF ← 0x" << std::hex << m_tbuf_data << std::dec);
        if (!m_tsr_busy && m_con.fields.R) loadTSR();
        break;

    case USART_RBUF_OFFSET:
        DBG(2, "RBUF write ignored (R-only)");
        break;

    case USART_BG_OFFSET:
        m_bg_reload.raw = value & BG_VAL_MASK;
        DBG(2, "BG reload ← " << m_bg_reload.fields.VAL);
        break;

    case USART_FDR_OFFSET:
        m_fdr.raw = value & FDR_WR_MASK;
        DBG(2, "FDR ← STEP=" << +m_fdr.fields.STEP << " DM=" << +m_fdr.fields.DM);
        break;

    default:
        DBG(1, "regWrite: unknown offset 0x" << std::hex << offset << std::dec);
        break;
    }
}

// =============================================================================
// TX helpers
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// loadTSR — move TBUF into the shift register, fire TBIR, schedule TIR
//
// The complete frame (data + parity + framing) is fully determined here.
// We write the whole frame to the txd port once (TLM abstraction: the
// physical bit sequence is knowable at t=0; only the end-time matters).
// m_tx_done_ev fires after get_frame_duration() to model serial line
// occupancy without simulating individual bit periods.
// ─────────────────────────────────────────────────────────────────────────────
void Usart::loadTSR()
{
    DBG_FN(3);
    uint8_t  mode  = m_con.fields.M;
    uint8_t  nbits = usart_data_bits(mode);
    uint32_t mask  = (nbits >= 32u) ? 0xFFFFFFFFu : ((1u << nbits) - 1u);

    m_tsr       = m_tbuf_data & mask;
    m_tsr_busy  = true;
    m_tbuf_full = false;

    // Compute parity locally — no need to cache it as member state.
    bool par = false;
    if (usart_mode_has_hw_parity(mode)) {
        bool even = computeEvenParity(m_tsr, nbits);
        par = m_con.fields.ODD ? !even : even;
    }

    // Write the complete frame to txd once (TLM level: all bits known now).
    USART_TxRx_Tlm f;
    f.data         = m_tsr;
    f.startOrStop  = (mode != USART_MODE_0);
    f.stopBitCount = usart_stop_bits(mode, m_con.fields.STP);
    f.parity       = par;
    txd.write(f);
    if (m_con.fields.LB) rxd.write(f);

    DBG(3, "loadTSR: data=0x" << std::hex << m_tsr
           << " par=" << par << std::dec);

    // TBIR: TBUF slot is now free — fire immediately.
    assertIrq(m_tbir_assert_time, tbir, "tbir");

    // Schedule TIR at end of frame.  SC_ZERO_TIME fallback if clock not yet
    // configured: txDoneMethod fires next delta and TIR is raised then.
    sc_core::sc_time fd = get_frame_duration();
    m_tx_done_ev.notify(fd);
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
        packed = ((m_rbuf.raw & 0x1FFu) << 1u); // placeholder; parity from caller
    } else {
        packed = m_rbuf.raw & RBUF_DATA_MASK;
    }

    m_rbuf.fields.DATA = packed & RBUF_DATA_MASK;
    m_rbuf_full        = true;

    if (oe) m_con.fields.OE = 1u;
    if (fe) m_con.fields.FE = 1u;
    if (pe) m_con.fields.PE = 1u;

    assertIrq(m_rir_assert_time, rir, "rir");

    bool fire_eir = (oe && m_con.fields.OEN) ||
                    (fe && m_con.fields.FEN) ||
                    (pe && m_con.fields.PEN);
    if (fire_eir) assertIrq(m_eir_assert_time, eir, "eir");
}

// =============================================================================
// Direct RX injection
// =============================================================================

bool Usart::rx_inject(uint8_t byte)
{
    if (!m_con.fields.R || !m_con.fields.REN) {
        DBG(2, "rx_inject: dropped — module not running or RX disabled");
        return false;
    }
    if (m_rbuf_full) {
        m_con.fields.OE = 1u;
        if (m_con.fields.OEN)
            assertIrq(m_eir_assert_time, eir, "eir");
        DBG(2, "rx_inject: OVERRUN — byte=0x"
               << std::hex << static_cast<unsigned>(byte) << std::dec);
        return false;
    }
    uint8_t  mode = m_con.fields.M;
    uint32_t packed;
    if (usart_mode_has_hw_parity(mode)) {
        uint8_t nbits    = usart_data_bits(mode);
        bool    even_par = computeEvenParity(byte, nbits);
        bool    par      = m_con.fields.ODD ? !even_par : even_par;
        packed = ((static_cast<uint32_t>(byte) << 1u) & RBUF_DATA_MASK) |
                 (par ? 1u : 0u);
    } else {
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

void Usart::assertIrq(sc_time       &assert_time,
                      sc_out<bool>  &port,
                      const char    *irq_name)
{
    DBG(4, "assertIrq: " << irq_name);
    sc_core::sc_time t = sc_core::sc_time_stamp();
    if (t == sc_core::SC_ZERO_TIME)
        t = sc_core::sc_time(1, sc_core::SC_PS);
    assert_time = t;
    port.write(true);

    // Schedule de-assertion in the NEXT delta cycle (SC_ZERO_TIME) rather than
    // after 2 × m_clk_period.  Advancing SC simulated time by 20 ns requires the
    // QEMU thread to have already pushed its local time past that point via the
    // quantum keeper — this cannot happen while the QEMU thread is blocked in
    // sleep_for(), causing irqDeassertMethod to fire OUTSIDE the sleep window and
    // accumulate as a pending SC-time event for the next byte's sleep.  Using
    // SC_ZERO_TIME keeps the deassert within the same delta epoch and eliminates
    // the cross-quantum scheduling backlog that caused ~70 % of test4 runs to hang.
    m_irq_deassert_ev.notify(sc_core::SC_ZERO_TIME);
}

// ─────────────────────────────────────────────────────────────────────────────
// updateIrqPulses — de-assert any IRQ output whose 2-cycle pulse has expired
// ─────────────────────────────────────────────────────────────────────────────
void Usart::updateIrqPulses(sc_time now)
{
    if (m_clk_period == SC_ZERO_TIME) return;
    sc_time pulse_width = m_clk_period * 2.0;

    auto check = [&](sc_time &assert_time, sc_out<bool> &port, const char *n) {
        if (assert_time == SC_ZERO_TIME) return;
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
// Baud-rate helpers (read-only; called from Usart2 for frame timing)
// =============================================================================

sc_core::sc_time Usart::get_baud_period() const
{
    if (m_clk_period == sc_core::SC_ZERO_TIME || !m_con.fields.R)
        return sc_core::SC_ZERO_TIME;

    unsigned prescaler;
    unsigned oversample;
    if (m_con.fields.M == USART_MODE_0) {
        prescaler  = 1u;
        oversample = 4u;
    } else {
        prescaler  = (m_con.fields.FDE || !m_con.fields.BRS) ? 2u : 3u;
        oversample = 16u;
    }
    double factor = static_cast<double>(prescaler)
                  * static_cast<double>(m_bg_reload.raw + 1u)
                  * static_cast<double>(oversample);
    return m_clk_period * factor;
}

sc_core::sc_time Usart::get_frame_duration() const
{
    sc_core::sc_time bp = get_baud_period();
    if (bp == sc_core::SC_ZERO_TIME) return sc_core::SC_ZERO_TIME;

    uint8_t mode = m_con.fields.M;
    int bits;
    if (mode == USART_MODE_0) {
        bits = static_cast<int>(usart_data_bits(mode));
    } else {
        int data = static_cast<int>(usart_data_bits(mode));
        int par  = usart_mode_has_hw_parity(mode) ? 1 : 0;
        int stop = static_cast<int>(usart_stop_bits(mode, m_con.fields.STP));
        bits = 1 + data + par + stop;
    }
    return bp * static_cast<double>(bits);
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
