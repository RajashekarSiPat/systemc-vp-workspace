// =============================================================================
// Usart_regdefs.h
// -----------------------------------------------------------------------------
// Register definitions for the QBox USART peripheral.
//
// All registers use the struct-inside-union idiom so that both named-field
// access and raw 32-bit word access are available without any casting.
//
// Register map (byte offsets from base address):
//   0x00  CON   – Control register                (R/W)
//   0x04  TBUF  – Transmit holding buffer         (W only)
//   0x08  RBUF  – Receive  holding buffer         (R only)
//   0x0C  BG    – Baud generator (dual-purpose)   (W=reload, R=live counter)
//   0x10  FDR   – Fractional divider              (R/W)
//
// Spec reference: §6 Register Description (pages 9-12 of specification PDF)
// =============================================================================
#ifndef USART_REGDEFS_H
#define USART_REGDEFS_H

#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// Register byte offsets
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint32_t USART_CON_OFFSET  = 0x00u;
static constexpr uint32_t USART_TBUF_OFFSET = 0x04u;
static constexpr uint32_t USART_RBUF_OFFSET = 0x08u;
static constexpr uint32_t USART_BG_OFFSET   = 0x0Cu;
static constexpr uint32_t USART_FDR_OFFSET  = 0x10u;

/// Highest valid byte offset (inclusive)
static constexpr uint32_t USART_REG_MAX_OFFSET = USART_FDR_OFFSET;

// ─────────────────────────────────────────────────────────────────────────────
// Mode encoding constants  (CON.M[2:0])
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t USART_MODE_0 = 0u; ///< Synchronous, half-duplex master
static constexpr uint8_t USART_MODE_1 = 1u; ///< Async 8N1   (8 data, no parity, 1/2 stop)
static constexpr uint8_t USART_MODE_3 = 3u; ///< Async 9-bit (9 data, no parity, 1/2 stop)
static constexpr uint8_t USART_MODE_4 = 4u; ///< Async 7+P   (7 data, even/odd parity)
static constexpr uint8_t USART_MODE_5 = 5u; ///< Async 8+P   (8 data, even/odd parity)
static constexpr uint8_t USART_MODE_7 = 7u; ///< Async 8N2   (8 data, no parity, 1/2 stop)

// ─────────────────────────────────────────────────────────────────────────────
// CON — Control Register  (offset 0x00, R/W, reset 0x00000000)
// ─────────────────────────────────────────────────────────────────────────────
// Exact bit layout from specification §6.1 (Figure 5):
//
//  Bit(s)  Field  Access  Description
//  [2:0]   M      R/W     Operating mode: 000=Sync, 001=8N1, 011=9-bit,
//                          100=7+Parity, 101=8+Parity, 111=8N2.
//                          Values 010 and 110 are reserved.
//  [3]     REN    R/W     Receiver enable.
//                          Async: 1 = activate RX logic + RXD sampling.
//                          Sync:  0 = RXD is output (TX turn);
//                                 1 = RXD is input  (RX turn).
//  [4]     ODD    R/W     Parity sense.  0=even, 1=odd.  Modes 4/5 only.
//  [5]     STP    R/W     Stop bit count.  0=1 stop bit, 1=2 stop bits.
//                          Async only; ignored in sync mode.
//  [6]     OEN    R/W     Overrun error interrupt enable.  1=OE → EIR fires.
//  [7]     FEN    R/W     Framing error interrupt enable.  1=FE → EIR fires.
//                          Async only (FE never set in Mode 0).
//  [8]     PEN    R/W     Parity error interrupt enable.   1=PE → EIR fires.
//                          Modes 4/5 only.
//  [9]     OE     RO      Overrun error flag (hardware-set).
//                          Set when RSR completes while RBUF still full.
//                          Clear: read RBUF, or write 0 to this bit.
//  [10]    FE     RO      Framing error flag (hardware-set, async only).
//                          Set when any expected stop bit is sampled as 0.
//                          Clears automatically on next valid start bit.
//  [11]    PE     RO      Parity error flag (hardware-set, modes 4/5 only).
//                          Set when received parity ≠ computed parity.
//                          Clears on RBUF read.
//  [12]    BRS    R/W     Baud prescaler.  0=f_PERIPH÷2, 1=f_PERIPH÷3.
//                          Active when FDE=0 and mode ≠ 0.
//  [13]    FDE    R/W     Fractional divider enable.  1=FDR register provides
//                          f_PRE, overriding BRS.  Must be 0 in sync mode.
//  [14]    LB     R/W     Loopback.  1=TSR output connected to RSR input
//                          internally; RXD pin input bypassed; TXD still active.
//  [15]    R      R/W     Run bit.  1=clock enabled, module operational.
//                          0=clock gated, state frozen, TXD idle (high).
//                          Set R last after all other CON configuration.
//  [31:16] —      —       Reserved.  Write 0; reads as 0.
// ─────────────────────────────────────────────────────────────────────────────
union CON_reg
{
    uint32_t raw;

    struct
    {
        uint32_t M    : 3;   ///< [2:0]  Mode select
        uint32_t REN  : 1;   ///< [3]    Receiver enable
        uint32_t ODD  : 1;   ///< [4]    Parity sense (0=even, 1=odd)
        uint32_t STP  : 1;   ///< [5]    Stop bit count (0=1, 1=2)
        uint32_t OEN  : 1;   ///< [6]    Overrun error IRQ enable
        uint32_t FEN  : 1;   ///< [7]    Framing error IRQ enable
        uint32_t PEN  : 1;   ///< [8]    Parity error IRQ enable
        uint32_t OE   : 1;   ///< [9]    Overrun error flag  (RO, hw-set)
        uint32_t FE   : 1;   ///< [10]   Framing error flag  (RO, hw-set)
        uint32_t PE   : 1;   ///< [11]   Parity error flag   (RO, hw-set)
        uint32_t BRS  : 1;   ///< [12]   Baud prescaler (0=÷2, 1=÷3)
        uint32_t FDE  : 1;   ///< [13]   Fractional divider enable
        uint32_t LB   : 1;   ///< [14]   Loopback enable
        uint32_t R    : 1;   ///< [15]   Run bit (module clock gate)
        uint32_t rsvd : 16;  ///< [31:16] Reserved
    } fields;

    CON_reg() : raw(0u) {}
};

// CON writable-field mask (all R/W bits; excludes RO error flags [11:9])
// Software may write bits [8:0] + [15:12] + [14].  Bits [11:9] are RO.
// Write-1-to-clear: OE (bit 9) — handled explicitly in regWrite().
static constexpr uint32_t CON_RW_MASK   = 0x0000F9FFu; ///< R/W writable bits
static constexpr uint32_t CON_RO_MASK   = 0x00000E00u; ///< RO bits [11:9]
static constexpr uint32_t CON_OE_BIT    = (1u << 9);
static constexpr uint32_t CON_FE_BIT    = (1u << 10);
static constexpr uint32_t CON_PE_BIT    = (1u << 11);
static constexpr uint32_t CON_ERR_MASK  = CON_OE_BIT | CON_FE_BIT | CON_PE_BIT;

// ─────────────────────────────────────────────────────────────────────────────
// TBUF — Transmit Holding Buffer  (offset 0x04, Write-only, reset undefined)
// ─────────────────────────────────────────────────────────────────────────────
// Writing TBUF:
//   • If TBUF is full (previous data not yet picked up by TSR) → write ignored.
//   • If TSR is idle → data transfers immediately to TSR and shifting begins.
//   • If TSR is busy → data held in TBUF until current frame completes.
//
// TBIR fires when the data is picked up by TSR (slot now free).
// Data is shifted out LSB first.
//
//  Bits   Description
//  [7:0]  D7..D0  — transmit data byte.  D0 transmitted first (LSB first).
//  [8]    TX9     — 9th transmit bit.  Valid in Mode 3 (CON.Mode=011) only.
//                   Used as the address/data flag in 9-bit protocols.
//  [31:9] Ignored on write.
// ─────────────────────────────────────────────────────────────────────────────
union TBUF_reg
{
    uint32_t raw;

    struct
    {
        uint32_t DATA : 9;   ///< [8:0] Transmit data (D7:D0 + TX9 for mode 3)
        uint32_t rsvd : 23;  ///< [31:9] Ignored on write
    } fields;

    TBUF_reg() : raw(0u) {}
};

static constexpr uint32_t TBUF_DATA_MASK = 0x000001FFu;  ///< 9-bit data mask

// ─────────────────────────────────────────────────────────────────────────────
// RBUF — Receive Holding Buffer  (offset 0x08, Read-only, reset undefined)
// ─────────────────────────────────────────────────────────────────────────────
// Read after RIR fires.  Reading RBUF clears CON.OE (if set).
//
// IMPORTANT — parity-mode packing (modes 4/5):
//   Received data is left-shifted (×2) before loading RBUF, and the raw
//   received parity bit is inserted at RBUF[0].  This allows direct software
//   parity inspection at RBUF[0] without masking.
//
//   Mode 1  — 8N1:    RBUF[8]=D7  RBUF[7:1]=D6..D0  RBUF[0]=0
//   Mode 3  — 9-bit:  RBUF[8]=D8  RBUF[7:1]=D7..D1  RBUF[0]=D0
//   Mode 4  — 7+P:    RBUF[8]=0   RBUF[7:2]=D6..D1  RBUF[1]=D0  RBUF[0]=P
//   Mode 5  — 8+P:    RBUF[8]=D7  RBUF[7:2]=D6..D1  RBUF[1]=D0  RBUF[0]=P
//   Mode 7  — 8N2:    RBUF[8]=D7  RBUF[7:1]=D6..D0  RBUF[0]=0
//
//  Bits    Description
//  [8:0]   Received data — layout is mode-dependent (see table above).
//  [31:9]  Reads as 0.
//
// Note: OE, FE, PE flags live in CON [11:9], NOT in RBUF.
// ─────────────────────────────────────────────────────────────────────────────
union RBUF_reg
{
    uint32_t raw;

    struct
    {
        uint32_t DATA : 9;   ///< [8:0] Received data (mode-dependent layout)
        uint32_t rsvd : 23;  ///< [31:9] Reads as 0
    } fields;

    RBUF_reg() : raw(0u) {}
};

static constexpr uint32_t RBUF_DATA_MASK = 0x000001FFu;  ///< 9-bit data mask

// ─────────────────────────────────────────────────────────────────────────────
// BG — Baud Generator Register  (offset 0x0C, dual-purpose, reset 0x00000000)
// ─────────────────────────────────────────────────────────────────────────────
// Physically dual-purpose: write targets the 13-bit RELOAD register; read
// returns the live 13-bit DOWNCOUNTER.  The two are independent — a write
// never disturbs the running count (glitch-free baud rate change takes effect
// at the next underflow boundary).
//
// On underflow: baud pulse emitted on that cycle; counter reloads from the
// reload register on the very next f_PRE clock edge (one-cycle latency).
//
// Async baud rate formula:
//   f_BAUD = f_PRE ÷ (16 × (BG_reload + 1))
//   where f_PRE is selected by FDE/BRS (see CON and FDR registers).
//
// Sync clock frequency:
//   f_SCLK = f_PERIPH ÷ (4 × (BG_reload + 1))
//   (BRS and FDE bypassed in sync mode)
//
//  Operation            Bits     Description
//  Write (→ reload reg) [12:0]   13-bit reload value.  Write 0 → maximum rate.
//  Write (→ reload reg) [31:13]  Ignored.
//  Read  (← downcounter)[12:0]   Live 13-bit downcounter state.
//  Read  (← downcounter)[31:13]  Reads as 0.
// ─────────────────────────────────────────────────────────────────────────────
union BG_reg
{
    uint32_t raw;

    struct
    {
        uint32_t VAL  : 13;  ///< [12:0]  Reload (write) / live counter (read)
        uint32_t rsvd : 19;  ///< [31:13] Ignored / reads as 0
    } fields;

    BG_reg() : raw(0u) {}
};

static constexpr uint32_t BG_VAL_MASK     = 0x00001FFFu;  ///< 13-bit mask
static constexpr uint32_t BG_MAX_RELOAD   = 0x00001FFFu;  ///< Maximum reload value

// ─────────────────────────────────────────────────────────────────────────────
// FDR — Fractional Divider Register  (offset 0x10, R/W, reset 0x00000000)
// ─────────────────────────────────────────────────────────────────────────────
// Active when CON.FDE = 1 (async modes only; must be 0 in sync mode).
//
// Implements a sigma-delta accumulator:
//   • STEP is added to an internal accumulator on every f_PERIPH cycle.
//   • A carry-out (baud pulse = f_PRE tick) is produced each time the
//     accumulator overflows its modulus.
//   • DM selects the modulus: 0 → 512 (9-bit), 1 → 256 (8-bit).
//
// Effective pre-divider output:
//   f_PRE = f_PERIPH × STEP ÷ denominator
//   where denominator = 512 (DM=0)  or  256 (DM=1)
//
// STEP = 0 is invalid when FDE=1 (produces no clock output).
//
//  Bits    Field  Access  Description
//  [7:0]   STEP   R/W     Fractional step (numerator).  Added to accumulator
//                          each f_PERIPH cycle.  Must not be 0 when FDE=1.
//  [8]     DM     R/W     Denominator mode.
//                            0 = 512 (9-bit accumulator, higher resolution).
//                            1 = 256 (8-bit accumulator).
//  [31:9]  —      —       Reserved.  Write 0; reads as 0.
// ─────────────────────────────────────────────────────────────────────────────
union FDR_reg
{
    uint32_t raw;

    struct
    {
        uint32_t STEP : 8;   ///< [7:0]  Fractional step value (0..255)
        uint32_t DM   : 1;   ///< [8]    Denominator mode (0=512, 1=256)
        uint32_t rsvd : 23;  ///< [31:9] Reserved
    } fields;

    FDR_reg() : raw(0u) {}
};

static constexpr uint32_t FDR_WR_MASK = 0x000001FFu; ///< STEP[7:0] + DM[8]

// ─────────────────────────────────────────────────────────────────────────────
// Helper inline functions
// ─────────────────────────────────────────────────────────────────────────────

/// Returns true if the mode value is one of the six supported modes.
inline bool usart_mode_valid(uint8_t m)
{
    return (m == USART_MODE_0 || m == USART_MODE_1 || m == USART_MODE_3 ||
            m == USART_MODE_4 || m == USART_MODE_5 || m == USART_MODE_7);
}

/// Returns true if the mode is asynchronous (all modes except Mode 0).
inline bool usart_mode_is_async(uint8_t m)
{
    return (m != USART_MODE_0);
}

/// Returns true if the mode includes a hardware-generated parity bit.
/// Parity is only in modes 4 (7+P) and 5 (8+P); mode 3 carries bit8 as
/// an application-level address/data flag, NOT a hardware parity bit.
inline bool usart_mode_has_hw_parity(uint8_t m)
{
    return (m == USART_MODE_4 || m == USART_MODE_5);
}

/// Returns the number of actual data bits for a given mode.
///   Mode 0 : 8  (synchronous, no start/stop)
///   Mode 1 : 8  (8N1)
///   Mode 3 : 9  (9-bit; bit8 = TX9/RX9, application address/data flag)
///   Mode 4 : 7  (7+Parity — only 7 data bits; parity is the 8th TX bit)
///   Mode 5 : 8  (8+Parity — 8 data bits; parity is the 9th TX bit)
///   Mode 7 : 8  (8N2)
inline uint8_t usart_data_bits(uint8_t m)
{
    if (m == USART_MODE_3) return 9u;
    if (m == USART_MODE_4) return 7u;
    return 8u;
}

/// Returns the number of stop bits for the given CON.STP value.
/// Always 0 for synchronous mode (Mode 0).
inline uint8_t usart_stop_bits(uint8_t mode, uint8_t stp)
{
    if (mode == USART_MODE_0) return 0u;
    return stp ? 2u : 1u;
}

#endif // USART_REGDEFS_H
