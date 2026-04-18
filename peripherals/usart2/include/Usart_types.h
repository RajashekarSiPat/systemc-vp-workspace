// =============================================================================
// Usart_types.h
// -----------------------------------------------------------------------------
// TLM payload type for the USART pin-level interface (txd / rxd ports).
//
// USART_TxRx_Tlm models one complete serial frame as it appears on the
// physical TXD / RXD pins.  It is used with sc_out<> and sc_inout<> and
// must therefore satisfy all SystemC signal-value requirements:
//   • default-constructible
//   • copy-assignable
//   • equality-comparable  (operator== / operator!=)
//   • stream-insertable    (operator<<)
//   • sc_trace-able        (free sc_trace overload)
//
// Spec reference: §2 Features, §3 Port list, §4.2–4.3
// =============================================================================
#ifndef USART_TYPES_H
#define USART_TYPES_H

#include <systemc>
#include <iostream>

// -----------------------------------------------------------------------------
// USART_TxRx_Tlm
// -----------------------------------------------------------------------------
//  data         – Up to 9 payload bits (bits [8:0]).
//                 Async TX: data field of the current frame (LSB-first order).
//                 Sync:     raw serial word shifted out / in.
//
//  startOrStop  – Framing qualifier (TX side):
//                   true  + data==0  → START bit being driven
//                   true  + data==1  → STOP  bit being driven
//                   false            → data bit or idle
//
//  stopBitCount – Number of STOP bits included in this frame.
//                   0 = synchronous / no stop bits
//                   1 = CON.STP = 0  (1 stop bit)
//                   2 = CON.STP = 1  (2 stop bits)
//                 Informational for bus monitors / scoreboard.
//
//  parity       – Parity bit value appended to the frame.
//                   true / false depending on computed parity.
//                   Meaningful only in modes 4 and 5.  Always false otherwise.
// -----------------------------------------------------------------------------
typedef struct USART_TxRx_Tlm_s
{
    unsigned int   data;           ///< Payload bits [8:0]; idle / mark = 1
    bool           startOrStop;    ///< true = START or STOP framing element
    unsigned char  stopBitCount;   ///< Number of STOP bits (0, 1, or 2)
    bool           parity;         ///< Parity bit value (modes 4/5 only)

    // -------------------------------------------------------------------------
    // Default constructor — idle / mark state (line high)
    // -------------------------------------------------------------------------
    USART_TxRx_Tlm_s()
        : data(1u)              // RXD/TXD idle = logic-high (mark)
        , startOrStop(false)
        , stopBitCount(1u)
        , parity(false)
    {}

    // -------------------------------------------------------------------------
    // Full constructor
    // -------------------------------------------------------------------------
    USART_TxRx_Tlm_s(unsigned int   d,
                      bool           ss,
                      unsigned char  sbc,
                      bool           p)
        : data(d), startOrStop(ss), stopBitCount(sbc), parity(p)
    {}

} USART_TxRx_Tlm;

// =============================================================================
// Equality operators
// Required by sc_signal<T> for change-detection on every write.
// =============================================================================
inline bool operator==(const USART_TxRx_Tlm &a, const USART_TxRx_Tlm &b)
{
    return (a.data         == b.data)         &&
           (a.startOrStop  == b.startOrStop)   &&
           (a.stopBitCount == b.stopBitCount)  &&
           (a.parity       == b.parity);
}

inline bool operator!=(const USART_TxRx_Tlm &a, const USART_TxRx_Tlm &b)
{
    return !(a == b);
}

// =============================================================================
// Stream insertion operator
// Required for sc_trace and general debug output.
// =============================================================================
inline std::ostream &operator<<(std::ostream &os, const USART_TxRx_Tlm &t)
{
    os << "USART_TxRx{"
       << "data=0x"     << std::hex << t.data << std::dec
       << " s/S="       << (t.startOrStop  ? "1" : "0")
       << " stopBits="  << static_cast<unsigned>(t.stopBitCount)
       << " parity="    << (t.parity ? "1" : "0")
       << "}";
    return os;
}

// =============================================================================
// sc_trace overload
// Enables VCD / WIF waveform tracing.  Each member is traced as an independent
// signal with a dotted sub-name so simulators can display them individually.
// =============================================================================
inline void sc_trace(sc_core::sc_trace_file  *tf,
                     const USART_TxRx_Tlm    &t,
                     const std::string        &name)
{
    sc_core::sc_trace(tf, t.data,         name + ".data");
    sc_core::sc_trace(tf, t.startOrStop,  name + ".startOrStop");
    sc_core::sc_trace(tf, t.stopBitCount, name + ".stopBitCount");
    sc_core::sc_trace(tf, t.parity,       name + ".parity");
}

#endif // USART_TYPES_H
