// =============================================================================
// serial_bridge.cc — Full-duplex biflow socket bridge implementation.
// =============================================================================

#include "serial_bridge.h"

using namespace sc_core;
using namespace tlm;

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
SerialBridge::SerialBridge(sc_module_name name)
    : sc_module(name)
    , socket_a("socket_a")
    , socket_b("socket_b")
{
    SCP_TRACE(()) << "SerialBridge constructor";

    socket_a.register_b_transport(this, &SerialBridge::recv_from_a);
    socket_b.register_b_transport(this, &SerialBridge::recv_from_b);
    /* NOTE: do NOT call can_receive_set() here — sockets not yet bound.
     * Call in start_of_simulation() after all connections are resolved. */

    SC_METHOD(wakeup_method);
    sensitive << m_wakeup_ev;
    dont_initialize();
}

// -----------------------------------------------------------------------------
// start_of_simulation — open receive windows on both sides
// -----------------------------------------------------------------------------
void SerialBridge::start_of_simulation()
{
    /* Use can_receive_any() (INFINITE_VALUE) rather than can_receive_set(N).
     *
     * can_receive_set(N) sends an ABSOLUTE_VALUE credit message to the peer,
     * which calls async_attach_suspending() on the peer's m_send_event.  When
     * QEMU's b_transport thread later calls enqueue() on the same socket, the
     * resulting m_send_event.notify() tries to acquire the SystemC kernel lock
     * while the SystemC thread holds it — an ABBA deadlock.
     *
     * can_receive_any() sends INFINITE_VALUE, which sets m_infinite=true on
     * the peer and does NOT call async_attach_suspending(), so QEMU can safely
     * call enqueue() / notify() without holding any conflicting lock. */
    socket_a.can_receive_any();
    socket_b.can_receive_any();
    SCP_TRACE(()) << "SerialBridge: receive windows opened (infinite, both sides)";
}

// -----------------------------------------------------------------------------
// recv_from_a — bytes arrived from USART-A (its TX output)
//               Forward each byte to socket_b so USART-B receives them as RX.
// -----------------------------------------------------------------------------
void SerialBridge::recv_from_a(tlm_generic_payload& txn, sc_time& /*t*/)
{
    uint8_t* ptr = txn.get_data_ptr();
    unsigned len = txn.get_data_length();

    SCP_INFO(()) << "bridge A→B: " << len << " byte(s) [0x" << std::hex << (unsigned)ptr[0] << "]";
    for (unsigned i = 0u; i < len; ++i) {
        socket_b.enqueue(ptr[i]);
        m_wakeup_ev.notify(sc_core::SC_ZERO_TIME);
    }
}

// -----------------------------------------------------------------------------
// recv_from_b — bytes arrived from USART-B (its TX output)
//               Forward each byte to socket_a so USART-A receives them as RX.
// -----------------------------------------------------------------------------
void SerialBridge::recv_from_b(tlm_generic_payload& txn, sc_time& /*t*/)
{
    uint8_t* ptr = txn.get_data_ptr();
    unsigned len = txn.get_data_length();

    SCP_TRACE(()) << "bridge B→A: " << len << " byte(s)";
    for (unsigned i = 0u; i < len; ++i) {
        socket_a.enqueue(ptr[i]);
        m_wakeup_ev.notify(sc_core::SC_ZERO_TIME);
    }
}

void SerialBridge::wakeup_method() { /* no-op: purpose is solely to wake SC scheduler */ }

// -----------------------------------------------------------------------------
// Module registration — required by QBOX ModuleFactory / gs_create_dymod
// -----------------------------------------------------------------------------
void module_register() { GSC_MODULE_REGISTER_C(SerialBridge); }
