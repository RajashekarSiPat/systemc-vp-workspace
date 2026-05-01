/*
 * serial_bridge.h — QBOX peripheral that bridges two biflow sockets.
 *
 * Connects two USART2 (or any biflow_socket) instances for full-duplex
 * communication without baud-rate serial framing:
 *
 *   Data received on socket_a (= TX from A) → forwarded to socket_b (= RX of B)
 *   Data received on socket_b (= TX from B) → forwarded to socket_a (= RX of A)
 *
 * Platform Lua wiring example:
 *   usart2_a.backend_socket  → serial_bridge_0.socket_a
 *   usart2_b.backend_socket  → serial_bridge_0.socket_b
 *
 * The bridge transparently relays any number of bytes per transaction and
 * restores flow-control credits on both sides after each forwarding.
 */

#pragma once

#include <systemc>
#include "tlm.h"
#include <ports/biflow-socket.h>
#include <async_event.h>
#include <module_factory_registery.h>
#include <scp/report.h>

class SerialBridge : public sc_core::sc_module
{
    SCP_LOGGER();

public:
    gs::biflow_socket<SerialBridge> socket_a; ///< Connected to USART instance A
    gs::biflow_socket<SerialBridge> socket_b; ///< Connected to USART instance B

    SC_HAS_PROCESS(SerialBridge);

    explicit SerialBridge(sc_core::sc_module_name name);
    void start_of_simulation() override;

private:
    static constexpr int FIFO_DEPTH = 64;

    /* Called when USART-A sends a byte (its TX) → forward to USART-B (its RX) */
    void recv_from_a(tlm::tlm_generic_payload& txn, sc_core::sc_time& t);

    /* Called when USART-B sends a byte (its TX) → forward to USART-A (its RX) */
    void recv_from_b(tlm::tlm_generic_payload& txn, sc_core::sc_time& t);

    /* biflow_socket's m_send_event is async_detach_suspending()-ed when
     * can_receive_any() (INFINITE_VALUE) is used, so enqueue() posts an
     * async_request_update() that does NOT wake a sleeping SC scheduler.
     * m_wakeup_ev (start_attached=true by default) is notified alongside
     * every enqueue() call to guarantee the SC scheduler wakes and processes
     * the biflow sendall SC_METHOD in the same delta cycle.               */
    gs::async_event m_wakeup_ev;
    void wakeup_method();
};

extern "C" void module_register();
