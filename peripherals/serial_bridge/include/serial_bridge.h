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
};

extern "C" void module_register();
