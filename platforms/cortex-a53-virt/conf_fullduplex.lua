-- conf_fullduplex.lua — Cortex-A53 virt platform: two USART2 instances
-- bridged for full-duplex communication with interrupt testing.
--
-- Hardware map:
--   0x80000000 .. 0x8FFFFFFF  256 MB RAM
--   0x09002000 .. 0x09002FFF  USART2 A  (bridge side A — test TX/RX)
--   0x09003000 .. 0x09003FFF  USART2 B  (bridge side B — test TX/RX)
--   0x09004000 .. 0x09004FFF  USART2 C  (console — firmware print output)
--   0x09010000 .. 0x09010FFF  exiter    (firmware stop)
--
-- Bridge topology:
--   usart2_a.backend_socket  ←→  serial_bridge_0.socket_a
--   usart2_b.backend_socket  ←→  serial_bridge_0.socket_b
--   TX of A → bridge → RX of B  (A sends, B receives)
--   TX of B → bridge → RX of A  (B sends, A receives)
--
-- Console output:
--   usart2_c.backend_socket  ←→  charbackend_stdio_0
--   Firmware writes chars to UC_TBUF (0x09004004) → backend_socket.enqueue
--   → charbackend_stdio_0 → stdout.  No PL011 needed.
--
-- STATUS register (per USART2 instance, at base + 0x20):
--   Bit 0: TBIR (TX buffer freed)     Bit 2: RIR (RX data ready)
--   Bit 1: TIR  (TX frame complete)   Bit 3: EIR (overrun/error)
--   Write-1-to-clear.
--
-- Run:
--   LD_LIBRARY_PATH=qbox/build/install/lib:...  \
--   ./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
--       --gs_luafile platforms/cortex-a53-virt/conf_fullduplex.lua

function script_path()
    local src = debug.getinfo(2, "S").source:sub(2)
    return src:match("(.*/)")
end
local base = script_path()

platform = {
    moduletype = "Container",
    -- Tests 1-4 (TBIR/RIR/EIR) use synchronous STATUS updates inside
    -- b_transport/rx_receive so do not require a small quantum.
    -- Test 5 (TIR) polls STATUS which calls advance() — it fires once
    -- sufficient simulation time has elapsed across quantum boundaries.
    quantum_ns = 10000000,

    router = { moduletype = "router", log_level = 0 },

    ram_0 = {
        moduletype    = "gs_memory",
        target_socket = {
            address = 0x80000000,
            size    = 0x10000000,
            bind    = "&router.initiator_socket",
        },
    },

    qemu_inst_mgr = { moduletype = "QemuInstanceManager" },

    qemu_inst = {
        moduletype  = "QemuInstance",
        args        = { "&qemu_inst_mgr", "AARCH64" },
        accel       = "tcg",
        sync_policy = "multithread-unconstrained",
    },

    cpu_0 = {
        moduletype   = "cpu_arm_cortexA53",
        args         = { "&qemu_inst" },
        mem          = { bind = "&router.target_socket" },
        rvbar        = 0x80000000,
        has_el3      = true,
        has_el2      = true,
        psci_conduit = "hvc",
    },

    -- -----------------------------------------------------------------------
    -- charbackend_stdio_0: console output (for usart2_c below)
    -- -----------------------------------------------------------------------
    charbackend_stdio_0 = {
        moduletype = "char_backend_stdio",
        read_write = true,
    },

    -- -----------------------------------------------------------------------
    -- USART2 C at 0x09004000  — console output only
    -- TBUF writes go: backend_socket.enqueue → charbackend_stdio_0 → stdout
    -- This is the same mechanism as hello_usart2.c used for its output.
    -- -----------------------------------------------------------------------
    usart2_c = {
        moduletype    = "Usart2",
        dylib_path    = "usart2",
        target_socket = {
            address = 0x09004000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
        backend_socket = { bind = "&charbackend_stdio_0.biflow_socket" },
    },

    -- -----------------------------------------------------------------------
    -- USART2 A at 0x09002000 — bridge side A (test sender/receiver)
    -- -----------------------------------------------------------------------
    usart2_a = {
        moduletype    = "Usart2",
        dylib_path    = "usart2",
        target_socket = {
            address = 0x09002000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
        backend_socket = { bind = "&serial_bridge_0.socket_a" },
    },

    -- -----------------------------------------------------------------------
    -- USART2 B at 0x09003000 — bridge side B (test sender/receiver)
    -- -----------------------------------------------------------------------
    usart2_b = {
        moduletype    = "Usart2",
        dylib_path    = "usart2",
        target_socket = {
            address = 0x09003000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
        backend_socket = { bind = "&serial_bridge_0.socket_b" },
    },

    -- -----------------------------------------------------------------------
    -- Serial bridge: wires A TX→B RX and B TX→A RX
    -- -----------------------------------------------------------------------
    serial_bridge_0 = {
        moduletype = "SerialBridge",
        dylib_path = "serial_bridge",
    },

    -- -----------------------------------------------------------------------
    -- Exiter: firmware writes 0 here to call sc_stop() cleanly.
    -- -----------------------------------------------------------------------
    exiter_0 = {
        moduletype    = "exiter",
        target_socket = {
            address = 0x09010000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
    },

    -- -----------------------------------------------------------------------
    -- ELF loader
    -- -----------------------------------------------------------------------
    load = {
        moduletype       = "loader",
        initiator_socket = { bind = "&router.target_socket" },
        { elf_file = base .. "hello_fullduplex.elf" },
    },
}
