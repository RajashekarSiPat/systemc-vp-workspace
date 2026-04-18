-- conf_usart2.lua — Cortex-A53 virt platform exercising the full USART2 model.
--
-- Hardware map:
--   0x80000000 .. 0x8FFFFFFF  256 MB RAM
--   0x09000000 .. 0x09000FFF  PL011 UART (kept for compatibility)
--   0x09001000 .. 0x09001FFF  Simple USART (peripherals/usart)
--   0x09002000 .. 0x09002FFF  Full USART2  (peripherals/usart2) ← under test
--
-- Run:
--   ./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
--       --gs_luafile platforms/cortex-a53-virt/conf_usart2.lua

function script_path()
    local src = debug.getinfo(2, "S").source:sub(2)
    return src:match("(.*/)")
end
local base = script_path()

platform = {
    moduletype = "Container",
    quantum_ns = 10000000,

    -- -----------------------------------------------------------------------
    -- TLM address router
    -- -----------------------------------------------------------------------
    router = { moduletype = "router", log_level = 0 },

    -- -----------------------------------------------------------------------
    -- RAM: 256 MB at 0x80000000
    -- -----------------------------------------------------------------------
    ram_0 = {
        moduletype    = "gs_memory",
        target_socket = {
            address = 0x80000000,
            size    = 0x10000000,
            bind    = "&router.initiator_socket",
        },
    },

    -- -----------------------------------------------------------------------
    -- QEMU instance (Cortex-A53, TCG)
    -- -----------------------------------------------------------------------
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
    -- Character backends — one per UART (biflow_socket is point-to-point).
    -- -----------------------------------------------------------------------
    charbackend_stdio_0 = {
        moduletype = "char_backend_stdio",
        read_write = true,
    },

    charbackend_stdio_1 = {
        moduletype = "char_backend_stdio",
        read_write = true,
    },

    charbackend_stdio_2 = {
        moduletype = "char_backend_stdio",
        read_write = true,
    },

    -- -----------------------------------------------------------------------
    -- Full USART2 peripheral at 0x09002000  ← under test
    -- dylib_path = "usart2" matches the .so built by peripherals/usart2/.
    -- -----------------------------------------------------------------------
    usart2_0 = {
        moduletype    = "Usart2",
        dylib_path    = "usart2",
        target_socket = {
            address = 0x09002000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
        backend_socket = { bind = "&charbackend_stdio_0.biflow_socket" },
    },

    -- -----------------------------------------------------------------------
    -- Simple USART peripheral at 0x09001000 (kept, not exercised by firmware)
    -- -----------------------------------------------------------------------
    usart_0 = {
        moduletype    = "Usart",
        dylib_path    = "usart",
        target_socket = {
            address = 0x09001000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
        backend_socket = { bind = "&charbackend_stdio_1.biflow_socket" },
    },

    -- -----------------------------------------------------------------------
    -- PL011 UART at 0x09000000
    -- -----------------------------------------------------------------------
    pl011_uart_0 = {
        moduletype    = "Pl011",
        dylib_path    = "uart-pl011",
        target_socket = {
            address = 0x09000000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
        backend_socket = { bind = "&charbackend_stdio_2.biflow_socket" },
    },

    -- -----------------------------------------------------------------------
    -- ELF loader
    -- -----------------------------------------------------------------------
    load = {
        moduletype       = "loader",
        initiator_socket = { bind = "&router.target_socket" },
        { elf_file = base .. "hello_usart2.elf" },
    },
}
