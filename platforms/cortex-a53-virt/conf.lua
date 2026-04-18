-- conf.lua — Cortex-A53 virt platform with custom USART peripheral.
--
-- Hardware map:
--   0x80000000 .. 0x8FFFFFFF  256 MB RAM (firmware loaded here)
--   0x09000000 .. 0x09000FFF  PL011 UART  (kept for compatibility / QEMU)
--   0x09001000 .. 0x09001FFF  Custom USART (our peripheral under test)
--
-- Run:
--   ./cortex-a53-virt-vp -c platforms/cortex-a53-virt/conf.lua

-- Resolve paths relative to this config file so the ELF is found regardless
-- of the working directory.
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

    -- -----------------------------------------------------------------------
    -- Custom USART peripheral at 0x09001000.
    -- The dylib_path must match the .so filename built by peripherals/usart/.
    -- -----------------------------------------------------------------------
    usart_0 = {
        moduletype    = "Usart",
        dylib_path    = "usart",
        target_socket = {
            address = 0x09001000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
        backend_socket = { bind = "&charbackend_stdio_0.biflow_socket" },
    },

    -- -----------------------------------------------------------------------
    -- PL011 UART at 0x09000000 (standard ARM virt UART — kept here so that
    -- any code that also targets PL011 still works alongside our USART demo).
    -- -----------------------------------------------------------------------
    pl011_uart_0 = {
        moduletype    = "Pl011",
        dylib_path    = "uart-pl011",
        target_socket = {
            address = 0x09000000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
        backend_socket = { bind = "&charbackend_stdio_1.biflow_socket" },
    },

    -- -----------------------------------------------------------------------
    -- ELF loader
    -- -----------------------------------------------------------------------
    load = {
        moduletype       = "loader",
        initiator_socket = { bind = "&router.target_socket" },
        { elf_file = base .. "hello_usart.elf" },
    },
}
