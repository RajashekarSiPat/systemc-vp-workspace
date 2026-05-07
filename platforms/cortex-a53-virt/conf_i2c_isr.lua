-- conf_i2c_isr.lua — Cortex-A53 I2C interrupt stress test platform.
--
-- Wiring:
--   i2c_stm32_0.ev_irq  →  gic_0.spi_in_2  (SPI 2, IRQ ID 34)
--   i2c_stm32_0.er_irq  →  gic_0.spi_in_3  (SPI 3, IRQ ID 35)
--
-- Console output: usart2_c at 0x09004000 (direct TBUF writes, no interrupt)
--
-- Run:
--   build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
--       --gs_luafile platforms/cortex-a53-virt/conf_i2c_isr.lua

function script_path()
    local src = debug.getinfo(2, "S").source:sub(2)
    return src:match("(.*/)")
end
local base = script_path()

platform = {
    moduletype = "Container",
    quantum_ns = 10000,

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
        sync_policy = "multithread",
    },

    cpu_0 = {
        moduletype   = "cpu_arm_cortexA53",
        args         = { "&qemu_inst" },
        mem          = { bind = "&router.target_socket" },
        rvbar        = 0x80000000,
        has_el3      = false,
        has_el2      = false,
        psci_conduit = "disabled",
    },

    -- GICv3: num_spi=32 supports SPIs 0-31 (IRQ IDs 32-63).
    -- SPI 2 → I2C EV (IRQ 34), SPI 3 → I2C ER (IRQ 35).
    gic_0 = {
        moduletype    = "arm_gicv3",
        args          = { "&qemu_inst" },
        dist_iface    = {
            address = 0x08000000,
            size    = 0x00010000,
            bind    = "&router.initiator_socket",
        },
        redist_iface_0 = {
            address = 0x080A0000,
            size    = 0x00020000,
            bind    = "&router.initiator_socket",
        },
        redist_region = { 1 },
        num_cpus      = 1,
        num_spi       = 32,
        irq_out_0     = { bind = "&cpu_0.irq_in" },
        fiq_out_0     = { bind = "&cpu_0.fiq_in" },
    },

    -- Console USART (firmware writes directly to TBUF; no IRQ needed)
    charbackend_stdio_0 = {
        moduletype = "char_backend_stdio",
        read_write = true,
    },

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

    -- STM32 I2C peripheral under test
    i2c_stm32_0 = {
        moduletype    = "I2cStm32",
        dylib_path    = "i2c_stm32",
        target_socket = {
            address = 0x09005000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
        slave_addr = 0x50,
        nack_addr  = 0xFF,
        ev_irq     = { bind = "&gic_0.spi_in_2" },
        er_irq     = { bind = "&gic_0.spi_in_3" },
    },

    exiter_0 = {
        moduletype    = "exiter",
        target_socket = {
            address = 0x09010000,
            size    = 0x1000,
            bind    = "&router.initiator_socket",
        },
    },

    load = {
        moduletype       = "loader",
        initiator_socket = { bind = "&router.target_socket" },
        { elf_file = os.getenv("I2C_ELF") or (base .. "hello_i2c_isr.elf") },
    },
}
