-- conf_isr.lua — Cortex-A53 ISR test platform: GICv3 + two USART2 instances.
--
-- Unlike conf_fullduplex.lua (which uses STATUS register polling), this
-- platform wires USART2 irq outputs through a GICv3 (arm_gicv3) so the
-- Cortex-A53 CPU takes real AArch64 IRQ exceptions on every USART event.
--
-- GICv3 register map:
--   0x08000000 .. 0x0800FFFF  GICD  (distributor, 64 KB)
--   0x080A0000 .. 0x080BFFFF  GICR  (redistributor, 128 KB = 1 × 64 KB × 2 frames)
--
-- SPI wiring (SPIs start at GIC interrupt ID 32):
--   usart2_a.irq  →  gic_0.spi_in_0  (SPI 0, IRQ ID 32)
--   usart2_b.irq  →  gic_0.spi_in_1  (SPI 1, IRQ ID 33)
--
-- CPU wiring:
--   gic_0.irq_out_0  →  cpu_0.irq_in
--   gic_0.fiq_out_0  →  cpu_0.fiq_in
--
-- has_el3/has_el2 = false: CPU starts directly at EL1 so firmware can set
-- VBAR_EL1, enable IRQ via MSR DAIFCLR, and handle exceptions without any
-- EL3→EL1 drop sequence.
--
-- Run:
--   ./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
--       --gs_luafile platforms/cortex-a53-virt/conf_isr.lua

function script_path()
    local src = debug.getinfo(2, "S").source:sub(2)
    return src:match("(.*/)")
end
local base = script_path()

-- TEST_MASK selects which tests to run (bit N-1 = Test N; default 0x1F = all).
-- The loader patches g_test_mask at 0x80FFFF00 after loading the ELF, so no
-- recompile is needed:
--   TEST_MASK=0x01 ./vp ...   -- only Test 1
--   TEST_MASK=0x07 ./vp ...   -- Tests 1, 2, 3
local test_mask = tonumber(os.getenv("TEST_MASK")) or 0x1F

-- Write test_mask as a little-endian uint32 binary file so the loader can
-- patch g_test_mask at 0x80FFFF00 via bin_file (unambiguous path; the data={}
-- loader path requires CCI integer-key support that is not always available).
local mask_file = "/tmp/hello_isr_mask.bin"
do
    local b0 = test_mask % 256
    local b1 = math.floor(test_mask / 256) % 256
    local b2 = math.floor(test_mask / 65536) % 256
    local b3 = math.floor(test_mask / 16777216) % 256
    local f   = assert(io.open(mask_file, "wb"))
    f:write(string.char(b0, b1, b2, b3))
    f:close()
end

platform = {
    moduletype = "Container",
    quantum_ns = 1000000,

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

    -- -----------------------------------------------------------------------
    -- Cortex-A53: no EL3/EL2 so CPU resets directly into EL1.
    -- -----------------------------------------------------------------------
    cpu_0 = {
        moduletype   = "cpu_arm_cortexA53",
        args         = { "&qemu_inst" },
        mem          = { bind = "&router.target_socket" },
        rvbar        = 0x80000000,
        has_el3      = false,
        has_el2      = false,
        psci_conduit = "disabled",
    },

    -- -----------------------------------------------------------------------
    -- GICv3: distributor at 0x08000000, redistributor at 0x080A0000.
    --
    -- arm_gicv3 avoids the arm_gicv2 v2m_iface binding problem (arm_gicv2
    -- has an unconditionally-declared TlmTargetSocket with no implementation
    -- when has_msi_support=false, causing SystemC E109 abort).
    --
    -- num_spi=32: (NUM_PPI=32 + num_spi) must be divisible by 32.
    -- Only SPI 0 (IRQ32) and SPI 1 (IRQ33) are wired to USARTs; extras unused.
    --
    -- redist_region={1}: one redistributor region covering 1 CPU.
    -- The firmware uses GICv3 system-register CPU interface (ICC_*_EL1).
    -- -----------------------------------------------------------------------
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
            size    = 0x00020000,   -- 2 × 64 KB frames for 1 CPU
            bind    = "&router.initiator_socket",
        },
        redist_region = { 1 },
        num_cpus      = 1,
        num_spi       = 32,
        irq_out_0     = { bind = "&cpu_0.irq_in" },
        fiq_out_0     = { bind = "&cpu_0.fiq_in" },
    },

    -- -----------------------------------------------------------------------
    -- Console output
    -- -----------------------------------------------------------------------
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

    -- -----------------------------------------------------------------------
    -- USART2 A — bridge side A; irq → GIC SPI 0 (IRQ ID 32)
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
        irq            = { bind = "&gic_0.spi_in_0" },
        vcd_file       = "/tmp/usart2_a",
    },

    -- -----------------------------------------------------------------------
    -- USART2 B — bridge side B; irq → GIC SPI 1 (IRQ ID 33)
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
        irq            = { bind = "&gic_0.spi_in_1" },
        vcd_file       = "/tmp/usart2_b",
    },

    -- -----------------------------------------------------------------------
    -- Serial bridge: TX of A → RX of B, TX of B → RX of A
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
    -- ELF loader + runtime test-mask patch
    --
    -- Entry 1: load the firmware ELF (ISR_ELF env var overrides the default).
    -- Entry 2: write test_mask (a uint32) to g_test_mask at 0x80FFFF00.
    --          This overwrites the ELF's default 0x1F with the value from
    --          the TEST_MASK env var (or 0x1F if unset), so the CPU sees the
    --          right mask before it reaches isr_main().
    -- -----------------------------------------------------------------------
    load = {
        moduletype       = "loader",
        initiator_socket = { bind = "&router.target_socket" },
        { elf_file = os.getenv("ISR_ELF") or (base .. "hello_isr.elf") },
        { address = 0x80FFFF00, bin_file = mask_file },
    },
}
