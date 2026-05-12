# SystemC VP Workspace

A reusable [SystemC](https://systemc.org/) / [TLM-2.0](https://www.accellera.org/downloads/standards/systemc) virtual-platform workspace built on top of the [QBOX](https://github.com/quic/qbox) framework.

The workspace provides a clean, scalable directory structure for developing custom SystemC peripherals and integrating them into virtual platforms that run real bare-metal firmware under a QEMU Cortex-A53 CPU model.

---

## Repository layout

```
systemc-vp-workspace/
├── CMakeLists.txt                     # Root build system (uses installed QBOX)
├── CLAUDE.md                          # Coding conventions and peripheral checklist
│
├── peripherals/
│   ├── usart/                         # Simple USART (STM32-style SR/DR/BRR/CR1)
│   │   ├── include/usart.h
│   │   └── src/usart.cc
│   │
│   ├── usart2/                        # Full-featured USART (spec-compliant)
│   │   ├── include/
│   │   │   ├── Usart_types.h          # USART_TxRx_Tlm serial frame type
│   │   │   ├── Usart_regdefs.h        # CON/TBUF/RBUF/BG/FDR register unions
│   │   │   ├── Usart.h                # Core model: TX/RX state machines, baud-gen
│   │   │   └── usart2.h               # QBOX wrapper (Usart2 + STATUS + IRQ + VCD)
│   │   └── src/
│   │       ├── Usart.cc               # Core model implementation
│   │       └── usart2.cc              # module_register()
│   │
│   └── serial_bridge/                 # Full-duplex USART bridge (no serial framing)
│       ├── include/serial_bridge.h    # SerialBridge: crosses two biflow sockets
│       └── src/serial_bridge.cc
│
├── platforms/
│   └── cortex-a53-virt/
│       ├── src/main.cc                # sc_main (ModuleFactory::Container)
│       ├── conf.lua                   # Simple USART demo
│       ├── conf_usart2.lua            # Full USART2 polling demo
│       ├── conf_fullduplex.lua        # USART2 A↔B full-duplex (STATUS polling)
│       └── conf_isr.lua               # GICv3 + USART2 interrupt test suite
│
└── baremetal/
    └── cortex-a53/
        └── src/
            ├── hello_usart.c          # Simple USART firmware
            ├── hello_usart2.c         # Full USART2 polling firmware
            ├── hello_fullduplex.c     # Full-duplex STATUS-polling firmware
            └── hello_isr.c            # AArch64 ISR test suite (GICv3, 5 tests)
```

---

## Peripheral descriptions

### `peripherals/usart` — Simple USART

STM32-inspired USART with a minimal register map:

| Offset | Register | Description |
|--------|----------|-------------|
| 0x00 | SR | Status: `[7]` TXE (always 1), `[5]` RXNE |
| 0x04 | DR | Data: write = TX byte, read = RX byte (clears RXNE) |
| 0x08 | BRR | Baud rate (ignored in simulation) |
| 0x0C | CR1 | Control: `[13]` UE, `[5]` RXNEIE, `[3]` TE, `[2]` RE |

---

### `peripherals/usart2` — Full-featured USART

Specification-compliant USART model with six operating modes, full TX/RX state machines, baud-rate generator (integer prescaler + sigma-delta FDR), and four interrupt outputs.

**Core register map:**

| Offset | Register | Description |
|--------|----------|-------------|
| 0x00 | CON | Control: mode, REN, parity, stop bits, BRS, FDE, LB, R (run) |
| 0x04 | TBUF | Transmit holding buffer (write-only) |
| 0x08 | RBUF | Receive holding buffer (read-only) |
| 0x0C | BG | Baud generator: write = reload value, read = live counter |
| 0x10 | FDR | Fractional divider (STEP + DM) |

Supported modes: `8N1`, `9-bit`, `7+Parity`, `8+Parity`, `8N2`, `synchronous`.

**Wrapper STATUS register (offset 0x20, sticky W1C):**

| Bit | Mask | Description |
|-----|------|-------------|
| 0 | `STATUS_TBIR` | TX buffer empty — TBUF freed into shift register |
| 1 | `STATUS_TIR` | TX complete — all bits shifted out |
| 2 | `STATUS_RIR` | RX data ready — RBUF has valid byte |
| 3 | `STATUS_EIR` | Error — overrun, framing, or parity |

The `Usart2` QBOX wrapper adapts the core model to the ModuleFactory + `biflow_socket` framework, combines the four IRQ outputs into a single `irq` line wired to the GIC, and maintains the sticky STATUS register.

**VCD waveform tracing:**

Set `vcd_file = "/tmp/basename"` in the Lua config to generate a VCD file containing seven 1-bit signals per instance:

| Signal | Description |
|--------|-------------|
| `txd` | Serial line level (extracted from TX state machine baud ticks) |
| `rxd` | Serial line level (idle in `rx_inject` / SerialBridge architecture) |
| `tbir` | 2-clock pulse on each TX-buffer-empty event |
| `tir` | 2-clock pulse on each TX-frame-complete event |
| `rir` | 2-clock pulse on each RX-data-ready event |
| `eir` | 2-clock pulse on each error event |
| `irq` | Combined GIC output — LOW→HIGH rising edge per new IRQ |

Open with: `gtkwave /tmp/basename.vcd`

---

### `peripherals/serial_bridge` — Full-duplex serial bridge

Connects two `Usart2` instances back-to-back (TX of A → RX of B, TX of B → RX of A) using `biflow_socket` without baud-rate serial framing. Used as the loopback fabric in all USART2 multi-instance tests.

---

## Platform configurations

### `conf.lua` — Simple USART demo

Single `Usart` instance at `0x09001000`. Firmware prints a greeting over the USART.

### `conf_usart2.lua` — Full USART2 polling demo

Single `Usart2` at `0x09002000` connected to stdio. Firmware exercises the full register map via STATUS polling.

### `conf_fullduplex.lua` — Full-duplex STATUS polling

Two `Usart2` instances (A at `0x09002000`, B at `0x09003000`) connected through a `SerialBridge`. Firmware sends bytes A→B and B→A using STATUS register polling — no interrupts.

### `conf_isr.lua` — GICv3 interrupt test suite

Two `Usart2` instances wired through a `SerialBridge` and a `GICv3` interrupt controller. The Cortex-A53 starts at EL1 (`has_el3 = false`, `has_el2 = false`). Firmware installs AArch64 exception vectors, enables IRQ via `MSR DAIFCLR`, and runs 5 interrupt-driven tests:

| Test | Mask bit | Description |
|------|----------|-------------|
| 1 | `0x01` | A → B one byte (`0x55`): TBIR\_A + RIR\_B |
| 2 | `0x02` | B → A one byte (`0xAA`): TBIR\_B + RIR\_A |
| 3 | `0x04` | Overrun: EIR\_B after second byte with RBUF full |
| 4 | `0x08` | Multi-byte stream A→B `[0xDE, 0xAD, 0xBE, 0xEF]` |
| 5 | `0x10` | TIR — TX-frame-complete via baud timing |

**GICv3 address map** (within `conf_isr.lua`):

| Range | Block |
|-------|-------|
| `0x08000000 – 0x0800FFFF` | GICD distributor (64 KB) |
| `0x080A0000 – 0x080BFFFF` | GICR redistributor (128 KB, 1 CPU) |

**SPI wiring:**

| USART IRQ | GIC SPI | IRQ ID |
|-----------|---------|--------|
| `usart2_a.irq` | `spi_in_0` | 32 |
| `usart2_b.irq` | `spi_in_1` | 33 |

**TEST_MASK** selects which tests to run (bit N−1 = Test N, default `0x1F` = all):

```bash
TEST_MASK=0x07 ./vp ...    # Tests 1, 2, 3 only
TEST_MASK=0x01 ./vp ...    # Test 1 only
```

---

## Dependencies

### Required

| Dependency | Version | Notes |
|------------|---------|-------|
| [QBOX](https://github.com/quic/qbox) | latest | Must be built and installed first |
| CMake | ≥ 3.14 | |
| C++17 compiler | GCC ≥ 9 / Clang ≥ 10 | |
| Python 3 | ≥ 3.6 | Required by QBOX build |
| Ninja or Make | any | CMake build backend |

### For bare-metal firmware (cross-compilation)

| Dependency | Install |
|------------|---------|
| `aarch64-linux-gnu-gcc` | `sudo apt install gcc-aarch64-linux-gnu` |

### For waveform viewing

| Dependency | Install |
|------------|---------|
| GTKWave | `sudo apt install gtkwave` |

### QBOX transitive dependencies (handled by QBOX's build system)
SystemC 3.x, SystemC CCI, SCP reporting, Lua 5.x, libqemu, rpclib, fmt.

---

## Build instructions

### Step 1 — Clone and build QBOX

QBOX is an external dependency and is **not** included in this repository.

```bash
git clone https://github.com/quic/qbox.git
cd qbox
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
cmake --install build          # installs to qbox/build/install/
cd ..
```

> The workspace `CMakeLists.txt` expects QBOX at `<workspace-root>/qbox/build/install/`.
> Override with `-DQBOX_INSTALL=/path/to/install` if needed.

### Step 2 — Clone this workspace

```bash
git clone https://github.com/RajashekarSiPat/systemc-vp-workspace.git
cd systemc-vp-workspace

# Place (or symlink) the qbox directory here
ln -s ../qbox qbox
```

### Step 3 — Build the workspace

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

This produces:

| Artifact | Description |
|----------|-------------|
| `build/platforms/cortex-a53-virt/cortex-a53-virt-vp` | VP executable |
| `build/usart.so` | Simple USART peripheral |
| `build/usart2.so` | Full USART2 peripheral |
| `build/serial_bridge.so` | Full-duplex serial bridge peripheral |
| `platforms/cortex-a53-virt/hello_usart.elf` | Simple USART firmware |
| `platforms/cortex-a53-virt/hello_usart2.elf` | Full USART2 polling firmware |
| `platforms/cortex-a53-virt/hello_fullduplex.elf` | Full-duplex polling firmware |
| `platforms/cortex-a53-virt/hello_isr.elf` | GICv3 ISR test firmware |

> Firmware ELFs require `aarch64-linux-gnu-gcc`. If not installed, only the VP binary
> and `.so` libraries are built. Install with: `sudo apt install gcc-aarch64-linux-gnu`

---

## Running the simulations and tests

Set `LD_LIBRARY_PATH` so the VP can find QBOX shared libraries and peripheral `.so` files:

```bash
export LD_LIBRARY_PATH="\
qbox/build/install/lib:\
qbox/build/install/lib/qbox-2.0:\
qbox/build/install/lib/libqemu:\
build"
```

Build the USART2 peripheral and test firmware after source changes:

```bash
cmake --build build --target usart2 cortex-a53-virt-fw2 cortex-a53-virt-fw3 cortex-a53-virt-fw4
```

### Simple USART

```bash
./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf.lua
```

### Full USART2 polling

```bash
./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf_usart2.lua
```

Expected result: the firmware prints the USART2 banner and exits with status 0.

### Full-duplex STATUS polling

```bash
./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf_fullduplex.lua
```

Expected result: all five full-duplex STATUS polling tests pass and the VP exits cleanly.

### GICv3 interrupt tests

Run all five tests:

```bash
./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf_isr.lua
```

Run a specific subset:

```bash
# Tests 1, 2, 3 only
TEST_MASK=0x07 ./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf_isr.lua

# Use a custom ELF
ISR_ELF=/path/to/hello_isr.elf ./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf_isr.lua
```

Expected output:

```
================================================
  Passed: 0x00000005   Failed: 0x00000000
  ALL TESTS PASSED
================================================
```

### Run all USART2 tests

Run the three USART2 simulations in sequence:

```bash
./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf_usart2.lua

./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf_fullduplex.lua

./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf_isr.lua
```

### VCD waveform tracing

`conf_isr.lua` generates `/tmp/usart2_a.vcd` and `/tmp/usart2_b.vcd` automatically:

```bash
gtkwave /tmp/usart2_a.vcd /tmp/usart2_b.vcd
```

To disable tracing, remove the `vcd_file` lines from the Lua config. To trace other
instances, add `vcd_file = "/tmp/basename"` to any `Usart2` block.

---

## Address map (conf_isr.lua)

| Address range | Module | Description |
|---------------|--------|-------------|
| `0x08000000 – 0x0800FFFF` | `arm_gicv3` GICD | GIC distributor |
| `0x080A0000 – 0x080BFFFF` | `arm_gicv3` GICR | GIC redistributor (1 CPU) |
| `0x80000000 – 0x8FFFFFFF` | `gs_memory` | 256 MB RAM |
| `0x09002000 – 0x09002FFF` | `Usart2` A | USART2 A → GIC SPI 0 (IRQ 32) |
| `0x09003000 – 0x09003FFF` | `Usart2` B | USART2 B → GIC SPI 1 (IRQ 33) |
| `0x09004000 – 0x09004FFF` | `Usart2` C | Console output (stdio, no IRQ) |
| `0x09010000 – 0x09010FFF` | `exiter` | Write 0 to call `sc_stop()` |

---

## Adding a new peripheral

Follow the checklist in `CLAUDE.md`. Short version:

```bash
mkdir -p peripherals/<name>/include peripherals/<name>/src
```

1. `peripherals/<name>/CMakeLists.txt` — one line: `gs_create_dymod(<name>)`
2. `peripherals/<name>/include/<name>.h` — `sc_module` subclass; class name = Lua `moduletype`
3. `peripherals/<name>/src/<name>.cc` — `void module_register() { GSC_MODULE_REGISTER_C(ClassName); }`
4. Add `add_subdirectory(<name>)` to `peripherals/CMakeLists.txt`
5. Reference in a platform `conf.lua` with `dylib_path = "<name>"`

---

## License

This workspace is provided as-is for educational and research purposes.
QBOX is licensed under its own terms — see [quic/qbox](https://github.com/quic/qbox).
