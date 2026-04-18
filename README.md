# SystemC VP Workspace

A reusable [SystemC](https://systemc.org/) / [TLM-2.0](https://www.accellera.org/downloads/standards/systemc) virtual-platform workspace built on top of the [QBOX](https://github.com/quic/qbox) framework.

The workspace provides a clean, scalable directory structure for developing custom SystemC peripherals and integrating them into virtual platforms that run real bare-metal firmware under a QEMU CPU model.

---

## Repository layout

```
systemc-vp-workspace/
├── CMakeLists.txt                  # Root build system (uses installed QBOX)
├── CLAUDE.md                       # Coding conventions and peripheral checklist
│
├── peripherals/
│   ├── usart/                      # Simple USART (STM32-style SR/DR/BRR/CR1)
│   │   ├── include/usart.h         # sc_module class + register map
│   │   └── src/usart.cc            # module_register()
│   │
│   └── usart2/                     # Full-featured USART (spec-compliant model)
│       ├── include/
│       │   ├── Usart_types.h       # USART_TxRx_Tlm serial frame type
│       │   ├── Usart_regdefs.h     # CON/TBUF/RBUF/BG/FDR register unions
│       │   ├── Usart.h             # Core model: TX/RX state machines, baud-gen
│       │   └── usart2.h            # QBOX wrapper (Usart2, ModuleFactory pattern)
│       └── src/
│           ├── Usart.cc            # Core model implementation
│           └── usart2.cc           # module_register()
│
├── platforms/
│   └── cortex-a53-virt/
│       ├── src/main.cc             # sc_main (ModuleFactory::Container boilerplate)
│       ├── conf.lua                # Platform config: exercises usart peripheral
│       └── conf_usart2.lua         # Platform config: exercises usart2 peripheral
│
└── baremetal/
    └── cortex-a53/
        └── src/
            ├── hello_usart.c       # AArch64 firmware for simple USART
            └── hello_usart2.c      # AArch64 firmware for full USART2
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

Mapped at `0x09001000` in the virtual platform.

### `peripherals/usart2` — Full-featured USART
Specification-compliant USART model with six operating modes, full TX/RX state machines, baud-rate generator (integer prescaler + sigma-delta FDR), and four interrupt outputs.

| Offset | Register | Description |
|--------|----------|-------------|
| 0x00 | CON | Control: mode, REN, parity, stop bits, BRS, FDE, LB, R (run) |
| 0x04 | TBUF | Transmit holding buffer (write-only) |
| 0x08 | RBUF | Receive holding buffer (read-only) |
| 0x0C | BG | Baud generator: write = reload value, read = live counter |
| 0x10 | FDR | Fractional divider (STEP + DM) |

Supported modes: `8N1`, `9-bit`, `7+Parity`, `8+Parity`, `8N2`, `synchronous`.

Mapped at `0x09002000` in the virtual platform.

The core model (`Usart.h` / `Usart.cc`) is wrapped by `Usart2` (`usart2.h` / `usart2.cc`) which adapts it to the QBOX `ModuleFactory` + `biflow_socket` framework.

---

## Dependencies

### Required

| Dependency | Version | Notes |
|------------|---------|-------|
| [QBOX](https://github.com/quic/qbox) | latest | Must be built and installed before this workspace |
| CMake | ≥ 3.14 | |
| C++17 compiler | GCC ≥ 9 / Clang ≥ 10 | |
| Python 3 | ≥ 3.6 | Required by QBOX build |
| Ninja or Make | any | Build backend for CMake |

### For bare-metal firmware (cross-compilation)
| Dependency | Install |
|------------|---------|
| `aarch64-linux-gnu-gcc` | `sudo apt install gcc-aarch64-linux-gnu` |

### QBOX transitive dependencies (handled by QBOX's build system)
SystemC 3.x, SystemC CCI, SCP reporting, Lua 5.x, libqemu, rpclib, fmt.

---

## Build instructions

### Step 1 — Clone and build QBOX

QBOX is an external dependency and is **not** included in this repository.

```bash
# Clone QBOX alongside this workspace
git clone https://github.com/quic/qbox.git

cd qbox
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
cmake --install build          # installs to qbox/build/install/
cd ..
```

> **Note:** The workspace CMakeLists.txt expects QBOX to be installed at
> `<workspace-root>/qbox/build/install/`. If you place it elsewhere, pass
> `-DQBOX_INSTALL=/path/to/install` to the workspace configure step.

### Step 2 — Clone this workspace

```bash
git clone https://github.com/RajashekarSiPat/systemc-vp-workspace.git
cd systemc-vp-workspace

# Place (or symlink) the qbox directory here
# e.g. if you cloned both side by side:
ln -s ../qbox qbox
```

### Step 3 — Build the workspace

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

This produces:
- `build/platforms/cortex-a53-virt/cortex-a53-virt-vp` — the VP executable
- `build/usart.so` — simple USART peripheral shared library
- `build/usart2.so` — full USART2 peripheral shared library
- `platforms/cortex-a53-virt/hello_usart.elf` — AArch64 firmware (simple USART)
- `platforms/cortex-a53-virt/hello_usart2.elf` — AArch64 firmware (full USART2)

> The firmware ELFs require `aarch64-linux-gnu-gcc`. If not installed, a warning
> is printed and only the VP binary and `.so` libraries are built. Install with:
> `sudo apt install gcc-aarch64-linux-gnu`

---

## Running the simulations

Set `LD_LIBRARY_PATH` so the VP binary can find the QBOX shared libraries and
the peripheral `.so` files at runtime.

```bash
export LD_LIBRARY_PATH="\
qbox/build/install/lib:\
qbox/build/install/lib/qbox-2.0:\
qbox/build/install/lib/libqemu:\
build"
```

### Test 1 — Simple USART (`peripherals/usart`)

```bash
./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf.lua
```

Expected output:
```
        SystemC 3.0.2-Accellera ...
[I] [ 0 s ] SystemC : SC_START
Hello from custom USART!
Peripheral: peripherals/usart/
Platform:   platforms/cortex-a53-virt/
```

### Test 2 — Full USART2 (`peripherals/usart2`)

```bash
./build/platforms/cortex-a53-virt/cortex-a53-virt-vp \
    --gs_luafile platforms/cortex-a53-virt/conf_usart2.lua
```

Expected output:
```
        SystemC 3.0.2-Accellera ...
[I] [ 0 s ] SystemC : SC_START
Hello from Usart2 (full model)!
Peripheral:  peripherals/usart2/
Register map: CON / TBUF / RBUF / BG / FDR
Modes: 8N1, 9-bit, 7+P, 8+P, 8N2, synchronous
```

---

## Address map

| Address range | Module | Peripheral |
|---------------|--------|------------|
| `0x80000000 – 0x8FFFFFFF` | `gs_memory` | 256 MB RAM (firmware loaded here) |
| `0x09000000 – 0x09000FFF` | `Pl011` | ARM PL011 UART |
| `0x09001000 – 0x09001FFF` | `Usart` | Simple USART (`peripherals/usart`) |
| `0x09002000 – 0x09002FFF` | `Usart2` | Full USART2 (`peripherals/usart2`) |

---

## Adding a new peripheral

Follow the checklist in `CLAUDE.md`. The short version:

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
