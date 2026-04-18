# SystemC VP Workspace — Rules for Claude

## Project overview

This workspace builds reusable SystemC/TLM virtual-platform components on top of
the QBOX framework.  The build is driven by the root `CMakeLists.txt`, which adds
`qbox/` as a sub-project, then adds our `peripherals/`, `platforms/`, and
`baremetal/` trees.

## Directory conventions

### qbox/
**Never modify any file inside `qbox/`.** It is the upstream QBOX framework.
All customisation lives outside this directory.

### peripherals/<name>/
Each reusable SystemC IP block lives here.  **Every peripheral MUST have:**

1. `peripherals/<name>/CMakeLists.txt` — contains exactly:
   ```cmake
   gs_create_dymod(<name>)
   ```
2. `peripherals/<name>/include/<name>.h` — public header.
   - Declares the `sc_module` subclass.
   - Class name must match the `moduletype` string used in platform Lua configs.
   - Must declare `extern "C" void module_register();`
3. `peripherals/<name>/src/<name>.cc` — implementation.
   - Must contain `void module_register() { GSC_MODULE_REGISTER_C(<ClassName>); }`
4. Register the new peripheral in `peripherals/CMakeLists.txt`:
   ```cmake
   add_subdirectory(<name>)
   ```

`gs_create_dymod` builds a shared library `<name>.so` and links it against
`TARGET_LIBS` (SystemC, CCI, libqemu, qbox, …).

### platforms/<name>/
Each virtual-platform integration lives here.  **Every platform MUST have:**

1. `platforms/<name>/CMakeLists.txt` — builds the VP executable.
2. `platforms/<name>/conf.lua` — platform topology (modules, address map,
   firmware path).  Reference peripherals with `dylib_path = "<name>"`.
3. `platforms/<name>/src/main.cc` — `sc_main` entry point.  Use the standard
   `gs::ModuleFactory::Container` boilerplate; put real topology in the Lua file.
4. Register the new platform in `platforms/CMakeLists.txt`:
   ```cmake
   add_subdirectory(<name>)
   ```

### baremetal/<arch>/
Cross-compiled bare-metal firmware.  **Every firmware target MUST have:**

1. `baremetal/<arch>/CMakeLists.txt` — cross-compilation rule.
2. `baremetal/<arch>/src/<name>.c` — C source (no stdlib, no OS).
3. Register in `baremetal/CMakeLists.txt`:
   ```cmake
   add_subdirectory(<arch>)
   ```

## Build instructions

```bash
# Configure (reuses qbox/build/_deps — no re-download)
cmake -B build -S .

# Build everything
cmake --build build -j$(nproc)

# Run the Cortex-A53 USART demo
./build/cortex-a53-virt-vp -c platforms/cortex-a53-virt/conf.lua
```

## Peripheral checklist (copy when adding a new peripheral)

- [ ] `peripherals/<name>/include/<name>.h` — `sc_module` class with TLM socket(s)
- [ ] `peripherals/<name>/src/<name>.cc`    — `module_register()`
- [ ] `peripherals/<name>/CMakeLists.txt`   — `gs_create_dymod(<name>)`
- [ ] Entry in `peripherals/CMakeLists.txt` — `add_subdirectory(<name>)`
- [ ] Entry in platform `conf.lua`          — `dylib_path = "<name>"`

## Coding rules

- Use `SCP_LOGGER()` / `SCP_TRACE` / `SCP_WARN` / `SCP_ERR` for all logging.
- Use `gs::biflow_socket` for character-stream I/O to backends.
- Use `InitiatorSignalSocket<bool>` for interrupt lines.
- Module class names must be unique across all peripherals (they are the Lua
  `moduletype` strings).
- Keep `usart.h` header-only for the module definition; put only
  `module_register()` in the `.cc` file to minimise recompilation.
- Do not add platform-specific code to `peripherals/`; keep IP blocks generic.
