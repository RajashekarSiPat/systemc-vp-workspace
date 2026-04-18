# ---------------------------------------------------------------------------
# Platform VP integrations.
# Each platform lives in its own subdirectory and should follow this layout:
#
#   platforms/<name>/
#     CMakeLists.txt    -- builds the VP executable
#     conf.lua          -- platform wiring (modules, address map, firmware)
#     src/main.cc       -- sc_main entry point (use the standard boilerplate)
#
# To add a new platform: mkdir -p platforms/<name>/src
# then add:  add_subdirectory(<name>)  below.
# ---------------------------------------------------------------------------

add_subdirectory(cortex-a53-virt)
