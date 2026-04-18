# ---------------------------------------------------------------------------
# Baremetal firmware (cross-compiled).
# Each target lives in its own subdirectory:
#
#   baremetal/<arch>/
#     CMakeLists.txt    -- standalone cross-compilation (no host deps)
#     src/<name>.c      -- bare-metal C source
#
# To add a new firmware target: mkdir -p baremetal/<arch>/src
# then add:  add_subdirectory(<arch>)  below.
# ---------------------------------------------------------------------------

add_subdirectory(cortex-a53)
