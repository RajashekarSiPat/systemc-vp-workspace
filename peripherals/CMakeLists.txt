# ---------------------------------------------------------------------------
# Reusable peripheral IP blocks.
# Each peripheral lives in its own subdirectory and MUST follow this layout:
#
#   peripherals/<name>/
#     CMakeLists.txt        -- must call: gs_create_dymod(<name>)
#     include/<name>.h      -- public header (module class, register map)
#     src/<name>.cc         -- implementation + void module_register()
#
# To add a new peripheral: mkdir -p peripherals/<name>/include peripherals/<name>/src
# then add:  add_subdirectory(<name>)  below.
# ---------------------------------------------------------------------------

add_subdirectory(usart)
add_subdirectory(usart2)
add_subdirectory(serial_bridge)
add_subdirectory(i2c_stm32)
