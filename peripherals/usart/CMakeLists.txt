# gs_create_dymod(<name>) does everything needed for a qbox peripheral:
#   - glob src/*.cc
#   - build a shared library named <name> (no "lib" prefix)
#   - link against TARGET_LIBS (SystemC, CCI, qbox, libqemu, ...)
#   - expose include/ as a public include directory
#   - output the .so into CMAKE_BINARY_DIR so the loader finds it
gs_create_dymod(usart)
