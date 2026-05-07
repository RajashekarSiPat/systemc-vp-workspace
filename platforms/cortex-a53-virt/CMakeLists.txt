add_executable(cortex-a53-virt-vp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc
)

if(APPLE)
    target_link_options(cortex-a53-virt-vp PRIVATE "LINKER:-dead_strip_dylibs")
else()
    target_link_options(cortex-a53-virt-vp PRIVATE "LINKER:-as-needed")
endif()

target_link_libraries(cortex-a53-virt-vp PRIVATE ${TARGET_LIBS})

# RPATH: find shared libraries next to the binary at runtime
set_target_properties(cortex-a53-virt-vp PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../../lib;$ORIGIN/../../../lib/libqemu;$ORIGIN"
    BUILD_RPATH   "${CMAKE_BINARY_DIR}"
)

# ---- Cross-compile the baremetal firmware (if toolchain is present) -------
find_program(AARCH64_GCC aarch64-linux-gnu-gcc)

if(AARCH64_GCC)
    set(FW_SRC  "${CMAKE_SOURCE_DIR}/baremetal/cortex-a53/src/hello_usart.c")
    set(FW_ELF  "${CMAKE_CURRENT_SOURCE_DIR}/hello_usart.elf")

    add_custom_command(
        OUTPUT  ${FW_ELF}
        COMMAND ${AARCH64_GCC}
            -nostdlib -static
            -Ttext=0x80000000 -e _start
            -Wl,--build-id=none -Wl,-n
            -o ${FW_ELF}
            ${FW_SRC}
        DEPENDS ${FW_SRC}
        COMMENT "Cross-compiling hello_usart.elf for AArch64"
    )
    add_custom_target(cortex-a53-virt-fw ALL DEPENDS ${FW_ELF})

    # ---- hello_usart2.elf — firmware for the full Usart2 peripheral --------
    set(FW2_SRC "${CMAKE_SOURCE_DIR}/baremetal/cortex-a53/src/hello_usart2.c")
    set(FW2_ELF "${CMAKE_CURRENT_SOURCE_DIR}/hello_usart2.elf")

    add_custom_command(
        OUTPUT  ${FW2_ELF}
        COMMAND ${AARCH64_GCC}
            -nostdlib -static
            -Ttext=0x80000000 -e _start
            -Wl,--build-id=none -Wl,-n
            -o ${FW2_ELF}
            ${FW2_SRC}
        DEPENDS ${FW2_SRC}
        COMMENT "Cross-compiling hello_usart2.elf for AArch64"
    )
    add_custom_target(cortex-a53-virt-fw2 ALL DEPENDS ${FW2_ELF})
    # ---- hello_fullduplex.elf — full-duplex USART2 interrupt test ----------
    set(FW3_SRC "${CMAKE_SOURCE_DIR}/baremetal/cortex-a53/src/hello_fullduplex.c")
    set(FW3_ELF "${CMAKE_CURRENT_SOURCE_DIR}/hello_fullduplex.elf")

    add_custom_command(
        OUTPUT  ${FW3_ELF}
        COMMAND ${AARCH64_GCC}
            -nostdlib -static
            -Ttext=0x80000000 -e _start
            -Wl,--build-id=none -Wl,-n
            -o ${FW3_ELF}
            ${FW3_SRC}
        DEPENDS ${FW3_SRC}
        COMMENT "Cross-compiling hello_fullduplex.elf for AArch64"
    )
    add_custom_target(cortex-a53-virt-fw3 ALL DEPENDS ${FW3_ELF})

    # ---- hello_isr.elf — ISR GICv3 USART2 interrupt test ----------------------
    # Test selection is done at run time via TEST_MASK env var (read by Lua).
    # g_test_mask lives at 0x80FFFF00 (--section-start); Lua patches it before
    # the CPU starts.  No per-test ELFs needed.
    set(FW4_SRC "${CMAKE_SOURCE_DIR}/baremetal/cortex-a53/src/hello_isr.c")
    set(FW4_ELF "${CMAKE_CURRENT_SOURCE_DIR}/hello_isr.elf")
    set(FW4_FLAGS -nostdlib -static -Ttext=0x80000000 -e _start
                  -Wl,--build-id=none -Wl,-n
                  "-Wl,--section-start=.test_cfg=0x80FFFF00")

    add_custom_command(
        OUTPUT  ${FW4_ELF}
        COMMAND ${AARCH64_GCC} ${FW4_FLAGS} -o ${FW4_ELF} ${FW4_SRC}
        DEPENDS ${FW4_SRC}
        COMMENT "Cross-compiling hello_isr.elf (runtime TEST_MASK via Lua)"
    )
    add_custom_target(cortex-a53-virt-fw-isr ALL DEPENDS ${FW4_ELF})

    # ---- hello_i2c_isr.elf — I2C interrupt stress test -------------------------
    set(FW5_SRC "${CMAKE_SOURCE_DIR}/baremetal/cortex-a53/src/hello_i2c_isr.c")
    set(FW5_ELF "${CMAKE_CURRENT_SOURCE_DIR}/hello_i2c_isr.elf")

    add_custom_command(
        OUTPUT  ${FW5_ELF}
        COMMAND ${AARCH64_GCC}
            -nostdlib -static
            -Ttext=0x80000000 -e _start
            -Wl,--build-id=none -Wl,-n
            -o ${FW5_ELF}
            ${FW5_SRC}
        DEPENDS ${FW5_SRC}
        COMMENT "Cross-compiling hello_i2c_isr.elf for AArch64"
    )
    add_custom_target(cortex-a53-virt-fw-i2c-isr ALL DEPENDS ${FW5_ELF})

else()
    message(WARNING
        "aarch64-linux-gnu-gcc not found. "
        "Build ELFs manually:\n"
        "  aarch64-linux-gnu-gcc -nostdlib -static -Ttext=0x80000000 -e _start "
        "-Wl,--build-id=none -Wl,-n -o platforms/cortex-a53-virt/hello_usart.elf "
        "baremetal/cortex-a53/src/hello_usart.c\n"
        "  aarch64-linux-gnu-gcc -nostdlib -static -Ttext=0x80000000 -e _start "
        "-Wl,--build-id=none -Wl,-n -o platforms/cortex-a53-virt/hello_usart2.elf "
        "baremetal/cortex-a53/src/hello_usart2.c\n"
        "  aarch64-linux-gnu-gcc -nostdlib -static -Ttext=0x80000000 -e _start "
        "-Wl,--build-id=none -Wl,-n -o platforms/cortex-a53-virt/hello_fullduplex.elf "
        "baremetal/cortex-a53/src/hello_fullduplex.c")
endif()
