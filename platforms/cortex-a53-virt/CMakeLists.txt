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

    # ---- hello_isr.elf — ISR GICv3 USART2 interrupt test (all tests) ---------
    set(FW4_SRC "${CMAKE_SOURCE_DIR}/baremetal/cortex-a53/src/hello_isr.c")
    set(FW4_ELF "${CMAKE_CURRENT_SOURCE_DIR}/hello_isr.elf")
    set(FW4_FLAGS -nostdlib -static -Ttext=0x80000000 -e _start
                  -Wl,--build-id=none -Wl,-n)

    add_custom_command(
        OUTPUT  ${FW4_ELF}
        COMMAND ${AARCH64_GCC} ${FW4_FLAGS} -o ${FW4_ELF} ${FW4_SRC}
        DEPENDS ${FW4_SRC}
        COMMENT "Cross-compiling hello_isr.elf (all tests, TEST_MASK=0x1F)"
    )
    add_custom_target(cortex-a53-virt-fw-isr ALL DEPENDS ${FW4_ELF})

    # ---- hello_isr_tN.elf — one ELF per test (TEST_MASK selects one test) ---
    set(_isr_nums  1    2    3    4    5   )
    set(_isr_masks 0x01 0x02 0x04 0x08 0x10)
    list(LENGTH _isr_nums _isr_count)
    math(EXPR _isr_last "${_isr_count} - 1")
    foreach(_i RANGE ${_isr_last})
        list(GET _isr_nums  ${_i} _n)
        list(GET _isr_masks ${_i} _mask)
        set(_elf "${CMAKE_CURRENT_SOURCE_DIR}/hello_isr_t${_n}.elf")
        add_custom_command(
            OUTPUT  ${_elf}
            COMMAND ${AARCH64_GCC} ${FW4_FLAGS}
                    -DTEST_MASK=${_mask} -o ${_elf} ${FW4_SRC}
            DEPENDS ${FW4_SRC}
            COMMENT "Cross-compiling hello_isr_t${_n}.elf (TEST_MASK=${_mask})"
        )
        add_custom_target(cortex-a53-virt-fw-isr-t${_n} ALL DEPENDS ${_elf})
    endforeach()

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
