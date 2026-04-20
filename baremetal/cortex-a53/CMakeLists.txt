if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # -----------------------------------------------------------------------
    # Standalone mode: invoked directly with the aarch64 cross-toolchain.
    #   cmake -B build -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc .
    #   cmake --build build
    # -----------------------------------------------------------------------
    cmake_minimum_required(VERSION 3.14)
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR aarch64)
    set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
    project(cortex-a53-firmware C)
endif()

find_program(AARCH64_GCC aarch64-linux-gnu-gcc)

if(AARCH64_GCC OR (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR))
    # Build the firmware ELF alongside the platform config so conf.lua can
    # reference it with a relative path.
    set(FW_OUT "${CMAKE_SOURCE_DIR}/platforms/cortex-a53-virt/hello_usart.elf")

    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        # Standalone: output next to the source
        set(FW_OUT "${CMAKE_CURRENT_SOURCE_DIR}/hello_usart.elf")
    endif()

    if(AARCH64_GCC)
        add_custom_command(
            OUTPUT  ${FW_OUT}
            COMMAND ${AARCH64_GCC}
                -nostdlib -static
                -Ttext=0x80000000 -e _start
                -Wl,--build-id=none -Wl,-n
                -o ${FW_OUT}
                ${CMAKE_CURRENT_SOURCE_DIR}/src/hello_usart.c
            DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/hello_usart.c
            COMMENT "Cross-compiling hello_usart.elf (Cortex-A53 AArch64)"
        )
        add_custom_target(cortex-a53-firmware ALL DEPENDS ${FW_OUT})

        set(FW_FD_OUT "${CMAKE_SOURCE_DIR}/platforms/cortex-a53-virt/hello_fullduplex.elf")
        add_custom_command(
            OUTPUT  ${FW_FD_OUT}
            COMMAND ${AARCH64_GCC}
                -nostdlib -static
                -Ttext=0x80000000 -e _start
                -Wl,--build-id=none -Wl,-n
                -o ${FW_FD_OUT}
                ${CMAKE_CURRENT_SOURCE_DIR}/src/hello_fullduplex.c
            DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/hello_fullduplex.c
            COMMENT "Cross-compiling hello_fullduplex.elf (Cortex-A53 AArch64)"
        )
        add_custom_target(cortex-a53-virt-fw-fullduplex ALL DEPENDS ${FW_FD_OUT})
    endif()
else()
    message(STATUS
        "baremetal/cortex-a53: aarch64-linux-gnu-gcc not found — "
        "skipping firmware cross-compilation.")
endif()
