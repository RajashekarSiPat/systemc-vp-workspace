cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(sc-vp-workspace VERSION 1.0)

# ---------------------------------------------------------------------------
# Paths to the already-built/installed QBOX framework.
# Run:   cmake --install qbox/build
# once before building this workspace.
# ---------------------------------------------------------------------------
set(QBOX_INSTALL "${CMAKE_CURRENT_SOURCE_DIR}/qbox/build/install"
    CACHE PATH "qbox install prefix (the result of cmake --install qbox/build)")

# ---------------------------------------------------------------------------
# Make all installed cmake packages findable via find_package.
# ---------------------------------------------------------------------------
list(PREPEND CMAKE_PREFIX_PATH "${QBOX_INSTALL}")

# Include qbox's boilerplate macros (gs_create_dymod, etc.) — read-only access.
include("${CMAKE_CURRENT_SOURCE_DIR}/qbox/cmake/boilerplate.cmake")

# ---------------------------------------------------------------------------
# Find packages that are properly installed in the qbox install tree.
# ---------------------------------------------------------------------------
find_package(Threads         REQUIRED)
find_package(SystemCLanguage REQUIRED)   # → SystemC::systemc
find_package(SystemCCCI      REQUIRED)   # → SystemC::cci-config, SystemC::cci
find_package(fmt             REQUIRED)   # → fmt::fmt  (scp::reporting dep)
find_package(reporting       REQUIRED)   # → scp::reporting
find_package(rpclib          REQUIRED)   # → rpclib::rpc
find_package(libqemu         REQUIRED)   # → libqemu (interface: QEMU .so loader)

# ---------------------------------------------------------------------------
# Create a minimal GreenSocs::lua imported target.
# The full qboxTargets.cmake requires Crow / pybind11 / GTest which are not
# installed; creating the targets manually avoids that chain of find_package.
# ---------------------------------------------------------------------------
if(NOT TARGET GreenSocs::lua)
    add_library(GreenSocs::lua SHARED IMPORTED GLOBAL)
    set_target_properties(GreenSocs::lua PROPERTIES
        IMPORTED_LOCATION "${QBOX_INSTALL}/lib/liblua.so"
    )
endif()

# ---------------------------------------------------------------------------
# Create a minimal GreenSocs::qbox imported target.
# Only exposes the dependencies that ARE installed; the rest (Crow, pybind11,
# gtest) are already baked into libqbox.so and do not need to be re-linked.
# ---------------------------------------------------------------------------
if(NOT TARGET GreenSocs::qbox)
    add_library(GreenSocs::qbox SHARED IMPORTED GLOBAL)
    set_target_properties(GreenSocs::qbox PROPERTIES
        IMPORTED_LOCATION "${QBOX_INSTALL}/lib/qbox-2.0/libqbox.so"
        INTERFACE_INCLUDE_DIRECTORIES
            # SystemC component headers (biflow-socket, module_factory_registery, …)
            "${QBOX_INSTALL}/systemc-components/common/include;\
${QBOX_INSTALL}/qemu-components/common/include;\
${QBOX_INSTALL}/include;\
${QBOX_INSTALL};\
${CMAKE_CURRENT_SOURCE_DIR}/qbox/build/_deps/lua-src"
        INTERFACE_LINK_LIBRARIES
            "SystemC::systemc;\
SystemC::cci-config;\
scp::reporting;\
GreenSocs::lua;\
libqemu;\
rpclib::rpc;\
Threads::Threads;\
${CMAKE_DL_LIBS}"
    )
endif()

# ---------------------------------------------------------------------------
# TARGET_LIBS — consumed by gs_create_dymod() in every peripheral.
# ---------------------------------------------------------------------------
set(TARGET_LIBS
    SystemC::systemc
    SystemC::cci-config
    scp::reporting
    GreenSocs::lua
    libqemu
    GreenSocs::qbox
    CACHE INTERNAL "qbox dependency targets for use in gs_create_dymod()")

# Make all installed .so files visible on the link RPATH / library search path.
list(APPEND CMAKE_BUILD_RPATH
    "${QBOX_INSTALL}/lib"
    "${QBOX_INSTALL}/lib/qbox-2.0"
    "${QBOX_INSTALL}/lib/libqemu"
)

# ---------------------------------------------------------------------------
# Reusable SystemC IP blocks
# Convention: peripherals/<name>/  with include/, src/, CMakeLists.txt
# ---------------------------------------------------------------------------
add_subdirectory(peripherals)

# ---------------------------------------------------------------------------
# Platform-specific virtual-platform integrations
# Convention: platforms/<name>/  with src/main.cc, conf.lua, CMakeLists.txt
# ---------------------------------------------------------------------------
add_subdirectory(platforms)

# ---------------------------------------------------------------------------
# Baremetal firmware (cross-compiled with aarch64-linux-gnu-gcc)
# ---------------------------------------------------------------------------
add_subdirectory(baremetal)
