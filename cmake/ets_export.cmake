# Regenerate the ETS product export as part of the firmware build.
#
# tools/ets_export is a host-native CMake project: it compiles the product
# definition in main/include/knx_product.hpp against knx_core, runs the result
# to emit JSON, and feeds that to the KNstaX Python exporter to produce
# ets_export/sensor_board_tp1_ets.knxprod.xml.
#
# It cannot be an add_subdirectory() of the firmware build, because that build
# is cross-compiled for the ESP32-C6 and the exporter binary has to run here.
# So it is driven as a nested cmake invocation with the host compilers named
# explicitly.  The nested build has its own dependency tracking, so a firmware
# build where nothing KNX-related changed costs a no-op ninja run.

option(SENSOR_BOARD_ETS_EXPORT "Regenerate the ETS .knxprod.xml during the build" ON)

if(NOT SENSOR_BOARD_ETS_EXPORT)
    return()
endif()

set(ETS_EXPORT_SRC_DIR   "${CMAKE_SOURCE_DIR}/tools/ets_export")
set(ETS_EXPORT_BUILD_DIR "${CMAKE_BINARY_DIR}/ets_export")
set(ETS_EXPORT_OUT_DIR   "${CMAKE_SOURCE_DIR}/ets_export")

# Plain names only: the xtensa/riscv toolchain binaries are prefixed, so these
# resolve to the host compilers even inside the IDF build.
find_program(ETS_EXPORT_HOST_CC  NAMES cc gcc clang)
find_program(ETS_EXPORT_HOST_CXX NAMES c++ g++ clang++)

if(NOT ETS_EXPORT_HOST_CC OR NOT ETS_EXPORT_HOST_CXX)
    message(WARNING
        "ETS export disabled: no host C/C++ compiler found. "
        "ets_export/sensor_board_tp1_ets.knxprod.xml will not be refreshed.")
    return()
endif()

# The exporter itself is pure stdlib, so IDF's venv python is fine.
find_program(ETS_EXPORT_PYTHON NAMES python3 python)
if(NOT ETS_EXPORT_PYTHON)
    message(WARNING "ETS export disabled: no python3 found.")
    return()
endif()

# Configure once; the stamp is the nested cache, which idf.py fullclean removes
# along with the rest of the build tree.
add_custom_command(
    OUTPUT  "${ETS_EXPORT_BUILD_DIR}/CMakeCache.txt"
    COMMAND "${CMAKE_COMMAND}"
            -S "${ETS_EXPORT_SRC_DIR}"
            -B "${ETS_EXPORT_BUILD_DIR}"
            -G "${CMAKE_GENERATOR}"
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_C_COMPILER=${ETS_EXPORT_HOST_CC}
            -DCMAKE_CXX_COMPILER=${ETS_EXPORT_HOST_CXX}
            -DETS_EXPORT_OUTPUT_DIR=${ETS_EXPORT_OUT_DIR}
    DEPENDS "${ETS_EXPORT_SRC_DIR}/CMakeLists.txt"
    COMMENT "Configuring host ETS export build"
    VERBATIM
)

# Always invoked; the nested build decides whether anything actually needs doing.
# USES_TERMINAL puts it in ninja's console pool so the nested build does not
# fight the outer one for job slots.
add_custom_target(ets_export ALL
    COMMAND "${CMAKE_COMMAND}" --build "${ETS_EXPORT_BUILD_DIR}"
            --target sensor_board_tp1_ets_knxprod
    # The .knxprod.xml is written straight to ETS_EXPORT_OUTPUT_DIR; the JSON
    # intermediate stays in the nested build tree, so bring it alongside — it is
    # the readable form of the same data and the better thing to diff.
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${ETS_EXPORT_BUILD_DIR}/sensor_board_tp1_ets.json"
            "${ETS_EXPORT_OUT_DIR}/sensor_board_tp1_ets.json"
    DEPENDS "${ETS_EXPORT_BUILD_DIR}/CMakeCache.txt"
    COMMENT "Regenerating ETS product export -> ${ETS_EXPORT_OUT_DIR}"
    USES_TERMINAL
    VERBATIM
)
