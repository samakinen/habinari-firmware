cmake_minimum_required(VERSION 3.21)

project(sensor_board_ets_export LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

set(KNX_PLATFORM linux CACHE STRING "Host platform for KNstaX export build" FORCE)
set(KNX_BUILD_EXAMPLES OFF CACHE BOOL "Disable KNstaX examples for export build" FORCE)
set(KNX_SECURE_ENABLED OFF CACHE BOOL "Disable KNX secure for export-only build" FORCE)
set(KNX_ZLIB_ENABLED OFF CACHE BOOL "Disable zlib for export-only build" FORCE)

add_subdirectory("${REPO_ROOT}/components/KNstaX" "${CMAKE_BINARY_DIR}/knstax")

include("${REPO_ROOT}/components/KNstaX/cmake/knx_product.cmake")

knx_commissioned_product(sensor_board_tp1_ets
    DEFINITION  "${REPO_ROOT}/main/include/knx_product.hpp"
    SYMBOL      sensor_board_knx::kSensorBoardProduct
    OUTPUT_DIR  "${CMAKE_BINARY_DIR}"
)