# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Sami Mäkinen

cmake_minimum_required(VERSION 3.21)

project(habinari_ets_export LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

# Where the finished .knxprod.xml / .json land.  Defaults to the build tree so a
# standalone `cmake -S tools/ets_export -B <dir>` keeps everything in one place;
# the firmware build overrides it to <repo>/ets_export so the artefacts survive
# an `idf.py fullclean`.
set(ETS_EXPORT_OUTPUT_DIR "${CMAKE_BINARY_DIR}" CACHE PATH
    "Directory receiving the generated ETS product export")

set(KNX_PLATFORM linux CACHE STRING "Host platform for KNstaX export build" FORCE)
set(KNX_BUILD_EXAMPLES OFF CACHE BOOL "Disable KNstaX examples for export build" FORCE)
set(KNX_SECURE_ENABLED OFF CACHE BOOL "Disable KNX secure for export-only build" FORCE)
set(KNX_ZLIB_ENABLED OFF CACHE BOOL "Disable zlib for export-only build" FORCE)

add_subdirectory("${REPO_ROOT}/components/KNstaX" "${CMAKE_BINARY_DIR}/knstax")

include("${REPO_ROOT}/components/KNstaX/cmake/knx_product.cmake")

# Which medium's catalogue entry to generate. The device model is identical on
# both; the medium decides the product key, the order number and what ETS will
# let the device be connected to (see knx_product.hpp). The firmware build
# passes whichever medium it is building, so the exported product always
# matches the image next to it.
set(HABINARI_ETS_MEDIUM "tp1" CACHE STRING "KNX medium for the exported product: tp1 or ip")
set_property(CACHE HABINARI_ETS_MEDIUM PROPERTY STRINGS tp1 ip)

if(HABINARI_ETS_MEDIUM STREQUAL "ip")
    set(HABINARI_ETS_TARGET habinari_ip_ets)
    set(HABINARI_ETS_DEFINES CONFIG_HABINARI_KNX_MEDIUM_IP=1)
elseif(HABINARI_ETS_MEDIUM STREQUAL "tp1")
    set(HABINARI_ETS_TARGET habinari_tp1_ets)
    set(HABINARI_ETS_DEFINES CONFIG_HABINARI_KNX_MEDIUM_TP1=1)
else()
    message(FATAL_ERROR "HABINARI_ETS_MEDIUM must be 'tp1' or 'ip', got '${HABINARI_ETS_MEDIUM}'")
endif()

knx_commissioned_product(${HABINARI_ETS_TARGET}
    DEFINITION  "${REPO_ROOT}/main/include/knx_product.hpp"
    SYMBOL      habinari_knx::kHabinariProduct
    OUTPUT_DIR  "${ETS_EXPORT_OUTPUT_DIR}"
)

# knx_commissioned_product() builds a host binary named <target>_knxprod_gen
# that prints the product as JSON; the medium symbol has to reach that
# compilation, not the custom target that runs it.
target_compile_definitions(${HABINARI_ETS_TARGET}_knxprod_gen PRIVATE ${HABINARI_ETS_DEFINES})