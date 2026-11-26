# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Sami Mäkinen

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Measurement builds only: per-function stack frames (.su) and call graphs (.ci)
# next to every object, for tools/stack_report.py. Use a build directory of its
# own: `idf.py -B build_su -DHABINARI_STACK_USAGE=ON build`. Shipped images
# leave it off.
option(HABINARI_STACK_USAGE "Emit -fstack-usage / -fcallgraph-info=su output" OFF)
if(HABINARI_STACK_USAGE)
    idf_build_set_property(COMPILE_OPTIONS "-fstack-usage;-fcallgraph-info=su" APPEND)
endif()

project(habinari)

# The ETS product of a KNX image, from main/include/knx_product.hpp and this
# build's sdkconfig (KNstaX cmake/knx_ets_product.cmake): <build>/
# habinari_<medium>.knxprod.xml, .ets.xml (to pack on a PC with ETS) and .json,
# and <build>/dist/habinari_<medium>/ with the image and a manifest. The medium
# follows the image: TP1 and KNXnet/IP are two catalogue entries for one device
# model. product.lock records each one's application version; a product that
# changes without a new version fails the build. Skip it with
# `idf.py -DKNX_ETS_PRODUCT=OFF build`.
if(CONFIG_HABINARI_PROTOCOL_KNX)
    include("${CMAKE_SOURCE_DIR}/components/KNstaX/cmake/knx_ets_product.cmake")
    if(CONFIG_HABINARI_KNX_MEDIUM_IP)
        set(_habinari_medium ip)
        set(_habinari_medium_define CONFIG_HABINARI_KNX_MEDIUM_IP=1)
    else()
        set(_habinari_medium tp1)
        set(_habinari_medium_define CONFIG_HABINARI_KNX_MEDIUM_TP1=1)
    endif()
    knx_ets_product(habinari_${_habinari_medium}
        DEFINITION          "${CMAKE_SOURCE_DIR}/main/include/knx_product.hpp"
        SYMBOL              habinari_knx::kHabinariProduct
        # knx_product.hpp reads the medium from sdkconfig.h, which the host
        # build does not have.
        COMPILE_DEFINITIONS ${_habinari_medium_define}
        LOCK                "${CMAKE_SOURCE_DIR}/product.lock"
        DIST_DIR            "${CMAKE_BINARY_DIR}/dist"
        DIST_FILES          "${CMAKE_BINARY_DIR}/habinari.bin"
        DIST_DEPENDS        app
    )
endif()
