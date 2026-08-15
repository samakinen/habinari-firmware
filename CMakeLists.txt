# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Sami Mäkinen

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(habinari)

# Keep the ETS product export in sync with main/include/knx_product.hpp.
# Disable with `idf.py -DHABINARI_ETS_EXPORT=OFF build`.
include("${CMAKE_SOURCE_DIR}/cmake/ets_export.cmake")
