# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Sami Mäkinen


idf_component_register(SRCS "scd4x.c"
                       INCLUDE_DIRS "include"
                       REQUIRES "esp_driver_i2c")
