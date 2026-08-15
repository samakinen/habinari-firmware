// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"
#include "sht4x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

typedef enum {
    EXT_PROBE_NOT_INITIALIZED = 0,
    EXT_PROBE_OPERATIONAL = 1,
    EXT_PROBE_INITIALIZATION_FAILED = 2,
    EXT_PROBE_POWER_OFF = 3,
    EXT_PROBE_NOT_CONNECTED = 4,
    EXT_PROBE_SHORT_CIRCUIT = 5
} ext_probe_status_t;

#define EXT_PROBE_STATUS_TO_TEXT(status) \
    ((status) == EXT_PROBE_NOT_INITIALIZED ? "Not Initialized" : \
    (status) == EXT_PROBE_OPERATIONAL ? "Operational" : \
    (status) == EXT_PROBE_INITIALIZATION_FAILED ? "Initialization Failed" : \
    (status) == EXT_PROBE_POWER_OFF ? "Power Off" : \
    (status) == EXT_PROBE_NOT_CONNECTED ? "Not Connected" : \
    (status) == EXT_PROBE_SHORT_CIRCUIT ? "Short Circuit" : "Unknown Status")

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t sht4x_device_handle;
    ext_probe_status_t status;
} ext_probe_t;

typedef ext_probe_t *ext_probe_handle_t;

typedef struct {
    float temperature;
    float humidity;
} ext_probe_results_t;

esp_err_t ext_probe_init(ext_probe_handle_t probe_handle);
esp_err_t ext_probe_read(ext_probe_handle_t probe_handle, ext_probe_results_t *results);
esp_err_t ext_probe_deinit(ext_probe_handle_t probe_handle);

#ifdef __cplusplus
}
#endif