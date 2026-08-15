// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

/*
 * Host stub for ESP-IDF's esp_err.h.
 *
 * control_state.h — the contract every protocol adapter maps onto — returns
 * esp_err_t, which is the only thing standing between the adapter mapping code
 * and a host test. It is an int and five constants, so this is the whole of it.
 *
 * Kept deliberately minimal: if a test needs more of ESP-IDF than this, the code
 * under test has a platform dependency it should not have, and the fix is in the
 * code, not here.
 */

#include <stdint.h>

typedef int esp_err_t;

#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_NOT_SUPPORTED 0x106
