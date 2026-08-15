// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file device_secret.h
 * @brief C view of the per-device secrets in device_secret.hpp.
 *
 * The derivation, the entropy gate and the eFuse handling are C++ because they
 * are host-tested against RFC 4231 vectors and want spans and arrays. What C
 * code needs is one number, so that is all this exposes — the same hpp/h split
 * control_service uses, and for the same reason.
 */

/**
 * @brief The BLE commissioning passkey for this device.
 *
 * Derived on every call from the eFuse root secret through the HMAC
 * peripheral, so it is never stored anywhere it could be read: it is the same
 * root the KNX FDSK comes from, under a different domain-separation label.
 *
 * @param out_passkey    receives 0..999999.
 * @param out_from_efuse optional; false means no root secret is provisioned and
 *                       @p out_passkey is the fleet-wide development fallback.
 *
 * Returns ESP_ERR_INVALID_ARG for a null @p out_passkey and ESP_OK otherwise —
 * a device with no root secret is a configuration problem to report, not a
 * reason to leave the channel unpairable.
 */
esp_err_t device_secret_ble_passkey(uint32_t *out_passkey, bool *out_from_efuse);

#ifdef __cplusplus
}
#endif
