// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file device_default_name.h
 * @brief The one hardcoded literal a channel is allowed to fall back on.
 *
 * Every channel that shows this device a name (the BLE advertisement, the
 * Home Assistant device card) prefers the installer-configured `dev.name`
 * (see device_config.h) and falls back to this when none is set. The literal
 * lives once, in CONFIG_HABINARI_DEVICE_NAME_PREFIX, instead of once per
 * channel.
 */

/**
 * @brief Render "<CONFIG_HABINARI_DEVICE_NAME_PREFIX> XXYYZZ" from the last
 * three bytes of a MAC address, truncating to fit if @p out_len is smaller
 * than the result — which a long configured prefix can do to the 24-byte BLE
 * advertisement buffer. Keep the prefix short if it needs to survive there.
 */
void device_default_name(const uint8_t mac_tail[3], char *out, size_t out_len);

/**
 * @brief The name a channel shows: the configured `dev.name` when it is set and
 * fits in @p out_len, otherwise device_default_name(). The BLE advertisement
 * and the Home Assistant device card both use it, so they agree on a name
 * until somebody sets one.
 */
void device_display_name(const uint8_t mac_tail[3], char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
