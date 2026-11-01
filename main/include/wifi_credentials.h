// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file wifi_credentials.h
 * @brief Copying a stored credential into a wifi_sta_config_t field.
 *
 * Apart from wifi_service.c only so the host tests can reach it: the station
 * config is the one place in the firmware where a string is not terminated.
 * Its ssid field is 32 bytes and holds a 32-character SSID with no NUL, as
 * 802.11 allows; copying with the usual "size minus one for the terminator"
 * cut the last character off, and a device given such a network could never
 * join it. The password field works the same way at 64 bytes.
 */

/**
 * Copy @p src into the fixed field @p dst of @p dst_len bytes, unterminated if
 * it fills the field exactly, zero-padded otherwise.
 *
 * @return false, leaving @p dst all zeros, if @p src is longer than the field:
 *         a credential cut short is a different credential, and joining the
 *         wrong network, or failing to authenticate for no visible reason, is
 *         worse than saying it does not fit.
 */
static inline bool wifi_credential_copy(uint8_t *dst, size_t dst_len, const char *src)
{
    memset(dst, 0, dst_len);
    const size_t len = (src != NULL) ? strlen(src) : 0;
    if (len > dst_len) {
        return false;
    }
    if (len > 0) {
        memcpy(dst, src, len);
    }
    return true;
}

#ifdef __cplusplus
}
#endif
