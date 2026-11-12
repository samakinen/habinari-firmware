// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "device_default_name.h"

#include <stdio.h>
#include <string.h>

#include "device_config.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "dev_name";

void device_default_name(const uint8_t mac_tail[3], char *out, size_t out_len)
{
    snprintf(out, out_len, "%s %02X%02X%02X", CONFIG_HABINARI_DEVICE_NAME_PREFIX, mac_tail[0],
             mac_tail[1], mac_tail[2]);
}

void device_display_name(const uint8_t mac_tail[3], char *out, size_t out_len)
{
    /* A configured name wins, because in a corridor of identical boards the one
     * the commissioner labelled is the one they are looking for. */
    const device_config_item_t *item = device_config_find("dev.name");
    if (item != NULL) {
        char configured[DEVICE_CONFIG_STRING_MAX];
        const int written = device_config_get_text(item, configured, sizeof(configured));
        /* The item bounds itself well inside every caller's buffer, but the
         * registry is the only thing enforcing that and it is in another file.
         * Checking the length here means a later relaxation of that bound
         * produces a fallback name rather than a silently halved one. */
        if (written > 0 && (size_t)written < out_len && (size_t)written < sizeof(configured)) {
            memcpy(out, configured, (size_t)written + 1u);
            return;
        }
        if (written > 0) {
            ESP_LOGW(TAG, "Configured device name does not fit %u bytes; using the fallback",
                     (unsigned)out_len);
        }
    }
    device_default_name(mac_tail, out, out_len);
}
