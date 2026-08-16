// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "device_default_name.h"

#include <stdio.h>

#include "sdkconfig.h"

void device_default_name(const uint8_t mac_tail[3], char *out, size_t out_len)
{
    snprintf(out, out_len, "%s %02X%02X%02X", CONFIG_HABINARI_DEVICE_NAME_PREFIX, mac_tail[0],
             mac_tail[1], mac_tail[2]);
}
