// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "knx_ip_addr.h"

#include <stddef.h>

bool knx_ip_parse_ipv4(const char *text, uint8_t out[4])
{
    if (text == NULL) {
        return false;
    }

    uint8_t octets[4] = {0};
    const char *p = text;

    for (int index = 0; index < 4; ++index) {
        if (index > 0) {
            if (*p != '.') {
                return false;
            }
            ++p;
        }

        if (*p < '0' || *p > '9') {
            return false;
        }

        /* Leading zeros are refused rather than interpreted: "023" is octal to
         * some parsers and decimal to others, and an address that means two
         * different things is worse than one that is rejected outright. */
        if (*p == '0' && p[1] >= '0' && p[1] <= '9') {
            return false;
        }

        unsigned value = 0;
        int digits = 0;
        while (*p >= '0' && *p <= '9') {
            value = (value * 10u) + (unsigned)(*p - '0');
            if (value > 255u) {
                return false;
            }
            ++p;
            if (++digits > 3) {
                return false;
            }
        }
        octets[index] = (uint8_t)value;
    }

    /* Anything after the fourth octet — a port, a mask, a stray space — means
     * this is not the address it looks like. */
    if (*p != '\0') {
        return false;
    }

    if (out != NULL) {
        for (int i = 0; i < 4; ++i) {
            out[i] = octets[i];
        }
    }
    return true;
}

bool knx_ip_is_multicast(const uint8_t address[4])
{
    if (address == NULL) {
        return false;
    }
    return address[0] >= 224u && address[0] <= 239u;
}

bool knx_ip_is_multicast_text(const char *text)
{
    uint8_t octets[4] = {0};
    if (!knx_ip_parse_ipv4(text, octets)) {
        return false;
    }
    return knx_ip_is_multicast(octets);
}
