// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file knx_ip_addr.h
 * @brief Parsing and checking the one address a KNXnet/IP installation sets.
 *
 * The routing multicast group is the single KNXnet/IP setting an installation
 * genuinely changes — two KNX projects sharing an IP network are told apart by
 * it — so it is writable in the field, which means it arrives as text from
 * whoever is holding the phone.
 *
 * A wrong value here is not a wrong value that shows up later: the device joins
 * the group at start-up, so a bad one is a device that boots and is never seen
 * again. Storing it is therefore gated on it being a group address the device
 * could actually join, and refusing a write is far kinder than accepting one
 * and disappearing on the next reboot.
 *
 * Plain C with no ESP-IDF and no KNX stack behind it, so it is host-tested in
 * main/test/test_knx_ip_addr.cpp rather than only on hardware.
 */

/**
 * @brief Parse a dotted-quad IPv4 address.
 *
 * Strict: exactly four decimal octets, each 0-255, no leading '+'/'-', no
 * whitespace, no partial forms ("10.1" is rejected rather than read as 10.0.0.1
 * the way inet_addr() would). Leading zeros are rejected too — "224.0.023.12"
 * is octal in some parsers and decimal in others, and a group address that
 * means two things is worse than one that is refused.
 *
 * @param text  NUL-terminated candidate; NULL is rejected.
 * @param out   receives the address as octets in network order (out[0] is the
 *              first octet written). Untouched on failure. May be NULL.
 * @return true when @p text is a well-formed IPv4 address.
 */
bool knx_ip_parse_ipv4(const char *text, uint8_t out[4]);

/**
 * @brief True for an address in 224.0.0.0/4, the IPv4 multicast range.
 *
 * That is the whole of what makes an address joinable as a group. Narrower
 * checks — insisting on 224.0.23.x, say — would reject the private ranges a
 * large site legitimately uses to separate installations.
 */
bool knx_ip_is_multicast(const uint8_t address[4]);

/**
 * @brief Convenience: parse @p text and check it names a multicast group.
 *
 * This is the predicate the config registry gates a write on.
 */
bool knx_ip_is_multicast_text(const char *text);

#ifdef __cplusplus
}
#endif
