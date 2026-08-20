// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * The KNXnet/IP routing group, as an installer types it.
 *
 * This is the one KNXnet/IP setting that is written in the field, and the
 * consequence of accepting a bad one is specific: the device joins the group at
 * start-up, so a value that is not a joinable group produces a board that
 * reboots and is never seen on the bus again. Refusing the write is the only
 * point at which that is still recoverable without a cable.
 */

#include "unity.h"

extern "C" {
#include "knx_ip_addr.h"
}

#include <cstdint>

void setUp(void) {}
void tearDown(void) {}

void test_parses_a_dotted_quad(void)
{
    uint8_t octets[4] = {0};
    TEST_ASSERT_TRUE(knx_ip_parse_ipv4("224.0.23.12", octets));
    TEST_ASSERT_EQUAL_UINT8(224, octets[0]);
    TEST_ASSERT_EQUAL_UINT8(0, octets[1]);
    TEST_ASSERT_EQUAL_UINT8(23, octets[2]);
    TEST_ASSERT_EQUAL_UINT8(12, octets[3]);
}

void test_accepts_the_range_boundaries(void)
{
    uint8_t octets[4] = {0};
    TEST_ASSERT_TRUE(knx_ip_parse_ipv4("0.0.0.0", octets));
    TEST_ASSERT_TRUE(knx_ip_parse_ipv4("255.255.255.255", octets));
    TEST_ASSERT_EQUAL_UINT8(255, octets[3]);
}

void test_rejects_out_of_range_octets(void)
{
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("256.0.23.12", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("224.0.23.256", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("999.999.999.999", nullptr));
}

void test_rejects_partial_and_overlong_forms(void)
{
    // inet_addr() reads "10.1" as 10.0.0.1. A group address is not somewhere to
    // be that helpful.
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("224.0.23", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("224.0.23.12.1", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("224", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4(nullptr, nullptr));
}

void test_rejects_leading_zeros(void)
{
    // Octal to some parsers, decimal to others. An address that means two
    // things is worse than one that is refused.
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("224.0.023.12", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("0224.0.23.12", nullptr));
    // A bare zero octet is not a leading zero.
    TEST_ASSERT_TRUE(knx_ip_parse_ipv4("224.0.23.12", nullptr));
}

void test_rejects_stray_characters(void)
{
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4(" 224.0.23.12", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("224.0.23.12 ", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("224.0.23.12:3671", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("224.0.23.12/4", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("+224.0.23.12", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("-1.0.23.12", nullptr));
    TEST_ASSERT_FALSE(knx_ip_parse_ipv4("224..23.12", nullptr));
}

void test_multicast_range_is_224_to_239(void)
{
    // The KNX System Setup address, and the default for every installation.
    TEST_ASSERT_TRUE(knx_ip_is_multicast_text("224.0.23.12"));
    // Boundaries of 224.0.0.0/4.
    TEST_ASSERT_TRUE(knx_ip_is_multicast_text("224.0.0.0"));
    TEST_ASSERT_TRUE(knx_ip_is_multicast_text("239.255.255.255"));
    // A large site separating installations inside the private multicast
    // range is doing something legitimate, so nothing narrower than /4.
    TEST_ASSERT_TRUE(knx_ip_is_multicast_text("239.192.1.5"));
}

void test_unicast_addresses_are_not_groups(void)
{
    // The commonest mistake: typing the device's own address, or the router's.
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text("192.168.1.50"));
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text("10.0.0.1"));
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text("223.255.255.255"));
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text("240.0.0.1"));
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text("255.255.255.255"));
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text("0.0.0.0"));
}

void test_malformed_text_is_not_a_group(void)
{
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text(""));
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text(nullptr));
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text("multicast"));
    TEST_ASSERT_FALSE(knx_ip_is_multicast_text("224.0.23"));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_parses_a_dotted_quad);
    RUN_TEST(test_accepts_the_range_boundaries);
    RUN_TEST(test_rejects_out_of_range_octets);
    RUN_TEST(test_rejects_partial_and_overlong_forms);
    RUN_TEST(test_rejects_leading_zeros);
    RUN_TEST(test_rejects_stray_characters);
    RUN_TEST(test_multicast_range_is_224_to_239);
    RUN_TEST(test_unicast_addresses_are_not_groups);
    RUN_TEST(test_malformed_text_is_not_a_group);
    return UNITY_END();
}
