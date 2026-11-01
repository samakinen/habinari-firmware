// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * Host tests for the small pure pieces the network and KNX services lean on.
 *
 * The services themselves need a radio, a bus and FreeRTOS. These pieces do
 * not, and each was a bug that only showed up on site: a 32-character SSID
 * the device could never join, and a neighbour's "invalid" DPT 9 telegram
 * taken as a CO2 setpoint of 670 760 ppm.
 */

#include "knx_dpt9.hpp"
#include "wifi_credentials.h"

#include "knx/application/dpt.hpp"

#include <unity.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

namespace {

// ---------------------------------------------------------------------------
// Wi-Fi credentials
// ---------------------------------------------------------------------------

void test_a_32_character_ssid_is_copied_whole()
{
    // wifi_sta_config_t::ssid is 32 bytes and needs no terminator.
    const std::string ssid(32, 'S');
    std::array<uint8_t, 32> field{};
    field.fill(0xAA);
    TEST_ASSERT_TRUE(wifi_credential_copy(field.data(), field.size(), ssid.c_str()));
    TEST_ASSERT_EQUAL_MEMORY(ssid.data(), field.data(), field.size());
}

void test_a_short_credential_is_zero_padded()
{
    std::array<uint8_t, 32> field{};
    field.fill(0xAA);
    TEST_ASSERT_TRUE(wifi_credential_copy(field.data(), field.size(), "home"));
    TEST_ASSERT_EQUAL_MEMORY("home", field.data(), 4);
    for (size_t i = 4; i < field.size(); ++i) {
        TEST_ASSERT_EQUAL_UINT8(0, field[i]);
    }
}

void test_a_credential_too_long_for_the_field_is_refused_not_cut()
{
    // Cutting it gives a different network name, not a shorter one.
    const std::string ssid(33, 'S');
    std::array<uint8_t, 32> field{};
    field.fill(0xAA);
    TEST_ASSERT_FALSE(wifi_credential_copy(field.data(), field.size(), ssid.c_str()));
    for (const uint8_t byte : field) {
        TEST_ASSERT_EQUAL_UINT8(0, byte);
    }

    // The 64-byte password field: 63 printable characters is the WPA2 limit
    // and fits; 64 fits the field too.
    std::array<uint8_t, 64> password{};
    const std::string fits(64, 'p');
    const std::string tooLong(65, 'p');
    TEST_ASSERT_TRUE(wifi_credential_copy(password.data(), password.size(), fits.c_str()));
    TEST_ASSERT_FALSE(wifi_credential_copy(password.data(), password.size(), tooLong.c_str()));
}

void test_an_empty_or_missing_credential_is_an_empty_field()
{
    std::array<uint8_t, 8> field{};
    field.fill(0xAA);
    TEST_ASSERT_TRUE(wifi_credential_copy(field.data(), field.size(), ""));
    for (const uint8_t byte : field) {
        TEST_ASSERT_EQUAL_UINT8(0, byte);
    }
    field.fill(0xAA);
    TEST_ASSERT_TRUE(wifi_credential_copy(field.data(), field.size(), nullptr));
    for (const uint8_t byte : field) {
        TEST_ASSERT_EQUAL_UINT8(0, byte);
    }
}

// ---------------------------------------------------------------------------
// DPT 9 invalid data
// ---------------------------------------------------------------------------

float decodeDpt9(uint8_t high, uint8_t low)
{
    // The stack's own codec, the one every inbound group write goes through,
    // so this tests what the adapter will actually be handed.
    const std::array<uint8_t, 2> raw{high, low};
    float value = 0.0f;
    TEST_ASSERT_TRUE(knx::application::Dpt9::decode(raw, value).isOk());
    return value;
}

void test_dpt9_invalid_code_is_recognised_as_decoded_by_the_stack()
{
    const float invalid = decodeDpt9(0x7F, 0xFF);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, habinari_knx::kDpt9InvalidDecoded, invalid);
    TEST_ASSERT_TRUE(habinari_knx::dpt9IsInvalid(invalid));
}

void test_the_top_of_the_dpt9_range_is_still_a_value()
{
    // 7FFEh, the largest valid code: 670 433.28.
    const float top = decodeDpt9(0x7F, 0xFE);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, habinari_knx::kDpt9MaxValid, top);
    TEST_ASSERT_FALSE(habinari_knx::dpt9IsInvalid(top));

    // And everything an HVAC input plausibly carries.
    const float ordinary[] = {-273.0f, -40.0f, 0.0f, 21.5f, 800.0f, 5000.0f, -671088.64f};
    for (const float value : ordinary) {
        TEST_ASSERT_FALSE(habinari_knx::dpt9IsInvalid(value));
    }
    // Every other code decodes to a value, including the most negative one.
    for (uint32_t code = 0; code <= 0xFFFFu; ++code) {
        if (code == 0x7FFFu) {
            continue;
        }
        const float value = decodeDpt9(static_cast<uint8_t>(code >> 8), static_cast<uint8_t>(code));
        if (habinari_knx::dpt9IsInvalid(value)) {
            char message[64];
            std::snprintf(message, sizeof(message), "code %04Xh taken as invalid", code);
            TEST_FAIL_MESSAGE(message);
        }
    }
}

void test_nan_is_never_taken_as_a_value()
{
    TEST_ASSERT_TRUE(habinari_knx::dpt9IsInvalid(std::numeric_limits<float>::quiet_NaN()));
    TEST_ASSERT_TRUE(habinari_knx::dpt9IsInvalid(std::numeric_limits<float>::infinity()));
}

}  // namespace

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_a_32_character_ssid_is_copied_whole);
    RUN_TEST(test_a_short_credential_is_zero_padded);
    RUN_TEST(test_a_credential_too_long_for_the_field_is_refused_not_cut);
    RUN_TEST(test_an_empty_or_missing_credential_is_an_empty_field);

    RUN_TEST(test_dpt9_invalid_code_is_recognised_as_decoded_by_the_stack);
    RUN_TEST(test_the_top_of_the_dpt9_range_is_still_a_value);
    RUN_TEST(test_nan_is_never_taken_as_a_value);

    return UNITY_END();
}
