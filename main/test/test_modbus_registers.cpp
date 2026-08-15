// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// Host tests for the Modbus register map in main/include/modbus_registers.h.
//
// Two things are worth testing without hardware, and they are exactly the two
// that break integrations: the scaling (including the "no reading" sentinels,
// which only exist because the redundancy layer can genuinely report a
// measurement as unavailable) and the register addresses, which are the public
// contract documented in docs/modbus-register-map.md.

#include "modbus_registers.h"

#include "unity.h"

#include <cstddef>
#include <cstdint>

namespace {

// The register address of a field is its word offset in the block, so these
// assertions are the same statement the documentation makes.
constexpr uint16_t inputAddress(size_t byteOffset)
{
    return static_cast<uint16_t>(byteOffset / 2);
}

} // namespace

// --- Scaling ---------------------------------------------------------------

void test_signed_encoding_round_trips(void)
{
    TEST_ASSERT_EQUAL_INT16(215, mb_encode_signed(21.5f, true, MB_SCALE_TEMPERATURE));
    TEST_ASSERT_EQUAL_INT16(-153, mb_encode_signed(-15.3f, true, MB_SCALE_TEMPERATURE));
    TEST_ASSERT_EQUAL_INT16(0, mb_encode_signed(0.0f, true, MB_SCALE_TEMPERATURE));

    float decoded = 0.0f;
    TEST_ASSERT_TRUE(mb_decode_signed(215, MB_SCALE_TEMPERATURE, &decoded));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 21.5f, decoded);
    TEST_ASSERT_TRUE(mb_decode_signed(-153, MB_SCALE_TEMPERATURE, &decoded));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -15.3f, decoded);
}

void test_signed_encoding_rounds_to_nearest(void)
{
    // Truncation would bias every reading downwards by up to 0.1 K, which is
    // half the default KNX send-on-change threshold.
    TEST_ASSERT_EQUAL_INT16(216, mb_encode_signed(21.55f, true, MB_SCALE_TEMPERATURE));
    TEST_ASSERT_EQUAL_INT16(-216, mb_encode_signed(-21.55f, true, MB_SCALE_TEMPERATURE));
}

void test_invalid_readings_use_the_sentinel(void)
{
    // A dead sensor must not look like 0.0 °C.
    TEST_ASSERT_EQUAL_INT16(MB_INVALID_SIGNED,
                            mb_encode_signed(0.0f, false, MB_SCALE_TEMPERATURE));
    TEST_ASSERT_EQUAL_UINT16(MB_INVALID_UNSIGNED, mb_encode_unsigned(0.0f, false, 1));

    float decoded = 42.0f;
    TEST_ASSERT_FALSE(mb_decode_signed(MB_INVALID_SIGNED, MB_SCALE_TEMPERATURE, &decoded));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 42.0f, decoded);  // left untouched
}

void test_saturation_never_produces_the_sentinel(void)
{
    // Otherwise an out-of-range reading would be indistinguishable from a
    // missing one.
    TEST_ASSERT_EQUAL_INT16(32767, mb_encode_signed(1.0e6f, true, MB_SCALE_TEMPERATURE));
    TEST_ASSERT_EQUAL_INT16(-32767, mb_encode_signed(-1.0e6f, true, MB_SCALE_TEMPERATURE));
    TEST_ASSERT_NOT_EQUAL(MB_INVALID_SIGNED, mb_encode_signed(-1.0e6f, true, 10));
    TEST_ASSERT_EQUAL_UINT16(65534, mb_encode_unsigned(1.0e6f, true, 1));
    TEST_ASSERT_NOT_EQUAL(MB_INVALID_UNSIGNED, mb_encode_unsigned(1.0e6f, true, 1));
}

void test_negative_values_are_invalid_on_unsigned_registers(void)
{
    // A negative CO2 or pressure is a fault, and wrapping it into a huge
    // unsigned number would hand the master a plausible-looking lie.
    TEST_ASSERT_EQUAL_UINT16(MB_INVALID_UNSIGNED, mb_encode_unsigned(-1.0f, true, 1));
}

void test_pressure_scaling_fits_a_single_register(void)
{
    // 101300 Pa does not fit in 16 bits, which is why the map carries 0.1 hPa.
    TEST_ASSERT_EQUAL_UINT16(10130, mb_encode_unsigned(101300.0f / 100.0f, true, MB_SCALE_PRESSURE));
    // The full standard-atmosphere range stays inside the register.
    TEST_ASSERT_EQUAL_UINT16(10133, mb_encode_unsigned(101325.0f / 100.0f, true, MB_SCALE_PRESSURE));
    TEST_ASSERT_EQUAL_UINT16(8700, mb_encode_unsigned(87000.0f / 100.0f, true, MB_SCALE_PRESSURE));
}

void test_baud_codes_map_to_line_rates(void)
{
    TEST_ASSERT_EQUAL_UINT32(9600, mb_baud_from_code(MB_BAUD_9600));
    TEST_ASSERT_EQUAL_UINT32(19200, mb_baud_from_code(MB_BAUD_19200));
    TEST_ASSERT_EQUAL_UINT32(115200, mb_baud_from_code(MB_BAUD_115200));
    // An unknown code reports 0 so the caller keeps the rate it already has,
    // rather than dropping to a default and going silent on the line.
    TEST_ASSERT_EQUAL_UINT32(0, mb_baud_from_code(999));
}

// --- Bit areas -------------------------------------------------------------

void test_bit_accessors_address_the_right_bit(void)
{
    uint8_t bits[MB_DISCRETE_BYTES] = {0};

    mb_bit_set(bits, MB_DISCRETE_FIRE_ALARM, true);
    TEST_ASSERT_TRUE(mb_bit_get(bits, MB_DISCRETE_FIRE_ALARM));
    TEST_ASSERT_FALSE(mb_bit_get(bits, MB_DISCRETE_DEVICE_FAULT));

    mb_bit_set(bits, MB_DISCRETE_FIRE_ALARM, false);
    TEST_ASSERT_FALSE(mb_bit_get(bits, MB_DISCRETE_FIRE_ALARM));

    // Every discrete input must be individually addressable within the
    // allocated bytes — an off-by-one here corrupts a neighbouring flag.
    for (unsigned i = 0; i < MB_DISCRETE_COUNT; ++i) {
        mb_bit_set(bits, i, true);
    }
    for (unsigned i = 0; i < MB_DISCRETE_COUNT; ++i) {
        TEST_ASSERT_TRUE(mb_bit_get(bits, i));
    }
}

// --- Addresses -------------------------------------------------------------

void test_input_register_addresses_match_the_documented_map(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, inputAddress(offsetof(mb_input_registers_t, device_id)));
    TEST_ASSERT_EQUAL_UINT16(5, inputAddress(offsetof(mb_input_registers_t, sensor_health_mask)));
    TEST_ASSERT_EQUAL_UINT16(10, inputAddress(offsetof(mb_input_registers_t, room_temperature)));
    TEST_ASSERT_EQUAL_UINT16(11, inputAddress(offsetof(mb_input_registers_t, room_humidity)));
    TEST_ASSERT_EQUAL_UINT16(14, inputAddress(offsetof(mb_input_registers_t, co2)));
    TEST_ASSERT_EQUAL_UINT16(20, inputAddress(offsetof(mb_input_registers_t, room_dew_point)));
    TEST_ASSERT_EQUAL_UINT16(26,
                             inputAddress(offsetof(mb_input_registers_t, temperature_trend)));
    TEST_ASSERT_EQUAL_UINT16(30, inputAddress(offsetof(mb_input_registers_t, active_setpoint)));
    TEST_ASSERT_EQUAL_UINT16(34, inputAddress(offsetof(mb_input_registers_t, heating_percent)));
    TEST_ASSERT_EQUAL_UINT16(45,
                             inputAddress(offsetof(mb_input_registers_t, temperature_source)));
    TEST_ASSERT_EQUAL_UINT16(48, inputAddress(offsetof(mb_input_registers_t, serial_0)));
}

void test_holding_register_addresses_match_the_documented_map(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, inputAddress(offsetof(mb_holding_registers_t, slave_address)));
    TEST_ASSERT_EQUAL_UINT16(1, inputAddress(offsetof(mb_holding_registers_t, baud_rate_code)));
    TEST_ASSERT_EQUAL_UINT16(2, inputAddress(offsetof(mb_holding_registers_t, config_commit)));
    TEST_ASSERT_EQUAL_UINT16(4, inputAddress(offsetof(mb_holding_registers_t, comfort_setpoint)));
    TEST_ASSERT_EQUAL_UINT16(9, inputAddress(offsetof(mb_holding_registers_t, co2_setpoint)));
    TEST_ASSERT_EQUAL_UINT16(12, inputAddress(offsetof(mb_holding_registers_t, serial_select_0)));
}

void test_register_blocks_have_no_padding(void)
{
    // A hole would shift every address after it, so the map would silently stop
    // matching the documentation on a compiler with different packing rules.
    TEST_ASSERT_EQUAL_UINT32(52 * 2, sizeof(mb_input_registers_t));
    TEST_ASSERT_EQUAL_UINT32(16 * 2, sizeof(mb_holding_registers_t));
}

// --- Commissioning ---------------------------------------------------------
//
// Modbus standardises no addressing mechanism, so this convention is the
// product's own and every part of it has to be pinned. What is being defended
// is one property above all: two devices must never answer the same request.

const uint8_t kSerialA[6] = {0x84, 0xF7, 0x03, 0xA1, 0xB2, 0xC3};
const uint8_t kSerialB[6] = {0x84, 0xF7, 0x03, 0x11, 0x22, 0x33};

mb_commissioning_state_t resolve(uint8_t assigned, bool programming, bool selected)
{
    mb_commissioning_inputs_t in;
    in.assigned_address = assigned;
    in.programming_mode = programming;
    in.serial_selected = selected;
    return mb_commissioning_resolve(in);
}

void test_a_fresh_device_is_silent(void)
{
    const mb_commissioning_state_t out = resolve(MB_SLAVE_ADDR_UNASSIGNED, false, false);

    // The whole point: a shelf of factory-fresh boards on one pair. None of them
    // transmits, so none of them can collide, and the bus stays usable.
    TEST_ASSERT_FALSE(out.answers_requests);
    TEST_ASSERT_EQUAL_UINT8(MB_SLAVE_ADDR_UNASSIGNED, out.listen_address);
    // Silent is not deaf: address 0 is the broadcast address, which is how the
    // serial-select frame still reaches it.
    TEST_ASSERT_FALSE(out.accept_line_settings);
}

void test_the_button_brings_an_unassigned_device_up_on_the_commissioning_address(void)
{
    const mb_commissioning_state_t out = resolve(MB_SLAVE_ADDR_UNASSIGNED, true, false);

    TEST_ASSERT_TRUE(out.answers_requests);
    TEST_ASSERT_EQUAL_UINT8(MB_SLAVE_ADDR_COMMISSIONING, out.listen_address);
    TEST_ASSERT_TRUE(out.accept_line_settings);
}

void test_an_addressed_device_keeps_its_address_while_selected(void)
{
    // Moving it to the commissioning address would drop it off a live bus for as
    // long as somebody leant on the button.
    const mb_commissioning_state_t out = resolve(17, true, false);

    TEST_ASSERT_TRUE(out.answers_requests);
    TEST_ASSERT_EQUAL_UINT8(17, out.listen_address);
    TEST_ASSERT_TRUE(out.accept_line_settings);
}

void test_line_settings_are_refused_unless_selected(void)
{
    // Reachable is not the same as selected: a stray master write must not be
    // able to re-address a device in a running building.
    TEST_ASSERT_FALSE(resolve(17, false, false).accept_line_settings);

    // Either selector is enough. The serial is the one that works with nobody
    // at the device.
    TEST_ASSERT_TRUE(resolve(17, false, true).accept_line_settings);
    TEST_ASSERT_TRUE(resolve(17, true, false).accept_line_settings);
}

void test_serial_selection_names_exactly_one_device(void)
{
    uint16_t selection[3];
    mb_serial_encode(kSerialA, selection);

    TEST_ASSERT_TRUE(mb_serial_selected(selection, kSerialA));
    TEST_ASSERT_FALSE(mb_serial_selected(selection, kSerialB));

    // Big-endian pairs, the same encoding the input registers publish, so a
    // master can read a serial off one device and select another with it.
    TEST_ASSERT_EQUAL_UINT16(0x84F7, selection[0]);
    TEST_ASSERT_EQUAL_UINT16(0x03A1, selection[1]);
    TEST_ASSERT_EQUAL_UINT16(0xB2C3, selection[2]);
}

void test_clearing_the_selection_releases_rather_than_matches(void)
{
    // All-zero has to mean "nobody", not "the device with serial 00:00:...".
    // Treating it as a match would leave every device selected after a clear.
    const uint16_t cleared[3] = {0, 0, 0};
    const uint8_t zeroSerial[6] = {0, 0, 0, 0, 0, 0};

    TEST_ASSERT_FALSE(mb_serial_selected(cleared, kSerialA));
    TEST_ASSERT_FALSE(mb_serial_selected(cleared, zeroSerial));
}

void test_the_commissioning_address_is_never_assignable(void)
{
    // Otherwise a legitimately addressed device at 247 would answer alongside
    // any unassigned board somebody had just put into programming mode.
    TEST_ASSERT_FALSE(mb_address_assignable(MB_SLAVE_ADDR_COMMISSIONING));
    TEST_ASSERT_FALSE(mb_address_assignable(MB_SLAVE_ADDR_UNASSIGNED));
    TEST_ASSERT_TRUE(mb_address_assignable(MB_SLAVE_ADDR_MIN));
    TEST_ASSERT_TRUE(mb_address_assignable(MB_SLAVE_ADDR_MAX));
    TEST_ASSERT_FALSE(mb_address_assignable(MB_SLAVE_ADDR_MAX + 1));
}

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_signed_encoding_round_trips);
    RUN_TEST(test_signed_encoding_rounds_to_nearest);
    RUN_TEST(test_invalid_readings_use_the_sentinel);
    RUN_TEST(test_saturation_never_produces_the_sentinel);
    RUN_TEST(test_negative_values_are_invalid_on_unsigned_registers);
    RUN_TEST(test_pressure_scaling_fits_a_single_register);
    RUN_TEST(test_baud_codes_map_to_line_rates);

    RUN_TEST(test_bit_accessors_address_the_right_bit);

    RUN_TEST(test_input_register_addresses_match_the_documented_map);
    RUN_TEST(test_holding_register_addresses_match_the_documented_map);
    RUN_TEST(test_register_blocks_have_no_padding);

    RUN_TEST(test_a_fresh_device_is_silent);
    RUN_TEST(test_the_button_brings_an_unassigned_device_up_on_the_commissioning_address);
    RUN_TEST(test_an_addressed_device_keeps_its_address_while_selected);
    RUN_TEST(test_line_settings_are_refused_unless_selected);
    RUN_TEST(test_serial_selection_names_exactly_one_device);
    RUN_TEST(test_clearing_the_selection_releases_rather_than_matches);
    RUN_TEST(test_the_commissioning_address_is_never_assignable);

    return UNITY_END();
}
