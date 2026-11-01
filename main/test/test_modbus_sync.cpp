// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// Host tests for main/include/mb_sync.h: how the Modbus slave task tells a
// master's write from its own read-back, one cycle at a time.
//
// The stack is replaced by two plain structs (the live register image the
// master writes into, and the task's record of it), and control_state_write()
// by a recorder, so the interleavings that lost writes on hardware — a master
// write landing between the task's snapshot and its refresh — can be placed
// exactly.

#include "mb_sync.h"

#include "unity.h"

#include <cstring>
#include <vector>

namespace {

struct Command {
    control_command_t command;
    float value;
};

/// The slave task's view: the live registers, its record, and the device.
struct Rig {
    mb_holding_registers_t live{};
    uint8_t coils[MB_COIL_BYTES]{};
    mb_writable_image_t applied{};
    control_state_t device{};
    uint8_t slaveAddress{17};
    uint16_t baudCode{MB_BAUD_19200};

    std::vector<Command> issued;
    esp_err_t verdict{ESP_OK};  ///< what the recorder answers

    mb_writable_image_t snapshot{};

    static esp_err_t record(control_command_t command, float value, void *ctx)
    {
        auto *rig = static_cast<Rig *>(ctx);
        rig->issued.push_back({command, value});
        return rig->verdict;
    }

    Rig()
    {
        device.comfort_setpoint_c = 21.0f;
        device.co2_setpoint_ppm = 900.0f;
        device.controller_on = true;
        // Settle: one full cycle with nothing written, as after start-up.
        cycle();
        issued.clear();
    }

    // The three steps of mb_apply_writes() / the refresh, separable so a test
    // can put a master write between them.
    unsigned takeAndApply()
    {
        mb_sync_take_snapshot(&live, coils, &snapshot);
        const unsigned refused = mb_sync_apply_commands(&snapshot, &applied, record, this);
        mb_sync_mark_applied(&snapshot, &applied);
        return refused;
    }

    void refresh()
    {
        mb_sync_refresh(&live, coils, &applied, &device, slaveAddress, baudCode);
    }

    void cycle()
    {
        takeAndApply();
        refresh();
    }
};

} // namespace

void test_a_write_between_snapshot_and_refresh_is_applied_next_cycle(void)
{
    Rig rig;
    TEST_ASSERT_EQUAL_INT16(210, rig.live.comfort_setpoint);

    rig.takeAndApply();
    rig.live.comfort_setpoint = 235;  // master writes 23.5 °C in the window
    rig.refresh();

    // Not overwritten by the read-back of the old value...
    TEST_ASSERT_EQUAL_INT16(235, rig.live.comfort_setpoint);
    TEST_ASSERT_EQUAL_UINT(0, rig.issued.size());

    // ...and, the point of the fix, not recorded as already applied either.
    rig.takeAndApply();
    TEST_ASSERT_EQUAL_UINT(1, rig.issued.size());
    TEST_ASSERT_EQUAL_INT(CONTROL_CMD_SETPOINT_BASE, rig.issued[0].command);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 23.5f, rig.issued[0].value);
}

void test_a_coil_write_between_snapshot_and_refresh_is_applied_next_cycle(void)
{
    Rig rig;
    rig.takeAndApply();
    mb_bit_set(rig.coils, MB_COIL_WINDOW_STATUS, true);
    rig.refresh();
    TEST_ASSERT_TRUE(mb_bit_get(rig.coils, MB_COIL_WINDOW_STATUS));

    rig.takeAndApply();
    TEST_ASSERT_EQUAL_UINT(1, rig.issued.size());
    TEST_ASSERT_EQUAL_INT(CONTROL_CMD_WINDOW_STATUS, rig.issued[0].command);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, rig.issued[0].value);
}

void test_a_commit_is_seen_exactly_once(void)
{
    Rig rig;
    rig.live.slave_address = 23;
    rig.live.config_commit = 1;

    rig.takeAndApply();
    TEST_ASSERT_EQUAL_UINT16(1, rig.snapshot.holding.config_commit);
    TEST_ASSERT_EQUAL_UINT16(0, rig.live.config_commit);  // consumed under the lock
    rig.refresh();

    for (int i = 0; i < 3; ++i) {
        rig.takeAndApply();
        TEST_ASSERT_EQUAL_UINT16(0, rig.snapshot.holding.config_commit);
        rig.refresh();
    }
}

void test_a_commit_written_again_in_the_window_is_not_lost(void)
{
    Rig rig;
    rig.live.config_commit = 1;
    rig.takeAndApply();
    rig.live.config_commit = 1;  // a second commit arrives before the refresh
    rig.refresh();
    TEST_ASSERT_EQUAL_UINT16(1, rig.live.config_commit);

    rig.takeAndApply();
    TEST_ASSERT_EQUAL_UINT16(1, rig.snapshot.holding.config_commit);
}

void test_the_devices_own_read_back_is_not_echoed_as_a_command(void)
{
    Rig rig;

    // The setpoint changes from another bus (KNX). The refresh shows it...
    rig.device.comfort_setpoint_c = 22.0f;
    rig.device.hvac_operating_mode = 3;
    rig.device.controller_on = false;
    rig.cycle();
    TEST_ASSERT_EQUAL_INT16(220, rig.live.comfort_setpoint);
    TEST_ASSERT_EQUAL_UINT16(3, rig.live.hvac_mode);
    TEST_ASSERT_FALSE(mb_bit_get(rig.coils, MB_COIL_CONTROLLER_ON));

    // ...and the next cycle must not mistake it for a master write and send it
    // back, which would undo a newer change made over KNX in the meantime.
    rig.cycle();
    TEST_ASSERT_EQUAL_UINT(0, rig.issued.size());
}

void test_a_refused_write_is_replaced_by_the_devices_value(void)
{
    Rig rig;
    rig.verdict = ESP_ERR_INVALID_ARG;
    rig.live.hvac_mode = 9;

    TEST_ASSERT_EQUAL_UINT(1, rig.takeAndApply());
    rig.refresh();
    TEST_ASSERT_EQUAL_UINT16(rig.device.hvac_operating_mode, rig.live.hvac_mode);

    // Not retried every cycle.
    rig.cycle();
    TEST_ASSERT_EQUAL_UINT(1, rig.issued.size());
}

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_a_write_between_snapshot_and_refresh_is_applied_next_cycle);
    RUN_TEST(test_a_coil_write_between_snapshot_and_refresh_is_applied_next_cycle);
    RUN_TEST(test_a_commit_is_seen_exactly_once);
    RUN_TEST(test_a_commit_written_again_in_the_window_is_not_lost);
    RUN_TEST(test_the_devices_own_read_back_is_not_echoed_as_a_command);
    RUN_TEST(test_a_refused_write_is_replaced_by_the_devices_value);

    return UNITY_END();
}
