// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "control_state.h"
#include "modbus_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file mb_sync.h
 * @brief How the Modbus slave tells a master's write from its own read-back.
 *
 * The writable registers are shared by two writers: the master, through the
 * stack, and the slave task, which reads the device's adopted values back into
 * them. Neither has a write callback — the area is plain memory — so a master
 * write is recognised as a register that no longer holds what the task last
 * synchronised it to. `applied` is that record.
 *
 * One cycle of the slave task is:
 *
 *   1. under the stack lock, mb_sync_take_snapshot(): copy the writable image
 *      and consume config_commit
 *   2. outside it, mb_sync_apply_commands() on the snapshot, then
 *      mb_sync_mark_applied() to record the snapshot as handled
 *   3. under the lock again, mb_sync_refresh(): read the device's state back,
 *      but only into registers that still hold what step 2 recorded
 *
 * The rule that makes this lossless: `applied` only ever records a value the
 * task has acted on (step 2) or has itself written (step 3). It used to be
 * overwritten wholesale with the register image at the end of the refresh,
 * which recorded a master write landing between steps 1 and 3 as already
 * handled, and dropped it.
 *
 * Kept free of ESP-IDF and of the stack so it is host-tested
 * (main/test/test_modbus_sync.cpp); mb_rtu_slave.c supplies the lock.
 */

/// The registers a master can write, as one unit.
typedef struct {
    mb_holding_registers_t holding;
    uint8_t coils[MB_COIL_BYTES];
} mb_writable_image_t;

/// Issues one command. mb_rtu_slave.c passes control_state_write() behind a
/// logging wrapper; the tests pass a recorder.
typedef esp_err_t (*mb_command_writer_t)(control_command_t command, float value, void *ctx);

/// Copy the live image into @p out and consume a pending commit. Call with the
/// stack locked: clearing config_commit in the same critical section that reads
/// it is what makes one commit act once. It used to be cleared only by the
/// refresh, a cycle later, so every commit was processed twice.
static inline void mb_sync_take_snapshot(mb_holding_registers_t *live_holding,
                                         const uint8_t *live_coils,
                                         mb_writable_image_t *out)
{
    out->holding = *live_holding;
    memcpy(out->coils, live_coils, sizeof(out->coils));
    live_holding->config_commit = 0;
}

/// Turn every control register and coil the master has changed since
/// @p applied into a command. Returns how many were refused. A refused value is
/// not retried: the refresh writes the device's actual value back over it, which
/// is how the master learns it was not adopted.
static inline unsigned mb_sync_apply_commands(const mb_writable_image_t *current,
                                              const mb_writable_image_t *applied,
                                              mb_command_writer_t write,
                                              void *ctx)
{
    const mb_holding_registers_t *c = &current->holding;
    const mb_holding_registers_t *a = &applied->holding;
    unsigned refused = 0;
    float value = 0.0f;

#define MB_SYNC_ISSUE(command, v)                  \
    do {                                           \
        if (write((command), (v), ctx) != ESP_OK) { \
            ++refused;                             \
        }                                          \
    } while (0)

    if (c->comfort_setpoint != a->comfort_setpoint
        && mb_decode_signed(c->comfort_setpoint, MB_SCALE_TEMPERATURE, &value)) {
        MB_SYNC_ISSUE(CONTROL_CMD_SETPOINT_BASE, value);
    }
    if (c->setpoint_shift != a->setpoint_shift
        && mb_decode_signed(c->setpoint_shift, MB_SCALE_TEMPERATURE, &value)) {
        MB_SYNC_ISSUE(CONTROL_CMD_SETPOINT_SHIFT, value);
    }
    if (c->hvac_mode != a->hvac_mode) {
        MB_SYNC_ISSUE(CONTROL_CMD_HVAC_MODE, (float)c->hvac_mode);
    }
    if (c->controller_mode != a->controller_mode) {
        MB_SYNC_ISSUE(CONTROL_CMD_CONTROLLER_MODE, (float)c->controller_mode);
    }
    if (c->ventilation_mode != a->ventilation_mode) {
        MB_SYNC_ISSUE(CONTROL_CMD_VENTILATION_MODE, (float)c->ventilation_mode);
    }
    if (c->co2_setpoint != a->co2_setpoint && c->co2_setpoint != MB_INVALID_UNSIGNED) {
        MB_SYNC_ISSUE(CONTROL_CMD_CO2_SETPOINT, (float)c->co2_setpoint);
    }

    for (unsigned coil = 0; coil < MB_COIL_COUNT; ++coil) {
        const bool now = mb_bit_get(current->coils, coil);
        if (now == mb_bit_get(applied->coils, coil)) {
            continue;
        }
        switch (coil) {
        case MB_COIL_CONTROLLER_ON:
            MB_SYNC_ISSUE(CONTROL_CMD_CONTROLLER_ON_OFF, now ? 1.0f : 0.0f);
            break;
        case MB_COIL_WINDOW_STATUS:
            MB_SYNC_ISSUE(CONTROL_CMD_WINDOW_STATUS, now ? 1.0f : 0.0f);
            break;
        case MB_COIL_PRESENCE:
            MB_SYNC_ISSUE(CONTROL_CMD_PRESENCE, now ? 1.0f : 0.0f);
            break;
        case MB_COIL_ALARM_ACKNOWLEDGE:
            if (now) {
                MB_SYNC_ISSUE(CONTROL_CMD_ACKNOWLEDGE_ALARMS, 1.0f);
            }
            break;
        default:  // MB_COIL_IDENTIFY_LED is read back by main, not a command
            break;
        }
    }

#undef MB_SYNC_ISSUE
    return refused;
}

/// Record @p current as handled. The commit is recorded as 0 because that is
/// what mb_sync_take_snapshot() left in the live register.
static inline void mb_sync_mark_applied(const mb_writable_image_t *current,
                                        mb_writable_image_t *applied)
{
    *applied = *current;
    applied->holding.config_commit = 0;
}

/// Read the device's adopted values back into the writable registers, so a
/// master sees what was actually adopted — a setpoint outside the installation's
/// limits comes back clamped, and a change made over KNX shows up here too.
///
/// A register is only refreshed while it still holds what @p applied records.
/// If it differs, a master wrote it after the snapshot; it is left alone, and
/// so is its `applied` entry, so the next cycle sees the change and applies it.
/// Where a register is refreshed, `applied` follows it, so the device's own
/// read-back is never mistaken for a master write and echoed back as a command
/// (which would undo a newer change from another bus). Call with the stack
/// locked.
static inline void mb_sync_refresh(mb_holding_registers_t *h,
                                   uint8_t *coils,
                                   mb_writable_image_t *applied,
                                   const control_state_t *state,
                                   uint8_t slave_address,
                                   uint16_t baud_code)
{
    mb_holding_registers_t *a = &applied->holding;

#define MB_SYNC_REFRESH(field, v)        \
    do {                                 \
        if (h->field == a->field) {      \
            h->field = (v);              \
            a->field = h->field;         \
        }                                \
    } while (0)

    MB_SYNC_REFRESH(slave_address, slave_address);
    MB_SYNC_REFRESH(baud_rate_code, baud_code);
    MB_SYNC_REFRESH(config_commit, 0);
    MB_SYNC_REFRESH(comfort_setpoint,
                    mb_encode_signed(state->comfort_setpoint_c, true, MB_SCALE_TEMPERATURE));
    MB_SYNC_REFRESH(setpoint_shift,
                    mb_encode_signed(state->setpoint_shift_k, true, MB_SCALE_TEMPERATURE));
    MB_SYNC_REFRESH(hvac_mode, state->hvac_operating_mode);
    MB_SYNC_REFRESH(controller_mode, state->controller_mode);
    MB_SYNC_REFRESH(ventilation_mode, state->ventilation_mode);
    MB_SYNC_REFRESH(co2_setpoint, mb_encode_unsigned(state->co2_setpoint_ppm, true, 1));

#undef MB_SYNC_REFRESH

    for (unsigned coil = 0; coil < MB_COIL_COUNT; ++coil) {
        if (mb_bit_get(coils, coil) != mb_bit_get(applied->coils, coil)) {
            continue;  // written since the snapshot; leave it for the next cycle
        }
        bool value;
        switch (coil) {
        case MB_COIL_CONTROLLER_ON:
            value = state->controller_on;
            break;
        case MB_COIL_WINDOW_STATUS:
            value = state->window_open;
            break;
        case MB_COIL_PRESENCE:
            value = state->presence;
            break;
        case MB_COIL_ALARM_ACKNOWLEDGE:
            value = false;  // momentary
            break;
        default:  // MB_COIL_IDENTIFY_LED is owned by the master
            continue;
        }
        mb_bit_set(coils, coil, value);
        mb_bit_set(applied->coils, coil, value);
    }
}

#ifdef __cplusplus
}
#endif
