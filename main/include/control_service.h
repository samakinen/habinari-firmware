// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control_state.h"
#include "esp_err.h"
#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file control_service.h
 * @brief The service that owns the device. Protocol adapters are its clients.
 *
 * This is the piece that used to be missing. The room-control model, the
 * settings, NVS and the 1 Hz tick all lived inside knx_service.cpp, so KNX was
 * not one way to reach the device — it *was* the device, and Modbus was a
 * passenger on a KNX stack. Turning KNX off left nothing running.
 *
 * Now the ownership is here:
 *
 *   control_service   owns Settings, Inputs, Outputs, NVS and the control task
 *   knx_service       adapter — maps group objects onto them
 *   mb_rtu_slave      adapter — maps holding/input registers onto them
 *   mqtt_service      adapter — maps JSON topics onto them
 *
 * Every adapter is optional and none of them can see another. A build with no
 * adapter at all still samples, still controls, still logs — it just has nobody
 * to tell, which is a sensible thing for a bench build to be.
 *
 * C++ code in-tree should prefer control_service.hpp, which exposes the typed
 * Settings/Inputs/Outputs structs directly instead of the flattened C view.
 */

/// Start the control task. Initialises NVS, so call it before any adapter.
/// Idempotent.
esp_err_t control_service_start(void);

/// Hand in a new fused measurement record. Stores it and fans it out to the
/// adapters; the control loops react on the next tick rather than in the
/// caller's context. Safe from any task.
void control_service_update_sensor_data(const sensor_data_t *data);

/// Ask for identify/programming mode to be toggled — what the PROG button does.
/// Protocol-neutral on purpose: on a KNX build the KNX adapter turns this into
/// programming mode, and on a build without KNX it still blinks the LED.
void control_service_request_identify_toggle(void);

/// Report identify/programming state back (the KNX adapter calls this when ETS
/// changes it, so the LED and control_state_t follow either route in).
void control_service_set_identify_active(bool active);

/// Erase all persisted state and reboot. Does not return.
void control_service_factory_reset(void);

/// Ticks completed since boot. Increments once per control cycle; adapters get
/// the same number in on_control_tick().
uint32_t control_service_generation(void);

/// Notification callback for identify-mode changes (drives the board LED).
typedef void (*control_identify_callback_t)(bool active);
void control_service_set_identify_callback(control_identify_callback_t callback);

#ifdef __cplusplus
}
#endif
