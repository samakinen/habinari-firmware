// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include "esp_err.h"
#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file control_state.h
 * @brief The device's runtime state and command surface, protocol-neutral.
 *
 * The board speaks two field protocols, and until now only one of them could
 * actually see the device: KNX carried the full room-controller model while
 * Modbus exposed six raw measurements and an LED coil. That was not a Modbus
 * limitation — it was a structural one, because the control model lived inside
 * knx_service.cpp with no way in from anywhere else.
 *
 * This header is that way in. It is the single description of what the device
 * knows (control_state_t) and what can be asked of it (control_command_t), with
 * no KNX or Modbus types in it. Both protocol adapters are now the same kind of
 * thing: a mapping between wire representation and this struct.
 *
 *   knx_service.cpp   owns the state and implements these functions; the KNX
 *                     group objects are its native expression
 *   mb_rtu_slave.c    maps the same state onto Modbus registers, and Modbus
 *                     writes onto the same commands
 *
 * A command from Modbus therefore takes exactly the path a KNX telegram takes,
 * including the clamping and the ETS-configured limits, and the two can never
 * drift into disagreeing about what "setpoint" means.
 */

/// Snapshot of everything the device currently knows. Copied under the control
/// mutex, so every field is from the same instant.
typedef struct {
    // --- Measurements -------------------------------------------------------
    sensor_data_t sensors;
    bool has_sensor_data;  ///< at least one sampling cycle has completed

    // --- Derived values (computed by the control tick) ----------------------
    float room_dew_point_c;
    float room_absolute_humidity_gm3;
    float floor_absolute_humidity_gm3;
    float sea_level_pressure_pa;
    float dew_point_margin_k;

    // --- Setpoints ----------------------------------------------------------
    float comfort_setpoint_c;   ///< the ETS/bus-writable comfort heating anchor
    float active_setpoint_c;    ///< what the controller is actually working to
    float heating_setpoint_c;
    float cooling_setpoint_c;
    float setpoint_shift_k;     ///< effective shift after clamping
    float co2_setpoint_ppm;

    // --- Controller outputs -------------------------------------------------
    uint8_t heating_percent;
    uint8_t cooling_percent;
    uint8_t ventilation_percent;
    uint8_t ventilation_level;      ///< 0=Off,1=Low,2=Medium,3=High,4=Boost
    bool heating_request;
    bool cooling_request;
    bool heat_cool_mode_heating;    ///< DPT 1.100: true = heating
    bool enable_heat;
    bool enable_cool;
    bool ventilation_boost_request;
    bool dehumidify_request;
    uint16_t controller_status;     ///< DPT 22.101 StatusRHCC
    uint16_t air_quality_status;    ///< IaqStatusBit bitset

    // --- Alarms and availability -------------------------------------------
    bool dew_point_alarm;
    bool floor_moisture_alarm;
    bool floor_limit_active;
    bool floor_comfort_active;
    bool free_cooling_available;
    bool free_drying_available;
    bool device_fault;
    uint8_t room_sensor_status;         ///< DPT 21.001 StatusGen
    uint8_t floor_probe_status;         ///< DPT 21.001 StatusGen
    uint8_t air_quality_sensor_status;  ///< DPT 21.001 StatusGen

    // --- Mode / inputs, whatever last set them ------------------------------
    bool controller_on;
    uint8_t hvac_operating_mode;  ///< habinari::hvac::OperatingPreset
    uint8_t controller_mode;      ///< habinari::hvac::ControllerMode
    uint8_t ventilation_mode;     ///< habinari::hvac::VentilationMode
    bool window_open;
    bool presence;
    bool programming_mode;        ///< KNX programming mode is active
} control_state_t;

/// Copy the current state. Safe from any task; never blocks on the bus.
void control_state_get(control_state_t *out);

/**
 * Commands a protocol adapter may issue. Each maps onto the same handling as
 * the equivalent KNX group-object write — same clamping, same ETS limits — so
 * a Modbus master and an ETS-linked switch cannot produce different results.
 */
typedef enum {
    CONTROL_CMD_CONTROLLER_ON_OFF = 0,  ///< 0 = off, non-zero = on
    CONTROL_CMD_HVAC_MODE,              ///< OperatingPreset code point
    /// ControllerMode code point (0=Auto, 1=Heat, 2=Cool, 3=Off) — the same
    /// encoding control_state_t::controller_mode reports, so read-modify-write
    /// round-trips. Protocol-specific encodings such as KNX DPT 20.105 are
    /// translated by the adapter that speaks them, never here.
    CONTROL_CMD_CONTROLLER_MODE,
    CONTROL_CMD_SETPOINT_BASE,          ///< comfort heating setpoint, °C
    CONTROL_CMD_SETPOINT_SHIFT,         ///< user offset, K
    CONTROL_CMD_WINDOW_STATUS,          ///< 0 = closed, non-zero = open
    CONTROL_CMD_PRESENCE,               ///< 0 = absent, non-zero = present
    CONTROL_CMD_VENTILATION_MODE,       ///< VentilationMode code point
    CONTROL_CMD_CO2_SETPOINT,           ///< ventilation CO2 setpoint, ppm
    CONTROL_CMD_ACKNOWLEDGE_ALARMS,     ///< clear latched alarms (value ignored)
} control_command_t;

/// Apply a command. `value` carries the payload for every command; booleans and
/// enumerations pass as 0/1/code point. Returns ESP_ERR_INVALID_ARG for an
/// unknown command and ESP_ERR_INVALID_STATE before the control task is up.
esp_err_t control_state_write(control_command_t command, float value);

#ifdef __cplusplus
}
#endif
