// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "control_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file mqtt_payload.h
 * @brief Wire format for the MQTT personality — the part worth testing.
 *
 * Deliberately separated from mqtt_service.c, which is Wi-Fi association, TLS
 * and an event loop: none of that can be exercised without hardware, whereas
 * *this* is where integration bugs actually live. A wrong topic suffix, a
 * setpoint published with the wrong unit or a command payload parsed as 0
 * because it was "21.5\r\n" are all silent in the field and all catchable here.
 * See main/test/test_mqtt_payload.cpp.
 *
 * Nothing in this header allocates, formats through a JSON library, or depends
 * on ESP-IDF beyond esp_err_t.
 *
 * Format choices, both taken from what Home Assistant's MQTT integration
 * actually consumes:
 *   - state goes out as one JSON document, so a subscriber gets a coherent
 *     snapshot rather than a scattering of topics that can disagree;
 *   - commands come in as bare scalar payloads on one topic each ("21.5",
 *     "heat", "ON"), because that is what HA's command topics publish.
 */

/// Longest state document mqtt_payload_state_json can emit, including the
/// terminating NUL. Sized from the field list with every value at full width.
#define MQTT_PAYLOAD_STATE_MAX 1024

/// Longest command topic suffix understood (see mqtt_payload_parse_command).
#define MQTT_PAYLOAD_TOPIC_MAX 32

/**
 * Serialise the device state as a JSON object.
 *
 * Invalid measurements are omitted rather than published as 0 — a subscriber
 * must be able to tell "the room is at 0 °C" from "there is no room
 * temperature", and the redundancy layer can genuinely report the latter.
 *
 * @return bytes written excluding the NUL, or -1 if the buffer was too small.
 */
int mqtt_payload_state_json(char *buf, size_t buf_len, const control_state_t *state);

/**
 * Map an inbound command topic and payload onto a control command.
 *
 * @param topic_suffix  the part after ".../cmd/", e.g. "setpoint"
 * @param payload       the raw payload; need not be NUL-terminated
 * @param payload_len   its length
 * @param out_command   receives the command on success
 * @param out_value     receives the value on success
 * @return true if the topic is known and the payload parsed
 */
bool mqtt_payload_parse_command(const char *topic_suffix, const char *payload, size_t payload_len,
                                control_command_t *out_command, float *out_value);

/// What every Home Assistant discovery document says about the device itself.
/// The strings are copied into the document JSON-escaped, so they may hold
/// anything an installer typed.
typedef struct {
    const char *device_id;           ///< also the prefix of every unique id
    const char *device_name;         ///< the name HA shows on the device card
    const char *manufacturer;
    const char *model;
    const char *state_topic;         ///< where the state document is published
    const char *availability_topic;  ///< "online" / "offline", retained
    const char *command_base;        ///< commands go to "<command_base>/cmd/<name>"
} mqtt_discovery_device_t;

/**
 * Home Assistant discovery configuration for one sensor entity reading
 * @p value_key from the state document. @p device_class and @p unit may be
 * NULL; a unit also marks the entity as a measurement.
 *
 * @return bytes written excluding the NUL, or -1 if the buffer was too small
 *         (the buffer then holds an empty string, never a truncated document).
 */
int mqtt_payload_discovery_sensor(char *buf, size_t buf_len, const mqtt_discovery_device_t *device,
                                  const char *object_id, const char *name,
                                  const char *device_class, const char *unit,
                                  const char *value_key);

/// Home Assistant discovery configuration for the climate entity. Returns as
/// mqtt_payload_discovery_sensor() does.
int mqtt_payload_discovery_climate(char *buf, size_t buf_len, const mqtt_discovery_device_t *device);

/// Home Assistant preset name for an OperatingPreset code point, or NULL.
const char *mqtt_payload_preset_name(uint8_t operating_mode);

/// Home Assistant hvac mode name for a ControllerMode code point, or NULL.
const char *mqtt_payload_mode_name(uint8_t controller_mode, bool controller_on);

/// The hvac_action Home Assistant shows: "heating", "cooling", "idle", "off".
const char *mqtt_payload_action_name(const control_state_t *state);

#ifdef __cplusplus
}
#endif
