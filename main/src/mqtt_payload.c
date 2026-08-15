// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/*
 * MQTT wire format: state out as JSON, commands in as scalars.
 *
 * Platform-free on purpose — see the header. Everything here is snprintf and
 * strtod against a fixed table, so it host-tests without a broker.
 */
#include "mqtt_payload.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Code points from habinari::hvac (hvac_control.hpp). Mirrored rather than
 * included because that header is C++; the host test asserts the two agree. */
#define PRESET_AUTO 0
#define PRESET_COMFORT 1
#define PRESET_STANDBY 2
#define PRESET_ECONOMY 3
#define PRESET_PROTECTION 4

#define MODE_AUTO 0
#define MODE_HEAT 1
#define MODE_COOL 2
#define MODE_OFF 3

#define VENT_AUTO 0
#define VENT_MANUAL 1
#define VENT_OFF 2
#define VENT_BOOST 3

/* --- JSON emission -------------------------------------------------------
 *
 * A tiny append helper rather than a JSON library: the document is a flat
 * object of numbers and short strings, and pulling in cJSON to build it would
 * cost heap churn every publish for no expressive gain. `used` saturates at
 * buf_len on overflow, which the caller detects as a -1 return.
 */
typedef struct {
    char *buf;
    size_t len;
    size_t used;
    bool overflow;
    bool first;
} json_writer_t;

static void json_raw(json_writer_t *w, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

static void json_raw(json_writer_t *w, const char *fmt, ...)
{
    if (w->overflow) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(w->buf + w->used, w->len - w->used, fmt, args);
    va_end(args);
    if (n < 0 || (size_t)n >= w->len - w->used) {
        w->overflow = true;
        return;
    }
    w->used += (size_t)n;
}

static void json_sep(json_writer_t *w)
{
    if (w->first) {
        w->first = false;
    } else {
        json_raw(w, ",");
    }
}

static void json_num(json_writer_t *w, const char *key, double value, int decimals)
{
    json_sep(w);
    json_raw(w, "\"%s\":%.*f", key, decimals, value);
}

static void json_int(json_writer_t *w, const char *key, long value)
{
    json_sep(w);
    json_raw(w, "\"%s\":%ld", key, value);
}

static void json_bool(json_writer_t *w, const char *key, bool value)
{
    json_sep(w);
    json_raw(w, "\"%s\":%s", key, value ? "true" : "false");
}

static void json_str(json_writer_t *w, const char *key, const char *value)
{
    if (value == NULL) {
        return;
    }
    json_sep(w);
    json_raw(w, "\"%s\":\"%s\"", key, value);
}

/* An invalid measurement is omitted, never published as 0. The redundancy layer
 * can genuinely report "no room temperature", and a subscriber that cannot tell
 * that from 0 °C will heat the room to the setpoint against a fabricated
 * reading. Absent keys are how JSON says "unknown". */
static void json_measurement(json_writer_t *w, const char *key, const sensor_value_t *v,
                             int decimals)
{
    if (!v->valid) {
        return;
    }
    json_num(w, key, v->value, decimals);
}

const char *mqtt_payload_preset_name(uint8_t operating_mode)
{
    switch (operating_mode) {
    case PRESET_AUTO:       return "auto";
    case PRESET_COMFORT:    return "comfort";
    case PRESET_STANDBY:    return "standby";
    case PRESET_ECONOMY:    return "eco";
    case PRESET_PROTECTION: return "away";
    default:                return NULL;
    }
}

const char *mqtt_payload_mode_name(uint8_t controller_mode, bool controller_on)
{
    /* Home Assistant treats "off" as an hvac_mode rather than a separate
     * switch, so a disabled controller reports off whatever mode it holds. */
    if (!controller_on) {
        return "off";
    }
    switch (controller_mode) {
    case MODE_AUTO: return "auto";
    case MODE_HEAT: return "heat";
    case MODE_COOL: return "cool";
    case MODE_OFF:  return "off";
    default:        return NULL;
    }
}

const char *mqtt_payload_action_name(const control_state_t *state)
{
    if (state == NULL || !state->controller_on) {
        return "off";
    }
    if (state->heating_percent > 0 || state->heating_request) {
        return "heating";
    }
    if (state->cooling_percent > 0 || state->cooling_request) {
        return "cooling";
    }
    return "idle";
}

int mqtt_payload_state_json(char *buf, size_t buf_len, const control_state_t *state)
{
    if (buf == NULL || state == NULL || buf_len == 0) {
        return -1;
    }

    json_writer_t w = {.buf = buf, .len = buf_len, .used = 0, .overflow = false, .first = true};
    json_raw(&w, "{");

    /* --- Measurements --- */
    if (state->has_sensor_data) {
        const sensor_data_t *s = &state->sensors;
        json_measurement(&w, "temperature", &s->temperature, 2);
        json_measurement(&w, "humidity", &s->humidity, 1);
        json_measurement(&w, "co2", &s->co2, 0);
        json_measurement(&w, "pressure", &s->pressure, 0);
        json_measurement(&w, "iaq", &s->iaq, 0);
        json_measurement(&w, "co2_equivalent", &s->co2_equivalent, 0);
        json_measurement(&w, "voc_equivalent", &s->voc_equivalent, 2);
        json_measurement(&w, "probe_temperature", &s->probe_temperature, 2);
        json_measurement(&w, "probe_humidity", &s->probe_humidity, 1);
        json_int(&w, "air_quality_accuracy", s->air_quality_accuracy);

        /* Derived values are only meaningful with a valid room reading. */
        if (s->temperature.valid && s->humidity.valid) {
            json_num(&w, "dew_point", state->room_dew_point_c, 2);
            json_num(&w, "absolute_humidity", state->room_absolute_humidity_gm3, 2);
        }
        if (s->pressure.valid) {
            json_num(&w, "pressure_sea_level", state->sea_level_pressure_pa, 0);
        }
    }

    /* --- Climate entity --- */
    json_num(&w, "setpoint", state->active_setpoint_c, 2);
    json_num(&w, "setpoint_base", state->comfort_setpoint_c, 2);
    json_num(&w, "setpoint_shift", state->setpoint_shift_k, 2);
    json_str(&w, "mode", mqtt_payload_mode_name(state->controller_mode, state->controller_on));
    json_str(&w, "preset", mqtt_payload_preset_name(state->hvac_operating_mode));
    json_str(&w, "action", mqtt_payload_action_name(state));

    /* --- Control outputs --- */
    json_int(&w, "heating_percent", state->heating_percent);
    json_int(&w, "cooling_percent", state->cooling_percent);
    json_int(&w, "ventilation_percent", state->ventilation_percent);
    json_int(&w, "ventilation_level", state->ventilation_level);
    json_int(&w, "ventilation_mode", state->ventilation_mode);
    json_num(&w, "co2_setpoint", state->co2_setpoint_ppm, 0);
    json_bool(&w, "heating_request", state->heating_request);
    json_bool(&w, "cooling_request", state->cooling_request);
    json_bool(&w, "ventilation_boost", state->ventilation_boost_request);
    json_bool(&w, "dehumidify_request", state->dehumidify_request);

    /* --- Inputs and alarms --- */
    json_bool(&w, "window_open", state->window_open);
    json_bool(&w, "presence", state->presence);
    json_bool(&w, "dew_point_alarm", state->dew_point_alarm);
    json_bool(&w, "floor_moisture_alarm", state->floor_moisture_alarm);
    json_bool(&w, "floor_limit_active", state->floor_limit_active);
    json_bool(&w, "free_cooling_available", state->free_cooling_available);
    json_bool(&w, "device_fault", state->device_fault);
    json_bool(&w, "identify", state->programming_mode);
    json_int(&w, "controller_status", state->controller_status);
    json_int(&w, "air_quality_status", state->air_quality_status);

    json_raw(&w, "}");

    if (w.overflow) {
        buf[0] = '\0';
        return -1;
    }
    return (int)w.used;
}

/* --- Command parsing ----------------------------------------------------- */

typedef struct {
    const char *suffix;
    control_command_t command;
    /* NULL for a plain number; otherwise a NUL-separated name->value table. */
    const char *const *names;
    const float *values;
    size_t name_count;
} command_map_t;

static const char *const kModeNames[] = {"auto", "heat", "cool", "off"};
static const float kModeValues[] = {MODE_AUTO, MODE_HEAT, MODE_COOL, MODE_OFF};

static const char *const kPresetNames[] = {"auto", "comfort", "standby", "eco", "away"};
static const float kPresetValues[] = {PRESET_AUTO, PRESET_COMFORT, PRESET_STANDBY, PRESET_ECONOMY,
                                      PRESET_PROTECTION};

static const char *const kVentNames[] = {"auto", "manual", "off", "boost"};
static const float kVentValues[] = {VENT_AUTO, VENT_MANUAL, VENT_OFF, VENT_BOOST};

static const command_map_t kCommands[] = {
    {"setpoint", CONTROL_CMD_SETPOINT_BASE, NULL, NULL, 0},
    {"setpoint_shift", CONTROL_CMD_SETPOINT_SHIFT, NULL, NULL, 0},
    {"co2_setpoint", CONTROL_CMD_CO2_SETPOINT, NULL, NULL, 0},
    {"mode", CONTROL_CMD_CONTROLLER_MODE, kModeNames, kModeValues,
     sizeof(kModeNames) / sizeof(kModeNames[0])},
    {"preset", CONTROL_CMD_HVAC_MODE, kPresetNames, kPresetValues,
     sizeof(kPresetNames) / sizeof(kPresetNames[0])},
    {"ventilation_mode", CONTROL_CMD_VENTILATION_MODE, kVentNames, kVentValues,
     sizeof(kVentNames) / sizeof(kVentNames[0])},
    {"power", CONTROL_CMD_CONTROLLER_ON_OFF, NULL, NULL, 0},
    {"window", CONTROL_CMD_WINDOW_STATUS, NULL, NULL, 0},
    {"presence", CONTROL_CMD_PRESENCE, NULL, NULL, 0},
    {"acknowledge_alarms", CONTROL_CMD_ACKNOWLEDGE_ALARMS, NULL, NULL, 0},
};

/* Copy into a NUL-terminated scratch buffer with surrounding whitespace
 * stripped. MQTT payloads are length-delimited and brokers/bridges routinely
 * append a newline; "21.5\n" must not parse as anything other than 21.5. */
static bool payload_to_token(const char *payload, size_t payload_len, char *out, size_t out_len)
{
    if (payload == NULL || out_len == 0) {
        return false;
    }
    size_t begin = 0;
    while (begin < payload_len && (unsigned char)payload[begin] <= ' ') {
        ++begin;
    }
    size_t end = payload_len;
    while (end > begin && (unsigned char)payload[end - 1] <= ' ') {
        --end;
    }
    const size_t n = end - begin;
    if (n == 0 || n >= out_len) {
        return false;
    }
    memcpy(out, payload + begin, n);
    out[n] = '\0';
    return true;
}

static bool token_equals_ci(const char *token, const char *name)
{
    for (size_t i = 0;; ++i) {
        char a = token[i];
        const char b = name[i];
        if (a >= 'A' && a <= 'Z') {
            a = (char)(a - 'A' + 'a');
        }
        if (a != b) {
            return false;
        }
        if (a == '\0') {
            return true;
        }
    }
}

/* Booleans arrive as "ON"/"OFF" from Home Assistant, as "true"/"false" from
 * scripts and as "1"/"0" from everything else. All three are the same intent. */
static bool parse_boolish(const char *token, float *out)
{
    if (token_equals_ci(token, "on") || token_equals_ci(token, "true")
        || token_equals_ci(token, "yes") || token_equals_ci(token, "open")) {
        *out = 1.0f;
        return true;
    }
    if (token_equals_ci(token, "off") || token_equals_ci(token, "false")
        || token_equals_ci(token, "no") || token_equals_ci(token, "closed")) {
        *out = 0.0f;
        return true;
    }
    return false;
}

static bool parse_number(const char *token, float *out)
{
    char *end = NULL;
    const double value = strtod(token, &end);
    if (end == token || *end != '\0') {
        return false;
    }
    *out = (float)value;
    return true;
}

bool mqtt_payload_parse_command(const char *topic_suffix, const char *payload, size_t payload_len,
                                control_command_t *out_command, float *out_value)
{
    if (topic_suffix == NULL || out_command == NULL || out_value == NULL) {
        return false;
    }

    const command_map_t *entry = NULL;
    for (size_t i = 0; i < sizeof(kCommands) / sizeof(kCommands[0]); ++i) {
        if (strcmp(topic_suffix, kCommands[i].suffix) == 0) {
            entry = &kCommands[i];
            break;
        }
    }
    if (entry == NULL) {
        return false;
    }

    /* The alarm acknowledgement ignores its payload by definition, so an empty
     * one is legitimate — a bare publish is the natural way to express it. */
    if (entry->command == CONTROL_CMD_ACKNOWLEDGE_ALARMS) {
        *out_command = entry->command;
        *out_value = 1.0f;
        return true;
    }

    char token[32];
    if (!payload_to_token(payload, payload_len, token, sizeof(token))) {
        return false;
    }

    if (entry->names != NULL) {
        for (size_t i = 0; i < entry->name_count; ++i) {
            if (token_equals_ci(token, entry->names[i])) {
                *out_command = entry->command;
                *out_value = entry->values[i];
                return true;
            }
        }
        /* Numeric code points stay acceptable for non-HA clients. */
        if (parse_number(token, out_value)) {
            *out_command = entry->command;
            return true;
        }
        return false;
    }

    if (parse_boolish(token, out_value) || parse_number(token, out_value)) {
        *out_command = entry->command;
        return true;
    }
    return false;
}
