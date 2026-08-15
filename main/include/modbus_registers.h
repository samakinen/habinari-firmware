#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file modbus_registers.h
 * @brief Modbus RTU register map and the scaling that defines it.
 *
 * The map mirrors the KNX object model one for one — same measurements, same
 * derived values, same controller outputs, same writable inputs — so the board
 * presents one device model on both field buses rather than a full room
 * controller on KNX and a handful of raw readings on RS-485.
 *
 * Representation follows what BMS and PLC masters actually expect, which is not
 * what the KNX side does:
 *
 *   - 16-bit scaled integers, not IEEE-754 floats. A float spans two registers
 *     and every vendor disagrees about the word order, so the previous float
 *     map was unreadable without knowing which end wrote it. Fixed-point
 *     integers have exactly one interpretation.
 *   - One documented sentinel per signedness for "no reading". A master must be
 *     able to tell 0.0 °C from a dead sensor, and the redundancy layer makes
 *     that distinction real: a value can genuinely be unavailable.
 *   - Fixed addresses with reserved gaps between blocks, so adding a
 *     measurement later does not renumber the map an integration was built on.
 *
 * This header is deliberately free of ESP-IDF and of the register structs'
 * consumers, so the scaling is covered by host tests
 * (main/test/test_modbus_registers.cpp) rather than only on hardware.
 */

// --- Sentinels --------------------------------------------------------------
// "Not available" markers. Chosen at the ends of the ranges so they can never
// collide with a real reading: -3276.8 °C and 65535 ppm are both impossible.
#define MB_INVALID_SIGNED   ((int16_t)0x8000)   /* -32768 */
#define MB_INVALID_UNSIGNED ((uint16_t)0xFFFF)  /* 65535  */

// --- Scaling ---------------------------------------------------------------
// Every scaled quantity is x10 unless noted: 0.1 K and 0.1 %RH are finer than
// any sensor here is accurate, and the range still covers anything physical.
#define MB_SCALE_TEMPERATURE 10  /* 0.1 °C   */
#define MB_SCALE_HUMIDITY    10  /* 0.1 %RH  */
#define MB_SCALE_PRESSURE    10  /* 0.1 hPa  */
#define MB_SCALE_DENSITY     10  /* 0.1 g/m³ */
#define MB_SCALE_TREND       10  /* 0.1 K/h  */

/// Product identity, register mb_input_registers_t.device_id. Lets a master
/// confirm it is talking to this device before trusting the rest of the map.
#define MB_DEVICE_ID 0x5B01
/// Bumped whenever the register map changes meaning, so an integration can
/// refuse to run against a layout it does not know.
#define MB_MAP_VERSION 2

/// Encode a float as a scaled signed register, or the sentinel when invalid.
static inline int16_t mb_encode_signed(float value, bool valid, int scale)
{
    if (!valid) {
        return MB_INVALID_SIGNED;
    }
    const float scaled = value * (float)scale;
    // Clamp one short of the sentinel so saturation can never be mistaken for
    // "no reading".
    if (scaled <= -32767.0f) {
        return (int16_t)-32767;
    }
    if (scaled >= 32767.0f) {
        return (int16_t)32767;
    }
    return (int16_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

/// Encode a float as a scaled unsigned register, or the sentinel when invalid.
static inline uint16_t mb_encode_unsigned(float value, bool valid, int scale)
{
    if (!valid || value < 0.0f) {
        return MB_INVALID_UNSIGNED;
    }
    const float scaled = value * (float)scale;
    if (scaled >= 65534.0f) {
        return (uint16_t)65534;
    }
    return (uint16_t)(scaled + 0.5f);
}

/// Decode a scaled signed register written by a master. Returns false — and
/// leaves @p out untouched — for the sentinel, so a master can explicitly write
/// "no value" to a register it does not wish to drive.
static inline bool mb_decode_signed(int16_t raw, int scale, float *out)
{
    if (raw == MB_INVALID_SIGNED || scale == 0) {
        return false;
    }
    *out = (float)raw / (float)scale;
    return true;
}

// --- Input registers (function 04), read-only ------------------------------
// Word offsets are the register addresses; the struct layout below IS the map.
typedef struct {
    // Identity and health, 0..9
    uint16_t device_id;          ///< MB_DEVICE_ID
    uint16_t map_version;        ///< MB_MAP_VERSION
    uint16_t firmware_version;   ///< major << 8 | minor
    uint16_t uptime_hours;
    uint16_t uptime_seconds;     ///< seconds within the current hour
    uint16_t sensor_health_mask; ///< bit per sensor_source_t currently healthy
    uint16_t sensor_present_mask;///< bit per sensor_source_t ever seen
    uint16_t sensor_suspect_mask;///< bit per sensor_source_t failing cross-check
    uint16_t sample_count;       ///< sampling cycles, wraps
    uint16_t error_count;        ///< cycles with at least one bus error

    // Room air measurements, 10..19
    int16_t room_temperature;     ///< 0.1 °C
    int16_t room_humidity;        ///< 0.1 %RH
    uint16_t air_pressure;        ///< 0.1 hPa
    uint16_t air_pressure_sea;    ///< 0.1 hPa, altitude-reduced
    uint16_t co2;                 ///< ppm
    uint16_t air_quality_index;   ///< BSEC IAQ 0..500
    uint16_t co2_equivalent;      ///< ppm
    uint16_t voc_equivalent;      ///< 0.1 ppm
    uint16_t air_quality_accuracy;///< 0..3
    uint16_t reserved_19;

    // Derived values, 20..29
    int16_t room_dew_point;         ///< 0.1 °C
    uint16_t room_absolute_humidity;///< 0.1 g/m³
    int16_t probe_temperature;      ///< 0.1 °C
    int16_t probe_humidity;         ///< 0.1 %RH
    uint16_t probe_absolute_humidity;///< 0.1 g/m³
    int16_t dew_point_margin;       ///< 0.1 K
    int16_t temperature_trend;      ///< 0.1 K/h
    uint16_t co2_baseline;          ///< ppm, tracked fresh-air level
    uint16_t estimated_occupants;
    uint16_t reserved_29;

    // Controller state, 30..44
    int16_t active_setpoint;      ///< 0.1 °C
    int16_t heating_setpoint;     ///< 0.1 °C
    int16_t cooling_setpoint;     ///< 0.1 °C
    int16_t setpoint_shift;       ///< 0.1 K, effective after clamping
    uint16_t heating_percent;     ///< 0..100
    uint16_t cooling_percent;     ///< 0..100
    uint16_t ventilation_percent; ///< 0..100
    uint16_t ventilation_stage;   ///< 0=Off,1=Low,2=Medium,3=High,4=Boost
    uint16_t hvac_mode_active;    ///< resolved operating preset
    uint16_t controller_mode_active;
    uint16_t controller_status;   ///< KNX DPT 22.101 StatusRHCC word
    uint16_t air_quality_status;  ///< IaqStatusBit word
    uint16_t room_sensor_status;  ///< KNX DPT 21.001 StatusGen octet
    uint16_t floor_probe_status;  ///< KNX DPT 21.001 StatusGen octet
    uint16_t air_quality_sensor_status;

    // Provenance, 45..47
    uint16_t temperature_source;  ///< sensor_source_t backing the published value
    uint16_t humidity_source;
    uint16_t fire_reason_mask;
} mb_input_registers_t;

// --- Holding registers (function 03/06/16), read/write ---------------------
typedef struct {
    // Device configuration, 0..3. Changes take effect when config_commit is
    // written, not on the address write itself: changing the address of the
    // device you are talking to mid-transaction loses the response.
    uint16_t slave_address;   ///< 1..247
    uint16_t baud_rate_code;  ///< mb_baud_code_t
    uint16_t config_commit;   ///< write 1: persist and apply, then self-clears
    uint16_t reserved_3;

    // Control inputs, 4..11. Each is applied exactly like the equivalent KNX
    // group-object write, including the ETS-configured clamping, then read back
    // with whatever the device actually adopted.
    int16_t comfort_setpoint; ///< 0.1 °C
    int16_t setpoint_shift;   ///< 0.1 K
    uint16_t hvac_mode;       ///< operating preset code point
    uint16_t controller_mode; ///< controller mode code point
    uint16_t ventilation_mode;///< 0=Auto,1=Manual,2=Off,3=Boost
    uint16_t co2_setpoint;    ///< ppm
    uint16_t reserved_10;
    uint16_t reserved_11;
} mb_holding_registers_t;

/// Supported line rates. 19200 8E1 is the Modbus default; 9600 is kept because
/// it is what long or noisy RS-485 runs fall back to.
typedef enum {
    MB_BAUD_9600 = 0,
    MB_BAUD_19200 = 1,
    MB_BAUD_38400 = 2,
    MB_BAUD_57600 = 3,
    MB_BAUD_115200 = 4,
    MB_BAUD_CODE_COUNT = 5,
} mb_baud_code_t;

static inline uint32_t mb_baud_from_code(uint16_t code)
{
    switch (code) {
    case MB_BAUD_9600: return 9600;
    case MB_BAUD_19200: return 19200;
    case MB_BAUD_38400: return 38400;
    case MB_BAUD_57600: return 57600;
    case MB_BAUD_115200: return 115200;
    default: return 0;  // unknown: caller keeps the current rate
    }
}

// --- Discrete inputs (function 02), read-only ------------------------------
// One bit each; the addresses are the enumerator values.
typedef enum {
    MB_DISCRETE_SERVICE_RUNNING = 0,
    MB_DISCRETE_DEVICE_FAULT,
    MB_DISCRETE_PROBE_PRESENT,
    MB_DISCRETE_HEATING_REQUEST,
    MB_DISCRETE_COOLING_REQUEST,
    MB_DISCRETE_HEAT_MODE,          ///< 1 = heating, 0 = cooling
    MB_DISCRETE_ENABLE_HEAT,
    MB_DISCRETE_ENABLE_COOL,
    MB_DISCRETE_DEW_POINT_ALARM,
    MB_DISCRETE_FLOOR_MOISTURE_ALARM,
    MB_DISCRETE_FLOOR_LIMIT_ACTIVE,
    MB_DISCRETE_FLOOR_COMFORT_ACTIVE,
    MB_DISCRETE_FREE_COOLING,
    MB_DISCRETE_FREE_DRYING,
    MB_DISCRETE_VENTILATION_BOOST,
    MB_DISCRETE_DEHUMIDIFY_REQUEST,
    MB_DISCRETE_FIRE_ALARM,
    MB_DISCRETE_FIRE_PRE_ALARM,
    MB_DISCRETE_OCCUPANCY_DETECTED,
    MB_DISCRETE_WINDOW_OPEN_DETECTED,
    MB_DISCRETE_SENSOR_DISAGREEMENT,
    MB_DISCRETE_PROGRAMMING_MODE,
    MB_DISCRETE_COUNT,
} mb_discrete_input_t;

// --- Coils (function 01/05), read/write ------------------------------------
typedef enum {
    MB_COIL_IDENTIFY_LED = 0,  ///< lights the board's LED, for finding it
    MB_COIL_CONTROLLER_ON,
    MB_COIL_WINDOW_STATUS,     ///< reports a window contact to the controller
    MB_COIL_PRESENCE,          ///< reports occupancy to the controller
    MB_COIL_ALARM_ACKNOWLEDGE, ///< self-clearing: clears latched alarms
    MB_COIL_COUNT,
} mb_coil_t;

/// Bit accessors for the packed coil/discrete arrays the Modbus stack maps.
static inline bool mb_bit_get(const uint8_t *bits, unsigned index)
{
    return (bits[index / 8] & (1u << (index % 8))) != 0u;
}

static inline void mb_bit_set(uint8_t *bits, unsigned index, bool value)
{
    const uint8_t mask = (uint8_t)(1u << (index % 8));
    if (value) {
        bits[index / 8] |= mask;
    } else {
        bits[index / 8] &= (uint8_t)~mask;
    }
}

/// Storage for the bit areas, sized from the enumerations above.
#define MB_DISCRETE_BYTES ((MB_DISCRETE_COUNT + 7) / 8)
#define MB_COIL_BYTES ((MB_COIL_COUNT + 7) / 8)

#ifdef __cplusplus
}
#endif
