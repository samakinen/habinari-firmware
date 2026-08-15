// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

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
///
/// v3 added the commissioning block: the serial number in the input registers,
/// the serial-select registers in the holding block, and the unassigned-address
/// convention below. Nothing was renumbered.
#define MB_MAP_VERSION 3

// --- Addressing and commissioning -------------------------------------------
//
// Modbus standardises no way to give a device an address. DALI has randomised
// binary-search arbitration, M-Bus has secondary addressing over fabrication
// numbers, CANopen has LSS, KNX has the programming button — Modbus has DIP
// switches and vendor conventions. The spec offers exactly two things to build
// on, and this map uses both:
//
//   * address 0 is broadcast: every slave processes the frame, none reply, so a
//     write to it can never collide no matter how many devices hear it;
//   * 248..255 are reserved, and 247 is by convention a service address.
//
// From those, the convention here:
//
//   * A device with no assigned address is SILENT. It answers nothing. That is
//     what lets any number of factory-fresh boards hang on one pair from the
//     first day without ever colliding — two devices sharing an address both
//     reply to a request for it, the frames overlap on the wire, and the master
//     gets a CRC error from a bus it cannot talk its way out of.
//   * A device is selected for commissioning in one of two ways, and only then
//     will it accept line settings:
//       - the programming button, exactly as on the KNX side. It then answers on
//         MB_SLAVE_ADDR_COMMISSIONING so the master can read its identity before
//         naming it. This needs somebody at the device.
//       - a write of its serial number to the serial-select registers, which
//         works over broadcast and therefore reaches a silent device. This needs
//         nobody at the device, which is the case that matters once a board is
//         above a ceiling.
//
// "Silent" is implemented by listening on the broadcast address itself rather
// than by shutting the stack down, and that distinction is the whole reason the
// serial-select path works. A slave configured with address 0 matches only
// frames addressed to 0, and the spec forbids answering those — so it hears
// every broadcast and can never transmit. A device that had simply stopped its
// UART would be unreachable by any means at all, including the one mechanism
// meant to reach devices nobody can get to.
//
// Assignable addresses stop at 246 so MB_SLAVE_ADDR_COMMISSIONING is never also
// somebody's real address. These names are prefixed away from the esp-modbus
// stack's own MB_ADDRESS_MIN/MAX, which say something different (1..247, the
// protocol's limits rather than this map's policy).
#define MB_SLAVE_ADDR_UNASSIGNED 0  /* the broadcast address: hears all, answers none */
#define MB_SLAVE_ADDR_MIN 1
#define MB_SLAVE_ADDR_MAX 246
#define MB_SLAVE_ADDR_COMMISSIONING 247

/// How long a serial-number selection stays armed. Long enough for a master to
/// follow it with the address write, short enough that a device is not left
/// writable because a commissioning script died halfway through.
#define MB_SERIAL_SELECT_TIMEOUT_S 30

/// True for an address a device may actually be given.
static inline bool mb_address_assignable(uint16_t address)
{
    return address >= MB_SLAVE_ADDR_MIN && address <= MB_SLAVE_ADDR_MAX;
}

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

    // Serial number, 48..50. The same six bytes the KNX device object reports
    // and the BLE channel advertises, so one board has one identity whatever is
    // asking. Big-endian pairs: serial[0]<<8 | serial[1], and so on.
    //
    // Readable rather than only printable because a master that has just found
    // a device on MB_SLAVE_ADDR_COMMISSIONING needs to know which one it is before
    // giving it a name.
    uint16_t serial_0;
    uint16_t serial_1;
    uint16_t serial_2;
    uint16_t reserved_51;
} mb_input_registers_t;

// --- Holding registers (function 03/06/16), read/write ---------------------
typedef struct {
    // Device configuration, 0..3. Changes take effect when config_commit is
    // written, not on the address write itself: changing the address of the
    // device you are talking to mid-transaction loses the response.
    //
    // A commit is accepted only while the device is selected for commissioning
    // — programming button or serial-number select, see the addressing note at
    // the top of this file. A stray master must not be able to re-address a
    // device in a running building, and a device that is merely reachable is
    // not the same thing as a device somebody meant to configure.
    uint16_t slave_address;   ///< MB_ADDRESS_MIN..MB_ADDRESS_MAX, or 0 to unassign
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

    // Commissioning, 12..15. Write this device's six-byte serial number here to
    // select it; the selection arms for MB_SERIAL_SELECT_TIMEOUT_S, during which
    // a config_commit is accepted. Writing zeros clears it.
    //
    // The point of this block is that it works over BROADCAST (address 0). A
    // broadcast needs no reply, so it reaches a device that is silent because it
    // has no address yet, and no two devices can collide answering it. That is
    // the whole commissioning sequence for a board already above a ceiling:
    //
    //   1. broadcast: write serial_select = the serial on the box label
    //   2. broadcast: write slave_address = 17, config_commit = 1
    //   3. read register 0 at address 17 to confirm
    //
    // Only the device whose serial matches acts on step 2. Every other device on
    // the pair sees both frames and ignores them.
    uint16_t serial_select_0;  ///< serial[0] << 8 | serial[1]
    uint16_t serial_select_1;  ///< serial[2] << 8 | serial[3]
    uint16_t serial_select_2;  ///< serial[4] << 8 | serial[5]
    uint16_t reserved_15;
} mb_holding_registers_t;

/// Pack a six-byte serial into three big-endian registers, the encoding both
/// the input block and the select block use.
static inline void mb_serial_encode(const uint8_t serial[6], uint16_t out[3])
{
    for (unsigned i = 0; i < 3; ++i) {
        out[i] = (uint16_t)(((uint16_t)serial[2u * i] << 8) | serial[2u * i + 1u]);
    }
}

/// True when @p selected names this device. All-zero is "no selection" rather
/// than a match, so clearing the registers releases the device instead of
/// selecting a board with a zero serial that cannot exist.
static inline bool mb_serial_selected(const uint16_t selected[3], const uint8_t serial[6])
{
    if (selected[0] == 0u && selected[1] == 0u && selected[2] == 0u) {
        return false;
    }
    uint16_t mine[3];
    mb_serial_encode(serial, mine);
    return selected[0] == mine[0] && selected[1] == mine[1] && selected[2] == mine[2];
}

// --- Commissioning state ----------------------------------------------------
// Pure decision, kept here rather than in the adapter so it is host-tested
// (main/test/test_modbus_registers.cpp). Everything about whether this device
// is on the bus, what it answers to, and whether it will take a new address
// comes out of these three inputs.

typedef struct {
    uint8_t assigned_address;  ///< MB_SLAVE_ADDR_UNASSIGNED when never commissioned
    bool programming_mode;     ///< the board's protocol-neutral "selected" state
    bool serial_selected;      ///< a matching, unexpired serial-select write
} mb_commissioning_inputs_t;

typedef struct {
    /// What the slave answers to. MB_SLAVE_ADDR_UNASSIGNED means it answers
    /// nothing while still receiving every broadcast.
    uint8_t listen_address;
    bool answers_requests;     ///< derived, for readability at the call sites
    bool accept_line_settings; ///< is a config_commit allowed right now?
} mb_commissioning_state_t;

static inline mb_commissioning_state_t
mb_commissioning_resolve(mb_commissioning_inputs_t in)
{
    mb_commissioning_state_t out;

    if (mb_address_assignable(in.assigned_address)) {
        // An addressed device keeps its address even while selected. Moving it
        // to the commissioning address would drop it off a live bus for as long
        // as somebody leant on the button, and the master already knows where
        // to find it.
        out.listen_address = in.assigned_address;
    } else if (in.programming_mode) {
        // Unassigned and somebody is standing at it. Answering on the
        // commissioning address is what lets a master read the serial number
        // back and confirm which board it just selected before naming it.
        out.listen_address = MB_SLAVE_ADDR_COMMISSIONING;
    } else {
        // Unassigned and unattended: mute. Any number of boards can sit here
        // together, because none of them will ever transmit — and all of them
        // are still listening for the broadcast that names one.
        out.listen_address = MB_SLAVE_ADDR_UNASSIGNED;
    }
    out.answers_requests = (out.listen_address != MB_SLAVE_ADDR_UNASSIGNED);

    // Either selector unlocks the write. The button proves physical presence;
    // the serial proves the master meant this device and no other. A broadcast
    // commit therefore lands on exactly one board.
    out.accept_line_settings = in.programming_mode || in.serial_selected;
    return out;
}

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
