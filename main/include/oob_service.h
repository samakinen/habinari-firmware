// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file oob_service.h
 * @brief The out-of-band service channel: a way in that is not the field bus.
 *
 * `device_config.h` explains *why* there has to be one — some settings decide
 * whether the bus works at all, so they cannot be written over it. This header
 * is the lifecycle of the channel that carries them. The implementation today
 * is BLE GATT (`src/oob_ble.c`); the contract deliberately says nothing about
 * radios, so a console or USB channel would replace the .c file and nothing
 * else.
 *
 * ## What it is not
 *
 * It is not a `protocol_adapter_t`. An adapter maps `control_state_t` onto a
 * wire format and is how a building talks to the room. This is a service
 * hatch: an installer standing at the device, during commissioning, reading
 * diagnostics and writing the handful of settings that bootstrap a personality.
 * Registering it as a fourth personality would say the device has four ways to
 * be operated, and it does not.
 *
 * ## Why KNX builds do not have one
 *
 * ETS is already a complete out-of-band configuration channel: it reaches the
 * device through the programming button and the individual address, so the
 * circularity never arises, and everything this channel could offer — settings,
 * identify, diagnostics — is in the application program. A KNX image therefore
 * links no BLE at all, and the build enforces it for the same two reasons the
 * Wi-Fi personality is excluded: the bit-banged TP1 receiver has about one
 * microsecond of ISR-jitter margin, and the KNX bus terminal cannot supply a
 * radio. Adding BLE "just for commissioning" would spend both.
 *
 * ## Access model
 *
 * The channel is closed by default. It advertises exactly while the board is in
 * **programming mode** — the one protocol-neutral "this device is selected"
 * state, raised by the programming button, shown on the LED, and shared with
 * KNX individual addressing and Modbus re-addressing. There is no separate BLE
 * window and deliberately no second gesture: one button, one LED, one meaning
 * across every protocol the board speaks.
 *
 * A device that has never been committed puts *itself* into programming mode at
 * boot, because there is no other way in and nobody has told it anything yet.
 * Programming mode then lapses on its own
 * (CONFIG_HABINARI_PROGRAMMING_MODE_TIMEOUT_S), so a board is never left
 * advertising because somebody walked away.
 *
 * Inside it, a link must be paired with LE Secure Connections and a six-digit
 * passkey derived from the device's eFuse root secret — the same root the KNX
 * FDSK comes from, so it is unique per device, unreadable by software,
 * printable at manufacture and survives a factory reset. Every characteristic
 * requires an encrypted, authenticated link.
 */

/* --- Wire contract -------------------------------------------------------- */
/* Little-endian, packed, versioned. Documented in docs/ble-commissioning.md and
 * parsed by tools/ble_config.py; change the version byte, never the meaning of
 * a field. */

#define OOB_WIRE_VERSION 1

/// Reported in oob_device_info_t::flags.
enum {
    OOB_INFO_FLAG_PROVISIONED = 1u << 0,     ///< configuration has been committed at least once
    OOB_INFO_FLAG_REBOOT_PENDING = 1u << 1,  ///< a stored setting needs a restart to take effect
    OOB_INFO_FLAG_SECRET_FROM_EFUSE = 1u << 2,  ///< passkey came from the eFuse root secret
};

/// Read from the Device Info characteristic. Fixed prefix, then NUL-separated
/// firmware version and comma-separated personality names.
typedef struct __attribute__((packed)) {
    uint8_t version;     ///< OOB_WIRE_VERSION
    uint8_t item_count;  ///< registered device_config items
    uint8_t flags;       ///< OOB_INFO_FLAG_*
    uint8_t reserved;
    uint8_t serial[6];  ///< the same six bytes the KNX serial number uses
    char text[64];      ///< "<firmware>\0<personality,personality>\0"
} oob_device_info_t;

/// Read from the Status characteristic, and notified on change. Scaling and the
/// "no reading" sentinel follow the Modbus register map, so the two views of
/// this device cannot disagree about what an absent sensor looks like.
enum {
    OOB_STATUS_FLAG_HAS_SENSOR_DATA = 1u << 0,
    OOB_STATUS_FLAG_IDENTIFY = 1u << 1,
    OOB_STATUS_FLAG_DEVICE_FAULT = 1u << 2,
    OOB_STATUS_FLAG_REBOOT_PENDING = 1u << 3,
};

#define OOB_STATUS_INVALID_SIGNED ((int16_t)0x8000)

typedef struct __attribute__((packed)) {
    uint8_t version;  ///< OOB_WIRE_VERSION
    uint8_t flags;    ///< OOB_STATUS_FLAG_*
    int16_t temperature_c_x100;
    int16_t humidity_pct_x100;
    int16_t co2_ppm;
    int16_t active_setpoint_c_x100;
    uint8_t heating_percent;
    uint8_t cooling_percent;
    uint8_t ventilation_percent;
    uint8_t hvac_mode;
    uint8_t controller_mode;
    uint8_t sensor_health_mask;
    uint32_t uptime_s;
} oob_status_t;

/// Opcodes for the Control characteristic. Every one of them requires an
/// authenticated link; the destructive one requires a magic word as well.
enum {
    OOB_CMD_COMMIT = 0x01,         ///< mark provisioned, leave programming mode, reboot
    OOB_CMD_REBOOT = 0x02,         ///< restart without marking anything
    OOB_CMD_IDENTIFY = 0x03,       ///< payload u8: 0 = off, non-zero = on
    OOB_CMD_FACTORY_RESET = 0x04,  ///< payload: the four bytes "RST!"
    /// Leave programming mode now. Stops advertising, and on a build with the
    /// Modbus personality also takes an unaddressed device back off the bus —
    /// one state, so leaving it means leaving all of it.
    OOB_CMD_END_COMMISSIONING = 0x05,
};

#define OOB_FACTORY_RESET_MAGIC "RST!"

/* --- Lifecycle ------------------------------------------------------------ */

#if CONFIG_HABINARI_OOB_BLE

/**
 * @brief Arm the channel. Call after every owner has registered its settings,
 *        i.e. after the protocol adapters have started.
 *
 * Arming loads the configuration and prepares the GATT content; it does not
 * start the BLE radio. The radio runs only in programming mode, because an
 * initialised controller keeps the Wi-Fi/BLE coexistence arbiter slicing the
 * one radio between them for the whole life of the device, and the field bus
 * on an IP build is riding on that Wi-Fi.
 *
 * A device that has never been committed asks for programming mode here, since
 * nothing else can. A failure is not fatal: the device keeps sampling and
 * controlling, it just cannot be reconfigured out of band.
 */
esp_err_t oob_service_start(void);

/**
 * @brief The board's programming mode changed; the radio follows it exactly.
 *
 * Called from whoever routes the board's programming state — main.c, from the
 * one control_service callback. Records intent and returns; the radio is
 * brought up and taken down by oob_service_loop(). Safe from any task,
 * including the NimBLE host task, precisely because it does no radio work.
 */
void oob_service_set_programming_mode(bool active);

/**
 * @brief Reconcile the radio with programming mode. Call from the main loop.
 *
 * The one place the BLE controller and host are started and stopped. Stopping
 * blocks until the NimBLE host task has exited, so it needs a caller that holds
 * no locks and is not the host task itself — the board's poll loop is both.
 * Cheap and idempotent when there is nothing to do.
 */
void oob_service_loop(void);

/// True while advertising or connected.
bool oob_service_advertising(void);

/// True while an authenticated client is connected.
bool oob_service_client_connected(void);

/// True while a connected client has asked for the identify blink
bool oob_service_identify_active(void);

#else /* !CONFIG_HABINARI_OOB_BLE */

/* No channel in this image. Inline no-ops rather than #ifdefs at every call
 * site: main.c should not know which service channels exist, for the same
 * reason it does not know which personalities do. */
static inline esp_err_t oob_service_start(void) { return ESP_OK; }
static inline void oob_service_set_programming_mode(bool active) { (void)active; }
static inline void oob_service_loop(void) { }
static inline bool oob_service_advertising(void) { return false; }
static inline bool oob_service_client_connected(void) { return false; }
static inline bool oob_service_identify_active(void) { return false; }

#endif /* CONFIG_HABINARI_OOB_BLE */

#ifdef __cplusplus
}
#endif
