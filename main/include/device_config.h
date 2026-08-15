// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file device_config.h
 * @brief The settings a field bus cannot configure, and who owns them.
 *
 * Every personality can configure the room controller: setpoints, modes and
 * limits are on the wire, in `control_state.h`, reachable from ETS, a Modbus
 * master or an MQTT topic. What none of them can configure is *themselves*.
 *
 *   - The Modbus address and baud rate live in NVS and are writable over
 *     Modbus. Two boards out of the box are both address 1 on the same pair,
 *     and neither can be reached to be moved.
 *   - The Wi-Fi SSID, passphrase and broker URI live in NVS and decide whether
 *     the MQTT personality has a network at all. Nothing on the network can
 *     write them, because there is no network until they are right.
 *
 * That circularity is the whole problem, and it is not a BLE problem — it is a
 * problem about settings that need a channel *other than* the one they
 * configure. KNX does not have it: ETS commissions over the same twisted pair
 * but reaches the device through the programming button and the individual
 * address, so the standard already broke the loop. Every other personality
 * needs somebody to break it for them.
 *
 * This header is the break. It is a registry of typed settings, contributed by
 * whoever owns them, with no idea what will read or write them:
 *
 *   mb_rtu_slave.c    registers  mb.addr, mb.baud
 *   mqtt_service.c    registers  net.ssid, net.pass, net.broker, ...
 *   control_service   registers  dev.name
 *
 * and one channel renders the registry to the outside world — today BLE
 * (`oob_service.h`), tomorrow a console or a USB device, without any of the
 * owners above changing a line. The registry itself is portable C with no NVS,
 * no FreeRTOS and no radio in it, so it is host-tested in
 * main/test/test_device_config.cpp.
 *
 * Threading: registration happens once, during start-up, from the task doing
 * the starting. Reads and writes afterwards come from the channel's task. The
 * item's own get/set hooks are responsible for whatever locking their storage
 * needs — which for the NVS-backed items above is none.
 */

/// Longest string value the registry carries, NUL included. Sized by the
/// longest real one — the broker URI, whose existing buffer in mqtt_service.c
/// is 128 bytes; a 63-character WPA2 passphrase fits comfortably inside it.
#define DEVICE_CONFIG_STRING_MAX 129

/// Longest key. Keys are `owner.name`, stable forever once shipped, and short
/// because they travel in a 23-byte default ATT payload.
#define DEVICE_CONFIG_KEY_MAX 16

/// Total items the registry can hold, across every owner.
#define DEVICE_CONFIG_MAX_ITEMS 32

typedef enum {
    DEVICE_CONFIG_TYPE_BOOL = 0,
    DEVICE_CONFIG_TYPE_UINT,    ///< unsigned integer, bounded by min/max
    DEVICE_CONFIG_TYPE_INT,     ///< signed integer, bounded by min/max
    DEVICE_CONFIG_TYPE_FLOAT,   ///< bounded by min/max
    DEVICE_CONFIG_TYPE_ENUM,    ///< unsigned code point with labels
    DEVICE_CONFIG_TYPE_STRING,  ///< max length carried in `max`
} device_config_type_t;

enum {
    DEVICE_CONFIG_FLAG_NONE = 0,
    /**
     * Write-only. The value never leaves the device: get() must report only
     * whether something is stored (empty string = unset, anything else = set)
     * and device_config_format() renders that as `<set>` / `<unset>`.
     *
     * This is a property of the *item*, not of the channel, on purpose. A
     * passphrase that only one transport declines to echo is a passphrase that
     * the next transport will echo.
     */
    DEVICE_CONFIG_FLAG_SECRET = 1u << 0,
    /// Diagnostics rather than configuration; writes are refused.
    DEVICE_CONFIG_FLAG_READ_ONLY = 1u << 1,
    /// Stored immediately, but only acted on at the next boot. Setting one
    /// raises device_config_reboot_pending(), which the channel reports so the
    /// installer is not left wondering why nothing changed.
    DEVICE_CONFIG_FLAG_REBOOT = 1u << 2,
};

typedef struct {
    device_config_type_t type;
    union {
        bool b;
        uint32_t u;  ///< also carries ENUM code points
        int32_t i;
        float f;
    } num;
    char str[DEVICE_CONFIG_STRING_MAX];
} device_config_value_t;

typedef struct device_config_item device_config_item_t;

struct device_config_item {
    const char *key;    ///< `owner.name`, <= DEVICE_CONFIG_KEY_MAX-1 chars
    const char *label;  ///< shown to a human; may be NULL
    const char *unit;   ///< may be NULL
    device_config_type_t type;
    uint32_t flags;

    /// Inclusive bounds for the numeric types; for STRING, `max` is the maximum
    /// length in characters and `min` is ignored. Out-of-range writes are
    /// *refused*, never clamped: a device that quietly turns Modbus address 300
    /// into 247 is a device nobody can find.
    float min;
    float max;

    /// ENUM only: `enum_count` labels indexed by code point. May be NULL.
    const char *const *enum_labels;
    size_t enum_count;

    /// Read the current value. Never returns secret material — see
    /// DEVICE_CONFIG_FLAG_SECRET. Required.
    esp_err_t (*get)(const device_config_item_t *item, device_config_value_t *out);

    /// Store a value that has already passed type and range validation.
    /// NULL means read-only regardless of the flags.
    esp_err_t (*set)(const device_config_item_t *item, const device_config_value_t *value);

    /// Owner's private pointer, so one hook pair can serve several items.
    void *ctx;
};

/**
 * @brief Contribute items to the registry.
 *
 * @p items must have static storage duration: the registry keeps the pointers,
 * not copies. Call once per owner, from its start() path. Returns
 * ESP_ERR_NO_MEM past DEVICE_CONFIG_MAX_ITEMS and ESP_ERR_INVALID_ARG for a
 * malformed item (missing or over-long key, missing get(), duplicate key) —
 * and registers nothing at all in that case, so a partial owner is never half
 * visible.
 */
esp_err_t device_config_register(const device_config_item_t *items, size_t count);

/// Drop every registration. Exists for the host tests; the firmware never
/// unregisters, because nothing in it ever stops.
void device_config_reset(void);

size_t device_config_count(void);

/// Item by index, in registration order, or NULL. The index is the handle a
/// channel uses on the wire, so it must stay stable for the life of a boot —
/// which it does, because registration only happens during start-up.
const device_config_item_t *device_config_at(size_t index);

const device_config_item_t *device_config_find(const char *key);

/// Index of @p item, or SIZE_MAX if it is not registered.
size_t device_config_index_of(const device_config_item_t *item);

/// Read through the item's own hook. Fails with ESP_ERR_INVALID_ARG on NULL.
esp_err_t device_config_get(const device_config_item_t *item, device_config_value_t *out);

/**
 * @brief Validate and store.
 *
 * Refuses with ESP_ERR_INVALID_ARG on a type mismatch, an out-of-range number,
 * an over-long string or an unknown enum code point, and with
 * ESP_ERR_NOT_SUPPORTED for a read-only item. On success, raises the
 * reboot-pending flag if the item carries DEVICE_CONFIG_FLAG_REBOOT.
 */
esp_err_t device_config_set(const device_config_item_t *item, const device_config_value_t *value);

/**
 * @brief Parse a textual value against an item's type and bounds.
 *
 * Text is the registry's interchange form: it is self-describing, it survives a
 * 23-byte ATT payload for every type the registry has, and it is the same thing
 * a console channel would need. `on`/`off`/`true`/`false`/`1`/`0` are all
 * accepted for BOOL; an ENUM accepts either its code point or a label.
 */
esp_err_t device_config_parse(const device_config_item_t *item,
                              const char *text,
                              device_config_value_t *out);

/// Convenience: parse then set.
esp_err_t device_config_set_text(const device_config_item_t *item, const char *text);

/**
 * @brief Render a value as text, redacting secrets.
 *
 * Returns the number of characters written (excluding the NUL), or -1 if the
 * buffer is too small. A SECRET item renders as `<set>` or `<unset>` whatever
 * its get() hook returned, so a channel cannot leak one by forgetting to check.
 */
int device_config_format(const device_config_item_t *item,
                         const device_config_value_t *value,
                         char *out,
                         size_t out_len);

/// Convenience: get then format.
int device_config_get_text(const device_config_item_t *item, char *out, size_t out_len);

/// Name of a type, for descriptors and logs. Never NULL.
const char *device_config_type_name(device_config_type_t type);

/// True once a DEVICE_CONFIG_FLAG_REBOOT item has been written this boot.
bool device_config_reboot_pending(void);

#ifdef __cplusplus
}
#endif
