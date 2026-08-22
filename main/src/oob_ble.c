// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/*
 * The out-of-band service channel, over BLE GATT.
 *
 * This file is a transport and nothing else. It owns no settings: it renders
 * whatever `device_config.h` was given by the owners of those settings, and it
 * reads live values through `control_state.h` like any other client. Replacing
 * BLE with a console or a USB device means replacing this file and touching
 * nothing else — which is the test of whether the split in device_config.h was
 * worth making.
 *
 * See oob_service.h for the access model and docs/ble-commissioning.md for the
 * GATT contract and the host-side tool.
 */
#include "oob_service.h"

#if CONFIG_HABINARI_OOB_BLE

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "control_service.h"
#include "control_state.h"
#include "device_config.h"
#include "device_default_name.h"
#include "device_secret.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs.h"
#include "protocol_adapter.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "oob-ble";

/* The wire structs are parsed byte-for-byte by tools/ble_config.py and by
 * anything else written against docs/ble-commissioning.md. Padding creeping in
 * would shift every field after it and the symptom would be nonsense readings
 * rather than a build failure, so pin the sizes here. */
_Static_assert(sizeof(oob_device_info_t) == 74, "device info layout changed");
_Static_assert(sizeof(oob_status_t) == 20, "status layout changed");

/* Bond storage lives in NVS and is set up by the NimBLE store; declared here
 * because esp-nimble exposes it through the example headers rather than a
 * public one. */
void ble_store_config_init(void);

/* --- Persistent state ----------------------------------------------------- */

#define OOB_NVS_NAMESPACE "oob"
#define OOB_NVS_KEY_PROVISIONED "prov"

/* --- Identifiers ---------------------------------------------------------- */
/*
 * One vendor 128-bit service. The base was generated once and is fixed forever;
 * the low byte selects the characteristic, so a reader of a packet capture can
 * tell them apart at a glance.
 *
 *   1d298b96-21f7-4109-87a0-17de995500NN
 */
#define OOB_UUID128(nn)                                                                          \
    BLE_UUID128_INIT(0x##nn, 0x00, 0x55, 0x99, 0xde, 0x17, 0xa0, 0x87, 0x09, 0x41, 0xf7, 0x21,   \
                     0x96, 0x8b, 0x29, 0x1d)

static const ble_uuid128_t s_uuid_service = OOB_UUID128(00);
static const ble_uuid128_t s_uuid_info = OOB_UUID128(01);
static const ble_uuid128_t s_uuid_select = OOB_UUID128(02);
static const ble_uuid128_t s_uuid_descriptor = OOB_UUID128(03);
static const ble_uuid128_t s_uuid_value = OOB_UUID128(04);
static const ble_uuid128_t s_uuid_control = OOB_UUID128(05);
static const ble_uuid128_t s_uuid_status = OOB_UUID128(06);

/* --- Runtime state -------------------------------------------------------- */

/* The descriptor blob is the largest thing served: a 13-byte header, three
 * strings and up to eight enum labels. Sized so no item can overflow it and
 * checked when it is built. */
#define OOB_DESCRIPTOR_MAX 256

typedef struct {
    bool started;
    bool synced;
    bool provisioned;

    uint8_t own_addr_type;
    char device_name[24];
    uint8_t serial[6];

    uint32_t passkey;
    bool passkey_from_efuse;

    esp_timer_handle_t leave_timer;
    esp_timer_handle_t status_timer;
    esp_timer_handle_t restart_timer;
    esp_timer_handle_t adv_timer;
    volatile bool advertising;
    /// Mirrors the board's programming mode; advertising follows it exactly.
    volatile bool programming_mode;
    /// The OOB_CMD_IDENTIFY blink, independent of programming_mode — see
    /// oob_service_identify_active(). Cleared on disconnect so a fresh session
    /// never inherits the previous client's blink.
    volatile bool identify_blink;

    /* Everything a GATT write asks for happens on a timer rather than in the
     * access callback. Two reasons, and they are both real: the callback runs on
     * the NimBLE host task with the host lock held, so calling back into
     * ble_gap_terminate() from there is asking for a self-deadlock; and
     * rebooting before returning would drop the ATT response, so the client
     * would see its own successful write as a dropped link. */
    volatile bool factory_reset_requested;

    uint16_t conn_handle;
    uint16_t status_val_handle;
    volatile bool authenticated;
    volatile bool status_subscribed;

    /* Which registry item the Value and Descriptor characteristics refer to.
     * One connection at a time (see CONFIG_BT_NIMBLE_MAX_CONNECTIONS in the
     * BLE defaults), so one selection is the whole story. */
    uint16_t selected;
} oob_ctx_t;

static oob_ctx_t s_ctx = {
    .conn_handle = BLE_HS_CONN_HANDLE_NONE,
};

static void advertise_start(void);

/* --- Provisioned marker --------------------------------------------------- */

static bool load_provisioned(void)
{
    nvs_handle_t handle;
    if (nvs_open(OOB_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    uint8_t value = 0;
    const esp_err_t err = nvs_get_u8(handle, OOB_NVS_KEY_PROVISIONED, &value);
    nvs_close(handle);
    return err == ESP_OK && value != 0;
}

static void store_provisioned(void)
{
    nvs_handle_t handle;
    if (nvs_open(OOB_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot open NVS to record commissioning");
        return;
    }
    if (nvs_set_u8(handle, OOB_NVS_KEY_PROVISIONED, 1) == ESP_OK) {
        nvs_commit(handle);
        s_ctx.provisioned = true;
    }
    nvs_close(handle);
}

/* --- Wire encoding -------------------------------------------------------- */

static uint8_t info_flags(void)
{
    uint8_t flags = 0;
    if (s_ctx.provisioned) {
        flags |= OOB_INFO_FLAG_PROVISIONED;
    }
    if (device_config_reboot_pending()) {
        flags |= OOB_INFO_FLAG_REBOOT_PENDING;
    }
    if (s_ctx.passkey_from_efuse) {
        flags |= OOB_INFO_FLAG_SECRET_FROM_EFUSE;
    }
    return flags;
}

static void build_device_info(oob_device_info_t *out)
{
    memset(out, 0, sizeof(*out));
    out->version = OOB_WIRE_VERSION;
    out->item_count = (uint8_t)device_config_count();
    out->flags = info_flags();
    memcpy(out->serial, s_ctx.serial, sizeof(out->serial));

    const esp_app_desc_t *app = esp_app_get_description();
    const int fw = snprintf(out->text, sizeof(out->text), "%s", app != NULL ? app->version : "?");
    if (fw < 0 || (size_t)fw + 1u >= sizeof(out->text)) {
        return;
    }

    /* The personalities this image carries, comma separated, after the NUL.
     * A technician holding a phone in a plant room should be able to see that
     * the board in front of them is the RS-485 one without reading a label. */
    size_t offset = (size_t)fw + 1u;
    size_t count = 0;
    const protocol_adapter_t *const *adapters = protocol_adapters(&count);
    for (size_t i = 0; i < count && offset + 1u < sizeof(out->text); ++i) {
        const int written = snprintf(out->text + offset, sizeof(out->text) - offset, "%s%s",
                                     i == 0 ? "" : ",", adapters[i]->name);
        if (written < 0 || (size_t)written >= sizeof(out->text) - offset) {
            break;
        }
        offset += (size_t)written;
    }
}

static int16_t scaled_or_invalid(float value, bool valid, float scale)
{
    if (!valid) {
        return OOB_STATUS_INVALID_SIGNED;
    }
    const float scaled = value * scale;
    if (scaled <= -32767.0f || scaled >= 32767.0f) {
        return OOB_STATUS_INVALID_SIGNED;
    }
    return (int16_t)(scaled < 0.0f ? scaled - 0.5f : scaled + 0.5f);
}

static void build_status(oob_status_t *out)
{
    control_state_t state;
    control_state_get(&state);

    memset(out, 0, sizeof(*out));
    out->version = OOB_WIRE_VERSION;
    if (state.has_sensor_data) {
        out->flags |= OOB_STATUS_FLAG_HAS_SENSOR_DATA;
    }
    if (s_ctx.identify_blink) {
        out->flags |= OOB_STATUS_FLAG_IDENTIFY;
    }
    if (state.device_fault) {
        out->flags |= OOB_STATUS_FLAG_DEVICE_FAULT;
    }
    if (device_config_reboot_pending()) {
        out->flags |= OOB_STATUS_FLAG_REBOOT_PENDING;
    }

    const bool have = state.has_sensor_data;
    out->temperature_c_x100 = scaled_or_invalid(state.sensors.temperature.value,
                                                have && state.sensors.temperature.valid, 100.0f);
    out->humidity_pct_x100 =
        scaled_or_invalid(state.sensors.humidity.value, have && state.sensors.humidity.valid,
                          100.0f);
    out->co2_ppm = scaled_or_invalid(state.sensors.co2.value,
                                     have && state.sensors.co2.valid, 1.0f);
    out->active_setpoint_c_x100 = scaled_or_invalid(state.active_setpoint_c, true, 100.0f);
    out->heating_percent = state.heating_percent;
    out->cooling_percent = state.cooling_percent;
    out->ventilation_percent = state.ventilation_percent;
    out->hvac_mode = state.hvac_operating_mode;
    out->controller_mode = state.controller_mode;
    out->sensor_health_mask = state.sensors.health.healthy_mask;
    out->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
}

/// Append a NUL-terminated string, returning false when it would not fit.
static bool append_string(uint8_t *buf, size_t cap, size_t *offset, const char *text)
{
    const char *source = text != NULL ? text : "";
    const size_t len = strlen(source) + 1u;
    if (*offset + len > cap) {
        return false;
    }
    memcpy(buf + *offset, source, len);
    *offset += len;
    return true;
}

/**
 * Item descriptor: a fixed little-endian header, then NUL-terminated key,
 * label and unit, then one NUL-terminated label per enum code point.
 *
 * Header, in order: u16 index, u8 type, u8 flags, f32 min, f32 max,
 * u8 enum_count. Documented in docs/ble-commissioning.md; the version byte in
 * the Device Info characteristic covers it.
 */
static int build_descriptor(uint16_t index, uint8_t *buf, size_t cap)
{
    const device_config_item_t *item = device_config_at(index);
    if (item == NULL) {
        return -1;
    }

    if (cap < 13u) {
        return -1;
    }
    const uint8_t enum_count = (uint8_t)(item->enum_count > 255u ? 255u : item->enum_count);

    size_t offset = 0;
    buf[offset++] = (uint8_t)(index & 0xFFu);
    buf[offset++] = (uint8_t)(index >> 8);
    buf[offset++] = (uint8_t)item->type;
    buf[offset++] = (uint8_t)item->flags;
    memcpy(buf + offset, &item->min, sizeof(float));
    offset += sizeof(float);
    memcpy(buf + offset, &item->max, sizeof(float));
    offset += sizeof(float);
    buf[offset++] = enum_count;

    if (!append_string(buf, cap, &offset, item->key)
        || !append_string(buf, cap, &offset, item->label)
        || !append_string(buf, cap, &offset, item->unit)) {
        return -1;
    }
    for (uint8_t i = 0; i < enum_count; ++i) {
        if (!append_string(buf, cap, &offset, item->enum_labels[i])) {
            return -1;
        }
    }
    return (int)offset;
}

/* --- GATT access ---------------------------------------------------------- */

static int read_flat(struct ble_gatt_access_ctxt *ctxt, const void *data, size_t len)
{
    return os_mbuf_append(ctxt->om, data, (uint16_t)len) == 0 ? 0
                                                              : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int write_flat(struct ble_gatt_access_ctxt *ctxt, void *out, size_t max, uint16_t *out_len)
{
    const uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len > max) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (ble_hs_mbuf_to_flat(ctxt->om, out, (uint16_t)max, out_len) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

/// Deferral delay for anything a GATT write asks for: long enough for the ATT
/// response to leave the radio, short enough that the client sees it happen.
#define OOB_DEFER_MS 500

static void schedule_restart(const char *why)
{
    ESP_LOGW(TAG, "Restarting: %s", why);
    esp_timer_start_once(s_ctx.restart_timer, OOB_DEFER_MS * 1000);
}

/// Restart advertising from the timer task rather than from the GAP callback.
///
/// NimBLE can report a failed connection from inside ble_gap_conn_broken(),
/// which frees the connection object only *after* the callback returns. Calling
/// ble_gap_adv_start() from there finds the (single-entry) connection pool still
/// full and fails with BLE_HS_ENOMEM, and no disconnect event follows to give a
/// second chance -- so the board goes quiet until programming mode lapses. Zero
/// delay is enough; the point is only to land after the host task unwinds.
static void schedule_advertise_restart(void)
{
    esp_timer_stop(s_ctx.adv_timer);
    esp_timer_start_once(s_ctx.adv_timer, 0);
}

/// Ask to leave programming mode from the timer task rather than from here.
/// Going out through control_service means the LED, the Modbus side and this
/// channel all end commissioning together, because they are one state.
static void schedule_leave_programming_mode(void)
{
    esp_timer_stop(s_ctx.leave_timer);
    esp_timer_start_once(s_ctx.leave_timer, OOB_DEFER_MS * 1000);
}

static int handle_control(const uint8_t *payload, uint16_t len)
{
    if (len < 1u) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    switch (payload[0]) {
    case OOB_CMD_COMMIT:
        /* Stored here, because it must not be lost if the restart beats it; the
         * restart leaves programming mode more thoroughly than anything this
         * function could. */
        store_provisioned();
        schedule_restart("configuration committed over BLE");
        return 0;

    case OOB_CMD_REBOOT:
        schedule_restart("restart requested over BLE");
        return 0;

    case OOB_CMD_IDENTIFY:
        if (len < 2u) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        /* Flashes the LED so an installer can tell which unit they are holding
         * a link to. */
        s_ctx.identify_blink = (payload[1] != 0u);
        return 0;

    case OOB_CMD_FACTORY_RESET:
        /* An authenticated link is not enough for something irreversible: the
         * magic word means the client had to mean it, not merely mistype an
         * opcode. */
        if (len < 1u + sizeof(OOB_FACTORY_RESET_MAGIC) - 1u
            || memcmp(payload + 1, OOB_FACTORY_RESET_MAGIC, sizeof(OOB_FACTORY_RESET_MAGIC) - 1u)
                   != 0) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        ESP_LOGW(TAG, "Factory reset requested over BLE");
        s_ctx.factory_reset_requested = true;
        schedule_restart("factory reset requested over BLE");
        return 0;

    case OOB_CMD_END_COMMISSIONING:
        schedule_leave_programming_mode();
        return 0;

    default:
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }
}

static int chr_access(uint16_t conn_handle,
                      uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt,
                      void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    const ble_uuid_t *uuid = ctxt->chr->uuid;

    if (ble_uuid_cmp(uuid, &s_uuid_info.u) == 0) {
        oob_device_info_t info;
        build_device_info(&info);
        return read_flat(ctxt, &info, sizeof(info));
    }

    if (ble_uuid_cmp(uuid, &s_uuid_select.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            const uint16_t selected = s_ctx.selected;
            return read_flat(ctxt, &selected, sizeof(selected));
        }
        uint16_t selected = 0;
        uint16_t len = 0;
        const int err = write_flat(ctxt, &selected, sizeof(selected), &len);
        if (err != 0) {
            return err;
        }
        if (len != sizeof(selected) || device_config_at(selected) == NULL) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        s_ctx.selected = selected;
        return 0;
    }

    if (ble_uuid_cmp(uuid, &s_uuid_descriptor.u) == 0) {
        uint8_t blob[OOB_DESCRIPTOR_MAX];
        const int len = build_descriptor(s_ctx.selected, blob, sizeof(blob));
        if (len < 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        return read_flat(ctxt, blob, (size_t)len);
    }

    if (ble_uuid_cmp(uuid, &s_uuid_value.u) == 0) {
        const device_config_item_t *item = device_config_at(s_ctx.selected);
        if (item == NULL) {
            return BLE_ATT_ERR_UNLIKELY;
        }

        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            /* Redaction is device_config_format()'s job, not this file's — see
             * DEVICE_CONFIG_FLAG_SECRET. A transport that had to remember would
             * eventually forget. */
            char text[DEVICE_CONFIG_STRING_MAX + 16];
            const int written = device_config_get_text(item, text, sizeof(text));
            if (written < 0) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            return read_flat(ctxt, text, (size_t)written);
        }

        char text[DEVICE_CONFIG_STRING_MAX] = {0};
        uint16_t len = 0;
        const int err = write_flat(ctxt, text, sizeof(text) - 1u, &len);
        if (err != 0) {
            return err;
        }
        text[len] = '\0';

        const esp_err_t set = device_config_set_text(item, text);
        if (set == ESP_ERR_NOT_SUPPORTED) {
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        if (set != ESP_OK) {
            /* The value was refused, not stored badly — a range or format
             * error the installer can correct. */
            ESP_LOGW(TAG, "Rejected write to %s: %s", item->key, esp_err_to_name(set));
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        ESP_LOGI(TAG, "Set %s", item->key);
        return 0;
    }

    if (ble_uuid_cmp(uuid, &s_uuid_control.u) == 0) {
        uint8_t payload[16] = {0};
        uint16_t len = 0;
        const int err = write_flat(ctxt, payload, sizeof(payload), &len);
        if (err != 0) {
            return err;
        }
        return handle_control(payload, len);
    }

    if (ble_uuid_cmp(uuid, &s_uuid_status.u) == 0) {
        oob_status_t status;
        build_status(&status);
        return read_flat(ctxt, &status, sizeof(status));
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* Every characteristic requires an encrypted *and* authenticated link with a
 * full-length key. Authenticated is the one that matters: encryption alone is
 * satisfied by Just Works pairing, which any passer-by can complete. */
#define OOB_READ_FLAGS (BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN)
#define OOB_WRITE_FLAGS                                                                          \
    (BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN)
#define OOB_MIN_KEY_SIZE 16

static const struct ble_gatt_svc_def s_gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_uuid_service.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &s_uuid_info.u,
                .access_cb = chr_access,
                .flags = OOB_READ_FLAGS,
                .min_key_size = OOB_MIN_KEY_SIZE,
            },
            {
                .uuid = &s_uuid_select.u,
                .access_cb = chr_access,
                .flags = OOB_READ_FLAGS | OOB_WRITE_FLAGS,
                .min_key_size = OOB_MIN_KEY_SIZE,
            },
            {
                .uuid = &s_uuid_descriptor.u,
                .access_cb = chr_access,
                .flags = OOB_READ_FLAGS,
                .min_key_size = OOB_MIN_KEY_SIZE,
            },
            {
                .uuid = &s_uuid_value.u,
                .access_cb = chr_access,
                .flags = OOB_READ_FLAGS | OOB_WRITE_FLAGS,
                .min_key_size = OOB_MIN_KEY_SIZE,
            },
            {
                .uuid = &s_uuid_control.u,
                .access_cb = chr_access,
                .flags = OOB_WRITE_FLAGS,
                .min_key_size = OOB_MIN_KEY_SIZE,
            },
            {
                .uuid = &s_uuid_status.u,
                .access_cb = chr_access,
                .flags = OOB_READ_FLAGS | BLE_GATT_CHR_F_NOTIFY,
                .min_key_size = OOB_MIN_KEY_SIZE,
                .val_handle = &s_ctx.status_val_handle,
            },
            {0},
        },
    },
    {0},
};

/* --- Status notifications ------------------------------------------------- */

static void status_timer_cb(void *arg)
{
    (void)arg;
    if (!s_ctx.status_subscribed || s_ctx.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    oob_status_t status;
    build_status(&status);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&status, sizeof(status));
    if (om == NULL) {
        return;
    }
    /* Failure here means the link went away between the check and the send,
     * which the disconnect handler is already dealing with. */
    ble_gatts_notify_custom(s_ctx.conn_handle, s_ctx.status_val_handle, om);
}

/* --- GAP ------------------------------------------------------------------ */

static void restart_timer_cb(void *arg)
{
    (void)arg;
    if (s_ctx.factory_reset_requested) {
        control_service_factory_reset(); /* erases NVS, bonds included; reboots */
    }
    esp_restart();
}

static void adv_timer_cb(void *arg)
{
    (void)arg;
    advertise_start();
}

static void leave_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Ending commissioning at the client's request");
    /* Not a private "stop advertising": programming mode is the state, so this
     * goes out the front door and comes back through main.c's programming-mode
     * callback, taking the LED and the Modbus commissioning address with it. */
    control_service_set_programming_mode(false);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "Connection failed (status %d)", event->connect.status);
            s_ctx.advertising = false;
            schedule_advertise_restart();
            return 0;
        }
        s_ctx.conn_handle = event->connect.conn_handle;
        s_ctx.authenticated = false;
        s_ctx.advertising = false;
        s_ctx.selected = 0;
        ESP_LOGI(TAG, "Client connected; awaiting pairing");
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Client disconnected (reason 0x%04x)", event->disconnect.reason);
        s_ctx.conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ctx.authenticated = false;
        s_ctx.status_subscribed = false;
        s_ctx.identify_blink = false;
        esp_timer_stop(s_ctx.status_timer);
        /* Still selected? Then advertise again: a dropped link during
         * commissioning must not mean a walk back to the button. Programming
         * mode's own timeout is what eventually ends this. */
        schedule_advertise_restart();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        if (event->enc_change.status != 0) {
            ESP_LOGE(TAG, "Encryption change failed: status 0x%04x", (unsigned)event->enc_change.status);
            return 0;
        }
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
            s_ctx.authenticated = desc.sec_state.encrypted && desc.sec_state.authenticated;
            ESP_LOGI(TAG, "Link security: encrypted=%d authenticated=%d bonded=%d",
                     desc.sec_state.encrypted, desc.sec_state.authenticated, desc.sec_state.bonded);
        }
        return 0;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            struct ble_sm_io io = {
                .action = BLE_SM_IOACT_DISP,
                .passkey = s_ctx.passkey,
            };
            /* The board has no display, so "display only" means the label. The
             * passkey is logged too, which is how it is read on the bench —
             * and why the console must not be exposed in the field. */
            ESP_LOGW(TAG, "Pairing passkey: %06" PRIu32, s_ctx.passkey);
            ble_sm_inject_io(event->passkey.conn_handle, &io);
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* A client that lost its bond (reinstalled app, new phone) would
         * otherwise be locked out until a factory reset. It still has to know
         * the passkey to get anywhere. */
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_ctx.status_val_handle) {
            s_ctx.status_subscribed = event->subscribe.cur_notify;
            if (s_ctx.status_subscribed) {
                esp_timer_start_periodic(s_ctx.status_timer,
                                         CONFIG_HABINARI_OOB_BLE_STATUS_PERIOD_MS * 1000);
            } else {
                esp_timer_stop(s_ctx.status_timer);
            }
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        s_ctx.advertising = false;
        return 0;

    default:
        return 0;
    }
}

static void advertise_start(void)
{
    if (s_ctx.advertising || !s_ctx.synced || !s_ctx.programming_mode
        || s_ctx.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    /* The name and the 128-bit service UUID do not both fit in 31 bytes, so the
     * UUID goes in the scan response. A scanner filtering on the service still
     * finds the device; a human scrolling a list still reads the name. */
    struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .name = (uint8_t *)s_ctx.device_name,
        .name_len = (uint8_t)strlen(s_ctx.device_name),
        .name_is_complete = 1,
        .tx_pwr_lvl_is_present = 1,
        .tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO,
    };
    int err = ble_gap_adv_set_fields(&fields);
    if (err != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", err);
        return;
    }

    struct ble_hs_adv_fields scan_response = {
        .uuids128 = (ble_uuid128_t *)&s_uuid_service,
        .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };
    err = ble_gap_adv_rsp_set_fields(&scan_response);
    if (err != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", err);
        return;
    }

    const struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    err = ble_gap_adv_start(s_ctx.own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (err != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", err);
        return;
    }
    s_ctx.advertising = true;
    ESP_LOGI(TAG, "Advertising as '%s'", s_ctx.device_name);
}

static void on_sync(void)
{
    if (ble_hs_util_ensure_addr(0) != 0 || ble_hs_id_infer_auto(0, &s_ctx.own_addr_type) != 0) {
        ESP_LOGE(TAG, "No usable BLE address");
        return;
    }
    s_ctx.synced = true;

    /* Programming mode may already be on — the boot-time unprovisioned check
     * asks for it before the host has finished coming up, and an impatient
     * button press during start-up would too. */
    advertise_start();
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset, reason %d", reason);
    s_ctx.synced = false;
    s_ctx.advertising = false;
    s_ctx.conn_handle = BLE_HS_CONN_HANDLE_NONE;
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* --- Public lifecycle ----------------------------------------------------- */

static void build_device_name(void)
{
    /* A configured name wins, because in a corridor of identical boards the one
     * the commissioner labelled is the one they are looking for. Otherwise
     * device_default_name() — the same fallback mqtt_service.c uses, so a
     * board's BLE advertisement and its Home Assistant device card agree on a
     * name until somebody sets one. */
    const device_config_item_t *item = device_config_find("dev.name");
    if (item != NULL) {
        char configured[DEVICE_CONFIG_STRING_MAX + 16];
        const int written = device_config_get_text(item, configured, sizeof(configured));
        /* The item bounds itself well inside device_name, but the registry is
         * the only thing enforcing that and it is in another file. Checking the
         * length here means a later relaxation of that bound produces a fallback
         * name rather than a silently halved one. */
        if (written > 0 && (size_t)written < sizeof(s_ctx.device_name)) {
            memcpy(s_ctx.device_name, configured, (size_t)written + 1u);
            return;
        }
        if (written >= (int)sizeof(s_ctx.device_name)) {
            ESP_LOGW(TAG, "Configured device name is too long to advertise; using the fallback");
        }
    }
    device_default_name(&s_ctx.serial[3], s_ctx.device_name, sizeof(s_ctx.device_name));
}

esp_err_t oob_service_start(void)
{
    if (s_ctx.started) {
        return ESP_OK;
    }

    if (esp_read_mac(s_ctx.serial, ESP_MAC_BASE) != ESP_OK) {
        ESP_LOGW(TAG, "esp_read_mac failed; device name and serial will be zeros");
    }
    build_device_name();
    s_ctx.provisioned = load_provisioned();
    device_secret_ble_passkey(&s_ctx.passkey, &s_ctx.passkey_from_efuse);

    const esp_timer_create_args_t leave_args = {
        .callback = leave_timer_cb,
        .name = "oob-leave",
    };
    const esp_timer_create_args_t status_args = {
        .callback = status_timer_cb,
        .name = "oob-status",
    };
    const esp_timer_create_args_t restart_args = {
        .callback = restart_timer_cb,
        .name = "oob-restart",
    };
    const esp_timer_create_args_t adv_args = {
        .callback = adv_timer_cb,
        .name = "oob-adv",
    };
    esp_err_t err = esp_timer_create(&leave_args, &s_ctx.leave_timer);
    if (err == ESP_OK) {
        err = esp_timer_create(&status_args, &s_ctx.status_timer);
    }
    if (err == ESP_OK) {
        err = esp_timer_create(&restart_args, &s_ctx.restart_timer);
    }
    if (err == ESP_OK) {
        err = esp_timer_create(&adv_args, &s_ctx.adv_timer);
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* LE Secure Connections with passkey entry. "Display only" is the truthful
     * I/O capability for a board whose only display is the label its passkey is
     * printed on, and it is what makes the pairing authenticated rather than
     * Just Works. Bonding so a return visit does not re-pair. */
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_services);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(s_gatt_services);
    }
    if (rc == 0) {
        rc = ble_svc_gap_device_name_set(s_ctx.device_name);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT registration failed: %d", rc);
        return ESP_FAIL;
    }

    ble_store_config_init();

    /* Before the host starts, both of them. on_sync() fires from the host task
     * as soon as the controller is up and is what actually begins advertising;
     * if it ran while `started` was still false, or before programming mode had
     * been raised, an unprovisioned device would come up silent and there would
     * be no second chance — nothing else can select it. */
    s_ctx.started = true;
    if (!s_ctx.provisioned) {
        /* Never commissioned: the board selects itself, because there is no
         * other way in and nobody has told it anything yet. Going through
         * control_service rather than setting the local flag means the LED lights
         * and — on a Modbus build — the commissioning address comes up too, which
         * is the same state by definition. It lapses on the programming-mode
         * timeout like any other selection. */
        ESP_LOGW(TAG, "Device has never been commissioned — entering programming mode");
        control_service_set_programming_mode(true);
    }

    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "Out-of-band channel ready: %u settings, name '%s', passkey %s",
             (unsigned)device_config_count(), s_ctx.device_name,
             s_ctx.passkey_from_efuse ? "derived from eFuse root secret" : "NOT PROVISIONED");
    return ESP_OK;
}

void oob_service_set_programming_mode(bool active)
{
    if (!s_ctx.started || s_ctx.programming_mode == active) {
        return;
    }
    s_ctx.programming_mode = active;

    if (active) {
        ESP_LOGI(TAG, "Programming mode: advertising as '%s'", s_ctx.device_name);
        advertise_start();
        return;
    }

    ESP_LOGI(TAG, "Programming mode ended: going silent");
    /* A restart may already be armed from a dropped link; going silent cancels
     * it. advertise_start() would refuse anyway now that the flag is clear —
     * this just keeps "going silent" from leaving work behind it. */
    esp_timer_stop(s_ctx.adv_timer);
    if (s_ctx.advertising) {
        ble_gap_adv_stop();
        s_ctx.advertising = false;
    }
    /* An installer who has walked away must not leave a link open behind them,
     * so leaving programming mode drops a connected client too. The bond
     * survives; only this connection does not. */
    if (s_ctx.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_ctx.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

bool oob_service_advertising(void)
{
    return s_ctx.started && (s_ctx.advertising || s_ctx.conn_handle != BLE_HS_CONN_HANDLE_NONE);
}

bool oob_service_client_connected(void)
{
    return s_ctx.conn_handle != BLE_HS_CONN_HANDLE_NONE && s_ctx.authenticated;
}

bool oob_service_identify_active(void)
{
    return s_ctx.identify_blink;
}

#endif /* CONFIG_HABINARI_OOB_BLE */
