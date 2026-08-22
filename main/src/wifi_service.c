// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/*
 * Station-mode Wi-Fi, shared by every personality that needs a network.
 *
 * Extracted from mqtt_service.c when KNXnet/IP arrived and needed exactly the
 * same association, the same credentials and the same waiting. The behaviour is
 * unchanged from that version; what is new is that two callers can ask for it.
 */
#include "wifi_service.h"

#include <string.h>

#include "device_config.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs.h"

static const char *TAG = "wifi";

/* One firmware image, many sites: credentials are provisioned into NVS, never
 * compiled in. The namespace is shared with the MQTT adapter's own settings —
 * it is the device's network configuration, not any one protocol's. */
#define NETCFG_NAMESPACE "netcfg"
#define NETCFG_KEY_SSID "ssid"
#define NETCFG_KEY_PASSWORD "pass"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_RECONNECT_DELAY_MS 2000

typedef struct {
    EventGroupHandle_t events;
    esp_timer_handle_t reconnect_timer;
    esp_netif_t *netif;
    bool stack_ready;
    bool started;
    bool config_registered;
    wifi_service_event_cb_t callback;
    void *callback_ctx;
} wifi_ctx_t;

static wifi_ctx_t s_ctx;

/* --- Out-of-band configuration -------------------------------------------- */

static esp_err_t netcfg_cfg_get(const device_config_item_t *item, device_config_value_t *out)
{
    out->str[0] = '\0';

    nvs_handle_t handle;
    if (nvs_open(NETCFG_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return ESP_OK; /* nothing provisioned yet is a valid answer, not an error */
    }

    char stored[DEVICE_CONFIG_STRING_MAX] = {0};
    size_t len = sizeof(stored);
    if (nvs_get_str(handle, (const char *)item->ctx, stored, &len) == ESP_OK) {
        if ((item->flags & DEVICE_CONFIG_FLAG_SECRET) != 0u) {
            /* Only the fact, never the value — device_config_format() renders
             * this as <set>/<unset>. The secret never reaches a buffer that a
             * transport could decide to send. */
            out->str[0] = (stored[0] != '\0') ? 'x' : '\0';
            out->str[1] = '\0';
        } else {
            snprintf(out->str, sizeof(out->str), "%s", stored);
        }
    }
    memset(stored, 0, sizeof(stored));
    nvs_close(handle);
    return ESP_OK;
}

static esp_err_t netcfg_cfg_set(const device_config_item_t *item,
                                const device_config_value_t *value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NETCFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(handle, (const char *)item->ctx, value->str);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

/* Wi-Fi limits, not ours: a 32-character SSID and a 63-character passphrase are
 * what 802.11 allows, and refusing a 64th character is more useful than storing
 * one the radio will not accept. */
static const device_config_item_t wifi_config_items[] = {
    {
        .key = "net.ssid",
        .label = "Wi-Fi SSID",
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .max = 32.0f,
        .get = netcfg_cfg_get,
        .set = netcfg_cfg_set,
        .ctx = (void *)NETCFG_KEY_SSID,
    },
    {
        .key = "net.pass",
        .label = "Wi-Fi passphrase",
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = DEVICE_CONFIG_FLAG_SECRET | DEVICE_CONFIG_FLAG_REBOOT,
        .max = 63.0f,
        .get = netcfg_cfg_get,
        .set = netcfg_cfg_set,
        .ctx = (void *)NETCFG_KEY_PASSWORD,
    },
};

esp_err_t wifi_service_register_config(void)
{
    if (s_ctx.config_registered) {
        return ESP_OK;
    }
    const esp_err_t err = device_config_register(
        wifi_config_items, sizeof(wifi_config_items) / sizeof(*wifi_config_items));
    if (err == ESP_OK) {
        s_ctx.config_registered = true;
    }
    return err;
}

/* --- Association ---------------------------------------------------------- */

static void notify(bool connected)
{
    if (s_ctx.callback != NULL) {
        s_ctx.callback(connected, s_ctx.callback_ctx);
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const bool was_connected =
            (xEventGroupGetBits(s_ctx.events) & WIFI_CONNECTED_BIT) != 0;
        xEventGroupClearBits(s_ctx.events, WIFI_CONNECTED_BIT);
        if (was_connected) {
            notify(false);
        }
        /* Reconnect forever rather than after N tries: this is a wall-mounted
         * device with no UI, and an AP that is down for an hour must not leave
         * it permanently offline. The control loops keep running regardless.
         *
         * Backed off through a one-shot timer rather than a vTaskDelay here:
         * this runs on the shared default event loop, and sleeping on it would
         * hold up every other event registered there. Without any backoff a
         * wrong passphrase would spin. */
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying in %d ms", WIFI_RECONNECT_DELAY_MS);
        if (s_ctx.reconnect_timer != NULL) {
            esp_timer_start_once(s_ctx.reconnect_timer, WIFI_RECONNECT_DELAY_MS * 1000);
        }
        return;
    }
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Wi-Fi up, IP " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_ctx.events, WIFI_CONNECTED_BIT);
        notify(true);
    }
}

static void wifi_reconnect_timer_cb(void *arg)
{
    (void)arg;
    esp_wifi_connect();
}

/* --- Network stack -------------------------------------------------------- */

/* esp_netif_init() is what starts the lwIP TCP/IP thread, and every socket in
 * the image goes through that thread's mailbox — so a personality that opens
 * one before this has run does not fail, it asserts ("Invalid mbox") and
 * panics. That is why the stack comes up here, unconditionally, ahead of the
 * credentials: a device with an empty NVS still runs KNXnet/IP, still binds its
 * routing socket, and simply has nothing to route over until it is provisioned.
 *
 * Everything except esp_wifi_start() lives here. Bringing the driver up without
 * associating costs a few kilobytes and no radio power — the PHY is only
 * powered from esp_wifi_start() — and it leaves wifi_service_mac() and
 * wifi_service_ip_info() with a real interface to answer from. */
static esp_err_t ensure_network_stack(void)
{
    if (s_ctx.stack_ready) {
        return ESP_OK;
    }

    if (s_ctx.events == NULL) {
        s_ctx.events = xEventGroupCreate();
        if (s_ctx.events == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }

    /* The default loop is shared with whatever else on the board posts to it,
     * so "already created" is the expected answer, not a fault. */
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    if (s_ctx.netif == NULL) {
        s_ctx.netif = esp_netif_create_default_wifi_sta();
        if (s_ctx.netif == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        return err;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = wifi_reconnect_timer_cb,
        .name = "wifi_reconnect",
    };
    err = esp_timer_create(&timer_args, &s_ctx.reconnect_timer);
    if (err != ESP_OK) {
        return err;
    }

    s_ctx.stack_ready = true;
    return ESP_OK;
}

esp_err_t wifi_service_start(void)
{
    if (s_ctx.started) {
        return ESP_OK;
    }

    /* Before the credentials check, not after: see ensure_network_stack(). */
    const esp_err_t stack_err = ensure_network_stack();
    if (stack_err != ESP_OK) {
        ESP_LOGE(TAG, "Network stack init failed: %s", esp_err_to_name(stack_err));
        return stack_err;
    }

    char ssid[64] = {0};
    char password[64] = {0};

    nvs_handle_t handle;
    if (nvs_open(NETCFG_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "No '%s' NVS namespace — provision Wi-Fi credentials first "
                      "(over BLE, see docs/ble-commissioning.md, or with "
                      "`idf.py nvs-partition-gen`)", NETCFG_NAMESPACE);
        return ESP_ERR_NOT_FOUND;
    }
    size_t len = sizeof(ssid);
    if (nvs_get_str(handle, NETCFG_KEY_SSID, ssid, &len) != ESP_OK) {
        ssid[0] = '\0';
    }
    len = sizeof(password);
    if (nvs_get_str(handle, NETCFG_KEY_PASSWORD, password, &len) != ESP_OK) {
        password[0] = '\0';
    }
    nvs_close(handle);

    if (ssid[0] == '\0') {
        ESP_LOGE(TAG, "Wi-Fi SSID not provisioned");
        return ESP_ERR_NOT_FOUND;
    }

    wifi_config_t config = {0};
    strncpy((char *)config.sta.ssid, ssid, sizeof(config.sta.ssid) - 1);
    strncpy((char *)config.sta.password, password, sizeof(config.sta.password) - 1);
    memset(password, 0, sizeof(password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    memset(&config, 0, sizeof(config));
    /* The board is mains/aux powered and must answer promptly, so no modem
     * sleep: the latency it saves is not worth the power it costs here. And a
     * KNX IP device that sleeps through a multicast is a device that misses
     * group telegrams addressed to it. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    const esp_err_t err = esp_wifi_start();
    if (err == ESP_OK) {
        s_ctx.started = true;
    }
    return err;
}

bool wifi_service_is_connected(void)
{
    if (s_ctx.events == NULL) {
        return false;
    }
    return (xEventGroupGetBits(s_ctx.events) & WIFI_CONNECTED_BIT) != 0;
}

esp_err_t wifi_service_wait_connected(uint32_t timeout_ms)
{
    /* Gated on started, not on the event group: the group now exists from the
     * moment the stack is up, and waiting 30 s for an association that was
     * never attempted only slows the boot of a device that has no credentials
     * to associate with. */
    if (!s_ctx.started || s_ctx.events == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    const EventBits_t bits = xEventGroupWaitBits(s_ctx.events, WIFI_CONNECTED_BIT,
                                                 pdFALSE, pdTRUE, ticks);
    return ((bits & WIFI_CONNECTED_BIT) != 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t wifi_service_ip_info(esp_netif_ip_info_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    if (s_ctx.netif == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_netif_get_ip_info(s_ctx.netif, out);
}

esp_err_t wifi_service_mac(uint8_t out[6])
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, 6);
    /* Read from the interface rather than from the eFuse base MAC: the station
     * MAC is what appears on the network, and it is what a DHCP server, an ARP
     * table and the KNXnet/IP Parameter Object all mean by "MAC address". */
    if (s_ctx.netif != NULL && esp_netif_get_mac(s_ctx.netif, out) == ESP_OK) {
        return ESP_OK;
    }
    return esp_read_mac(out, ESP_MAC_WIFI_STA);
}

void wifi_service_set_event_callback(wifi_service_event_cb_t callback, void *ctx)
{
    s_ctx.callback = callback;
    s_ctx.callback_ctx = ctx;
}
