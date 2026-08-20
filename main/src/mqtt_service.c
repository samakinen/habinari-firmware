// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/*
 * Wi-Fi + MQTT personality.
 *
 * Everything protocol-shaped is in mqtt_payload.c, which host-tests. What is
 * left here is association, session management and topic plumbing — the parts
 * that need a radio and a broker to exercise, so they are kept thin and dull.
 *
 * This adapter needs the 5-30 V auxiliary supply. It cannot be built into the
 * same image as the KNX personality; protocol_registry.c enforces that and says
 * why.
 */
#include "mqtt_service.h"

#include <stdio.h>
#include <string.h>

#include "control_state.h"
#include "device_config.h"
#include "device_default_name.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "mqtt_payload.h"
#include "nvs.h"
#include "protocol_adapter.h"
#include "wifi_service.h"
#include "sdkconfig.h"

static const char *TAG = "mqtt";

/* One firmware image, many sites: credentials are provisioned into NVS, never
 * compiled in. See docs/mqtt-integration.md for the keys. */
#define NETCFG_NAMESPACE "netcfg"
#define NETCFG_KEY_SSID "ssid"
#define NETCFG_KEY_PASSWORD "pass"
#define NETCFG_KEY_BROKER "broker"
#define NETCFG_KEY_MQTT_USER "mq_user"
#define NETCFG_KEY_MQTT_PASS "mq_pass"

/* The base is bounded well below TOPIC_MAX so that every derived topic
 * provably fits: the longest suffix appended below is "/availability" at 13
 * characters, and 96 + 13 < 160. Sizing both the same would be a truncation
 * the compiler is right to refuse. */
#define BASE_MAX 96
#define TOPIC_MAX 160
#define DEVICE_ID_MAX 16
#define DEVICE_NAME_MAX 64

typedef struct {
    esp_mqtt_client_handle_t client;
    TaskHandle_t task;

    char device_id[DEVICE_ID_MAX];
    char device_name[DEVICE_NAME_MAX]; /* HA display name; see build_device_name() */
    char base[BASE_MAX];       /* "<prefix>/<device-id>" */
    char topic_state[TOPIC_MAX];
    char topic_avail[TOPIC_MAX];
    char topic_cmd_wild[TOPIC_MAX];

    char broker_uri[128];
    char mqtt_user[64];
    char mqtt_pass[64];

    volatile bool connected;
    volatile bool discovery_pending;
    volatile bool publish_forced;

    /* Send-on-change plus heartbeat, the same model the KNX transmit policy
     * uses and for the same reason: a room sensor that republishes an unchanged
     * document every second is just noise on someone's broker. Keeping the last
     * document costs 1 kB of static RAM and turns "has anything changed?" into
     * a memcmp. */
    char last_payload[MQTT_PAYLOAD_STATE_MAX];
    int64_t last_publish_us;
} mqtt_ctx_t;

static mqtt_ctx_t s_ctx;

/* --- NVS configuration ---------------------------------------------------- */

static esp_err_t nvs_get_string(nvs_handle_t handle, const char *key, char *out, size_t out_len)
{
    size_t len = out_len;
    const esp_err_t err = nvs_get_str(handle, key, out, &len);
    if (err != ESP_OK) {
        out[0] = '\0';
    }
    return err;
}

/* --- Out-of-band configuration --------------------------------------------
 *
 * The other circular one, and the worse of the two: these five strings decide
 * whether the device has a network at all, and nothing on the network can write
 * them until they are already right. Registered before the adapter tries to
 * associate, precisely so that the failure path — no credentials — still leaves
 * them reachable from the service channel (oob_service.h).
 *
 * One hook pair serves all five; the NVS key travels in item->ctx.
 */

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

/* The broker settings only. The SSID and passphrase moved to wifi_service.c
 * when KNXnet/IP started needing the same network: they are the device's
 * connection to a network, not this adapter's connection to a broker, and a
 * KNX IP image should not offer settings for a broker it does not have. */
static const device_config_item_t netcfg_config_items[] = {
    {
        .key = "net.broker",
        .label = "MQTT broker URI",
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .max = 127.0f,
        .get = netcfg_cfg_get,
        .set = netcfg_cfg_set,
        .ctx = (void *)NETCFG_KEY_BROKER,
    },
    {
        .key = "net.user",
        .label = "MQTT username",
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .max = 63.0f,
        .get = netcfg_cfg_get,
        .set = netcfg_cfg_set,
        .ctx = (void *)NETCFG_KEY_MQTT_USER,
    },
    {
        .key = "net.mqpass",
        .label = "MQTT password",
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = DEVICE_CONFIG_FLAG_SECRET | DEVICE_CONFIG_FLAG_REBOOT,
        .max = 63.0f,
        .get = netcfg_cfg_get,
        .set = netcfg_cfg_set,
        .ctx = (void *)NETCFG_KEY_MQTT_PASS,
    },
};

/* --- Home Assistant discovery --------------------------------------------- */

#if defined(CONFIG_HABINARI_MQTT_DISCOVERY_PREFIX)
#define DISCOVERY_PREFIX CONFIG_HABINARI_MQTT_DISCOVERY_PREFIX
#else
#define DISCOVERY_PREFIX ""
#endif

/* The device block every entity repeats, so Home Assistant groups them under
 * one device card rather than scattering nine unrelated entities. */
static int append_device_block(char *buf, size_t len, const mqtt_ctx_t *ctx)
{
    return snprintf(buf, len,
                    "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\","
                    "\"mf\":\"%s\",\"mdl\":\"%s\"},"
                    "\"avty_t\":\"%s\"",
                    ctx->device_id, ctx->device_name,
                    CONFIG_HABINARI_MQTT_MANUFACTURER, CONFIG_HABINARI_MQTT_MODEL,
                    ctx->topic_avail);
}

static void publish_discovery_sensor(mqtt_ctx_t *ctx, const char *object_id, const char *name,
                                     const char *device_class, const char *unit,
                                     const char *value_key)
{
    char topic[TOPIC_MAX];
    char payload[512];

    snprintf(topic, sizeof(topic), "%s/sensor/%s/%s/config", DISCOVERY_PREFIX, ctx->device_id,
             object_id);

    int n = snprintf(payload, sizeof(payload),
                     "{\"uniq_id\":\"%s_%s\",\"name\":\"%s\",\"stat_t\":\"%s\","
                     "\"val_tpl\":\"{{value_json.%s|default('')}}\",",
                     ctx->device_id, object_id, name, ctx->topic_state, value_key);
    if (device_class != NULL) {
        n += snprintf(payload + n, sizeof(payload) - n, "\"dev_cla\":\"%s\",", device_class);
    }
    if (unit != NULL) {
        n += snprintf(payload + n, sizeof(payload) - n, "\"unit_of_meas\":\"%s\",\"stat_cla\":\"measurement\",",
                      unit);
    }
    n += append_device_block(payload + n, sizeof(payload) - n, ctx);
    snprintf(payload + n, sizeof(payload) - n, "}");

    /* Retained: Home Assistant must find the configuration after a restart
     * without waiting for this device to reboot. */
    esp_mqtt_client_publish(ctx->client, topic, payload, 0, 1, 1);
}

static void publish_discovery_climate(mqtt_ctx_t *ctx)
{
    char topic[TOPIC_MAX];
    char payload[1024];

    snprintf(topic, sizeof(topic), "%s/climate/%s/thermostat/config", DISCOVERY_PREFIX,
             ctx->device_id);

    int n = snprintf(
        payload, sizeof(payload),
        "{\"uniq_id\":\"%s_thermostat\",\"name\":\"Room thermostat\","
        "\"stat_t\":\"%s\","
        "\"curr_temp_tpl\":\"{{value_json.temperature|default('')}}\","
        "\"curr_hum_tpl\":\"{{value_json.humidity|default('')}}\","
        "\"temp_stat_tpl\":\"{{value_json.setpoint}}\","
        "\"temp_cmd_t\":\"%s/cmd/setpoint\","
        "\"mode_stat_tpl\":\"{{value_json.mode}}\","
        "\"mode_cmd_t\":\"%s/cmd/mode\","
        "\"modes\":[\"off\",\"heat\",\"cool\",\"auto\"],"
        "\"act_tpl\":\"{{value_json.action}}\","
        "\"pr_mode_stat_tpl\":\"{{value_json.preset}}\","
        "\"pr_mode_cmd_t\":\"%s/cmd/preset\","
        "\"pr_modes\":[\"comfort\",\"standby\",\"eco\",\"away\"],"
        "\"min_temp\":7,\"max_temp\":35,\"temp_step\":0.5,\"temp_unit\":\"C\",",
        ctx->device_id, ctx->topic_state, ctx->base, ctx->base, ctx->base);
    n += append_device_block(payload + n, sizeof(payload) - n, ctx);
    snprintf(payload + n, sizeof(payload) - n, "}");

    esp_mqtt_client_publish(ctx->client, topic, payload, 0, 1, 1);
}

static void publish_discovery(mqtt_ctx_t *ctx)
{
    if (DISCOVERY_PREFIX[0] == '\0') {
        ESP_LOGI(TAG, "Discovery prefix empty — publishing no discovery config");
        return;
    }

    publish_discovery_climate(ctx);
    publish_discovery_sensor(ctx, "temperature", "Room temperature", "temperature", "°C",
                             "temperature");
    publish_discovery_sensor(ctx, "humidity", "Room humidity", "humidity", "%", "humidity");
    publish_discovery_sensor(ctx, "co2", "Room CO2", "carbon_dioxide", "ppm", "co2");
    publish_discovery_sensor(ctx, "pressure", "Air pressure", "atmospheric_pressure", "Pa",
                             "pressure");
    publish_discovery_sensor(ctx, "iaq", "Air quality index", "aqi", NULL, "iaq");
    publish_discovery_sensor(ctx, "dew_point", "Dew point", "temperature", "°C", "dew_point");
    publish_discovery_sensor(ctx, "probe_temperature", "Floor temperature", "temperature", "°C",
                             "probe_temperature");
    publish_discovery_sensor(ctx, "heating_percent", "Heating demand", "power_factor", "%",
                             "heating_percent");
    publish_discovery_sensor(ctx, "ventilation_percent", "Ventilation demand", "power_factor", "%",
                             "ventilation_percent");
    ESP_LOGI(TAG, "Published Home Assistant discovery under %s/", DISCOVERY_PREFIX);
}

/* --- MQTT ----------------------------------------------------------------- */

static void handle_command(mqtt_ctx_t *ctx, const char *topic, int topic_len, const char *data,
                           int data_len)
{
    /* Everything after ".../cmd/" is the command name. */
    const size_t prefix_len = strlen(ctx->base) + 5; /* "<base>/cmd/" */
    if ((size_t)topic_len <= prefix_len) {
        return;
    }

    char suffix[MQTT_PAYLOAD_TOPIC_MAX];
    const size_t suffix_len = (size_t)topic_len - prefix_len;
    if (suffix_len >= sizeof(suffix)) {
        return;
    }
    memcpy(suffix, topic + prefix_len, suffix_len);
    suffix[suffix_len] = '\0';

    control_command_t command;
    float value = 0.0f;
    if (!mqtt_payload_parse_command(suffix, data, (size_t)data_len, &command, &value)) {
        ESP_LOGW(TAG, "Ignoring command '%s' (unknown topic or unparseable payload)", suffix);
        return;
    }

    const esp_err_t err = control_state_write(command, value);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Command '%s' rejected: %s", suffix, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Command %s = %.2f", suffix, value);

    /* Publish the resulting state immediately rather than at the next interval:
     * a UI that has just sent a command is waiting to see it take effect, and
     * the device may well have clamped the value it was given. Forced, because
     * a rejected or clamped-to-unchanged command still owes the sender an
     * answer. */
    ctx->publish_forced = true;
    if (ctx->task != NULL) {
        xTaskNotifyGive(ctx->task);
    }
}

static void mqtt_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)base;
    mqtt_ctx_t *ctx = (mqtt_ctx_t *)arg;
    const esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)data;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Broker connected");
        ctx->connected = true;
        esp_mqtt_client_subscribe(ctx->client, ctx->topic_cmd_wild, 1);
        esp_mqtt_client_publish(ctx->client, ctx->topic_avail, "online", 0, 1, 1);
        ctx->discovery_pending = true;
        /* A reconnect must republish even if nothing changed while we were
         * away: the broker may have lost the retained document. */
        ctx->last_payload[0] = '\0';
        if (ctx->task != NULL) {
            xTaskNotifyGive(ctx->task);
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Broker disconnected");
        ctx->connected = false;
        break;
    case MQTT_EVENT_DATA:
        handle_command(ctx, event->topic, event->topic_len, event->data, event->data_len);
        break;
    default:
        break;
    }
}

/* Publish if the document changed, if the heartbeat is due, or if something
 * asked for it (a command was just applied, or the session just came up).
 * Returns true if a message went out. */
static bool publish_state_if_due(mqtt_ctx_t *ctx, bool force)
{
    control_state_t state;
    control_state_get(&state);

    char payload[MQTT_PAYLOAD_STATE_MAX];
    const int n = mqtt_payload_state_json(payload, sizeof(payload), &state);
    if (n < 0) {
        ESP_LOGE(TAG, "State document did not fit in %d bytes", (int)sizeof(payload));
        return false;
    }

    const int64_t now = esp_timer_get_time();
    const int64_t heartbeat_us = (int64_t)CONFIG_HABINARI_MQTT_STATE_INTERVAL_S * 1000000;
    const bool changed = strcmp(payload, ctx->last_payload) != 0;
    const bool heartbeat_due = (now - ctx->last_publish_us) >= heartbeat_us;

    if (!force && !changed && !heartbeat_due) {
        return false;
    }

    /* Retained, QoS 0: a late subscriber should see the current room state
     * immediately, and a dropped update is superseded by the next one a few
     * seconds later — redelivery would only ever deliver a stale reading. */
    esp_mqtt_client_publish(ctx->client, ctx->topic_state, payload, n, 0, 1);
    memcpy(ctx->last_payload, payload, (size_t)n + 1);
    ctx->last_publish_us = now;
    return true;
}

static void mqtt_task(void *arg)
{
    mqtt_ctx_t *ctx = (mqtt_ctx_t *)arg;
    /* An upper bound on the wait, not the publish cadence: the control tick
     * wakes us every second so a change reaches the broker promptly, and
     * publish_state_if_due() decides whether it is worth a message. This bound
     * only matters if the control task stops notifying. */
    const TickType_t max_wait = pdMS_TO_TICKS(CONFIG_HABINARI_MQTT_STATE_INTERVAL_S * 1000);

    for (;;) {
        /* Wake on a control tick, a command, or a connect — whichever comes
         * first. Nothing here polls. */
        (void)ulTaskNotifyTake(pdTRUE, max_wait);

        if (!ctx->connected) {
            continue;
        }
        if (ctx->discovery_pending) {
            ctx->discovery_pending = false;
            publish_discovery(ctx);
            /* The retained state document must follow the discovery config, or
             * Home Assistant creates the entities and shows them as unknown
             * until the next change. */
            ctx->publish_forced = true;
        }

        const bool force = ctx->publish_forced;
        ctx->publish_forced = false;
        (void)publish_state_if_due(ctx, force);
    }
}

/* --- Adapter -------------------------------------------------------------- */

static void build_device_name(mqtt_ctx_t *ctx, const uint8_t mac_tail[3])
{
    const device_config_item_t *item = device_config_find("dev.name");
    if (item != NULL) {
        char configured[DEVICE_CONFIG_STRING_MAX];
        const int written = device_config_get_text(item, configured, sizeof(configured));
        if (written > 0 && (size_t)written < sizeof(ctx->device_name)) {
            memcpy(ctx->device_name, configured, (size_t)written + 1u);
            return;
        }
        if (written >= (int)sizeof(ctx->device_name)) {
            ESP_LOGW(TAG, "Configured device name is too long; using the fallback");
        }
    }
    device_default_name(mac_tail, ctx->device_name, sizeof(ctx->device_name));
}

static void build_topics(mqtt_ctx_t *ctx)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(ctx->device_id, sizeof(ctx->device_id), "%02x%02x%02x", mac[3], mac[4], mac[5]);
    build_device_name(ctx, &mac[3]);

    snprintf(ctx->base, sizeof(ctx->base), "%s/%s", CONFIG_HABINARI_MQTT_BASE_TOPIC,
             ctx->device_id);
    snprintf(ctx->topic_state, sizeof(ctx->topic_state), "%s/state", ctx->base);
    snprintf(ctx->topic_avail, sizeof(ctx->topic_avail), "%s/availability", ctx->base);
    snprintf(ctx->topic_cmd_wild, sizeof(ctx->topic_cmd_wild), "%s/cmd/+", ctx->base);
}

static esp_err_t mqtt_adapter_start(void)
{
    /* First, before anything can fail. An unprovisioned device returns from
     * this function two branches down, and the whole point of the service
     * channel is that it can still be provisioned afterwards. */
    const esp_err_t wifi_cfg = wifi_service_register_config();
    if (wifi_cfg != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi settings not exposed out of band: %s", esp_err_to_name(wifi_cfg));
    }
    const esp_err_t cfg = device_config_register(
        netcfg_config_items, sizeof(netcfg_config_items) / sizeof(*netcfg_config_items));
    if (cfg != ESP_OK) {
        ESP_LOGW(TAG, "Broker settings not exposed out of band: %s", esp_err_to_name(cfg));
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NETCFG_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No '%s' NVS namespace — provision Wi-Fi and broker settings first "
                      "(over BLE, see docs/ble-commissioning.md, or with "
                      "`idf.py nvs-partition-gen`)", NETCFG_NAMESPACE);
        return ESP_ERR_NOT_FOUND;
    }
    nvs_get_string(handle, NETCFG_KEY_BROKER, s_ctx.broker_uri, sizeof(s_ctx.broker_uri));
    nvs_get_string(handle, NETCFG_KEY_MQTT_USER, s_ctx.mqtt_user, sizeof(s_ctx.mqtt_user));
    nvs_get_string(handle, NETCFG_KEY_MQTT_PASS, s_ctx.mqtt_pass, sizeof(s_ctx.mqtt_pass));
    nvs_close(handle);

    if (s_ctx.broker_uri[0] == '\0') {
        ESP_LOGE(TAG, "MQTT broker URI not provisioned");
        return ESP_ERR_INVALID_STATE;
    }

    build_topics(&s_ctx);

    err = wifi_service_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi start failed: %s", esp_err_to_name(err));
        return err;
    }

    const esp_mqtt_client_config_t config = {
        .broker.address.uri = s_ctx.broker_uri,
        /* Only exercised for mqtts:// and wss:// URIs — plain mqtt:// ignores
         * it — but esp-tls refuses to open *any* TLS connection without a
         * verification option set, so a broker configured as wss:// with no
         * cert_bundle_attach fails with ESP_ERR_MBEDTLS_SSL_SETUP_FAILED
         * before it ever reaches the network. Mozilla's root set (enabled via
         * CONFIG_MBEDTLS_CERTIFICATE_BUNDLE) covers any broker with a
         * publicly-trusted certificate; a broker on a private CA needs
         * .broker.verification.certificate with the CA's PEM instead. */
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.username = s_ctx.mqtt_user[0] ? s_ctx.mqtt_user : NULL,
        .credentials.authentication.password = s_ctx.mqtt_pass[0] ? s_ctx.mqtt_pass : NULL,
        .credentials.client_id = s_ctx.device_id,
        /* Retained last will, so a device that drops off the network is shown
         * as unavailable rather than frozen at its last reading. */
        .session.last_will.topic = s_ctx.topic_avail,
        .session.last_will.msg = "offline",
        .session.last_will.qos = 1,
        .session.last_will.retain = 1,
        .session.keepalive = 30,
    };

    s_ctx.client = esp_mqtt_client_init(&config);
    if (s_ctx.client == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_ctx.client, ESP_EVENT_ANY_ID,
                                                   mqtt_event_handler, &s_ctx));

    if (xTaskCreate(mqtt_task, "mqtt", 4096, &s_ctx, 4, &s_ctx.task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    /* Non-blocking: the client retries the broker on its own, so a broker that
     * is down at boot delays reporting and nothing else. */
    return esp_mqtt_client_start(s_ctx.client);
}

static void mqtt_adapter_on_control_tick(uint32_t generation)
{
    (void)generation;
    /* Wake the publish task rather than publishing here: this runs on the
     * control task, and esp_mqtt_client_publish can block on the outbox. The
     * task rate-limits itself, so a nudge every second is not a publish every
     * second. */
    if (s_ctx.task != NULL && s_ctx.connected) {
        xTaskNotifyGive(s_ctx.task);
    }
}

bool mqtt_service_connected(void)
{
    return s_ctx.connected;
}

esp_err_t mqtt_service_provision(const char *ssid, const char *password, const char *broker_uri)
{
    /* Still writes the Wi-Fi pair even though wifi_service.c owns them now:
     * they share the "netcfg" namespace, and this is the one call a console
     * user makes to bring an unprovisioned board onto a network and a broker in
     * a single step. Splitting it would trade one obvious entry point for two. */
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NETCFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    if (ssid != NULL) {
        err = nvs_set_str(handle, NETCFG_KEY_SSID, ssid);
    }
    if (err == ESP_OK && password != NULL) {
        err = nvs_set_str(handle, NETCFG_KEY_PASSWORD, password);
    }
    if (err == ESP_OK && broker_uri != NULL) {
        err = nvs_set_str(handle, NETCFG_KEY_BROKER, broker_uri);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGW(TAG, "Network settings stored — rebooting");
    esp_restart();
    return ESP_OK;
}

const protocol_adapter_t mqtt_protocol_adapter = {
    .name = "mqtt-wifi",
    .start = mqtt_adapter_start,
    .on_control_tick = mqtt_adapter_on_control_tick,
    /* No on_sensor_data hook: measurements ride along in the state document
     * published each control tick, and publishing twice per cycle would double
     * the broker traffic for readings that only change every 30 s anyway. */
    .on_sensor_data = NULL,
    .identify_active = NULL,
    /* Optional: an unprovisioned or out-of-range device must still boot, run its
     * control loops and be reachable over any other personality present. */
    .required = false,
};
