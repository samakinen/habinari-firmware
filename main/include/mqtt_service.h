#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file mqtt_service.h
 * @brief Wi-Fi + MQTT personality: the same room controller, over IP.
 *
 * This is the third mapping onto control_state.h, and it exists mostly to prove
 * that the second one was not a coincidence — adding it changed no line of the
 * control core, no line of main.c and no line of the other two adapters.
 *
 * It publishes the full device state as one JSON document and accepts scalar
 * command payloads on per-topic endpoints, which is the shape Home Assistant's
 * MQTT integration expects. Discovery configuration is published on connect, so
 * the board appears as a climate entity plus its sensors with no YAML.
 *
 * Credentials live in the NVS namespace "netcfg" and are deliberately NOT
 * compiled in: one firmware image, many sites. Provision them over the console
 * or with `idf.py nvs-partition-gen`; the keys are listed in
 * docs/mqtt-integration.md.
 *
 * The adapter descriptor (mqtt_protocol_adapter) is the registry's entry point;
 * these two functions exist for diagnostics and for the console.
 */

/// True once the Wi-Fi station has an IP and the MQTT session is up.
bool mqtt_service_connected(void);

/// Store Wi-Fi and broker settings in NVS and reboot into them. Any argument
/// may be NULL to leave that field unchanged. Returns without rebooting on a
/// storage error.
esp_err_t mqtt_service_provision(const char *ssid, const char *password, const char *broker_uri);

#ifdef __cplusplus
}
#endif
