// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file wifi_service.h
 * @brief Joining a Wi-Fi network, for whichever personality needs one.
 *
 * This used to live inside the MQTT adapter, which was fine while MQTT was the
 * only thing on the board that needed a radio. KNXnet/IP needs the same
 * network, the same credentials and the same "wait until we actually have an
 * address" behaviour, and two copies of that would be two things to get subtly
 * different.
 *
 * It owns the SSID and the passphrase — and only those. The MQTT broker URI and
 * its credentials stay with the adapter that uses them, because a KNXnet/IP
 * image has no broker and should not offer settings for one.
 *
 * Those two strings are the circular case device_config.h exists for: they
 * decide whether the device has a network at all, and nothing on the network
 * can write them until they are already correct. They are registered with the
 * config registry before the first association attempt, so a device with the
 * wrong passphrase is still reachable over the service channel to be given the
 * right one.
 *
 * Threading: start() is called once from app_main's start-up path. Everything
 * else is safe from any task afterwards.
 */

/**
 * Bring the station interface up and start associating.
 *
 * Idempotent: the second caller in an image that has two personalities on the
 * radio gets ESP_OK and the existing connection, not a second stack.
 *
 * Returns ESP_ERR_NOT_FOUND when no credentials are provisioned, which is a
 * normal state for a device out of the box rather than a fault — the caller
 * should say so and carry on, since the service channel is how they arrive.
 *
 * Association continues in the background and retries forever; a device on a
 * wall with no UI must not give up on an access point that is down for an hour.
 */
esp_err_t wifi_service_start(void);

/// True once the interface holds an IP address. False while associating.
bool wifi_service_is_connected(void);

/**
 * Block until the interface has an address, or the timeout expires.
 *
 * @param timeout_ms 0 waits forever.
 * @return ESP_OK when connected, ESP_ERR_TIMEOUT otherwise.
 */
esp_err_t wifi_service_wait_connected(uint32_t timeout_ms);

/// Current IPv4 configuration. Zeroed and ESP_ERR_INVALID_STATE when down.
esp_err_t wifi_service_ip_info(esp_netif_ip_info_t *out);

/// Station MAC address. Zeroed if the interface does not exist yet.
esp_err_t wifi_service_mac(uint8_t out[6]);

/**
 * Called when the interface gains or loses its address.
 *
 * A KNX IP device has to republish its IP identity to the KNXnet/IP Parameter
 * Object whenever the address changes, or ETS reads back an address the device
 * no longer answers on. Runs on the default event loop, so it must not block.
 *
 * One callback, last registration wins: there is one address and the
 * personalities that care about it do not coexist.
 */
typedef void (*wifi_service_event_cb_t)(bool connected, void *ctx);
void wifi_service_set_event_callback(wifi_service_event_cb_t callback, void *ctx);

/**
 * Expose the SSID and passphrase through the device_config registry.
 *
 * Separate from start() and called before it, so that the failure path — no
 * credentials at all — still leaves them writable from the service channel.
 * Safe to call more than once.
 */
esp_err_t wifi_service_register_config(void);

#ifdef __cplusplus
}
#endif
