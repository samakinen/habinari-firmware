// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

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
 * compiled in: one firmware image, many sites. Provision them over BLE
 * (`tools/ble_config.py`) or with `idf.py nvs-partition-gen`; the keys are
 * listed in docs/mqtt-integration.md.
 *
 * The adapter descriptor (mqtt_protocol_adapter) is the registry's entry point.
 */
