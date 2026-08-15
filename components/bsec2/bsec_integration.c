// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * @file bsec_integration.c
 * @brief BME688 + Bosch BSEC air-quality integration.
 *
 * Two build configurations:
 *  - default (CONFIG_BME688_USE_BSEC unset): inert stubs; the firmware builds
 *    with no external dependency and produces no air-quality data.
 *  - CONFIG_BME688_USE_BSEC set: the real integration, compiled against the
 *    proprietary Bosch BSEC library + headers dropped into components/bsec2/bosch
 *    (see README.md). The library is NOT redistributable, so it is not committed.
 *
 * The real path follows Bosch's reference `bsec_integration` control loop:
 * bsec_sensor_control() dictates when to run a BME688 forced measurement and
 * with what heater temperature/duration; the raw signals are fed to
 * bsec_do_steps(); the virtual-sensor outputs (IAQ/CO2-eq/VOC-eq/compensated
 * T&H) are latched for the sensor bus to read. Calibration state is persisted to
 * NVS so the baseline and accuracy survive reboots.
 */

#include "bsec_integration.h"

#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "bsec";

// Latest outputs, shared between the BSEC task and get_latest() callers.
static bsec_air_quality_t s_latest;
static SemaphoreHandle_t  s_latest_mutex;

bool bsec_integration_get_latest(bsec_air_quality_t *out)
{
    if (out == NULL || s_latest_mutex == NULL) {
        return false;
    }
    xSemaphoreTake(s_latest_mutex, portMAX_DELAY);
    const bool valid = s_latest.valid;
    if (valid) {
        *out = s_latest;
    }
    xSemaphoreGive(s_latest_mutex);
    return valid;
}

#if !CONFIG_BME688_USE_BSEC

// --------------------------------------------------------------------------
// Stub build: no BSEC library. Everything is inert; no air-quality data.
// --------------------------------------------------------------------------

bool bsec_integration_available(void) { return false; }

esp_err_t bsec_integration_init(struct bme68x_dev *dev, SemaphoreHandle_t bus_mutex)
{
    (void)dev;
    (void)bus_mutex;
    ESP_LOGI(TAG, "BSEC disabled (CONFIG_BME688_USE_BSEC unset) — no air-quality output");
    return ESP_OK;
}

esp_err_t bsec_integration_start(void) { return ESP_OK; }

#else // CONFIG_BME688_USE_BSEC

// --------------------------------------------------------------------------
// Real build: proprietary Bosch BSEC library. Headers/lib live in
// components/bsec2/bosch/ (git-ignored). See README.md.
// --------------------------------------------------------------------------

#include <stdlib.h>
#include <string.h>

#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/task.h"

#include "bsec_interface.h"
#include "bsec_datatypes.h"

#define BSEC_NVS_NAMESPACE "bsec"
#define BSEC_NVS_STATE_KEY "state"

#if CONFIG_BME688_BSEC_SAMPLE_RATE_ULP
#define BSEC_SAMPLE_RATE BSEC_SAMPLE_RATE_ULP
#else
#define BSEC_SAMPLE_RATE BSEC_SAMPLE_RATE_LP
#endif

static struct bme68x_dev *s_dev;
static SemaphoreHandle_t  s_bus_mutex;
static TaskHandle_t       s_task;
static int64_t            s_last_state_save_us;

static inline void bus_lock(void)
{
    if (s_bus_mutex != NULL) xSemaphoreTake(s_bus_mutex, portMAX_DELAY);
}
static inline void bus_unlock(void)
{
    if (s_bus_mutex != NULL) xSemaphoreGive(s_bus_mutex);
}

static void store_latest(const bsec_air_quality_t *out)
{
    if (s_latest_mutex != NULL) {
        xSemaphoreTake(s_latest_mutex, portMAX_DELAY);
    }
    s_latest = *out;
    if (s_latest_mutex != NULL) {
        xSemaphoreGive(s_latest_mutex);
    }
}

// ---- BSEC calibration-state persistence (NVS) ----------------------------

// The BSEC work buffer alone is BSEC_MAX_WORKBUFFER_SIZE (4 KB); together with
// the state blob it far exceeds a typical task stack (bsec_state_load runs in
// the main task during init). Heap-allocate both to avoid a stack overflow.
static void bsec_state_load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(BSEC_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "no saved BSEC state (cold start)");
        return;
    }
    uint8_t *state = malloc(BSEC_MAX_STATE_BLOB_SIZE);
    uint8_t *work = malloc(BSEC_MAX_WORKBUFFER_SIZE);
    if (state == NULL || work == NULL) {
        ESP_LOGE(TAG, "OOM loading BSEC state");
    } else {
        size_t len = BSEC_MAX_STATE_BLOB_SIZE;
        esp_err_t err = nvs_get_blob(nvs, BSEC_NVS_STATE_KEY, state, &len);
        if (err == ESP_OK) {
            bsec_library_return_t rc = bsec_set_state(state, len, work, BSEC_MAX_WORKBUFFER_SIZE);
            if (rc != BSEC_OK) {
                ESP_LOGW(TAG, "bsec_set_state failed: %d", rc);
            } else {
                ESP_LOGI(TAG, "restored BSEC calibration state (%u bytes)", (unsigned)len);
            }
        } else {
            ESP_LOGI(TAG, "no saved BSEC state blob");
        }
    }
    nvs_close(nvs);
    free(state);
    free(work);
}

// BSEC ships a per-configuration tuning blob (sensor supply voltage, sample
// rate, calibration horizon) that must be applied before subscribing. Without
// it BSEC runs its built-in default, whose rate does not match an LP/ULP
// subscription — bsec_update_subscription() then answers
// BSEC_W_SU_SAMPLERATEMISMATCH (14) and never emits the gas-derived outputs, so
// IAQ / CO2-eq / VOC-eq stay at zero forever.
//
// The blob is part of the Bosch BSEC release and is no more redistributable
// than the library itself, so it is not committed. Drop the matching
// bsec_iaq.txt byte array in as components/bsec2/bosch/config/bsec_config.inc
// (see README.md) and it is compiled in; without it we keep the previous
// behaviour and say plainly why air quality will not work.
//
// BSEC_HAVE_CONFIG is set by CMakeLists.txt, deliberately NOT by __has_include
// here. ccache's direct mode hashes the source plus the headers a previous
// compile actually opened; a file that did not exist when __has_include was
// evaluated is not among them. Dropping the blob in later therefore left the
// hash unchanged and ccache served the stale "no blob" object — the build
// looked clean while the firmware still logged warning 14. Deciding in CMake
// puts the flag on the command line, which ccache does hash.
#ifdef BSEC_HAVE_CONFIG
static const uint8_t s_bsec_config[] = {
#include "bosch/config/bsec_config.inc"
};
#endif

static void bsec_config_load(void)
{
#ifdef BSEC_HAVE_CONFIG
    uint8_t *work = malloc(BSEC_MAX_WORKBUFFER_SIZE);
    if (work == NULL) {
        ESP_LOGE(TAG, "OOM applying BSEC configuration");
        return;
    }
    bsec_library_return_t rc =
        bsec_set_configuration(s_bsec_config, sizeof(s_bsec_config), work, BSEC_MAX_WORKBUFFER_SIZE);
    if (rc != BSEC_OK) {
        ESP_LOGW(TAG, "bsec_set_configuration failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "applied BSEC configuration (%u bytes)", (unsigned)sizeof(s_bsec_config));
    }
    free(work);
#else
    ESP_LOGW(TAG,
             "no BSEC configuration blob compiled in — expect "
             "BSEC_W_SU_SAMPLERATEMISMATCH (14) and no IAQ/CO2-eq/VOC-eq output. "
             "See components/bsec2/README.md.");
#endif
}

static void bsec_state_save(void)
{
    uint8_t *state = malloc(BSEC_MAX_STATE_BLOB_SIZE);
    uint8_t *work = malloc(BSEC_MAX_WORKBUFFER_SIZE);
    if (state == NULL || work == NULL) {
        ESP_LOGE(TAG, "OOM saving BSEC state");
        free(state);
        free(work);
        return;
    }
    uint32_t n_state = 0;
    bsec_library_return_t rc =
        bsec_get_state(0, state, BSEC_MAX_STATE_BLOB_SIZE, work, BSEC_MAX_WORKBUFFER_SIZE, &n_state);
    if (rc == BSEC_OK) {
        nvs_handle_t nvs;
        if (nvs_open(BSEC_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            if (nvs_set_blob(nvs, BSEC_NVS_STATE_KEY, state, n_state) == ESP_OK) {
                nvs_commit(nvs);
                ESP_LOGI(TAG, "saved BSEC calibration state (%u bytes)", (unsigned)n_state);
            }
            nvs_close(nvs);
        }
    } else {
        ESP_LOGW(TAG, "bsec_get_state failed: %d", rc);
    }
    free(state);
    free(work);
}

static void maybe_save_state(uint8_t accuracy)
{
#if CONFIG_BME688_BSEC_STATE_SAVE_INTERVAL_MIN > 0
    // Only persist once BSEC is at least somewhat calibrated, and no more often
    // than the configured interval, to spare flash wear.
    const int64_t interval_us =
        (int64_t)CONFIG_BME688_BSEC_STATE_SAVE_INTERVAL_MIN * 60 * 1000000;
    const int64_t now = esp_timer_get_time();
    if (accuracy >= 3 && (now - s_last_state_save_us) >= interval_us) {
        bsec_state_save();
        s_last_state_save_us = now;
    }
#else
    (void)accuracy;
#endif
}

// ---- Output parsing ------------------------------------------------------

static void apply_outputs(const bsec_output_t *outputs, uint8_t n_outputs, float pressure)
{
    bsec_air_quality_t aq;
    (void)bsec_integration_get_latest(&aq);  // start from previous (keeps fields)
    aq.pressure = pressure;
    for (uint8_t i = 0; i < n_outputs; i++) {
        switch (outputs[i].sensor_id) {
            case BSEC_OUTPUT_IAQ:
                aq.iaq = outputs[i].signal;
                aq.iaq_accuracy = outputs[i].accuracy;
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                aq.co2_equivalent = outputs[i].signal;
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                aq.voc_equivalent = outputs[i].signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                aq.comp_temperature = outputs[i].signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                aq.comp_humidity = outputs[i].signal;
                break;
            default:
                break;
        }
    }
    aq.valid = true;
    store_latest(&aq);
    maybe_save_state(aq.iaq_accuracy);
}

// ---- One BSEC control step (measure + process) ---------------------------

static void bsec_step(void)
{
    const int64_t now_ns = esp_timer_get_time() * 1000;  // us -> ns

    bsec_bme_settings_t settings;
    memset(&settings, 0, sizeof(settings));
    // Only a negative return is fatal; positive codes (e.g.
    // BSEC_W_SC_CALL_TIMING_VIOLATION) still yield valid settings.
    if (bsec_sensor_control(now_ns, &settings) < BSEC_OK) {
        return;
    }

    if (settings.trigger_measurement && settings.op_mode == BME68X_FORCED_MODE) {
        struct bme68x_heatr_conf heatr = {
            .enable = BME68X_ENABLE,
            .heatr_temp = settings.heater_temperature,
            .heatr_dur = settings.heater_duration,
        };
        bus_lock();
        int8_t rc = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr, s_dev);
        if (rc == BME68X_OK) rc = bme68x_set_op_mode(BME68X_FORCED_MODE, s_dev);
        bus_unlock();
        if (rc != BME68X_OK) {
            ESP_LOGW(TAG, "BME688 trigger failed: %d", rc);
            return;
        }

        struct bme68x_conf conf;
        bme68x_get_conf(&conf, s_dev);
        const uint32_t dur_us =
            bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, s_dev) +
            settings.heater_duration * 1000u;
        vTaskDelay(pdMS_TO_TICKS(dur_us / 1000 + 5));

        struct bme68x_data data;
        uint8_t n_fields = 0;
        bus_lock();
        rc = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, s_dev);
        bus_unlock();
        if (rc != BME68X_OK || n_fields == 0) {
            return;
        }

        // Feed the raw signals BSEC subscribed to (see process_data flags).
        bsec_input_t inputs[4];
        uint8_t n_inputs = 0;
        if (settings.process_data & BSEC_PROCESS_TEMPERATURE) {
            inputs[n_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
            inputs[n_inputs].signal = data.temperature / 100.0f;
            inputs[n_inputs].time_stamp = now_ns;
            n_inputs++;
        }
        if (settings.process_data & BSEC_PROCESS_HUMIDITY) {
            inputs[n_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
            inputs[n_inputs].signal = data.humidity / 1000.0f;
            inputs[n_inputs].time_stamp = now_ns;
            n_inputs++;
        }
        if (settings.process_data & BSEC_PROCESS_PRESSURE) {
            inputs[n_inputs].sensor_id = BSEC_INPUT_PRESSURE;
            inputs[n_inputs].signal = data.pressure;
            inputs[n_inputs].time_stamp = now_ns;
            n_inputs++;
        }
        if ((settings.process_data & BSEC_PROCESS_GAS) &&
            (data.status & BME68X_GASM_VALID_MSK)) {
            inputs[n_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
            inputs[n_inputs].signal = data.gas_resistance;
            inputs[n_inputs].time_stamp = now_ns;
            n_inputs++;
        }

        if (n_inputs > 0) {
            bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
            uint8_t n_outputs = BSEC_NUMBER_OUTPUTS;
            // Warnings (positive) still return usable outputs in n_outputs.
            if (bsec_do_steps(inputs, n_inputs, outputs, &n_outputs) >= BSEC_OK) {
                apply_outputs(outputs, n_outputs, data.pressure);
            }
        }
    }

    // Sleep until BSEC's next scheduled call.
    const int64_t sleep_ns = settings.next_call - (esp_timer_get_time() * 1000);
    const int32_t sleep_ms = sleep_ns > 0 ? (int32_t)(sleep_ns / 1000000) : 0;
    vTaskDelay(pdMS_TO_TICKS(sleep_ms > 0 ? sleep_ms : 10));
}

static void bsec_task(void *arg)
{
    (void)arg;
    for (;;) {
        bsec_step();
    }
}

bool bsec_integration_available(void) { return true; }

esp_err_t bsec_integration_init(struct bme68x_dev *dev, SemaphoreHandle_t bus_mutex)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_dev = dev;
    s_bus_mutex = bus_mutex;

    if (s_latest_mutex == NULL) {
        s_latest_mutex = xSemaphoreCreateMutex();
        if (s_latest_mutex == NULL) return ESP_ERR_NO_MEM;
    }

    if (bsec_init() != BSEC_OK) {
        ESP_LOGE(TAG, "bsec_init failed");
        return ESP_FAIL;
    }

    // Order matters: configuration first (it resets the library state), then
    // the saved calibration state, then the subscription.
    bsec_config_load();
    bsec_state_load();

    // Subscribe to the virtual sensors we expose, all at the configured rate.
    const bsec_virtual_sensor_t wanted[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };
    bsec_sensor_configuration_t requested[sizeof(wanted) / sizeof(wanted[0])];
    for (size_t i = 0; i < sizeof(wanted) / sizeof(wanted[0]); i++) {
        requested[i].sample_rate = BSEC_SAMPLE_RATE;
        requested[i].sensor_id = (float)wanted[i];
    }
    bsec_sensor_configuration_t required[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required = BSEC_MAX_PHYSICAL_SENSOR;
    bsec_library_return_t rc = bsec_update_subscription(
        requested, sizeof(wanted) / sizeof(wanted[0]), required, &n_required);
    // BSEC convention: negative = error, positive = warning/info, 0 = OK.
    // A positive code (e.g. BSEC_W_SU_SAMPLERATEMISMATCH = 14 when running on
    // the built-in default config) means the subscription still succeeded, so
    // log it and carry on — only a negative code is fatal.
    if (rc < BSEC_OK) {
        ESP_LOGE(TAG, "bsec_update_subscription failed: %d", rc);
        return ESP_FAIL;
    }
    if (rc != BSEC_OK) {
        ESP_LOGW(TAG, "bsec_update_subscription warning: %d", rc);
    }

    ESP_LOGI(TAG, "BSEC initialised (%s rate)",
             BSEC_SAMPLE_RATE == BSEC_SAMPLE_RATE_LP ? "LP/3s" : "ULP/300s");
    return ESP_OK;
}

esp_err_t bsec_integration_start(void)
{
    if (s_task != NULL) {
        return ESP_OK;
    }
    // 8 KB: BSEC's do_steps/sensor_control plus the BME688 driver call tree need
    // comfortable headroom (the 4 KB state/work buffers are heap-allocated).
    if (xTaskCreate(bsec_task, "bsec", 8192, NULL, tskIDLE_PRIORITY + 2, &s_task) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

#endif // CONFIG_BME688_USE_BSEC
