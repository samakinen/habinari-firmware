// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "sensor_service.h"

#include "ext_probe.h"
#include "sensor_bus.h"
#include "sensor_fusion_service.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor_service";

esp_err_t sensor_service_init(sensor_service_handle_t handle, sensor_data_updated_callback_t callback)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing sensor service...");
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    handle->callback = callback;
    handle->task_handle = NULL;
    handle->stop_requested = false;
    ret = sensor_bus_init(&handle->sensor_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize sensor bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = ext_probe_init(&handle->ext_probe);
    if (ret != ESP_OK)
    {
        // Not fatal: the probe is an optional accessory, and the task retries
        // for it periodically so it can be plugged in later.
        ESP_LOGW(TAG, "External probe unavailable: %s", EXT_PROBE_STATUS_TO_TEXT(handle->ext_probe.status));
    }
    ESP_LOGI(TAG, "Sensor service initialized");
    return ESP_OK;
}

// Try to bring up a probe that was not present at startup. Cheap when there is
// nothing there (one I2C probe transaction per address), so it is run on a slow
// cycle rather than every sample.
static void sensor_service_retry_probe(sensor_service_handle_t service_handle)
{
    if (service_handle->ext_probe.status == EXT_PROBE_OPERATIONAL)
    {
        return;
    }
    // A short circuit is a wiring fault, not a missing accessory: retrying it
    // just re-powers a shorted rail every few minutes.
    if (service_handle->ext_probe.status == EXT_PROBE_SHORT_CIRCUIT)
    {
        return;
    }
    ext_probe_deinit(&service_handle->ext_probe);
    if (ext_probe_init(&service_handle->ext_probe) == ESP_OK
        && service_handle->ext_probe.status == EXT_PROBE_OPERATIONAL)
    {
        ESP_LOGI(TAG, "External probe detected and initialized");
    }
}

static void sensor_service_task(void *param)
{
    sensor_service_handle_t service_handle = (sensor_service_handle_t)param;
    sensor_bus_results_t bus_results;
    ext_probe_results_t probe_results;
    sensor_data_t sensor_data = {0};
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_sample_us = esp_timer_get_time();
    uint32_t cycle = 0;

    while (!service_handle->stop_requested)
    {
        const int64_t now_us = esp_timer_get_time();
        // Measured, not assumed: a cycle that overran (a slow I2C retry, a
        // preempting KNX burst) must feed the filters and rate detectors its
        // real elapsed time or every rate it computes is wrong.
        const float dt_seconds = (float)(now_us - last_sample_us) / 1000000.0f;
        last_sample_us = now_us;

        const esp_err_t bus_ret = sensor_bus_read(&service_handle->sensor_bus, &bus_results);
        if (bus_ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Sensor bus produced no readings this cycle: %s", esp_err_to_name(bus_ret));
        }

        bool have_probe = false;
        if (service_handle->ext_probe.status == EXT_PROBE_OPERATIONAL)
        {
            if (ext_probe_read(&service_handle->ext_probe, &probe_results) == ESP_OK)
            {
                have_probe = true;
            }
            else
            {
                ESP_LOGW(TAG, "Failed to read external probe: %s",
                         EXT_PROBE_STATUS_TO_TEXT(service_handle->ext_probe.status));
            }
        }

        // One place turns raw readings into the record everything else uses:
        // conditioning, redundancy, cross-validation, trends and events.
        sensor_fusion_update(&bus_results, have_probe ? &probe_results : NULL, dt_seconds,
                             &sensor_data);

        if (service_handle->callback)
        {
            service_handle->callback(&sensor_data);
        }

        if (++cycle % SENSOR_SERVICE_PROBE_RETRY_CYCLES == 0)
        {
            sensor_service_retry_probe(service_handle);
        }

        // Fixed cadence regardless of how long the cycle itself took, so the
        // sampling interval the filters are tuned for is the one they get.
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_SERVICE_SAMPLE_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "Sensor service task exiting");
    service_handle->task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t sensor_service_start(sensor_service_handle_t service_handle)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Starting sensor service...");
    if (service_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (service_handle->task_handle != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    ret = sensor_bus_enable(&service_handle->sensor_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable sensor bus: %s", esp_err_to_name(ret));
        return ret;
    }

    service_handle->stop_requested = false;
    BaseType_t task_created = xTaskCreate(
        sensor_service_task,
        "SensorServiceTask",
        4096,
        service_handle,
        tskIDLE_PRIORITY + 1,
        &service_handle->task_handle
    );

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create sensor service task");
        sensor_bus_disable(&service_handle->sensor_bus);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Sensor service running, sampling every %d ms",
             SENSOR_SERVICE_SAMPLE_INTERVAL_MS);
    return ESP_OK;
}

esp_err_t sensor_service_stop(sensor_service_handle_t service_handle)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Stopping sensor service...");
    if (service_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Ask, then wait. Deleting the task outright (what this used to do) could
    // strand the BME688 mutex mid-transaction and hang the BSEC task with it.
    service_handle->stop_requested = true;
    for (int waited_ms = 0;
         service_handle->task_handle != NULL && waited_ms < SENSOR_SERVICE_SAMPLE_INTERVAL_MS * 2;
         waited_ms += 50)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (service_handle->task_handle != NULL)
    {
        ESP_LOGW(TAG, "Sensor task did not exit in time; deleting it");
        vTaskDelete(service_handle->task_handle);
        service_handle->task_handle = NULL;
    }

    ret = sensor_bus_disable(&service_handle->sensor_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to disable sensor bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = ext_probe_deinit(&service_handle->ext_probe);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to deinitialize external probe: %s",
                 EXT_PROBE_STATUS_TO_TEXT(service_handle->ext_probe.status));
        return ret;
    }
    ESP_LOGI(TAG, "Sensor service stopped");
    return ESP_OK;
}
