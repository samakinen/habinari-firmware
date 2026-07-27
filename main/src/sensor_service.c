#include "sensor_service.h"
#include "sensor_bus.h"
#include "ext_probe.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const int REFRESH_RATE = 30000; // 30 seconds
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
    ret = sensor_bus_init(&handle->sensor_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize sensor bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = ext_probe_init(&handle->ext_probe);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to initialize external probe: %s", EXT_PROBE_STATUS_TO_TEXT(handle->ext_probe.status));
    }
    ESP_LOGI(TAG, "Sensor service initialized");
    return ESP_OK;
}

static void sensor_service_update_results(sensor_data_t *sensor_data, sensor_bus_results_t *sensor_bus_results, ext_probe_results_t *ext_probe_results)
{
    sensor_data->updated_mask = 0;

    if (sensor_bus_results->updated_mask & SENSOR_HDC302X_TEMPERATURE)
    {
        sensor_data->temperature = sensor_bus_results->hdc302x_temperature;
        sensor_data->updated_mask |= SENSOR_TEMPERATURE;
    }
    if (sensor_bus_results->updated_mask & SENSOR_HDC302X_HUMIDITY)
    {
        sensor_data->humidity = sensor_bus_results->hdc302x_humidity;
        sensor_data->updated_mask |= SENSOR_HUMIDITY;
    }
    if (sensor_bus_results->updated_mask & SENSOR_BME68X_PRESSURE)
    {
        sensor_data->pressure = sensor_bus_results->bme68x_pressure;
        sensor_data->updated_mask |= SENSOR_PRESSURE;
    }
    if (sensor_bus_results->updated_mask & SENSOR_BME68X_IAQ)
    {
        sensor_data->iaq = sensor_bus_results->bme68x_iaq;
        sensor_data->air_quality_accuracy = sensor_bus_results->bme68x_iaq_accuracy;
        sensor_data->updated_mask |= SENSOR_IAQ;
    }
    if (sensor_bus_results->updated_mask & SENSOR_BME68X_CO2_EQUIVALENT)
    {
        sensor_data->co2_equivalent = sensor_bus_results->bme68x_co2_equivalent;
        sensor_data->updated_mask |= SENSOR_CO2_EQUIVALENT;
    }
    if (sensor_bus_results->updated_mask & SENSOR_BME68X_VOC_EQUIVALENT)
    {
        sensor_data->voc_equivalent = sensor_bus_results->bme68x_voc_equivalent;
        sensor_data->updated_mask |= SENSOR_VOC_EQUIVALENT;
    }
    if (sensor_bus_results->updated_mask & SENSOR_SCD4X_CO2)
    {
        sensor_data->co2 = sensor_bus_results->scd4x_co2;
        sensor_data->updated_mask |= SENSOR_CO2;
    }

    if (ext_probe_results != NULL)
    {
        sensor_data->ext_probe_temperature = ext_probe_results->temperature;
        sensor_data->updated_mask |= SENSOR_EXT_PROBE_TEMPERATURE;
        sensor_data->ext_probe_humidity = ext_probe_results->humidity;
        sensor_data->updated_mask |= SENSOR_EXT_PROBE_HUMIDITY;
    }
}

static void sensor_service_task(void *param)
{
    sensor_service_handle_t service_handle = (sensor_service_handle_t)param;
    sensor_data_t sensor_data;
    sensor_bus_results_t sensor_bus_results;
    ext_probe_results_t ext_probe_results;
    bool has_ext_probe = false;

    while (1)
    {
        // Delay to control the frequency of sensor data updates
        vTaskDelay(pdMS_TO_TICKS(REFRESH_RATE));
        esp_err_t ret = sensor_bus_read(&service_handle->sensor_bus, &sensor_bus_results);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read sensor bus: %s", esp_err_to_name(ret));
            //continue;
        }

        has_ext_probe = (service_handle->ext_probe.status == EXT_PROBE_OPERATIONAL);
        if (has_ext_probe)
        {
            ret = ext_probe_read(&service_handle->ext_probe, &ext_probe_results);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to read external probe: %s", EXT_PROBE_STATUS_TO_TEXT(service_handle->ext_probe.status));
                has_ext_probe = false;
            }
        }
        else
        {
            ESP_LOGI(TAG, "External probe not operational or not connected (%s)", EXT_PROBE_STATUS_TO_TEXT(service_handle->ext_probe.status));
        }

        sensor_service_update_results(&sensor_data, &sensor_bus_results, has_ext_probe ? &ext_probe_results : NULL);

        if (service_handle->callback)
        {
            service_handle->callback(&sensor_data);
        }
    }
}

esp_err_t sensor_service_start(sensor_service_handle_t service_handle)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Starting sensor service...");
    if (service_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    ret = sensor_bus_enable(&service_handle->sensor_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable sensor bus: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t task_created = xTaskCreate(
        sensor_service_task,         // Task function
        "SensorServiceTask",         // Task name
        4096,                        // Stack size in words
        service_handle,              // Task parameter
        tskIDLE_PRIORITY + 1,        // Task priority
        &service_handle->task_handle // Task handle
    );

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create sensor service task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Sensor service running");
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
    if (service_handle->task_handle != NULL)
    {
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
        ESP_LOGE(TAG, "Failed to deinitialize external probe: %s", EXT_PROBE_STATUS_TO_TEXT(service_handle->ext_probe.status));
        return ret;
    }
    ESP_LOGI(TAG, "Sensor service stopped");
    return ESP_OK;
}