#include "board.h"
#include "sensor_bus.h"
#include "hdc302x.h"
#include "scd4x.h"
#include "bme68x_esp.h"
#include "bsec_integration.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor_bus";

esp_err_t sensor_bus_init(sensor_bus_handle_t sensor_bus)
{
    if (sensor_bus == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (sensor_bus->bus_handle != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    sensor_bus->hdc302x_device_handle = NULL;
    sensor_bus->scd4x_device_handle = NULL;
    if (sensor_bus->bme68x_mutex == NULL)
    {
        sensor_bus->bme68x_mutex = xSemaphoreCreateMutex();
        if (sensor_bus->bme68x_mutex == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

static inline esp_err_t sensor_bus_reg_enable(void)
{
    esp_err_t ret;
    ret = gpio_set_level(PIN_2V8_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_BUS_STARTUP_TIME));
    return ret;
}

static inline esp_err_t sensor_bus_reg_disable(void)
{
    return gpio_set_level(PIN_2V8_EN, 0);
}

static esp_err_t init_bme68x(sensor_bus_handle_t sensor_bus_handle)
{
    esp_err_t ret;
    ret = bme68x_device_create_i2c(&sensor_bus_handle->bme68x_dev, sensor_bus_handle->bus_handle, BME68X_I2C_ADDR_LOW, SENSOR_BUS_SPEED);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create BME68x device: %s", esp_err_to_name(ret));
        return ret;
    }
    bme68x_init(&sensor_bus_handle->bme68x_dev);
    ESP_LOGI(TAG, "BME68x device created");
    struct bme68x_conf conf;
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_8X;
    conf.os_pres = BME68X_OS_16X;
    conf.os_temp = BME68X_OS_8X;
    int8_t rslt = bme68x_set_conf(&conf, &sensor_bus_handle->bme68x_dev);
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(TAG, "Failed to set BME68x conf: %d", rslt);
        return ESP_FAIL;
    }
    bme68x_get_conf(&conf, &sensor_bus_handle->bme68x_dev);
    ESP_LOGI(TAG, "BME68x conf: os_hum: %d, os_temp: %d, os_pres: %d, filter: %d, odr: %d",
        conf.os_hum, conf.os_temp, conf.os_pres, conf.filter, conf.odr);

    sensor_bus_handle->bme68x_wait_time = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &sensor_bus_handle->bme68x_dev) / 1000;

    // Hand the BME688 to BSEC for air-quality fusion when compiled in
    // (CONFIG_BME688_USE_BSEC). BSEC then owns the gas-measurement cadence
    // (heater profile + timing) via its own task, and this reader pulls the
    // fused outputs. Inert no-op in stub builds — then this reader does its own
    // BME688 forced T/H/P reads instead (see sensor_bus_read).
    esp_err_t bsec_ret = bsec_integration_init(&sensor_bus_handle->bme68x_dev,
                                               sensor_bus_handle->bme68x_mutex);
    if (bsec_ret == ESP_OK)
    {
        bsec_integration_start();
    }
    else
    {
        ESP_LOGW(TAG, "BSEC init failed: %s", esp_err_to_name(bsec_ret));
    }
    return ESP_OK;
}

esp_err_t sensor_bus_enable(sensor_bus_handle_t sensor_bus_handle)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Enabling sensor bus...");
    ret = sensor_bus_reg_enable();
    if (ret != ESP_OK)
    {
        return ret;
    }
    if (sensor_bus_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (sensor_bus_handle->bus_handle != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    ret = i2c_bus_init(&sensor_bus_handle->bus_handle,
            PIN_SDA,
            PIN_SCL,
            true);
    sensor_bus_handle->hdc302x_device_handle = hdc302x_device_create(
                                                sensor_bus_handle->bus_handle,
                                                HDC203X_ADDR_0,
                                                SENSOR_BUS_SPEED);
    if (sensor_bus_handle->hdc302x_device_handle == NULL)
    {
        return ESP_FAIL;
    }
    sensor_bus_handle->scd4x_device_handle = scd4x_device_create(
                                                sensor_bus_handle->bus_handle,
                                                SCD4X_ADDR_0,
                                                SENSOR_BUS_SPEED);
    if (sensor_bus_handle->scd4x_device_handle == NULL)
    {
        return ESP_FAIL;
    }
    ret = scd4x_start_low_power_periodic_measurement(sensor_bus_handle->scd4x_device_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start SCD4x measurement: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = init_bme68x(sensor_bus_handle);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ESP_LOGI(TAG, "Sensor bus enabled");
    return ESP_OK;
}

esp_err_t sensor_bus_disable(sensor_bus_handle_t sensor_bus_handle)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Disabling sensor bus...");
    if (sensor_bus_handle->hdc302x_device_handle != NULL)
    {
        ret = hdc302x_device_delete(sensor_bus_handle->hdc302x_device_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to delete HDC302x device: %s", esp_err_to_name(ret));
        }
        sensor_bus_handle->hdc302x_device_handle = NULL;
    }
    if (sensor_bus_handle->scd4x_device_handle != NULL)
    {
        ret = scd4x_device_delete(sensor_bus_handle->scd4x_device_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to delete SCD4x device: %s", esp_err_to_name(ret));
        }
        sensor_bus_handle->scd4x_device_handle = NULL;
    }
    if (sensor_bus_handle->bme68x_dev.intf_ptr != NULL)
    {
        ret = bme68x_device_delete_i2c(&sensor_bus_handle->bme68x_dev);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to delete BME68x device: %s", esp_err_to_name(ret));
        }
        sensor_bus_handle->bme68x_dev.intf_ptr = NULL;
    }
    if (sensor_bus_handle->bus_handle != NULL)
    {
        ret = i2c_del_master_bus(sensor_bus_handle->bus_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
        }
        sensor_bus_handle->bus_handle = NULL;
    }
    sensor_bus_reg_disable();
    ESP_LOGI(TAG, "Sensor bus disabled");
    return ESP_OK;
}

esp_err_t sensor_bus_read(sensor_bus_handle_t sensor_bus_handle, sensor_bus_results_t *results)
{
    esp_err_t ret;
    struct bme68x_data data;
    uint8_t n_data;
    int8_t rslt;

    if (sensor_bus_handle == NULL || results == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    results->updated_mask = 0;
    results->error_count = 0;

    ret = hdc302x_start_measurement(sensor_bus_handle->hdc302x_device_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to start HDC302x measurement: %s", esp_err_to_name(ret));
        results->error_count++;
    }
    vTaskDelay(pdMS_TO_TICKS(25)); // wait for measurement to complete
    ret = hdc302x_read_measurement(
            sensor_bus_handle->hdc302x_device_handle,
            &results->hdc302x_temperature,
            &results->hdc302x_humidity);
    if (ret == ESP_OK)
    {
        results->updated_mask |= SENSOR_BUS_HDC302X_TEMPERATURE | SENSOR_BUS_HDC302X_HUMIDITY;
    }
    else
    {
        ESP_LOGW(TAG, "Failed to read HDC302x: %s", esp_err_to_name(ret));
        results->error_count++;
    }
    if (bsec_integration_available())
    {
        // BSEC owns the BME688 and produces fused T/H/P + air-quality outputs on
        // its own cadence; just snapshot the latest here.
        bsec_air_quality_t aq;
        if (bsec_integration_get_latest(&aq))
        {
            results->bme68x_temperature = aq.comp_temperature;
            results->bme68x_humidity = aq.comp_humidity;
            results->bme68x_pressure = aq.pressure;
            results->updated_mask |= SENSOR_BUS_BME68X_TEMPERATURE | SENSOR_BUS_BME68X_HUMIDITY
                                     | SENSOR_BUS_BME68X_PRESSURE;
            // Only surface air quality once BSEC has started calibrating
            // (accuracy 0 = unreliable warm-up).
            if (aq.iaq_accuracy > 0)
            {
                results->bme68x_iaq = aq.iaq;
                results->bme68x_iaq_accuracy = aq.iaq_accuracy;
                results->bme68x_co2_equivalent = aq.co2_equivalent;
                results->bme68x_voc_equivalent = aq.voc_equivalent;
                results->updated_mask |= SENSOR_BUS_BME68X_IAQ | SENSOR_BUS_BME68X_CO2_EQUIVALENT
                                         | SENSOR_BUS_BME68X_VOC_EQUIVALENT;
            }
        }
    }
    else
    {
        // No BSEC: read BME688 temperature/humidity/pressure directly (forced
        // mode, gas heater unused). The mutex is uncontended here since the BSEC
        // task does not run, but keeps the access pattern uniform.
        xSemaphoreTake(sensor_bus_handle->bme68x_mutex, portMAX_DELAY);
        rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &sensor_bus_handle->bme68x_dev);
        if (rslt != BME68X_OK)
        {
            ESP_LOGW(TAG, "Failed to set BME68x op mode: %d", rslt);
            results->error_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(sensor_bus_handle->bme68x_wait_time + 10)); // wait for measurement to complete
        rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_data, &sensor_bus_handle->bme68x_dev);
        xSemaphoreGive(sensor_bus_handle->bme68x_mutex);
        if (rslt != BME68X_OK && rslt != BME68X_W_NO_NEW_DATA)
        {
            ESP_LOGW(TAG, "Failed to get BME68x data: %d", rslt);
            results->error_count++;
        }
        if (n_data > 0 && rslt == BME68X_OK)
        {
            results->bme68x_temperature = data.temperature / 100.0f;
            results->bme68x_humidity = data.humidity / 1000.0f;
            results->bme68x_pressure = data.pressure;
            results->updated_mask |= SENSOR_BUS_BME68X_TEMPERATURE | SENSOR_BUS_BME68X_HUMIDITY
                                     | SENSOR_BUS_BME68X_PRESSURE;
        }
    }

    if (results->updated_mask & SENSOR_BUS_BME68X_PRESSURE)
    {
        ret = scd4x_set_ambient_pressure(sensor_bus_handle->scd4x_device_handle,
                (uint16_t)((float)results->bme68x_pressure / 100.0f));
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to set SCD4x ambient pressure: %s", esp_err_to_name(ret));
            results->error_count++;
        }
    }
    // The SCD4x runs its own low-power periodic cadence (~30 s), so most cycles
    // legitimately find no new sample. That is not an error and must not be
    // counted as one — the fusion layer's staleness window is what decides when
    // the CO2 reading has actually gone quiet.
    if (scd4x_data_ready_status(sensor_bus_handle->scd4x_device_handle))
    {
        ret = scd4x_read_measurement(
                sensor_bus_handle->scd4x_device_handle,
                &results->scd4x_co2,
                &results->scd4x_temperature,
                &results->scd4x_humidity);
        if (ret == ESP_OK)
        {
            results->updated_mask |= SENSOR_BUS_SCD4X_TEMPERATURE | SENSOR_BUS_SCD4X_HUMIDITY
                                     | SENSOR_BUS_SCD4X_CO2;
        }
        else
        {
            // A failed read here used to abort the whole cycle, throwing away
            // the HDC3020 and BME688 readings already collected above.
            ESP_LOGW(TAG, "Failed to read SCD4x: %s", esp_err_to_name(ret));
            results->error_count++;
        }
    }

    return results->updated_mask != 0 ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t sensor_bus_deinit(sensor_bus_handle_t sensor_bus_handle)
{
    return sensor_bus_disable(sensor_bus_handle);
}
