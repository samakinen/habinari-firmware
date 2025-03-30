#include "board.h"
#include "sensor_bus.h"
#include "hdc302x.h"
#include "scd4x.h"
#include "bme68x_esp.h"
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

esp_err_t init_bme68x(sensor_bus_handle_t sensor_bus_handle)
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
    results->updated_mask = 0;
    struct bme68x_data data;
    uint8_t n_data;
    int8_t rslt;

    if (sensor_bus_handle == NULL || results == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    ret = hdc302x_start_measurement(sensor_bus_handle->hdc302x_device_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start HDC302x measurement: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(25)); // wait for measurement to complete
    ret = hdc302x_read_measurement(
            sensor_bus_handle->hdc302x_device_handle,
            &results->hdc302x_temperature,
            &results->hdc302x_humidity);
    if (ret == ESP_OK)
    {
        results->updated_mask |= SENSOR_HDC302X_TEMPERATURE | SENSOR_HDC302X_HUMIDITY;
    }
    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &sensor_bus_handle->bme68x_dev);
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(TAG, "Failed to set BME68x op mode: %d", rslt);
    }
    vTaskDelay(pdMS_TO_TICKS(sensor_bus_handle->bme68x_wait_time + 10)); // wait for measurement to complete
    rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_data, &sensor_bus_handle->bme68x_dev);
    if (rslt != BME68X_OK && rslt != BME68X_W_NO_NEW_DATA)
    {
        ESP_LOGE(TAG, "Failed to get BME68x data: %d", rslt);
    }
    if (n_data > 0 && rslt==BME68X_OK)
    {
        results->bme68x_temperature = data.temperature / 100.0f;
        results->bme68x_humidity = data.humidity / 1000.0f;
        results->bme68x_pressure = data.pressure;
        results->bme68x_gas_resistance = data.gas_resistance;
        results->updated_mask |= SENSOR_BME68X_TEMPERATURE | SENSOR_BME68X_HUMIDITY | SENSOR_BME68X_PRESSURE | SENSOR_BME68X_GAS_RESISTANCE;
    }

    if (results->updated_mask & SENSOR_BME68X_PRESSURE)
    {
        ret = scd4x_set_ambient_pressure(sensor_bus_handle->scd4x_device_handle,
                (uint16_t)((float)results->bme68x_pressure / 100.0f));
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set SCD4x ambient pressure: %s", esp_err_to_name(ret));
        }
    }
    if (scd4x_data_ready_status(sensor_bus_handle->scd4x_device_handle))
    {
        ret = scd4x_read_measurement(
                sensor_bus_handle->scd4x_device_handle,
                &results->scd4x_co2,
                &results->scd4x_temperature,
                &results->scd4x_humidity);
        if (ret != ESP_OK)
        {
            return ret;
        }
        results->updated_mask |= SENSOR_SCD4X_TEMPERATURE | SENSOR_SCD4X_HUMIDITY | SENSOR_SCD4X_CO2;
    }
    return ESP_OK;
}

esp_err_t sensor_bus_deinit(sensor_bus_handle_t sensor_bus_handle)
{
    return sensor_bus_disable(sensor_bus_handle);
}
