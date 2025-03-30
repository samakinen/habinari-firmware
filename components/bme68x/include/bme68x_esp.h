#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "bme68x.h"

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bme68x_delay_us(uint32_t period_us, void *intf_ptr);

esp_err_t bme68x_device_create_i2c(struct bme68x_dev *dev, i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);
esp_err_t bme68x_device_delete_i2c(struct bme68x_dev *dev);

#ifdef __cplusplus
}
#endif
