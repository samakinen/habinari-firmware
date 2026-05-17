#pragma once

#include "esp_err.h"
#include "sensor_service.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t knx_service_start(void);
void knx_service_update_sensor_data(const sensor_data_t *data);
void knx_service_toggle_programming_mode(void);

#ifdef __cplusplus
}
#endif