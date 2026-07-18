#pragma once

#include "esp_err.h"
#include "sensor_service.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t knx_service_start(void);
void knx_service_update_sensor_data(const sensor_data_t *data);
void knx_service_toggle_programming_mode(void);
void knx_service_reset_nvm(void);
typedef void (*knx_programming_mode_callback_t)(bool enabled);
void knx_service_set_programming_mode_callback(knx_programming_mode_callback_t callback);

#ifdef __cplusplus
}
#endif