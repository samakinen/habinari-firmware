#pragma once

#include "board.h"
#include "mbcontroller.h"
#include "sensor_service.h"

#pragma pack(push, 1)
typedef struct
{
    uint8_t led_on; // 000001
    
} coil_reg_params_t;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct
{
    uint8_t service_running:1; // 100001
    uint8_t error_flag:1; // 100002
    uint8_t probe_present:1; // 100003

} discrete_reg_params_t;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct
{
    float temperature; // 300001
    float humidity; // 300003
    float pressure; // 300005
    float probe_temperature; // 300007
    float probe_humidity; // 300009
    uint16_t co2_ppm; // 300011
} input_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint16_t slave_address; // 400001
} holding_reg_params_t;
#pragma pack(pop)

#define MB_SLAVE_NAME_MAX_LEN 32 // Max length of device name
typedef struct {
    void *mbc_slave_handle; // Modbus controller handle
    holding_reg_params_t holding_reg_params;
    input_reg_params_t input_reg_params;
    coil_reg_params_t coil_reg_params;
    discrete_reg_params_t discrete_reg_params;
} mb_rtu_slave_t;

typedef mb_rtu_slave_t *mb_rtu_slave_handle_t; // Modbus RTU slave handle type

esp_err_t mb_rtu_slave_set_sensor_data(mb_rtu_slave_t *mb_rtu_slave, const sensor_data_t *sensor_data);
esp_err_t mb_rtu_slave_init(mb_rtu_slave_t *mb_rtu_slave, uint8_t slave_address);
esp_err_t mb_rtu_slave_deinit(mb_rtu_slave_t *mb_rtu_slave);
