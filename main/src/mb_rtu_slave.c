#define CONFIG_FMB_COMM_MODE_RTU_EN 1
#define CONFIG_FMB_CONTROLLER_SLAVE_ID_SUPPORT 1
#include "mb_rtu_slave.h"
#include "esp_log.h"

static const char *TAG = "modbus_rtu_slave"; // Log tag for Modbus RTU slave

#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
#define HOLD_SIZE(first, last) ((size_t)((HOLD_OFFSET(last) - HOLD_OFFSET(first)) << 1))
#define INPUT_SIZE(first, last) ((size_t)((INPUT_OFFSET(last) - INPUT_OFFSET(first)) << 1))

#define MB_REG_DISCRETE_INPUT_START         (0x0000)
#define MB_REG_COILS_START                  (0x0000)
#define MB_REG_INPUT_START_AREA0            (INPUT_OFFSET(input_data0)) // register offset input area 0
#define MB_REG_INPUT_START_AREA1            (INPUT_OFFSET(input_data4)) // register offset input area 1
#define MB_REG_HOLDING_START_AREA0          (HOLD_OFFSET(holding_data0))
#define MB_REG_HOLDING_START_AREA0_SIZE     ((size_t)((HOLD_OFFSET(holding_data4) - HOLD_OFFSET(holding_data0)) << 1))
#define MB_REG_HOLDING_START_AREA1          (HOLD_OFFSET(holding_data4))
#define MB_REG_HOLDING_START_AREA1_SIZE     ((size_t)((HOLD_OFFSET(holding_area1_end) - HOLD_OFFSET(holding_data4)) << 1))
#define MB_REG_HOLDING_START_AREA2          (HOLD_OFFSET(holding_u8_a))
#define MB_REG_HOLDING_START_AREA2_SIZE     ((size_t)((HOLD_OFFSET(holding_area2_end) - HOLD_OFFSET(holding_u8_a)) << 1))

const char *device_name = "air quality sensor"; // Device name for Modbus slave

void init_registers(mb_rtu_slave_t *mb_rtu_slave)
{
    mb_rtu_slave->holding_reg_params.slave_address = 0x01; // Set slave address
    mb_rtu_slave->input_reg_params.temperature = 0.0; // Initialize temperature register
    mb_rtu_slave->input_reg_params.humidity = 0.0; // Initialize humidity register
    mb_rtu_slave->input_reg_params.co2_ppm = 0; // Initialize CO2 register
    mb_rtu_slave->input_reg_params.pressure = 0; // Initialize pressure register
    mb_rtu_slave->input_reg_params.probe_temperature = 0.0; // Initialize probe temperature register
    mb_rtu_slave->input_reg_params.probe_humidity = 0.0; // Initialize probe humidity register
    mb_rtu_slave->coil_reg_params.led_on = 0; // Initialize LED state register
    mb_rtu_slave->discrete_reg_params.service_running = 0; // Set server running flag
    mb_rtu_slave->discrete_reg_params.error_flag = 0; // Clear error flag
    mb_rtu_slave->discrete_reg_params.probe_present = 0; // Set probe present flag
}

esp_err_t mb_rtu_slave_set_sensor_data(mb_rtu_slave_t *mb_rtu_slave, const sensor_data_t *sensor_data)
{
    esp_err_t ret;
    if (sensor_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ret = mbc_slave_lock(mb_rtu_slave->mbc_slave_handle); // Lock the Modbus controller
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to lock Modbus controller: %s", esp_err_to_name(ret));
        return ret;
    }
    mb_rtu_slave->input_reg_params.temperature = sensor_data->temperature; // Set temperature register
    mb_rtu_slave->input_reg_params.humidity = sensor_data->humidity; // Set humidity register
    mb_rtu_slave->input_reg_params.co2_ppm = sensor_data->co2; // Set CO2 register
    mb_rtu_slave->input_reg_params.pressure = sensor_data->pressure; // Set pressure register
    mb_rtu_slave->input_reg_params.probe_temperature = sensor_data->ext_probe_temperature; // Set probe temperature register
    mb_rtu_slave->input_reg_params.probe_humidity = sensor_data->ext_probe_humidity; // Set probe humidity register
    ret = mbc_slave_unlock(mb_rtu_slave->mbc_slave_handle); // Unlock the Modbus controller
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unlock Modbus controller: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}
esp_err_t mb_rtu_slave_init(mb_rtu_slave_t *mb_rtu_slave, uint8_t slave_address)
{
    esp_err_t ret;
    mb_register_area_descriptor_t reg_area = {0}; // Modbus register area descriptor structure

    if (mb_rtu_slave == NULL) {
        ESP_LOGE(TAG, "mb_rtu_slave is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (slave_address == 0 || slave_address > 247) {
        ESP_LOGE(TAG, "Invalid slave address: %d", slave_address);
        return ESP_ERR_INVALID_ARG;
    }
    if (mb_rtu_slave->mbc_slave_handle != NULL) {
        ESP_LOGE(TAG, "Modbus controller already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    init_registers(mb_rtu_slave); // Initialize registers
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // Initialize Modbus controller
    mb_communication_info_t comm_config = {
        .ser_opts.port = MODBUS_RTU_UART_NUM,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = MODBUS_RTU_UART_BAUDRATE,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.uid = slave_address,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1
    };
  
    ret = mbc_slave_create_serial(&comm_config, &mb_rtu_slave->mbc_slave_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create Modbus controller: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialization of Coils register area
    reg_area.type = MB_PARAM_COIL;
    reg_area.start_offset = 0;
    reg_area.address = (void*)&mb_rtu_slave->coil_reg_params;
    reg_area.size = sizeof(coil_reg_params_t);
    reg_area.access = MB_ACCESS_RW;
    ret = mbc_slave_set_descriptor(mb_rtu_slave->mbc_slave_handle, reg_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set descriptor for coils register area: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialization of Discrete Inputs register area
    reg_area.type = MB_PARAM_DISCRETE;
    reg_area.start_offset = MB_REG_DISCRETE_INPUT_START;
    reg_area.address = (void*)&mb_rtu_slave->discrete_reg_params;
    reg_area.size = sizeof(discrete_reg_params_t);
    ret = mbc_slave_set_descriptor(mb_rtu_slave->mbc_slave_handle, reg_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set descriptor for discrete register area: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialization of Input Registers area
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = 0;
    reg_area.address = (void*)&mb_rtu_slave->input_reg_params.temperature;
    reg_area.size = sizeof(input_reg_params_t) << 1;
    ret = mbc_slave_set_descriptor(mb_rtu_slave->mbc_slave_handle, reg_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set descriptor for input register area 0: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialization of Holding Registers area
    reg_area.type = MB_PARAM_HOLDING; // Set type of register area
    reg_area.start_offset = HOLD_OFFSET(slave_address); // Offset of register area in Modbus protocol
    reg_area.address = (void*)&mb_rtu_slave->holding_reg_params.slave_address; // Set pointer to storage instance
    reg_area.size = sizeof(holding_reg_params_t);
    reg_area.access = MB_ACCESS_RW;
    ret = mbc_slave_set_descriptor(mb_rtu_slave->mbc_slave_handle, reg_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set descriptor for holding register area 0: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set UART pin numbers
    ret = uart_set_pin(
            MODBUS_RTU_UART_NUM,
            PIN_RS485_TX,
            PIN_RS485_RX,
            PIN_RS485_TEN,
            UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART driver mode to Half Duplex
    ret = uart_set_mode(MODBUS_RTU_UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Starts of modbus controller and stack
    esp_err_t err = mbc_slave_start(mb_rtu_slave->mbc_slave_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Modbus controller: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize the new slave identificator structure (example)
    uint8_t is_running = (bool)(err == ESP_OK);

    // This is the way to set Slave ID fields to retrieve it by master using report slave ID command.
    err = mbc_set_slave_id(mb_rtu_slave->mbc_slave_handle, comm_config.ser_opts.uid, is_running, (uint8_t*)device_name, strlen(device_name));
    if (err == ESP_OK) {
        ESP_LOGW("SET_SLAVE_ID", "dev_name: %s", device_name);
    } else {
        ESP_LOGE("SET_SLAVE_ID", "Set slave ID fail, err=%d.", err);
    }

    ESP_LOGI(TAG, "Modbus slave stack initialized.");
    return ESP_OK;
}

esp_err_t mb_rtu_slave_deinit(mb_rtu_slave_handle_t slave_handle)
{
    esp_err_t ret;
    if (slave_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ret = mbc_slave_stop(slave_handle->mbc_slave_handle); // Stop Modbus controller
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop Modbus controller: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mbc_slave_delete(slave_handle->mbc_slave_handle); // Delete Modbus controller
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete Modbus controller: %s", esp_err_to_name(ret));
        return ret;
    }
    slave_handle->mbc_slave_handle = NULL;
    ESP_LOGI(TAG, "Modbus slave deinitialized.");
    return ESP_OK;
}