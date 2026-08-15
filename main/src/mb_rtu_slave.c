#include "mb_rtu_slave.h"

#include <inttypes.h>
#include <string.h>

#include "board.h"
#include "control_state.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mbcontroller.h"
#include "nvs.h"
#include "nvs_flash.h"

// Modbus RTU adapter. Everything protocol-specific — scaling, addressing, line
// settings — is here; everything about what the device *is* comes from
// control_state.h, the same source the KNX side publishes from.

static const char *TAG = "modbus_rtu_slave";

#define MB_NVS_NAMESPACE "modbus"
#define MB_NVS_KEY_ADDRESS "address"
#define MB_NVS_KEY_BAUD "baud"

#define MB_DEFAULT_SLAVE_ADDRESS 1
#define MB_DEFAULT_BAUD_CODE MB_BAUD_19200

// How often the register images are refreshed from control_state and master
// writes are picked up. 200 ms is far below any BMS poll interval and keeps a
// written setpoint feeling immediate, at negligible cost.
#define MB_SERVICE_PERIOD_MS 200

#define MB_DEVICE_NAME "Room HVAC Sensor/Controller"

#define MB_REG_OFFSET(type, field) ((uint16_t)(offsetof(type, field) / 2))

// --- Line settings ---------------------------------------------------------

static void mb_load_line_settings(mb_rtu_slave_t *slave)
{
    slave->slave_address = MB_DEFAULT_SLAVE_ADDRESS;
    slave->baud_code = MB_DEFAULT_BAUD_CODE;

    nvs_handle_t handle = 0;
    if (nvs_open(MB_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGI(TAG, "No stored Modbus settings; using address %u at %" PRIu32 " baud",
                 slave->slave_address, mb_baud_from_code(slave->baud_code));
        return;
    }

    uint8_t address = 0;
    if (nvs_get_u8(handle, MB_NVS_KEY_ADDRESS, &address) == ESP_OK && address >= 1
        && address <= 247) {
        slave->slave_address = address;
    }
    uint16_t baud = 0;
    if (nvs_get_u16(handle, MB_NVS_KEY_BAUD, &baud) == ESP_OK && baud < MB_BAUD_CODE_COUNT) {
        slave->baud_code = baud;
    }
    nvs_close(handle);
    ESP_LOGI(TAG, "Modbus settings: address %u at %" PRIu32 " baud", slave->slave_address,
             mb_baud_from_code(slave->baud_code));
}

static esp_err_t mb_store_line_settings(const mb_rtu_slave_t *slave)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(MB_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(handle, MB_NVS_KEY_ADDRESS, slave->slave_address);
    if (err == ESP_OK) {
        err = nvs_set_u16(handle, MB_NVS_KEY_BAUD, slave->baud_code);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

// --- Stack lifecycle -------------------------------------------------------

static esp_err_t mb_register_area(mb_rtu_slave_t *slave,
                                  mb_param_type_t type,
                                  uint16_t start_offset,
                                  void *address,
                                  size_t size,
                                  mb_param_access_t access)
{
    mb_register_area_descriptor_t area = {
        .type = type,
        .start_offset = start_offset,
        .address = address,
        .size = size,
        .access = access,
    };
    const esp_err_t err = mbc_slave_set_descriptor(slave->mbc_slave_handle, area);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register area type %d: %s", (int)type, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t mb_stack_start(mb_rtu_slave_t *slave)
{
    mb_communication_info_t comm_config = {
        .ser_opts.port = MODBUS_RTU_UART_NUM,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = mb_baud_from_code(slave->baud_code),
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.uid = slave->slave_address,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1,
    };

    esp_err_t err = mbc_slave_create_serial(&comm_config, &slave->mbc_slave_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create Modbus controller: %s", esp_err_to_name(err));
        return err;
    }

    // sizeof() in bytes is what the descriptor wants. The previous code passed
    // `sizeof(...) << 1` for the input area, publishing twice the struct and
    // letting a master read whatever followed it in memory.
    err = mb_register_area(slave, MB_PARAM_INPUT, 0, &slave->input_regs,
                           sizeof(slave->input_regs), MB_ACCESS_RO);
    if (err == ESP_OK) {
        err = mb_register_area(slave, MB_PARAM_HOLDING, 0, &slave->holding_regs,
                               sizeof(slave->holding_regs), MB_ACCESS_RW);
    }
    if (err == ESP_OK) {
        err = mb_register_area(slave, MB_PARAM_DISCRETE, 0, slave->discrete_bits,
                               sizeof(slave->discrete_bits), MB_ACCESS_RO);
    }
    if (err == ESP_OK) {
        err = mb_register_area(slave, MB_PARAM_COIL, 0, slave->coil_bits,
                               sizeof(slave->coil_bits), MB_ACCESS_RW);
    }
    if (err != ESP_OK) {
        mbc_slave_delete(slave->mbc_slave_handle);
        slave->mbc_slave_handle = NULL;
        return err;
    }

    err = uart_set_pin(MODBUS_RTU_UART_NUM, PIN_RS485_TX, PIN_RS485_RX, PIN_RS485_TEN,
                       UART_PIN_NO_CHANGE);
    if (err == ESP_OK) {
        err = uart_set_mode(MODBUS_RTU_UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
    }
    if (err == ESP_OK) {
        err = mbc_slave_start(slave->mbc_slave_handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Modbus slave: %s", esp_err_to_name(err));
        mbc_slave_delete(slave->mbc_slave_handle);
        slave->mbc_slave_handle = NULL;
        return err;
    }

    const esp_err_t id_err =
        mbc_set_slave_id(slave->mbc_slave_handle, slave->slave_address, true,
                         (uint8_t *)MB_DEVICE_NAME, strlen(MB_DEVICE_NAME));
    if (id_err != ESP_OK) {
        ESP_LOGW(TAG, "Report Slave ID unavailable: %s", esp_err_to_name(id_err));
    }

    ESP_LOGI(TAG, "Modbus RTU slave listening as address %u at %" PRIu32 " baud",
             slave->slave_address, mb_baud_from_code(slave->baud_code));
    return ESP_OK;
}

static void mb_stack_stop(mb_rtu_slave_t *slave)
{
    if (slave->mbc_slave_handle == NULL) {
        return;
    }
    mbc_slave_stop(slave->mbc_slave_handle);
    mbc_slave_delete(slave->mbc_slave_handle);
    slave->mbc_slave_handle = NULL;
}

// --- Register images -------------------------------------------------------

static void mb_fill_identity(mb_rtu_slave_t *slave)
{
    const uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    slave->input_regs.device_id = MB_DEVICE_ID;
    slave->input_regs.map_version = MB_MAP_VERSION;
    slave->input_regs.firmware_version = (1 << 8) | 0;
    slave->input_regs.uptime_hours = (uint16_t)(uptime_s / 3600);
    slave->input_regs.uptime_seconds = (uint16_t)(uptime_s % 3600);
}

static void mb_fill_measurements(mb_rtu_slave_t *slave, const sensor_data_t *d)
{
    mb_input_registers_t *r = &slave->input_regs;

    r->room_temperature = mb_encode_signed(d->temperature.value, d->temperature.valid,
                                           MB_SCALE_TEMPERATURE);
    r->room_humidity = mb_encode_signed(d->humidity.value, d->humidity.valid, MB_SCALE_HUMIDITY);
    // Pa to 0.1 hPa: a station pressure in Pa does not fit a 16-bit register.
    r->air_pressure = mb_encode_unsigned(d->pressure.value / 100.0f, d->pressure.valid,
                                         MB_SCALE_PRESSURE);
    r->co2 = mb_encode_unsigned(d->co2.value, d->co2.valid, 1);
    r->air_quality_index = mb_encode_unsigned(d->iaq.value, d->iaq.valid, 1);
    r->co2_equivalent = mb_encode_unsigned(d->co2_equivalent.value, d->co2_equivalent.valid, 1);
    r->voc_equivalent = mb_encode_unsigned(d->voc_equivalent.value, d->voc_equivalent.valid, 10);
    r->air_quality_accuracy = d->air_quality_accuracy;

    r->probe_temperature = mb_encode_signed(d->probe_temperature.value,
                                            d->probe_temperature.valid, MB_SCALE_TEMPERATURE);
    r->probe_humidity =
        mb_encode_signed(d->probe_humidity.value, d->probe_humidity.valid, MB_SCALE_HUMIDITY);

    r->temperature_trend = mb_encode_signed(d->trends.temperature.per_minute * 60.0f,
                                            d->trends.temperature.valid, MB_SCALE_TREND);
    r->co2_baseline = mb_encode_unsigned(d->events.co2_baseline_ppm,
                                         d->events.co2_baseline_ppm > 0.0f, 1);
    r->estimated_occupants = d->events.estimated_occupants;
    r->fire_reason_mask = d->events.fire_reason_mask;

    r->sensor_health_mask = d->health.healthy_mask;
    r->sensor_present_mask = d->health.present_mask;
    r->sensor_suspect_mask = d->health.suspect_mask;
    r->sample_count = (uint16_t)d->health.sample_count;
    r->error_count = (uint16_t)d->health.error_count;
    r->temperature_source = d->temperature.source;
    r->humidity_source = d->humidity.source;

    mb_bit_set(slave->discrete_bits, MB_DISCRETE_PROBE_PRESENT,
               (d->health.present_mask & SENSOR_SOURCE_BIT(SENSOR_SOURCE_SHT4X)) != 0);
    mb_bit_set(slave->discrete_bits, MB_DISCRETE_FIRE_ALARM, d->events.fire_alarm);
    mb_bit_set(slave->discrete_bits, MB_DISCRETE_FIRE_PRE_ALARM, d->events.fire_pre_alarm);
    mb_bit_set(slave->discrete_bits, MB_DISCRETE_OCCUPANCY_DETECTED,
               d->events.occupancy_detected);
    mb_bit_set(slave->discrete_bits, MB_DISCRETE_WINDOW_OPEN_DETECTED,
               d->events.window_open_detected);
    mb_bit_set(slave->discrete_bits, MB_DISCRETE_SENSOR_DISAGREEMENT,
               d->temperature.disagreement || d->humidity.disagreement);
}

static void mb_fill_control(mb_rtu_slave_t *slave, const control_state_t *state)
{
    mb_input_registers_t *r = &slave->input_regs;

    r->air_pressure_sea = mb_encode_unsigned(state->sea_level_pressure_pa / 100.0f,
                                             state->sensors.pressure.valid, MB_SCALE_PRESSURE);
    r->room_dew_point = mb_encode_signed(state->room_dew_point_c,
                                         state->sensors.temperature.valid
                                             && state->sensors.humidity.valid,
                                         MB_SCALE_TEMPERATURE);
    r->room_absolute_humidity = mb_encode_unsigned(state->room_absolute_humidity_gm3,
                                                   state->sensors.humidity.valid,
                                                   MB_SCALE_DENSITY);
    r->probe_absolute_humidity = mb_encode_unsigned(state->floor_absolute_humidity_gm3,
                                                    state->sensors.probe_humidity.valid,
                                                    MB_SCALE_DENSITY);
    r->dew_point_margin = mb_encode_signed(state->dew_point_margin_k, state->has_sensor_data,
                                           MB_SCALE_TEMPERATURE);

    r->active_setpoint =
        mb_encode_signed(state->active_setpoint_c, true, MB_SCALE_TEMPERATURE);
    r->heating_setpoint =
        mb_encode_signed(state->heating_setpoint_c, true, MB_SCALE_TEMPERATURE);
    r->cooling_setpoint =
        mb_encode_signed(state->cooling_setpoint_c, true, MB_SCALE_TEMPERATURE);
    r->setpoint_shift = mb_encode_signed(state->setpoint_shift_k, true, MB_SCALE_TEMPERATURE);
    r->heating_percent = state->heating_percent;
    r->cooling_percent = state->cooling_percent;
    r->ventilation_percent = state->ventilation_percent;
    r->ventilation_stage = state->ventilation_level;
    r->hvac_mode_active = state->hvac_operating_mode;
    r->controller_mode_active = state->controller_mode;
    r->controller_status = state->controller_status;
    r->air_quality_status = state->air_quality_status;
    r->room_sensor_status = state->room_sensor_status;
    r->floor_probe_status = state->floor_probe_status;
    r->air_quality_sensor_status = state->air_quality_sensor_status;

    uint8_t *bits = slave->discrete_bits;
    mb_bit_set(bits, MB_DISCRETE_SERVICE_RUNNING, true);
    mb_bit_set(bits, MB_DISCRETE_DEVICE_FAULT, state->device_fault);
    mb_bit_set(bits, MB_DISCRETE_HEATING_REQUEST, state->heating_request);
    mb_bit_set(bits, MB_DISCRETE_COOLING_REQUEST, state->cooling_request);
    mb_bit_set(bits, MB_DISCRETE_HEAT_MODE, state->heat_cool_mode_heating);
    mb_bit_set(bits, MB_DISCRETE_ENABLE_HEAT, state->enable_heat);
    mb_bit_set(bits, MB_DISCRETE_ENABLE_COOL, state->enable_cool);
    mb_bit_set(bits, MB_DISCRETE_DEW_POINT_ALARM, state->dew_point_alarm);
    mb_bit_set(bits, MB_DISCRETE_FLOOR_MOISTURE_ALARM, state->floor_moisture_alarm);
    mb_bit_set(bits, MB_DISCRETE_FLOOR_LIMIT_ACTIVE, state->floor_limit_active);
    mb_bit_set(bits, MB_DISCRETE_FLOOR_COMFORT_ACTIVE, state->floor_comfort_active);
    mb_bit_set(bits, MB_DISCRETE_FREE_COOLING, state->free_cooling_available);
    mb_bit_set(bits, MB_DISCRETE_FREE_DRYING, state->free_drying_available);
    mb_bit_set(bits, MB_DISCRETE_VENTILATION_BOOST, state->ventilation_boost_request);
    mb_bit_set(bits, MB_DISCRETE_DEHUMIDIFY_REQUEST, state->dehumidify_request);
    mb_bit_set(bits, MB_DISCRETE_PROGRAMMING_MODE, state->programming_mode);
}

// Read back the writable registers from the device's own state, so a master
// that reads them sees what was actually adopted rather than what it asked for
// — a setpoint outside the ETS limits comes back clamped, and a change made
// over KNX shows up here too.
//
// A register is only refreshed while it still holds what this task last wrote.
// If it has changed, a master wrote it in the window between mb_apply_writes()
// reading the block and this refresh; overwriting it there would silently
// swallow that write. Leaving it alone lets the next cycle apply it.
#define MB_REFRESH(field, value)                            \
    do {                                                    \
        if (h->field == slave->applied_holding.field) {     \
            h->field = (value);                             \
        }                                                   \
    } while (0)

static void mb_refresh_writables(mb_rtu_slave_t *slave, const control_state_t *state)
{
    mb_holding_registers_t *h = &slave->holding_regs;

    MB_REFRESH(slave_address, slave->slave_address);
    MB_REFRESH(baud_rate_code, slave->baud_code);
    MB_REFRESH(config_commit, 0);
    MB_REFRESH(comfort_setpoint,
               mb_encode_signed(state->comfort_setpoint_c, true, MB_SCALE_TEMPERATURE));
    MB_REFRESH(setpoint_shift,
               mb_encode_signed(state->setpoint_shift_k, true, MB_SCALE_TEMPERATURE));
    MB_REFRESH(hvac_mode, state->hvac_operating_mode);
    MB_REFRESH(controller_mode, state->controller_mode);
    MB_REFRESH(ventilation_mode, state->ventilation_mode);
    MB_REFRESH(co2_setpoint, mb_encode_unsigned(state->co2_setpoint_ppm, true, 1));

    for (unsigned coil = 0; coil < MB_COIL_COUNT; ++coil) {
        if (mb_bit_get(slave->coil_bits, coil) != mb_bit_get(slave->applied_coils, coil)) {
            continue;  // written since the last apply; leave it for the next cycle
        }
        switch (coil) {
        case MB_COIL_CONTROLLER_ON:
            mb_bit_set(slave->coil_bits, coil, state->controller_on);
            break;
        case MB_COIL_WINDOW_STATUS:
            mb_bit_set(slave->coil_bits, coil, state->window_open);
            break;
        case MB_COIL_PRESENCE:
            mb_bit_set(slave->coil_bits, coil, state->presence);
            break;
        case MB_COIL_ALARM_ACKNOWLEDGE:
            mb_bit_set(slave->coil_bits, coil, false);  // momentary
            break;
        default:  // MB_COIL_IDENTIFY_LED is owned by the master
            break;
        }
    }

    slave->applied_holding = *h;
    memcpy(slave->applied_coils, slave->coil_bits, sizeof(slave->applied_coils));
}

#undef MB_REFRESH

// --- Applying master writes ------------------------------------------------

// True when the master has changed this register since we last wrote it.
#define MB_CHANGED(field) (current.field != slave->applied_holding.field)

static bool mb_apply_writes(mb_rtu_slave_t *slave)
{
    mb_holding_registers_t current;
    uint8_t coils[MB_COIL_BYTES];

    if (mbc_slave_lock(slave->mbc_slave_handle) != ESP_OK) {
        return false;
    }
    current = slave->holding_regs;
    memcpy(coils, slave->coil_bits, sizeof(coils));
    mbc_slave_unlock(slave->mbc_slave_handle);

    float value = 0.0f;
    if (MB_CHANGED(comfort_setpoint)
        && mb_decode_signed(current.comfort_setpoint, MB_SCALE_TEMPERATURE, &value)) {
        control_state_write(CONTROL_CMD_SETPOINT_BASE, value);
    }
    if (MB_CHANGED(setpoint_shift)
        && mb_decode_signed(current.setpoint_shift, MB_SCALE_TEMPERATURE, &value)) {
        control_state_write(CONTROL_CMD_SETPOINT_SHIFT, value);
    }
    if (MB_CHANGED(hvac_mode)) {
        control_state_write(CONTROL_CMD_HVAC_MODE, (float)current.hvac_mode);
    }
    if (MB_CHANGED(controller_mode)) {
        control_state_write(CONTROL_CMD_CONTROLLER_MODE, (float)current.controller_mode);
    }
    if (MB_CHANGED(ventilation_mode)) {
        control_state_write(CONTROL_CMD_VENTILATION_MODE, (float)current.ventilation_mode);
    }
    if (MB_CHANGED(co2_setpoint) && current.co2_setpoint != MB_INVALID_UNSIGNED) {
        control_state_write(CONTROL_CMD_CO2_SETPOINT, (float)current.co2_setpoint);
    }

    for (unsigned coil = 0; coil < MB_COIL_COUNT; ++coil) {
        const bool now = mb_bit_get(coils, coil);
        if (now == mb_bit_get(slave->applied_coils, coil)) {
            continue;
        }
        switch (coil) {
        case MB_COIL_CONTROLLER_ON:
            control_state_write(CONTROL_CMD_CONTROLLER_ON_OFF, now ? 1.0f : 0.0f);
            break;
        case MB_COIL_WINDOW_STATUS:
            control_state_write(CONTROL_CMD_WINDOW_STATUS, now ? 1.0f : 0.0f);
            break;
        case MB_COIL_PRESENCE:
            control_state_write(CONTROL_CMD_PRESENCE, now ? 1.0f : 0.0f);
            break;
        case MB_COIL_ALARM_ACKNOWLEDGE:
            if (now) {
                control_state_write(CONTROL_CMD_ACKNOWLEDGE_ALARMS, 1.0f);
            }
            break;
        default:  // MB_COIL_IDENTIFY_LED is read back by main, not a command
            break;
        }
    }

    // Line settings only change on an explicit commit: rewriting the address of
    // the device you are mid-conversation with would drop the response to the
    // very write that requested it.
    bool reconfigure = false;
    if (current.config_commit != 0) {
        const bool address_ok = current.slave_address >= 1 && current.slave_address <= 247;
        const bool baud_ok = current.baud_rate_code < MB_BAUD_CODE_COUNT;
        if (address_ok && baud_ok) {
            slave->slave_address = (uint8_t)current.slave_address;
            slave->baud_code = current.baud_rate_code;
            const esp_err_t err = mb_store_line_settings(slave);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to persist Modbus settings: %s", esp_err_to_name(err));
            }
            reconfigure = true;
            ESP_LOGW(TAG, "Modbus line settings committed: address %u at %" PRIu32 " baud",
                     slave->slave_address, mb_baud_from_code(slave->baud_code));
        } else {
            ESP_LOGW(TAG, "Rejected invalid Modbus settings: address %u, baud code %u",
                     current.slave_address, current.baud_rate_code);
        }
    }
    return reconfigure;
}

#undef MB_CHANGED

// --- Task ------------------------------------------------------------------

static void mb_service_task(void *param)
{
    mb_rtu_slave_t *slave = (mb_rtu_slave_t *)param;
    control_state_t state;

    while (!slave->stop_requested) {
        const bool reconfigure = mb_apply_writes(slave);

        control_state_get(&state);
        if (mbc_slave_lock(slave->mbc_slave_handle) == ESP_OK) {
            mb_fill_identity(slave);
            mb_fill_control(slave, &state);
            mb_refresh_writables(slave, &state);
            mbc_slave_unlock(slave->mbc_slave_handle);
        }

        if (reconfigure) {
            // Give the stack time to finish the response to the commit before
            // the line settings change underneath it.
            vTaskDelay(pdMS_TO_TICKS(100));
            mb_stack_stop(slave);
            if (mb_stack_start(slave) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restart with the new settings; slave is offline");
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MB_SERVICE_PERIOD_MS));
    }

    ESP_LOGI(TAG, "Modbus service task exiting");
    slave->task_handle = NULL;
    vTaskDelete(NULL);
}

// --- Public API ------------------------------------------------------------

esp_err_t mb_rtu_slave_start(mb_rtu_slave_t *slave)
{
    if (slave == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (slave->mbc_slave_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(slave, 0, sizeof(*slave));
    mb_load_line_settings(slave);

    // Until the first sampling cycle every measurement is genuinely unknown,
    // and the map has a way to say that. Zero would have claimed 0.0 °C.
    memset(&slave->input_regs, 0xFF, sizeof(slave->input_regs));

    esp_err_t err = mb_stack_start(slave);
    if (err != ESP_OK) {
        return err;
    }

    slave->stop_requested = false;
    if (xTaskCreate(mb_service_task, "ModbusService", 3072, slave, tskIDLE_PRIORITY + 1,
                    &slave->task_handle)
        != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Modbus service task");
        mb_stack_stop(slave);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mb_rtu_slave_stop(mb_rtu_slave_t *slave)
{
    if (slave == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    slave->stop_requested = true;
    for (int waited_ms = 0; slave->task_handle != NULL && waited_ms < 1000; waited_ms += 50) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    mb_stack_stop(slave);
    return ESP_OK;
}

esp_err_t mb_rtu_slave_publish(mb_rtu_slave_t *slave, const sensor_data_t *data)
{
    if (slave == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (slave->mbc_slave_handle == NULL) {
        return ESP_ERR_INVALID_STATE;  // Modbus is optional; sampling continues
    }

    const esp_err_t err = mbc_slave_lock(slave->mbc_slave_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to lock Modbus controller: %s", esp_err_to_name(err));
        return err;
    }
    mb_fill_measurements(slave, data);
    return mbc_slave_unlock(slave->mbc_slave_handle);
}

bool mb_rtu_slave_led_requested(const mb_rtu_slave_t *slave)
{
    if (slave == NULL) {
        return false;
    }
    return mb_bit_get(slave->coil_bits, MB_COIL_IDENTIFY_LED);
}
