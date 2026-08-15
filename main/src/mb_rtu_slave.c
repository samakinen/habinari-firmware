#include "mb_rtu_slave.h"

#include <inttypes.h>
#include <string.h>

#include "board.h"
#include "control_state.h"
#include "device_config.h"
#include "protocol_adapter.h"
#include "esp_log.h"
#include "esp_mac.h"
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

#define MB_DEFAULT_SLAVE_ADDRESS MB_SLAVE_ADDR_UNASSIGNED
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
        ESP_LOGI(TAG, "No stored Modbus settings; this device has no address yet");
        return;
    }

    uint8_t address = 0;
    if (nvs_get_u8(handle, MB_NVS_KEY_ADDRESS, &address) == ESP_OK
        && mb_address_assignable(address)) {
        slave->slave_address = address;
    }
    uint16_t baud = 0;
    if (nvs_get_u16(handle, MB_NVS_KEY_BAUD, &baud) == ESP_OK && baud < MB_BAUD_CODE_COUNT) {
        slave->baud_code = baud;
    }
    nvs_close(handle);
    if (mb_address_assignable(slave->slave_address)) {
        ESP_LOGI(TAG, "Modbus settings: address %u at %" PRIu32 " baud", slave->slave_address,
                 mb_baud_from_code(slave->baud_code));
    } else {
        ESP_LOGW(TAG,
                 "Modbus address not assigned: this device stays silent until the programming "
                 "button selects it, or a master writes its serial to the select registers");
    }
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

// --- Out-of-band configuration ---------------------------------------------
//
// The circular one. Address and baud rate are writable over Modbus, which works
// exactly until it does not: two boards out of the box are both address 1 on
// the same pair, and a master cannot address either of them to move one. The
// service channel (oob_service.h) breaks that loop; this is the Modbus side's
// half of the contract, and it does not know or care what renders it.
//
// Both take effect at the next boot rather than immediately. Re-opening a UART
// underneath a live master mid-transaction is a way to lose the frame that told
// you to do it, and the installer is standing at the device anyway.

static uint8_t mb_stored_address(void)
{
    nvs_handle_t handle = 0;
    if (nvs_open(MB_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return MB_DEFAULT_SLAVE_ADDRESS;
    }
    uint8_t address = MB_DEFAULT_SLAVE_ADDRESS;
    if (nvs_get_u8(handle, MB_NVS_KEY_ADDRESS, &address) != ESP_OK) {
        address = MB_DEFAULT_SLAVE_ADDRESS;
    }
    nvs_close(handle);
    return address;
}

static uint16_t mb_stored_baud_code(void)
{
    nvs_handle_t handle = 0;
    if (nvs_open(MB_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return MB_DEFAULT_BAUD_CODE;
    }
    uint16_t code = MB_DEFAULT_BAUD_CODE;
    if (nvs_get_u16(handle, MB_NVS_KEY_BAUD, &code) != ESP_OK || code >= MB_BAUD_CODE_COUNT) {
        code = MB_DEFAULT_BAUD_CODE;
    }
    nvs_close(handle);
    return code;
}

static esp_err_t mb_cfg_get_address(const device_config_item_t *item, device_config_value_t *out)
{
    (void)item;
    out->num.u = mb_stored_address();
    return ESP_OK;
}

static esp_err_t mb_cfg_set_address(const device_config_item_t *item,
                                    const device_config_value_t *value)
{
    (void)item;
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(MB_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(handle, MB_NVS_KEY_ADDRESS, (uint8_t)value->num.u);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static esp_err_t mb_cfg_get_baud(const device_config_item_t *item, device_config_value_t *out)
{
    (void)item;
    out->num.u = mb_stored_baud_code();
    return ESP_OK;
}

static esp_err_t mb_cfg_set_baud(const device_config_item_t *item,
                                 const device_config_value_t *value)
{
    (void)item;
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(MB_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u16(handle, MB_NVS_KEY_BAUD, (uint16_t)value->num.u);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

// Indexed by mb_baud_code_t, so the registry's enum bound and the UART's are
// the same list. Adding a rate means adding it in both places or neither.
static const char *const mb_baud_labels[] = {"9600", "19200", "38400", "57600", "115200"};
_Static_assert(sizeof(mb_baud_labels) / sizeof(mb_baud_labels[0]) == MB_BAUD_CODE_COUNT,
               "baud labels must match mb_baud_code_t");

static const device_config_item_t mb_config_items[] = {
    {
        .key = "mb.addr",
        // 0 is offered deliberately: it is how a board being pulled out of one
        // installation and into another is put back into the silent, unaddressed
        // state it left the factory in.
        .label = "Modbus slave address (0 = unassigned, silent)",
        .type = DEVICE_CONFIG_TYPE_UINT,
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .min = 0.0f,
        .max = (float)MB_SLAVE_ADDR_MAX,
        .get = mb_cfg_get_address,
        .set = mb_cfg_set_address,
    },
    {
        .key = "mb.baud",
        .label = "Modbus baud rate",
        .unit = "bit/s",
        .type = DEVICE_CONFIG_TYPE_ENUM,
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .enum_labels = mb_baud_labels,
        .enum_count = MB_BAUD_CODE_COUNT,
        .get = mb_cfg_get_baud,
        .set = mb_cfg_set_baud,
    },
};

/// True while a serial-number selection is armed and unexpired.
static bool mb_serial_selection_active(mb_rtu_slave_t *slave)
{
    if (slave->serial_select_expiry_us == 0) {
        return false;
    }
    if (esp_timer_get_time() >= slave->serial_select_expiry_us) {
        slave->serial_select_expiry_us = 0;
        ESP_LOGI(TAG, "Serial selection expired");
        return false;
    }
    return true;
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
        .ser_opts.uid = slave->listen_address,
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
        mbc_set_slave_id(slave->mbc_slave_handle, slave->listen_address, true,
                         (uint8_t *)MB_DEVICE_NAME, strlen(MB_DEVICE_NAME));
    if (id_err != ESP_OK) {
        ESP_LOGW(TAG, "Report Slave ID unavailable: %s", esp_err_to_name(id_err));
    }

    if (slave->listen_address == MB_SLAVE_ADDR_UNASSIGNED) {
        // Not an error and not idle: the stack is running and receiving. It just
        // cannot answer, because the only address it matches is the broadcast
        // address and the spec forbids replying to that. This is how a shelf of
        // unaddressed boards shares one pair in silence and still hears the
        // broadcast that names one of them.
        ESP_LOGI(TAG,
                 "Modbus RTU slave listening for broadcasts only at %" PRIu32
                 " baud (no address assigned)",
                 mb_baud_from_code(slave->baud_code));
    } else {
        ESP_LOGI(TAG, "Modbus RTU slave listening as address %u at %" PRIu32 " baud%s",
                 slave->listen_address, mb_baud_from_code(slave->baud_code),
                 slave->listen_address == MB_SLAVE_ADDR_COMMISSIONING
                     ? " (commissioning address)"
                     : "");
    }
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

    uint16_t serial[3];
    mb_serial_encode(slave->serial, serial);
    slave->input_regs.serial_0 = serial[0];
    slave->input_regs.serial_1 = serial[1];
    slave->input_regs.serial_2 = serial[2];
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

static bool mb_apply_writes(mb_rtu_slave_t *slave, bool programming_mode)
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

    // A serial-number write selects this device, and only this device. It is
    // checked before the commit below so that one broadcast frame carrying both
    // — select and commit — is honoured in the order the master wrote it.
    const uint16_t selection[3] = {
        current.serial_select_0,
        current.serial_select_1,
        current.serial_select_2,
    };
    //
    // Detected as a change rather than as a write, because a memory-mapped
    // register area has no write callback and rewriting the same value looks
    // like nothing happening. The consequence is worth knowing: to re-arm an
    // expired selection, write zeros and then the serial again.
    if (memcmp(selection, &slave->applied_holding.serial_select_0, sizeof(selection)) != 0) {
        if (mb_serial_selected(selection, slave->serial)) {
            slave->serial_select_expiry_us =
                esp_timer_get_time() + (int64_t)MB_SERIAL_SELECT_TIMEOUT_S * 1000000;
            ESP_LOGW(TAG, "Selected by serial number for %d s", MB_SERIAL_SELECT_TIMEOUT_S);
        } else if (slave->serial_select_expiry_us != 0) {
            // Somebody else's serial, or an explicit clear. Either way this
            // device is no longer the one being addressed.
            slave->serial_select_expiry_us = 0;
            ESP_LOGI(TAG, "Serial selection released");
        }
    }

    const mb_commissioning_inputs_t commissioning = {
        .assigned_address = slave->slave_address,
        .programming_mode = programming_mode,
        .serial_selected = mb_serial_selection_active(slave),
    };
    const mb_commissioning_state_t allowed = mb_commissioning_resolve(commissioning);

    // Line settings only change on an explicit commit: rewriting the address of
    // the device you are mid-conversation with would drop the response to the
    // very write that requested it.
    bool reconfigure = false;
    if (current.config_commit != 0) {
        if (!allowed.accept_line_settings) {
            // Reachable is not the same as selected. Without this a stray or
            // mistargeted master write could re-address a device in a running
            // building, and the first anyone would know is a BMS point going
            // dark.
            ESP_LOGW(TAG,
                     "Refused a line-settings commit: this device is not selected. Press the "
                     "programming button, or write its serial to registers 12..14 first.");
        } else {
            const bool address_ok = mb_address_assignable(current.slave_address)
                                    || current.slave_address == MB_SLAVE_ADDR_UNASSIGNED;
            const bool baud_ok = current.baud_rate_code < MB_BAUD_CODE_COUNT;
            if (address_ok && baud_ok) {
                slave->slave_address = (uint8_t)current.slave_address;
                slave->baud_code = current.baud_rate_code;
                const esp_err_t err = mb_store_line_settings(slave);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to persist Modbus settings: %s", esp_err_to_name(err));
                }
                reconfigure = true;
                // The selection is spent: a commissioning script that crashes
                // after this must not leave the device writable.
                slave->serial_select_expiry_us = 0;
                if (slave->slave_address == MB_SLAVE_ADDR_UNASSIGNED) {
                    ESP_LOGW(TAG, "Modbus address cleared; this device is silent again");
                } else {
                    ESP_LOGW(TAG, "Modbus line settings committed: address %u at %" PRIu32 " baud",
                             slave->slave_address, mb_baud_from_code(slave->baud_code));
                }
            } else {
                ESP_LOGW(TAG,
                         "Rejected invalid Modbus settings: address %u (allowed %u..%u, or %u to "
                         "unassign), baud code %u",
                         current.slave_address, MB_SLAVE_ADDR_MIN, MB_SLAVE_ADDR_MAX,
                         MB_SLAVE_ADDR_UNASSIGNED, current.baud_rate_code);
            }
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
        control_state_get(&state);

        const bool reconfigure = mb_apply_writes(slave, state.programming_mode);

        if (mbc_slave_lock(slave->mbc_slave_handle) == ESP_OK) {
            mb_fill_identity(slave);
            mb_fill_control(slave, &state);
            mb_refresh_writables(slave, &state);
            mbc_slave_unlock(slave->mbc_slave_handle);
        }

        // What this device should be answering to right now. It changes when a
        // commit lands, and also when programming mode does — an unaddressed
        // board comes up on the commissioning address while somebody is standing
        // at it and goes mute again when they leave.
        const mb_commissioning_state_t want = mb_commissioning_resolve((mb_commissioning_inputs_t){
            .assigned_address = slave->slave_address,
            .programming_mode = state.programming_mode,
            .serial_selected = mb_serial_selection_active(slave),
        });

        if (reconfigure || want.listen_address != slave->listen_address) {
            // Give the stack time to finish the response to the commit before
            // the line settings change underneath it.
            vTaskDelay(pdMS_TO_TICKS(100));
            mb_stack_stop(slave);
            slave->listen_address = want.listen_address;
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

    // The identity a master selects this device by, and the same six bytes the
    // KNX device object and the BLE advertisement use. Without it the device can
    // still be commissioned from the button, just not remotely.
    if (esp_read_mac(slave->serial, ESP_MAC_BASE) != ESP_OK) {
        ESP_LOGW(TAG, "esp_read_mac failed; serial-number selection will not work");
    }

    // Start where the commissioning state says, not where the stored address
    // says. A fresh device comes up mute; the task moves it when programming
    // mode changes.
    control_state_t initial;
    control_state_get(&initial);
    slave->listen_address =
        mb_commissioning_resolve((mb_commissioning_inputs_t){
                                     .assigned_address = slave->slave_address,
                                     .programming_mode = initial.programming_mode,
                                     .serial_selected = false,
                                 })
            .listen_address;

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

// ---------------------------------------------------------------------------
// Protocol adapter descriptor.
//
// The instance-based API above is kept — it is the testable shape, and it is
// what docs/modbus-register-map.md describes — and this is the thin binding
// that lets the registry start it alongside whatever else is compiled in.
// ---------------------------------------------------------------------------

static mb_rtu_slave_t s_slave;

static esp_err_t mb_adapter_start(void)
{
    // Registered even if the stack below fails to come up. A board whose RS-485
    // driver did not start is exactly the board somebody needs to reconfigure.
    const esp_err_t cfg =
        device_config_register(mb_config_items, sizeof(mb_config_items) / sizeof(*mb_config_items));
    if (cfg != ESP_OK) {
        ESP_LOGW(TAG, "Modbus line settings not exposed out of band: %s", esp_err_to_name(cfg));
    }
    return mb_rtu_slave_start(&s_slave);
}

static void mb_adapter_on_sensor_data(const sensor_data_t *data)
{
    // Measurements go straight into the input-register image so a master
    // polling faster than the control tick still sees fresh readings. The rest
    // of the map is refreshed by the slave's own task from control_state.
    (void)mb_rtu_slave_publish(&s_slave, data);
}

static bool mb_adapter_identify_active(void)
{
    return mb_rtu_slave_led_requested(&s_slave);
}

const protocol_adapter_t modbus_protocol_adapter = {
    .name = "modbus-rtu",
    .start = mb_adapter_start,
    // No tick hook: the slave's own task refreshes the register image on its
    // own cadence, and a master reads whenever it likes. There is nothing to
    // push.
    .on_control_tick = NULL,
    .on_sensor_data = mb_adapter_on_sensor_data,
    .identify_active = mb_adapter_identify_active,
    // Optional: a board with nothing wired to the RS-485 terminal is a
    // correctly installed board, not a broken one.
    .required = false,
};
