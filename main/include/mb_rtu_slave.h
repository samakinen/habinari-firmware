#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus_registers.h"
#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file mb_rtu_slave.h
 * @brief Modbus RTU slave: the second protocol adapter onto the same device
 *        model the KNX side exposes.
 *
 * The slave owns a task that keeps its register images in step with
 * control_state.h — publishing the device's state into the read areas and
 * turning master writes into control_state_write() commands. It therefore
 * carries the whole room-controller model, not just measurements, and a Modbus
 * master can drive the device exactly as far as a KNX installation can.
 *
 * Line settings (address and baud rate) are held in NVS and are themselves
 * writable over Modbus, so a device can be re-addressed in the field without
 * reflashing. See docs/modbus-register-map.md for the wire-level map.
 */

typedef struct {
    void *mbc_slave_handle;
    TaskHandle_t task_handle;
    volatile bool stop_requested;

    // Register images the Modbus stack serves directly. The layouts are in
    // modbus_registers.h, which is also what the documentation is generated
    // from, so wire and docs cannot drift.
    mb_input_registers_t input_regs;
    mb_holding_registers_t holding_regs;
    uint8_t discrete_bits[MB_DISCRETE_BYTES];
    uint8_t coil_bits[MB_COIL_BYTES];

    // Last values this task itself wrote, so a master's write is told apart
    // from the device's own read-back.
    mb_holding_registers_t applied_holding;
    uint8_t applied_coils[MB_COIL_BYTES];

    // Commissioning. `slave_address` is what the device has been *given*
    // (MB_SLAVE_ADDR_UNASSIGNED until somebody does); `listen_address` is what
    // the stack is currently answering to, which differs while an unassigned
    // device is selected or muted. See the addressing note in
    // modbus_registers.h.
    uint8_t slave_address;
    uint8_t listen_address;
    uint16_t baud_code;

    uint8_t serial[6];              ///< base MAC: the identity on the box label
    int64_t serial_select_expiry_us; ///< 0 when no serial selection is armed
} mb_rtu_slave_t;

typedef mb_rtu_slave_t *mb_rtu_slave_handle_t;

/// Load the persisted line settings, bring the stack up and start the task that
/// keeps the registers in step with control_state.
esp_err_t mb_rtu_slave_start(mb_rtu_slave_t *slave);

/// Stop the task and release the Modbus controller.
esp_err_t mb_rtu_slave_stop(mb_rtu_slave_t *slave);

/// Publish a fresh measurement record. Called from the sensor task; the rest of
/// the register image is refreshed by the slave's own task from control_state.
esp_err_t mb_rtu_slave_publish(mb_rtu_slave_t *slave, const sensor_data_t *data);

/// Whether a master has asked for the identify LED.
bool mb_rtu_slave_led_requested(const mb_rtu_slave_t *slave);

#ifdef __cplusplus
}
#endif
