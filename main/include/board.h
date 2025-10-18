#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

// GPIO pin number
#define PIN_PROBE_EN GPIO_NUM_0
#define PIN_2V8_EN GPIO_NUM_1
#define PIN_KNX_OK GPIO_NUM_3
#define PIN_KNX_TX GPIO_NUM_4
#define PIN_KNX_RX GPIO_NUM_5
#define PIN_SDA GPIO_NUM_6
#define PIN_SCL GPIO_NUM_7
#define PIN_PROG_BTN GPIO_NUM_9
#define PIN_USB_DP GPIO_NUM_12
#define PIN_USB_DM GPIO_NUM_13
#define PIN_RS485_TX GPIO_NUM_14
#define PIN_RS485_RX GPIO_NUM_15
#define PIN_PROBE_SCL GPIO_NUM_18
#define PIN_PROBE_SDA GPIO_NUM_19
#define PIN_LED GPIO_NUM_20
#define PIN_RS485_TEN GPIO_NUM_21
#define PIN_UNUSED_1 GPIO_NUM_22
#define PIN_UNUSED_2 GPIO_NUM_23

#define MODBUS_RTU_UART_NUM UART_NUM_1
#define MODBUS_RTU_UART_BAUDRATE 9600

// Time to wait for the probe to stabilize
#define PROBE_STABILIZE_TIME 250

esp_err_t init_pins(void);
esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus_handle, uint8_t sda_io, uint8_t scl_io, bool lp_i2c);
