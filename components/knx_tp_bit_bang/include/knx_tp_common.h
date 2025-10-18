#pragma once

#include <stdint.h>
#include "driver/gpio.h"

// Forward declaration of event and priority types
// The actual enums are defined in knx_tp_bit_bang_events.h
struct knx_event_type;
struct knx_priority_type;

// Status types
typedef uint8_t knx_tp_bit_bang_rx_status_t;
typedef uint8_t knx_tp_bit_bang_tx_status_t;

// Set pin configuration
typedef struct {
    gpio_num_t tx_pin;          // KNX TP TX pin
    gpio_num_t rx_pin;          // KNX TP RX pin
    gpio_num_t prog_btn_pin;    // Program button pin (optional)
    gpio_num_t led_pin;         // Status LED pin (optional)
} knx_tp_pin_config_t;
