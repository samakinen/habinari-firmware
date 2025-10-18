#pragma once

#include "knx_tp_bit_bang.h"

/**
 * @brief Create and initialize the KNX TP bit-bang interface and task with individual pins
 * 
 * This function automatically creates a new KNX TP bit-bang interface and initializes it,
 * eliminating the need for the caller to manually create the knx_tp_bit_bang_t structure.
 * It creates the required queues, initializes the GPIO pins, and starts the processing task.
 * 
 * @param tx_pin GPIO pin number for KNX transmission
 * @param rx_pin GPIO pin number for KNX reception
 * @param prog_btn_pin GPIO pin number for programming button
 * @param led_pin GPIO pin number for LED indication
 * @return knx_tp_bit_bang_handle_t Handle to the created KNX TP bit-bang interface, or NULL on failure
 */
knx_tp_bit_bang_handle_t knx_tp_bit_bang_task_create(gpio_num_t tx_pin, gpio_num_t rx_pin,
                                                gpio_num_t prog_btn_pin, gpio_num_t led_pin);

/**
 * @brief Create and initialize the KNX TP bit-bang interface and task with pin configuration structure
 * 
 * This function automatically creates a new KNX TP bit-bang interface and initializes it,
 * eliminating the need for the caller to manually create the knx_tp_bit_bang_t structure.
 * It creates the required queues, initializes the GPIO pins, and starts the processing task.
 * 
 * @param pin_config Pin configuration structure
 * @return knx_tp_bit_bang_handle_t Handle to the created KNX TP bit-bang interface, or NULL on failure
 */
knx_tp_bit_bang_handle_t knx_tp_bit_bang_task_create_with_config(const knx_tp_pin_config_t *pin_config);
