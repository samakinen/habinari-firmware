#pragma once

//#include "knx_tp_common.h"
#include "knx_tp_bit_bang.h"
#include "esp_err.h"
#include "driver/uart.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief TP-UART / NCN5121 emulation configuration
 */
typedef struct {
    uart_port_t uart_port;               /*!< UART port to use for host communication */
    uint32_t uart_baud_rate;             /*!< UART baud rate (typically 19200) */
    gpio_num_t uart_tx_pin;              /*!< UART TX pin */
    gpio_num_t uart_rx_pin;              /*!< UART RX pin */
    uint16_t device_address;             /*!< KNX device address */
    size_t rx_buffer_size;               /*!< Size of UART RX buffer */
    size_t tx_buffer_size;               /*!< Size of UART TX buffer */
    knx_tp_pin_config_t knx_pins;        /*!< KNX TP pin configuration - pins used for KNX bus communication */
} knx_tp_uart_config_t;

/**
 * @brief TP-UART service request command codes
 */
typedef enum {
    KNX_TPUART_RESET_REQ = 0x01,         /*!< Reset request */
    KNX_TPUART_STATE_REQ = 0x02,         /*!< State request */
    KNX_TPUART_SET_ADDR_REQ = 0x28,      /*!< Set address request */
    KNX_TPUART_DATA_START_CONTINUE = 0x80,/*!< Data start/continue (bit 7 set) */
    KNX_TPUART_DATA_END_REQ = 0x40,      /*!< Data end request (bit 6 set) */
    KNX_TPUART_RESET_INDICATION = 0x03,  /*!< Reset indication */
    KNX_TPUART_STATE_INDICATION = 0x07,  /*!< State indication */
    KNX_TPUART_CONFIGURE = 0x0C,         /*!< Configure */
} knx_tp_uart_command_t;

/**
 * @brief TP-UART emulation context 
 * This structure is internal to the implementation
 */
typedef struct knx_tp_uart_context_s knx_tp_uart_context_t;

/**
 * @brief TP-UART emulation handle
 */
typedef knx_tp_uart_context_t* knx_tp_uart_handle_t;

/**
 * @brief Initialize the TP-UART emulation
 * 
 * @param config Configuration parameters
 * @param[out] handle_out Pointer to store the created handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t knx_tp_uart_init(const knx_tp_uart_config_t* config, knx_tp_uart_handle_t* handle_out);

/**
 * @brief Deinitialize the TP-UART emulation
 * 
 * @param handle TP-UART emulation handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t knx_tp_uart_deinit(knx_tp_uart_handle_t handle);

/**
 * @brief Send a byte to the TP-UART emulation
 * This function is meant to be used by the host application
 * to send commands and data to the TP-UART interface
 * 
 * @param handle TP-UART emulation handle
 * @param byte Byte to send
 * @return esp_err_t ESP_OK on success
 */
esp_err_t knx_tp_uart_send_byte(knx_tp_uart_handle_t handle, uint8_t byte);

/**
 * @brief Send multiple bytes to the TP-UART emulation
 * 
 * @param handle TP-UART emulation handle
 * @param data Data to send
 * @param len Length of data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t knx_tp_uart_send_data(knx_tp_uart_handle_t handle, const uint8_t* data, size_t len);

/**
 * @brief Set a callback function to be called when the TP-UART receives data
 * 
 * @param handle TP-UART emulation handle
 * @param callback Function to call on data received
 * @param user_data User data to pass to the callback
 * @return esp_err_t ESP_OK on success
 */
esp_err_t knx_tp_uart_set_rx_callback(knx_tp_uart_handle_t handle, 
                                    void (*callback)(uint8_t byte, void* user_data),
                                    void* user_data);

/**
 * @brief Set callback for received KNX telegrams
 * 
 * @param handle TP-UART emulation handle
 * @param callback Function to call when a telegram is received
 * @param user_data User data to pass to the callback
 * @return esp_err_t ESP_OK on success
 */
esp_err_t knx_tp_uart_set_telegram_received_cb(knx_tp_uart_handle_t handle, 
                                           void (*callback)(uint8_t *data, uint8_t length, void* user_data),
                                           void* user_data);

/**
 * @brief Start the TP-UART emulation
 * This initializes the UART and starts the task that handles UART communication
 * 
 * @param handle TP-UART emulation handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t knx_tp_uart_start(knx_tp_uart_handle_t handle);

/**
 * @brief Stop the TP-UART emulation
 * 
 * @param handle TP-UART emulation handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t knx_tp_uart_stop(knx_tp_uart_handle_t handle);
