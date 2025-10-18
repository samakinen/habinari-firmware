// #include "knx_tp_uart_emulation.h"
// #include "knx_app_interface.h"  // Replace direct bit_bang with app interface
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "sdkconfig.h"  // Required for ESP_LOG macros
// #include "esp_log.h"
// #include "driver/uart.h"
// #include <string.h>

// #include "freertos/semphr.h"

// static const char *TAG = "KNX_TPUART_EMUL";

// // TP-UART control bytes
// #define TPUART_RESET_IND        0x03
// #define TPUART_STATE_IND        0x07
// #define TPUART_DATA_CONFIRM     0x8B
// #define TPUART_DATA_RECEIVED    0x89
// #define TPUART_RECEIVE_ERROR    0x0A
// #define TPUART_ACK              0x0C
// #define TPUART_NACK             0x0D
// #define TPUART_BUSY             0x0E

// // Maximum buffer sizes
// #define TPUART_MAX_TELEGRAM_LENGTH 64
// #define TPUART_RX_BUFFER_SIZE 256
// #define TPUART_TX_BUFFER_SIZE 256

// /**
//  * @brief TP-UART states
//  */
// typedef enum {
//     KNX_TPUART_STATE_IDLE,
//     KNX_TPUART_STATE_RECEIVING_DATA,
//     KNX_TPUART_STATE_TRANSMITTING_DATA,
//     KNX_TPUART_STATE_WAITING_FOR_ACK,
//     KNX_TPUART_STATE_ERROR
// } knx_tp_uart_state_t;

// /**
//  * @brief TP-UART emulation context 
//  */
// /**
//  * @brief TP-UART emulation context 
//  */
// struct knx_tp_uart_context_s {
//     uart_port_t uart_port;
//     uint32_t uart_baud_rate;
//     uint16_t device_address;
//     knx_tp_uart_state_t state;
//     TaskHandle_t task_handle;
//     QueueHandle_t uart_rx_queue;
//     QueueHandle_t uart_tx_queue;
//     SemaphoreHandle_t mutex;          // Mutex for thread safety
//     uint8_t tx_buffer[TPUART_MAX_TELEGRAM_LENGTH];
//     uint8_t rx_buffer[TPUART_MAX_TELEGRAM_LENGTH];
//     uint8_t tx_length;
//     uint8_t rx_length;
//     TickType_t ack_wait_start;        // Track when ACK timeout starts
//     bool is_running;                  // Flag indicating if the task is running
    
//     // Store the KNX pin configuration
//     knx_tp_pin_config_t knx_pins;
    
//     void (*rx_callback)(uint8_t byte, void* user_data);
//     void* rx_callback_user_data;
//     void (*telegram_received_cb)(uint8_t* data, uint8_t length, void* user_data);
//     void* telegram_received_user_data;
// };

// // Forward declaration of the telegram received callback
// static void knx_uart_telegram_received_callback(uint8_t* data, uint8_t length, void* user_data);

// /**
//  * @brief Process a byte from the UART (host to TP-UART)
//  * 
//  * @param handle TP-UART context
//  * @param byte Byte to process
//  */
// static void process_uart_byte(knx_tp_uart_handle_t handle, uint8_t byte)
// {
//     if (handle == NULL) {
//         ESP_LOGE(TAG, "NULL handle in process_uart_byte");
//         return;
//     }

//     // Take mutex for thread-safe access
//     if (handle->mutex != NULL && xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
//         ESP_LOGE(TAG, "Failed to take mutex in process_uart_byte");
//         return;
//     }
    
//     // Check for control commands
//     if (byte == KNX_TPUART_RESET_REQ) {
//         // Reset the TP-UART
//         ESP_LOGI(TAG, "TP-UART reset requested");
        
//         // Reset the state
//         handle->state = KNX_TPUART_STATE_IDLE;
//         handle->tx_length = 0;
//         handle->rx_length = 0;
        
//         // Send reset indication to host
//         uint8_t reset_ind = TPUART_RESET_IND;
//         int written = uart_write_bytes(handle->uart_port, &reset_ind, 1);
//         if (written != 1) {
//             ESP_LOGE(TAG, "Failed to send reset indication");
//         }
        
//         if (handle->mutex != NULL) {
//             xSemaphoreGive(handle->mutex);
//         }
//         return;
//     }
    
//     if (byte == KNX_TPUART_STATE_REQ) {
//         // State request
//         ESP_LOGI(TAG, "TP-UART state requested");
        
//         // Send state indication to host
//         uint8_t state_ind = TPUART_STATE_IND;
//         int written = uart_write_bytes(handle->uart_port, &state_ind, 1);
//         if (written != 1) {
//             ESP_LOGE(TAG, "Failed to send state indication");
//         }
        
//         if (handle->mutex != NULL) {
//             xSemaphoreGive(handle->mutex);
//         }
//         return;
//     }
    
//     // Check for data start/continue
//     if ((byte & 0x80) == 0x80) {
//         // Data byte with bit 7 set (start/continue)
//         if (handle->state == KNX_TPUART_STATE_IDLE) {
//             // Start of new telegram
//             ESP_LOGI(TAG, "Starting new telegram");
//             handle->state = KNX_TPUART_STATE_RECEIVING_DATA;
//             handle->tx_length = 0;
//         } else if (handle->state != KNX_TPUART_STATE_RECEIVING_DATA) {
//             // Invalid state for receiving data
//             ESP_LOGE(TAG, "Received data byte in invalid state: %d", handle->state);
//             handle->state = KNX_TPUART_STATE_ERROR;
            
//             // Send error indication
//             uint8_t error = TPUART_RECEIVE_ERROR;
//             int written = uart_write_bytes(handle->uart_port, &error, 1);
//             if (written != 1) {
//                 ESP_LOGE(TAG, "Failed to send error indication");
//             }
//             return;
//         }
        
//         // Extract data byte (remove control bit)
//         uint8_t data_byte = byte & 0x7F;
        
//         // Add to buffer if there's room
//         if (handle->tx_length < TPUART_MAX_TELEGRAM_LENGTH) {
//             handle->tx_buffer[handle->tx_length++] = data_byte;
            
//             // Send data confirm to host
//             uint8_t confirm = TPUART_DATA_CONFIRM;
//             int written = uart_write_bytes(handle->uart_port, &confirm, 1);
//             if (written != 1) {
//                 ESP_LOGE(TAG, "Failed to send data confirmation");
//                 // Don't change state, try to continue
//             }
//         } else {
//             // Buffer overflow
//             ESP_LOGE(TAG, "TX buffer overflow");
//             handle->state = KNX_TPUART_STATE_ERROR;
            
//             // Send error indication
//             uint8_t error = TPUART_RECEIVE_ERROR;
//             int written = uart_write_bytes(handle->uart_port, &error, 1);
//             if (written != 1) {
//                 ESP_LOGE(TAG, "Failed to send error indication");
//             }
//             // Reset buffer state to prevent further issues
//             handle->tx_length = 0;
//         }
//         return;
//     }
    
//     // Check for data end
//     if ((byte & 0x40) == 0x40) {
//         // Data byte with bit 6 set (end of telegram)
//         if (handle->state == KNX_TPUART_STATE_RECEIVING_DATA) {
//             // Extract data byte (remove control bit)
//             uint8_t data_byte = byte & 0x3F; // Remove bits 6-7
            
//             // Add to buffer if there's room
//             if (handle->tx_length < TPUART_MAX_TELEGRAM_LENGTH) {
//                 handle->tx_buffer[handle->tx_length++] = data_byte;
                
//                 // Validate the telegram length - basic validation 
//                 if (handle->tx_length < 2) {
//                     ESP_LOGE(TAG, "Telegram too short: %d bytes", handle->tx_length);
//                     handle->state = KNX_TPUART_STATE_ERROR;
                    
//                     // Send error indication
//                     uint8_t error = TPUART_RECEIVE_ERROR;
//                     int written = uart_write_bytes(handle->uart_port, &error, 1);
//                     if (written != 1) {
//                         ESP_LOGE(TAG, "Failed to send error indication");
//                     }
//                     if (handle->mutex != NULL) {
//                         xSemaphoreGive(handle->mutex);
//                     }
//                     return;
//                 }
                
//                 // Send the complete telegram to KNX bus
//                 ESP_LOGI(TAG, "Sending telegram to KNX bus, length: %d", handle->tx_length);
                
//                 // Use the app interface to send the telegram - use lowest priority as default
//                 esp_err_t err = knx_app_send_telegram(handle->tx_buffer, handle->tx_length, KNX_PRIORITY_LOW);
//                 if (err != ESP_OK) {
//                     ESP_LOGE(TAG, "Failed to send telegram: %s", esp_err_to_name(err));
                    
//                     // Send a negative acknowledgment
//                     uint8_t nack = TPUART_NACK;
//                     int written = uart_write_bytes(handle->uart_port, &nack, 1);
//                     if (written != 1) {
//                         ESP_LOGE(TAG, "Failed to send NACK");
//                     }
//                 }
                
//                 // Reset state to wait for ACK
//                 handle->state = KNX_TPUART_STATE_WAITING_FOR_ACK;
//                     if (written != 1) {
//                         ESP_LOGE(TAG, "Failed to send error indication");
//                     }
//                 }
                
//                 // Reset state to wait for ACK
//                 handle->state = KNX_TPUART_STATE_WAITING_FOR_ACK;
//             } else {
//                 // Buffer overflow
//                 ESP_LOGE(TAG, "TX buffer overflow at end of telegram");
//                 handle->state = KNX_TPUART_STATE_ERROR;
                
//                 // Send error indication
//                 uint8_t error = TPUART_RECEIVE_ERROR;
//                 int written = uart_write_bytes(handle->uart_port, &error, 1);
//                 if (written != 1) {
//                     ESP_LOGE(TAG, "Failed to send error indication");
//                 }
                
//                 // Reset buffer state to prevent further issues
//                 handle->tx_length = 0;
//             }
//         } else {
//             // Received end of telegram in wrong state
//             ESP_LOGE(TAG, "Received end of telegram in wrong state: %d", handle->state);
//             handle->state = KNX_TPUART_STATE_ERROR;
            
//             // Send error indication
//             uint8_t error = TPUART_RECEIVE_ERROR;
//             int written = uart_write_bytes(handle->uart_port, &error, 1);
//             if (written != 1) {
//                 ESP_LOGE(TAG, "Failed to send error indication");
//             }
//         }
        
//         // Release mutex before returning
//         if (handle->mutex != NULL) {
//             xSemaphoreGive(handle->mutex);
//         }
//         return;
//     }
//     // If we get here, it's a command we don't recognize
//     ESP_LOGW(TAG, "Unrecognized command: 0x%02x", byte);
    
//     // Release mutex at the end of function for all other cases
//     if (handle->mutex != NULL) {
//         xSemaphoreGive(handle->mutex);
//     }
//     return;
// }

// /**
//  * @brief Process a telegram received from KNX bus
//  * 
//  * @param handle TP-UART context
//  * @param data Telegram data
//  * @param length Telegram length
//  */
// static void process_knx_telegram(knx_tp_uart_handle_t handle, uint8_t* data, uint8_t length)
// {
//     ESP_LOGI(TAG, "Processing received KNX telegram, length: %d", length);
    
//     if (handle == NULL) {
//         ESP_LOGE(TAG, "NULL handle in process_knx_telegram");
//         return;
//     }
    
//     if (data == NULL) {
//         ESP_LOGE(TAG, "Invalid data pointer (NULL)");
//         return;
//     }
    
//     // Take mutex for thread-safe access
//     bool mutex_taken = false;
//     if (handle->mutex != NULL) {
//         if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//             mutex_taken = true;
//         } else {
//             ESP_LOGE(TAG, "Failed to take mutex in process_knx_telegram");
//             // Continue without mutex in this case
//         }
//     }
    
//     // Limit length to maximum allowed
//     if (length > TPUART_MAX_TELEGRAM_LENGTH) {
//         ESP_LOGW(TAG, "Telegram length %d exceeds maximum %d, truncating", length, TPUART_MAX_TELEGRAM_LENGTH);
//         length = TPUART_MAX_TELEGRAM_LENGTH;
//     }
    
//     // Make a copy of the data for the callback
//     uint8_t telegram_copy[TPUART_MAX_TELEGRAM_LENGTH];
//     if (length > 0) {
//         memcpy(telegram_copy, data, length);
//     }
    
//     // Call telegram received callback if registered - Call outside mutex lock to prevent deadlock
//     if (mutex_taken) {
//         xSemaphoreGive(handle->mutex);
//         mutex_taken = false;
//     }
    
//     // Now call the callback safely outside the mutex
//     if (handle->telegram_received_cb != NULL) {
//         handle->telegram_received_cb(telegram_copy, length, handle->telegram_received_user_data);
//     }
    
//     // Take mutex again
//     if (handle->mutex != NULL) {
//         if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//             mutex_taken = true;
//         } else {
//             ESP_LOGE(TAG, "Failed to take mutex again in process_knx_telegram");
//             return;
//         }
//     }
    
//     // Validate telegram format - basic validation
//     if (length < 2) {
//         ESP_LOGW(TAG, "Telegram too short (%d bytes), not forwarding to host", length);
        
//         if (mutex_taken) {
//             xSemaphoreGive(handle->mutex);
//         }
//         return;
//     }
    
//     // Send data received indication to host
//     uint8_t rx_start = TPUART_DATA_RECEIVED;
//     int written = uart_write_bytes(handle->uart_port, &rx_start, 1);
//     if (written != 1) {
//         ESP_LOGE(TAG, "Failed to write data received indication to UART");
        
//         if (mutex_taken) {
//             xSemaphoreGive(handle->mutex);
//         }
//         return;
//     }
    
//     // Send each byte with appropriate control bits
//     for (int i = 0; i < length - 1; i++) {
//         // Data bytes except the last one have bit 7 set
//         uint8_t tx_byte = 0x80 | data[i];
//         written = uart_write_bytes(handle->uart_port, &tx_byte, 1);
//         if (written != 1) {
//             ESP_LOGE(TAG, "Failed to write telegram byte %d to UART", i);
            
//             if (mutex_taken) {
//                 xSemaphoreGive(handle->mutex);
//             }
//             return;
//         }
//     }
    
//     // Last byte has bit 6 set
//     if (length > 0) {
//         uint8_t tx_byte = 0x40 | data[length - 1];
//         written = uart_write_bytes(handle->uart_port, &tx_byte, 1);
//         if (written != 1) {
//             ESP_LOGE(TAG, "Failed to write last telegram byte to UART");
            
//             if (mutex_taken) {
//                 xSemaphoreGive(handle->mutex);
//             }
//             return;
//         }
//     }
    
//     if (mutex_taken) {
//         xSemaphoreGive(handle->mutex);
//     }
// }

// /**
//  * @brief UART event handler task
//  * 
//  * @param pvParameters TP-UART context
//  */
// static void knx_tp_uart_task(void* pvParameters)
// {
//     knx_tp_uart_handle_t handle = (knx_tp_uart_handle_t)pvParameters;
//     if (handle == NULL) {
//         ESP_LOGE(TAG, "Null handle provided to task, exiting");
//         vTaskDelete(NULL);
//         return;
//     }

//     uint8_t data[TPUART_RX_BUFFER_SIZE];
//     int length;
    
//     ESP_LOGI(TAG, "TP-UART emulation task started");

//     // Setup UART event queue for more efficient reception
//     QueueHandle_t uart_queue;
//     ESP_LOGI(TAG, "Setting up UART event queue");
//     esp_err_t err = uart_driver_install(handle->uart_port, TPUART_RX_BUFFER_SIZE, 
//                                        TPUART_TX_BUFFER_SIZE, 10, &uart_queue, 0);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to setup UART queue: %s", esp_err_to_name(err));
//         handle->is_running = false;
//         vTaskDelete(NULL);
//         return;
//     }
    
//     // Clear any existing data in UART buffer
//     uart_flush(handle->uart_port);
    
//     // Main task loop
//     TickType_t last_activity_time = xTaskGetTickCount();
//     const TickType_t max_idle_time = pdMS_TO_TICKS(5000); // 5 seconds max idle time
    
//     while (handle->is_running) {
//         // Check for bytes received from UART (host) with short timeout
//         length = uart_read_bytes(handle->uart_port, data, TPUART_RX_BUFFER_SIZE, 10 / portTICK_PERIOD_MS);
//         if (length > 0) {
//             ESP_LOGI(TAG, "Received %d bytes from UART", length);
//             last_activity_time = xTaskGetTickCount();
            
//             // Process each byte
//             for (int i = 0; i < length; i++) {
//                 process_uart_byte(handle, data[i]);
                
//                 // Call callback if registered
//                 if (handle->rx_callback != NULL) {
//                     handle->rx_callback(data[i], handle->rx_callback_user_data);
//                 }
//             }
//         }
        
//         // Check for KNX data received from the app interface
//         uint8_t rx_data[TPUART_MAX_TELEGRAM_LENGTH];
//         uint8_t rx_length;
        
//         if (knx_app_telegram_available()) {
//             if (knx_app_receive_telegram(rx_data, &rx_length) == ESP_OK) {
//                 // Process the received telegram
//                 process_knx_telegram(handle, rx_data, rx_length);
//             }
//         }
        
//         // Check for state transitions
//         if (handle->state == KNX_TPUART_STATE_WAITING_FOR_ACK) {
//             // If we've been waiting for ACK for too long, reset
//             static const TickType_t max_ack_wait = pdMS_TO_TICKS(1000); // 1 second
            
//             if (handle->ack_wait_start == 0) {
//                 handle->ack_wait_start = xTaskGetTickCount();
//             } else if ((xTaskGetTickCount() - handle->ack_wait_start) > max_ack_wait) {
//                 ESP_LOGW(TAG, "ACK timeout, resetting state");
//                 handle->state = KNX_TPUART_STATE_IDLE;
//                 handle->ack_wait_start = 0;
                
//                 // Send NACK to host
//                 uint8_t nack = TPUART_NACK;
//                 uart_write_bytes(handle->uart_port, &nack, 1);
//             }
//         } else {
//             // Not waiting for ACK, reset timer
//             handle->ack_wait_start = 0;
//         }
        
//         // Allow other tasks to run with shorter delay
//         vTaskDelay(1);
        
//         // Check for extended inactivity
//         if ((xTaskGetTickCount() - last_activity_time) > max_idle_time) {
//             // No activity for too long, perform a health check
//             ESP_LOGW(TAG, "No activity for extended period, performing health check");
            
//             // Reset the timer
//             last_activity_time = xTaskGetTickCount();
//         }
//     }
    
//     ESP_LOGI(TAG, "TP-UART emulation task ended");
    
//     // Clean up resources before exiting
//     uart_driver_delete(handle->uart_port);
//     vTaskDelete(NULL);
// }

// // Callback function to receive telegrams from the app task
// static void knx_uart_telegram_received_callback(uint8_t* data, uint8_t length, void* user_data)
// {
//     knx_tp_uart_handle_t handle = (knx_tp_uart_handle_t)user_data;
//     if (handle == NULL) {
//         ESP_LOGE(TAG, "NULL handle in telegram received callback");
//         return;
//     }
    
//     // Process the received telegram
//     process_knx_telegram(handle, data, length);
// }

// esp_err_t knx_tp_uart_init(const knx_tp_uart_config_t* config, knx_tp_uart_handle_t* handle_out)
// {
//     ESP_LOGI(TAG, "Initializing KNX TP-UART emulation");
    
//     if (config == NULL || handle_out == NULL) {
//         ESP_LOGE(TAG, "Invalid arguments (NULL pointers)");
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     // Check KNX pins are valid
//     if (config->knx_pins.tx_pin == GPIO_NUM_NC || config->knx_pins.rx_pin == GPIO_NUM_NC) {
//         ESP_LOGE(TAG, "KNX pins not provided");
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     // Validate configuration
//     if (config->rx_buffer_size == 0 || config->tx_buffer_size == 0) {
//         ESP_LOGE(TAG, "Invalid buffer sizes");
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     // Allocate context
//     knx_tp_uart_handle_t handle = calloc(1, sizeof(struct knx_tp_uart_context_s));
//     if (handle == NULL) {
//         ESP_LOGE(TAG, "Failed to allocate memory for handle");
//         return ESP_ERR_NO_MEM;
//     }
    
//     // Initialize fields
//     handle->uart_port = config->uart_port;
//     handle->uart_baud_rate = config->uart_baud_rate;
//     handle->device_address = config->device_address;
//     handle->state = KNX_TPUART_STATE_IDLE;
//     handle->ack_wait_start = 0;
//     handle->is_running = false;
    
//     // Store KNX pin configuration for later use
//     memcpy(&handle->knx_pins, &config->knx_pins, sizeof(knx_tp_pin_config_t));
    
//     // Create the mutex
//     handle->mutex = xSemaphoreCreateMutex();
//     if (handle->mutex == NULL) {
//         ESP_LOGE(TAG, "Failed to create mutex");
//         free(handle);
//         return ESP_ERR_NO_MEM;
//     }
    
//     // Configure UART
//     uart_config_t uart_config = {
//         .baud_rate = config->uart_baud_rate,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT,
//     };
    
//     // Note: We don't install the UART driver here, the task will do that
//     // to prevent race conditions between task initialization and driver setup
    
//     esp_err_t err;
//     err = uart_param_config(config->uart_port, &uart_config);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(err));
//         vSemaphoreDelete(handle->mutex);
//         free(handle);
//         return err;
//     }
    
//     err = uart_set_pin(config->uart_port, config->uart_tx_pin, config->uart_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
//         vSemaphoreDelete(handle->mutex);
//         free(handle);
//         return err;
//     }
    
//     // Set the device address in the app layer
//     // Initialize the KNX app interface
//     // Create a new bit_bang instance inside the app interface
//     if (knx_app_create_with_config(&config->knx_pins) == NULL) {
//         ESP_LOGE(TAG, "Failed to create app interface");
//         vSemaphoreDelete(handle->mutex);
//         free(handle);
//         return ESP_FAIL;
//     }
    
//     ESP_LOGI(TAG, "TP-UART emulation initialized successfully");
//     *handle_out = handle;
//     return ESP_OK;
// }

// esp_err_t knx_tp_uart_deinit(knx_tp_uart_handle_t handle)
// {
//     if (handle == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     ESP_LOGI(TAG, "Deinitializing TP-UART emulation");
    
//     // Stop the task if running
//     if (handle->is_running) {
//         esp_err_t err = knx_tp_uart_stop(handle);
//         if (err != ESP_OK) {
//             ESP_LOGW(TAG, "Failed to stop TP-UART task: %s", esp_err_to_name(err));
//             // Continue with cleanup despite errors
//         }
//     }
    
//     // We don't need to call uart_driver_delete here since it's done in the task
//     // when it exits. Calling it here might cause a race condition.

//     // Reset handle fields to safe values
//     handle->rx_callback = NULL;
//     handle->telegram_received_cb = NULL;
//         }
//         if (handle->bit_bang->rx_queue != NULL) {
//             vQueueDelete(handle->bit_bang->rx_queue);
//         }
        
//         // Free the bit_bang structure
//         free(handle->bit_bang);
//         ESP_LOGI(TAG, "Freed KNX bit_bang interface");
//     }

//     // Free memory
//     if (handle->mutex != NULL) {
//         vSemaphoreDelete(handle->mutex);
//     }
//     free(handle);
    
//     return ESP_OK;
// }

// esp_err_t knx_tp_uart_send_byte(knx_tp_uart_handle_t handle, uint8_t byte)
// {
//     if (handle == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     if (!handle->is_running) {
//         ESP_LOGW(TAG, "Trying to send byte when TP-UART is not running");
//         return ESP_ERR_INVALID_STATE;
//     }
    
//     // Process the byte
//     process_uart_byte(handle, byte);
    
//     return ESP_OK;
// }

// esp_err_t knx_tp_uart_send_data(knx_tp_uart_handle_t handle, const uint8_t* data, size_t len)
// {
//     if (handle == NULL) {
//         ESP_LOGE(TAG, "NULL handle in send_data");
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     if (data == NULL) {
//         ESP_LOGE(TAG, "NULL data pointer in send_data");
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     if (len == 0) {
//         ESP_LOGW(TAG, "Zero length data in send_data");
//         return ESP_ERR_INVALID_SIZE;
//     }
    
//     if (!handle->is_running) {
//         ESP_LOGW(TAG, "Trying to send data when TP-UART is not running");
//         return ESP_ERR_INVALID_STATE;
//     }
    
//     // Validate length
//     if (len > TPUART_MAX_TELEGRAM_LENGTH) {
//         ESP_LOGW(TAG, "Data length %zu exceeds max telegram size %d", len, TPUART_MAX_TELEGRAM_LENGTH);
//         return ESP_ERR_INVALID_SIZE;
//     }
    
//     // Process each byte
//     for (size_t i = 0; i < len; i++) {
//         esp_err_t err = knx_tp_uart_send_byte(handle, data[i]);
//         if (err != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to send byte %zu: %s", i, esp_err_to_name(err));
//             return err;
//         }
//     }
    
//     return ESP_OK;
// }

// esp_err_t knx_tp_uart_set_rx_callback(knx_tp_uart_handle_t handle, 
//                                   void (*callback)(uint8_t byte, void* user_data),
//                                   void* user_data)
// {
//     if (handle == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     bool mutex_taken = false;
//     if (handle->mutex != NULL) {
//         if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//             mutex_taken = true;
//         } else {
//             ESP_LOGW(TAG, "Failed to take mutex when setting RX callback");
//         }
//     }
    
//     handle->rx_callback = callback;
//     handle->rx_callback_user_data = user_data;
    
//     if (mutex_taken) {
//         xSemaphoreGive(handle->mutex);
//     }
    
//     return ESP_OK;
// }

// /**
//  * @brief Set callback for received KNX telegrams
//  * 
//  * @param handle TP-UART emulation handle
//  * @param callback Function to call when a telegram is received
//  * @param user_data User data to pass to the callback
//  * @return esp_err_t ESP_OK on success
//  */
// esp_err_t knx_tp_uart_set_telegram_received_cb(knx_tp_uart_handle_t handle, 
//                                            void (*callback)(uint8_t *data, uint8_t length, void* user_data),
//                                            void* user_data)
// {
//     if (handle == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     bool mutex_taken = false;
//     if (handle->mutex != NULL) {
//         if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//             mutex_taken = true;
//         } else {
//             ESP_LOGW(TAG, "Failed to take mutex when setting telegram callback");
//         }
//     }
    
//     handle->telegram_received_cb = callback;
//     handle->telegram_received_user_data = user_data;
    
//     if (mutex_taken) {
//         xSemaphoreGive(handle->mutex);
//     }
    
//     return ESP_OK;
// }

// esp_err_t knx_tp_uart_start(knx_tp_uart_handle_t handle)
// {
//     if (handle == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     ESP_LOGI(TAG, "Starting TP-UART emulation");
    
//     // Check if already running
//     if (handle->is_running) {
//         ESP_LOGW(TAG, "TP-UART emulation is already running");
//         return ESP_OK;
//     }
    
//     if (handle->mutex != NULL) {
//         if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
//             ESP_LOGE(TAG, "Failed to take mutex when starting");
//             return ESP_ERR_TIMEOUT;
//         }
//     }
    
//     // Reset state
//     handle->state = KNX_TPUART_STATE_IDLE;
//     handle->tx_length = 0;
//     handle->rx_length = 0;
    
//     // Set running flag
//     handle->is_running = true;
    
//     if (handle->mutex != NULL) {
//         xSemaphoreGive(handle->mutex);
//     }
    
//     // Create task with increased stack size for safety
//     const uint32_t stack_size = 4096;
//     BaseType_t result = xTaskCreate(knx_tp_uart_task, "tp_uart_task", stack_size, handle, 5, &handle->task_handle);
//     if (result != pdPASS) {
//         ESP_LOGE(TAG, "Failed to create TP-UART task");
        
//         if (handle->mutex != NULL) {
//             if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//                 handle->is_running = false;
//                 xSemaphoreGive(handle->mutex);
//             }
//         } else {
//             handle->is_running = false;
//         }
        
//         return ESP_ERR_NO_MEM;
//     }
    
//     ESP_LOGI(TAG, "TP-UART emulation started successfully");
//     return ESP_OK;
// }

// esp_err_t knx_tp_uart_stop(knx_tp_uart_handle_t handle)
// {
//     if (handle == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     ESP_LOGI(TAG, "Stopping TP-UART emulation");
    
//     // Check if running
//     if (!handle->is_running) {
//         ESP_LOGW(TAG, "TP-UART emulation is not running");
//         return ESP_OK;
//     }
    
//     if (handle->mutex != NULL) {
//         if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
//             ESP_LOGW(TAG, "Failed to take mutex when stopping - continuing anyway");
//         }
//     }
    
//     // Clear running flag
//     handle->is_running = false;
    
//     if (handle->mutex != NULL) {
//         xSemaphoreGive(handle->mutex);
//     }
    
//     // Give the task time to exit
//     TaskHandle_t task_to_wait = handle->task_handle;
//     if (task_to_wait != NULL) {
//         const TickType_t stop_timeout = pdMS_TO_TICKS(1000);  // 1 second timeout
//         TickType_t start_time = xTaskGetTickCount();
        
//         // Wait for task to exit with timeout
//         while (eTaskGetState(task_to_wait) != eDeleted && 
//                (xTaskGetTickCount() - start_time) < stop_timeout) {
//             vTaskDelay(10 / portTICK_PERIOD_MS);
//         }
        
//         if (eTaskGetState(task_to_wait) != eDeleted) {
//             ESP_LOGW(TAG, "TP-UART task did not exit in time, may leak resources");
//         }
//     }
    
//     // Task should be exiting
//     handle->task_handle = NULL;
    
//     ESP_LOGI(TAG, "TP-UART emulation stopped");
//     return ESP_OK;
// }
