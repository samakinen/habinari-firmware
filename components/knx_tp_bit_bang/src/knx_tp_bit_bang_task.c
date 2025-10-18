#ifdef USE_KNX_TP_BIT_BANG_TASK

#include "knx_tp_bit_bang.h"
#include "knx_tp_bit_bang_task.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_random.h" // For esp_random()


static const char *TAG = "KNX_TP_BIT_BANG_TASK";

// Structure to keep track of telegram processing state
typedef struct {
    knx_tp_bit_bang_t *bit_bang;
    uint8_t retry_count;
    uint64_t next_retry_time;
    uint8_t telegram_buffer[KNX_TP_BIT_BANG_TX_BUFFER_SIZE];
    uint8_t telegram_length;
    knx_priority_t pending_priority;
    bool telegram_pending;
} knx_tp_bit_bang_task_context_t;

// Function to handle collision using a random backoff strategy
static void handle_collision(knx_tp_bit_bang_task_context_t *context) {
    knx_tp_bit_bang_t *bit_bang = context->bit_bang;
    
    // Implement a random backoff strategy
    context->retry_count++;
    
    // Limit retries
    if (context->retry_count > 3) {
        ESP_LOGI(TAG, "Max retries reached, giving up");
        context->telegram_pending = false;
        return;
    }
    
    // Calculate random backoff time (20-50 bit times)
    uint32_t backoff_time = (KNX_BIT_TIME_US * (20 + (esp_random() % 30)));
    
    // Get current time
    uint64_t current_time;
    gptimer_get_raw_count(bit_bang->timer, &current_time);
    
    // Set next retry time
    context->next_retry_time = current_time + backoff_time;
    
    ESP_LOGI(TAG, "Collision detected, retry %d scheduled in %lu us", 
            context->retry_count, (unsigned long)backoff_time);
}

// Function to initiate transmission when the bus is idle
static void start_transmission(knx_tp_bit_bang_task_context_t *context) {
    knx_tp_bit_bang_t *bit_bang = context->bit_bang;
    
    if (!context->telegram_pending) {
        return;
    }
    
    // Check if we're in a backoff period
    if (context->retry_count > 0) {
        uint64_t current_time;
        gptimer_get_raw_count(bit_bang->timer, &current_time);
        
        if (current_time < context->next_retry_time) {
            // Not time to retry yet
            return;
        }
    }
    
    // Copy telegram to the bit bang buffer
    memcpy(bit_bang->tx_buffer, context->telegram_buffer, context->telegram_length);
    bit_bang->tx_telegram_length = context->telegram_length;
    bit_bang->telegram_priority = context->pending_priority;
    
    // Start the transmission
    esp_err_t ret = knx_tp_bit_bang_tx_enable(bit_bang);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start transmission: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Transmission started with priority %d", context->pending_priority);
    }
}

// Process received telegrams
static void process_received_telegram(knx_tp_bit_bang_task_context_t *context) {
    knx_tp_bit_bang_t *bit_bang = context->bit_bang;
    
    // Check if there's any data in the RX buffer
    if (bit_bang->rx_telegram_length == 0) {
        return;
    }
    
    // Process the received telegram
    ESP_LOGI(TAG, "Received telegram with length %d", bit_bang->rx_telegram_length);
    
    // Extract priority from control byte (bits 2-3)
    uint8_t ctrl_byte = bit_bang->rx_buffer[0];
    knx_priority_t priority = (ctrl_byte >> 2) & 0x03;
    
    // Log the priority
    ESP_LOGI(TAG, "Telegram priority: %d", priority);
    
    // Process the telegram according to its type, destination, etc.
    // This is application-specific code
    // ...
    
    // Reset for next reception
    bit_bang->rx_telegram_length = 0;
}

// KNX processing task - handles events from the bit bang ISR
void knx_processing_task(void *pvParameters) {
    knx_tp_bit_bang_task_context_t context = {0};
    context.bit_bang = (knx_tp_bit_bang_t *)pvParameters;
    
    // Register this task for notifications
    context.bit_bang->xTaskToNotify = xTaskGetCurrentTaskHandle();
    
    ESP_LOGI(TAG, "KNX processing task started");
    
    while (1) {
        // Wait for notification from ISR
        uint32_t notification_value = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        
        if (notification_value > 0) {
            knx_tp_bit_bang_t *bit_bang = context.bit_bang;
            
            // Check status flags and process accordingly
            
            // Check for received telegram
            if (bit_bang->rx_state == KNX_TP_BIT_BANG_RX_STATE_TELEGRAM_RECEIVED) {
                ESP_LOGI(TAG, "Telegram received");
                process_received_telegram(&context);
                // Reset rx_status after processing
                bit_bang->rx_status = KNX_TP_BIT_BANG_RX_STATUS_IDLE;
            }
            
            // Check for transmission related events
            switch (bit_bang->tx_status) {
                case KNX_TP_BIT_BANG_TX_STATE_IDLE:
                    // Check if bus is idle to start transmission
                    ESP_LOGI(TAG, "Bus detected as idle");
                    start_transmission(&context);
                    break;
                    
                case KNX_TP_BIT_BANG_TX_STATE_COLLISION:
                    ESP_LOGI(TAG, "Collision detected");
                    handle_collision(&context);
                    // Reset tx_status after handling collision
                    bit_bang->tx_status = KNX_TP_BIT_BANG_TX_STATE_IDLE;
                    break;
                    
                case KNX_TP_BIT_BANG_TX_STATE_ACK_RECEIVED:
                    ESP_LOGI(TAG, "ACK received");
                    context.telegram_pending = false;
                    context.retry_count = 0;
                    // Reset tx_status after handling ACK
                    bit_bang->tx_status = KNX_TP_BIT_BANG_TX_STATE_IDLE;
                    break;
                    
                case KNX_TP_BIT_BANG_TX_STATE_ACK_NOT_RECEIVED:
                    ESP_LOGI(TAG, "ACK not received, retry");
                    handle_collision(&context); // Treat as collision for retry purposes
                    // Reset tx_status after handling ACK not received
                    bit_bang->tx_status = KNX_TP_BIT_BANG_TX_STATE_IDLE;
                    break;
                    
                case KNX_TP_BIT_BANG_TX_STATE_ACK_BUSY:
                    ESP_LOGI(TAG, "Receiver busy (BUSY), waiting before retry");
                    handle_collision(&context); // Use the same backoff mechanism
                    // Reset tx_status after handling busy
                    bit_bang->tx_status = KNX_TP_BIT_BANG_TX_STATE_IDLE;
                    break;
                    
                case KNX_TP_BIT_BANG_TX_STATE_ACK_NACK:
                    ESP_LOGI(TAG, "Negative acknowledgment (NACK) received, retry");
                    handle_collision(&context); // Use the same backoff mechanism
                    // Reset tx_status after handling NACK
                    bit_bang->tx_status = KNX_TP_BIT_BANG_TX_STATE_IDLE;
                    break;
                    
                case KNX_TP_BIT_BANG_TX_STATE_ACK_NACK_BUSY:
                    ESP_LOGI(TAG, "NACK and BUSY received, retry after longer delay");
                    // Double the retry count to increase backoff time
                    context.retry_count++;
                    handle_collision(&context);
                    // Reset tx_status after handling NACK+BUSY
                    bit_bang->tx_status = KNX_TP_BIT_BANG_TX_STATE_IDLE;
                    break;
                    
                default:
                    // Other TX states don't need immediate action
                    break;
            }
        }
        
        // Periodic processing
        // Check if we should prepare a telegram (based on application logic)
        // ...
    }
}

// Function to initialize and start the KNX processing task
void knx_tp_bit_bang_task_init(knx_tp_bit_bang_t *bit_bang, gpio_num_t tx_pin, gpio_num_t rx_pin, 
                  gpio_num_t prog_btn_pin, gpio_num_t led_pin) {
    // Store pin configurations or use them for hardware setup if needed
    
    // Create the KNX processing task
    xTaskCreate(knx_processing_task, "knx_proc", 4096, bit_bang, 5, NULL);
}

/**
 * @brief Initialize the KNX application layer with pin configuration structure
 */
void knx_tp_bit_bang_task_init_with_config(knx_tp_bit_bang_t *bit_bang, const knx_tp_pin_config_t *pin_config) {
    if (bit_bang == NULL || pin_config == NULL) {
        return;
    }
    
    knx_tp_bit_bang_task_init(bit_bang, pin_config->tx_pin, pin_config->rx_pin, 
                 pin_config->prog_btn_pin, pin_config->led_pin);
}

/**
 * @brief Create and initialize the KNX TP bit-bang interface and task with individual pins
 */
knx_tp_bit_bang_handle_t knx_tp_bit_bang_task_create(gpio_num_t tx_pin, gpio_num_t rx_pin,
                                                gpio_num_t prog_btn_pin, gpio_num_t led_pin) {
    static const char *TAG = "KNX_TP_BIT_BANG_TASK_CREATE";
                                                    
    // Allocate memory for the bit_bang structure
    knx_tp_bit_bang_t *bit_bang = (knx_tp_bit_bang_t *)malloc(sizeof(knx_tp_bit_bang_t));
    if (bit_bang == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for bit_bang structure");
        return NULL;
    }
    
    // Initialize the bit_bang structure
    esp_err_t ret = knx_tp_bit_bang_init(bit_bang, tx_pin, rx_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bit_bang: %s", esp_err_to_name(ret));
        free(bit_bang);
        return NULL;
    }
    
    // Initialize the task
    knx_tp_bit_bang_task_init(bit_bang, tx_pin, rx_pin, prog_btn_pin, led_pin);
    
    return bit_bang;
}

// /**
//  * @brief Create and initialize the KNX TP bit-bang interface and task with pin configuration structure
//  */
// knx_tp_bit_bang_handle_t knx_tp_bit_bang_task_create_with_config(const knx_tp_pin_config_t *pin_config) {
//     if (pin_config == NULL) {
//         return NULL;
//     }
    
//     return knx_tp_bit_bang_task_create(pin_config->tx_pin, pin_config->rx_pin,
//                                     pin_config->prog_btn_pin, pin_config->led_pin);
// }
#endif // USE_KNX_TP_BIT_BANG_TASK