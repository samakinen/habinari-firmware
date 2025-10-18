
#include "esp_log.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "knx_tp_bit_bang.h"

static const char *TAG = "KNX_TP_BIT_BANG";

// Forward declarations for ISR functions defined in knx_tp_bit_bang_isr.c
void rearm_timer(knx_tp_bit_bang_t *bit_bang);

esp_err_t knx_tp_bit_bang_init(knx_tp_bit_bang_t *bit_bang)
{
    static const int timer_intr_priority = 3; // Set high priority for timer interrupts to keep timing accurate
    esp_err_t ret;
    
    if (bit_bang == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(bit_bang, 0, sizeof(knx_tp_bit_bang_t)); // Init all fields to 0
    
    // Initialize ring buffer
    knx_ring_buffer_init(&bit_bang->rx_ring_buffer);
    
    // Use compile-time configured pins for maximum optimization
    bit_bang->timer = NULL;
    bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_IDLE;
    bit_bang->rx_state = KNX_TP_BIT_BANG_RX_STATE_IDLE;

    bit_bang->tx_timer_count = 0;
    bit_bang->rx_timer_count = 0;
    
    // Initialize address filtering
    bit_bang->device_address = 0x0000;  // Default device address (can be changed later)
    bit_bang->group_address_count = 0;  // No group addresses by default

    // Configure timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
        .intr_priority = timer_intr_priority,
    };
    ret = gptimer_new_timer(&timer_config, &bit_bang->timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TX timer: %s", esp_err_to_name(ret));
        return ret;
    }    gptimer_event_callbacks_t cbs = {
        .on_alarm = knx_tp_bit_bang_timer_isr, // register user callback
    };
    ret = gptimer_register_event_callbacks(bit_bang->timer, &cbs, bit_bang);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register event callbacks: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gptimer_enable(bit_bang->timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable timer: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gptimer_start(bit_bang->timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(ret));
        return ret;
    }    // Configure RX pin using compile-time constant
    gpio_config_t rx_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // Trigger on rising edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_KNX_TP_RX_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ret = gpio_config(&rx_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure RX GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure TX pin using compile-time constant
    gpio_config_t tx_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_KNX_TP_TX_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ret = gpio_config(&tx_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure TX GPIO: %s", esp_err_to_name(ret));
        // Clean up and return error
        return ret;
    }
    
    // Initialize TX pin to idle state using optimized operation
    knx_tp_bit_bang_reset_tx(bit_bang);
    
    ret = gpio_isr_handler_add(CONFIG_KNX_TP_RX_PIN, knx_tp_bit_bang_rx_pin_isr, bit_bang);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        // Clean up and return error
        return ret;
    }
    return ESP_OK;
}

// Performance monitoring function
void knx_tp_bit_bang_get_performance_stats(knx_tp_bit_bang_t *bit_bang, 
                                           uint32_t *tx_timer_count, 
                                           uint32_t *rx_timer_count)
{
    if (bit_bang == NULL) {
        return;
    }
    
    if (tx_timer_count != NULL) {
        *tx_timer_count = bit_bang->tx_timer_count;
    }
    
    if (rx_timer_count != NULL) {
        *rx_timer_count = bit_bang->rx_timer_count;
    }
}

esp_err_t knx_tp_bit_bang_tx_enable(knx_tp_bit_bang_t *bit_bang)
{
    if (bit_bang->tx_state != KNX_TP_BIT_BANG_TX_STATE_IDLE) {
        ESP_LOGE(TAG, "Bit-bang is busy, cannot send data.");
        return ESP_ERR_INVALID_STATE;
    }

    bit_bang->tx_byte_position = 0;
    bit_bang->tx_bit_position = -1; // Start with start bit
    bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT; // start with waiting slot state


    gptimer_alarm_config_t alarm_config = {
        .flags.auto_reload_on_alarm = false
    };
    gptimer_get_raw_count(bit_bang->timer, &alarm_config.alarm_count);
    bit_bang->tx_alarm_value = alarm_config.alarm_count + KNX_BIT_TIME_US; // Set the timer for the next bit
    knx_tp_bit_bang_rearm_timer(bit_bang);


    ESP_LOGI(TAG, "Sending %d bytes, tx-status: %d", bit_bang->tx_telegram_length, bit_bang->tx_state);
    return ESP_OK;
}

esp_err_t knx_tp_bit_bang_receive(knx_tp_bit_bang_t *bit_bang, uint8_t *data, uint16_t length) {
    return ESP_OK;
}

// Ring buffer API implementation

// Ring buffer API implementation

bool knx_tp_bit_bang_pop_telegram(knx_tp_bit_bang_t *bit_bang, knx_ring_buffer_entry_t *entry)
{
    if (bit_bang == NULL || entry == NULL) {
        return false;
    }
    
    return knx_ring_buffer_pop(&bit_bang->rx_ring_buffer, entry);
}

uint8_t knx_tp_bit_bang_telegrams_available(knx_tp_bit_bang_t *bit_bang)
{
    if (bit_bang == NULL) {
        return 0;
    }
    
    return knx_ring_buffer_available(&bit_bang->rx_ring_buffer);
}

uint32_t knx_tp_bit_bang_get_dropped_count(knx_tp_bit_bang_t *bit_bang)
{
    if (bit_bang == NULL) {
        return 0;
    }
    
    return knx_ring_buffer_get_dropped_count(&bit_bang->rx_ring_buffer);
}

// Address filtering API implementation

void knx_format_individual_address(uint16_t addr, char* out, size_t out_size)
{
    uint8_t area = (addr >> 12) & 0x0F;
    uint8_t line = (addr >> 8) & 0x0F;
    uint8_t device = addr & 0xFF;
    snprintf(out, out_size, "%u.%u.%u", area, line, device);
}

void knx_format_group_address(uint16_t addr, char* out, size_t out_size)
{
    uint8_t main = (addr >> 11) & 0x1F;
    uint8_t middle = (addr >> 8) & 0x07;
    uint8_t sub = addr & 0xFF;
    snprintf(out, out_size, "%u/%u/%u", main, middle, sub);
}

uint16_t knx_tp_bit_bang_triplet_to_address(uint8_t main, uint8_t middle, uint8_t sub)
{
    return ((main & 0x1F) << 11) | ((middle & 0x07) << 8) | (sub & 0xFF);
}

esp_err_t knx_tp_bit_bang_set_device_address(knx_tp_bit_bang_t *bit_bang, uint16_t address)
{
    if (bit_bang == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    bit_bang->device_address = address;
    char addr_str[16];
    knx_format_individual_address(address, addr_str, sizeof(addr_str));
    ESP_LOGI(TAG, "Device individual address set to %s", addr_str);
    
    return ESP_OK;
}

esp_err_t knx_tp_bit_bang_add_group_address(knx_tp_bit_bang_t *bit_bang, uint16_t group_address)
{
    if (bit_bang == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    char addr_str[16];
    knx_format_group_address(group_address, addr_str, sizeof(addr_str));
    // Check if list is full
    if (bit_bang->group_address_count >= KNX_TP_BIT_BANG_MAX_GROUP_ADDRESSES) {
        ESP_LOGE(TAG, "Group address list is full (%d addresses)", KNX_TP_BIT_BANG_MAX_GROUP_ADDRESSES);
        return ESP_ERR_NO_MEM;
    }
    
    // Check if address already exists
    for (uint8_t i = 0; i < bit_bang->group_address_count; i++) {
        if (bit_bang->group_addresses[i] == group_address) {
            ESP_LOGW(TAG, "Group address %s already in list", addr_str);
            return ESP_OK;  // Already exists, not an error
        }
    }
    
    // Add new address
    bit_bang->group_addresses[bit_bang->group_address_count] = group_address;
    bit_bang->group_address_count++;

    ESP_LOGI(TAG, "Added group address %s (total: %d)", addr_str, bit_bang->group_address_count);

    return ESP_OK;
}

esp_err_t knx_tp_bit_bang_remove_group_address(knx_tp_bit_bang_t *bit_bang, uint16_t group_address)
{
    if (bit_bang == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find and remove the address
    for (uint8_t i = 0; i < bit_bang->group_address_count; i++) {
        if (bit_bang->group_addresses[i] == group_address) {
            // Shift remaining addresses down
            for (uint8_t j = i; j < bit_bang->group_address_count - 1; j++) {
                bit_bang->group_addresses[j] = bit_bang->group_addresses[j + 1];
            }
            bit_bang->group_address_count--;
            char addr_str[16];
            knx_format_group_address(group_address, addr_str, sizeof(addr_str));
            ESP_LOGI(TAG, "Removed group address %s (remaining: %d)", addr_str, bit_bang->group_address_count);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "Group address 0x%04X not found in list", group_address);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t knx_tp_bit_bang_clear_group_addresses(knx_tp_bit_bang_t *bit_bang)
{
    if (bit_bang == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    bit_bang->group_address_count = 0;
    ESP_LOGI(TAG, "Cleared all group addresses");
    
    return ESP_OK;
}

