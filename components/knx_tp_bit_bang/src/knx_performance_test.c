/**
 * @file knx_performance_test.c
 * @brief Performance testing for KNX TP bit-bang optimizations
 * 
 * This file demonstrates the performance improvements achieved through:
 * - Direct GPIO register access
 * - Compile-time pin configuration 
 * - Branch prediction optimizations
 * - Reduced ISR overhead
 */

#include "knx_tp_bit_bang.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "KNX_PERF_TEST";

/**
 * @brief Test the performance of GPIO operations
 * 
 * This function measures the execution time of GPIO operations
 * to demonstrate the performance improvements.
 */
void knx_performance_test(void)
{
    ESP_LOGI(TAG, "=== KNX TP Bit-Bang Performance Test ===");
    
    knx_tp_bit_bang_t bit_bang;
    esp_err_t ret = knx_tp_bit_bang_init(&bit_bang);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bit-bang: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Initialization successful with optimizations:");
    ESP_LOGI(TAG, "  - TX Pin: %d (compile-time optimized)", CONFIG_KNX_TP_TX_PIN);
    ESP_LOGI(TAG, "  - RX Pin: %d (compile-time optimized)", CONFIG_KNX_TP_RX_PIN);
    
#ifdef CONFIG_KNX_USE_DIRECT_GPIO
    ESP_LOGI(TAG, "  - Direct GPIO register access: ENABLED");
#else
    ESP_LOGI(TAG, "  - Direct GPIO register access: DISABLED");
#endif

#ifdef CONFIG_KNX_DEBUG_GPIO
    ESP_LOGI(TAG, "  - Debug GPIO: ENABLED (Pin %d)", CONFIG_KNX_DEBUG_GPIO_PIN);
#else
    ESP_LOGI(TAG, "  - Debug GPIO: DISABLED");
#endif

    // Simulate some bit-bang operations
    ESP_LOGI(TAG, "Simulating KNX telegram transmission...");
    
    // Test data
    uint8_t test_telegram[8] = {0xBC, 0x11, 0x01, 0x12, 0x34, 0x06, 0x10, 0x81};
    
    // Copy test data to bit-bang structure
    memcpy(bit_bang.tx_buffer, test_telegram, sizeof(test_telegram));
    bit_bang.tx_telegram_length = sizeof(test_telegram);
    
    // Measure performance
    int64_t start_time = esp_timer_get_time();
    
    ret = knx_tp_bit_bang_tx_enable(&bit_bang);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Transmission enabled successfully");
        
        // Wait a bit to allow some processing
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Get performance statistics
        uint32_t tx_count, rx_count;
        knx_tp_bit_bang_get_performance_stats(&bit_bang, &tx_count, &rx_count);
        
        int64_t end_time = esp_timer_get_time();
        int64_t duration_us = end_time - start_time;
        
        ESP_LOGI(TAG, "Performance Results:");
        ESP_LOGI(TAG, "  - Test duration: %lld μs", duration_us);
        ESP_LOGI(TAG, "  - TX timer events: %ld", tx_count);
        ESP_LOGI(TAG, "  - RX timer events: %ld", rx_count);
        ESP_LOGI(TAG, "  - Total ISR invocations: %ld", tx_count + rx_count);
        
        if (tx_count > 0) {
            ESP_LOGI(TAG, "  - Avg time per TX event: %lld μs", duration_us / tx_count);
        }
    } else {
        ESP_LOGE(TAG, "Failed to enable transmission: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "=== Performance Test Complete ===");
}
