#include "esp_log.h"
#include "esp_err.h"
#include "board.h"
#include "esp_idf_platform.h"

static const char *TAG = "KNX_PLATFORM_EXAMPLE";

extern "C" void knx_platform_example(void) {
    
    ESP_LOGI(TAG, "Initializing KNX ESP-IDF Platform");
    
    // Define KNX TP pin configuration
    knx_tp_pin_config_t pin_config = {
        .tx_pin = PIN_KNX_TX,
        .rx_pin = PIN_KNX_RX,
        .prog_btn_pin = PIN_PROG_BTN,
        .led_pin = PIN_LED
    };
    
    // Create the ESP-IDF Platform instance
    EspIdFPlatform* platform = new EspIdFPlatform(pin_config);
    
    // Setup the UART interface to connect to the KNX TP bit-bang implementation
    platform->setupUart();
    
    ESP_LOGI(TAG, "KNX ESP-IDF Platform initialized successfully");
    
    // Here you would typically initialize the KNX stack with this platform
    // For example:
    // knx.init(platform);
    
    // Example: Keep the application running for demo purposes
    ESP_LOGI(TAG, "KNX Platform example running...");
    
    // Process messages in a loop
    int count = 0;
    while (count < 10) {
        // Check if there's data available from the KNX bus
        if (platform->uartAvailable()) {
            // Read the data
            uint8_t data = platform->readUart();
            ESP_LOGI(TAG, "Received data from KNX bus: 0x%02X", data);
        }
        
        // Send a test byte every second
        if (count % 5 == 0) {
            uint8_t test_byte = 0xA5;
            ESP_LOGI(TAG, "Sending test byte to KNX bus: 0x%02X", test_byte);
            platform->writeUart(test_byte);
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        count++;
    }
    
    // Cleanup
    platform->closeUart();
    delete platform;
    
    ESP_LOGI(TAG, "KNX Platform example completed");
}
