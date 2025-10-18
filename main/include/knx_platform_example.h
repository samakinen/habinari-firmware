#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Example demonstrating the use of the ESP-IDF platform for KNX
 * 
 * This function creates an instance of the EspIdFPlatform class, which implements
 * the KNX Platform interface for ESP32 using the KNX TP-UART emulation.
 */
void knx_platform_example(void);

#ifdef __cplusplus
}
#endif
