#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Example demonstrating the use of the KNX stack with ESP-IDF platform
 * 
 * This function creates an instance of the EspIdFPlatform class and initializes
 * the KNX stack to send and receive KNX messages. It demonstrates how to:
 * 1. Set up the physical address
 * 2. Create a group object and assign a group address
 * 3. Register a message handler
 * 4. Send and receive KNX messages
 */
void knx_stack_example(void);

#ifdef __cplusplus
}
#endif
