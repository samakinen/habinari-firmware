# Define source groups
set(TPUART_SRCS
    "tpuart/src/TPUart/DataLinkLayer.cpp"
    "tpuart/src/TPUart/Receiver.cpp"
    "tpuart/src/TPUart/Transmitter.cpp"
    "tpuart/src/TPUart/RingBuffer.cpp"
    "tpuart/src/TPUart/SearchBuffer.cpp"
    "tpuart/src/TPUart/Statistics.cpp"
    "tpuart/src/TPUart/SystemState.cpp"
    "tpuart/src/TPUart/RepetitionFilter.cpp"
)

set(KNX_MIN_SRCS
    "knx/src/knx/cemi_frame.cpp"
    "knx/src/knx/apdu.cpp"
    "knx/src/knx/tpdu.cpp"
    "knx/src/knx/npdu.cpp"
    "knx/src/knx/bits.cpp"
)

idf_component_register(
    SRCS 
        "src/knx_tp_bit_bang.c"
        "src/knx_tp_bit_bang_isr.c"
        "src/knx_ring_buffer.c"
        "src/knx_tp_bit_bang_task.c"
        "src/knx_tp_uart_emulation.c"
        "src/knx_tp_bit_bang_interface.cpp"
        ${TPUART_SRCS}
        ${KNX_MIN_SRCS}
    INCLUDE_DIRS 
        "include"
        "tpuart/src"
        "knx/src/knx"
    REQUIRES 
        driver
    PRIV_REQUIRES
        esp_timer
)

# Enable ESP32-specific code paths only for the TPUART stack files
set_source_files_properties(${TPUART_SRCS} PROPERTIES COMPILE_DEFINITIONS "ARDUINO_ARCH_ESP32")

# Provide a MASK_VERSION for the minimal KNX CEMI/APDU/TPDU sources used by tests
set_source_files_properties(${KNX_MIN_SRCS} PROPERTIES COMPILE_DEFINITIONS "MASK_VERSION=0x07B0")

# Disable specific compiler errors for external TPUART library files  
# The DataLinkLayer.cpp has issues with non-virtual destructor and boolean context
target_compile_options(${COMPONENT_LIB} PRIVATE 
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=delete-non-virtual-dtor>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=int-in-bool-context>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=ignored-qualifiers>
)
