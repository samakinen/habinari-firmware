idf_component_register(
    SRCS 
        "src/knx_tp_bit_bang.c"
        "src/knx_tp_bit_bang_isr.c"
        "src/knx_ring_buffer.c"
        "src/knx_tp_bit_bang_task.c"
        "src/knx_tp_uart_emulation.c"
        "src/knx_performance_test.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        driver
    PRIV_REQUIRES
        esp_timer

)
