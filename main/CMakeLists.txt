idf_component_register(SRCS "src/sensor_service.c"
                            "src/board.c"
                            "src/main.c"
                            "src/ext_probe.c"
                            "src/sensor_bus.c"
                            "src/mb_rtu_slave.c"
                            "src/knx_service.cpp"
                            "src/device_identity.cpp"
                            "src/device_secret_core.cpp"
                            "src/device_secret_esp.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES
                         driver
                         bme68x
                         bsec2
                         hdc302x
                         scd4x
                         nvs_flash
                         efuse
                         esp_security
                         bootloader_support
                         mbedtls
                         KNstaX)

set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
