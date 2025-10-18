idf_component_register(SRCS "src/sensor_service.c"
                            "src/board.c"
                            "src/main.c"
                            "src/ext_probe.c"
                            "src/sensor_bus.c"
                            "src/sensor_service.c"
                            "src/mb_rtu_slave.c"
                       INCLUDE_DIRS "include"
                       REQUIRES
                         driver
                         bme68x
                         hdc302x
                         scd4x
                         knx_tp_bit_bang)
