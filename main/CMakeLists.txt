# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Sami Mäkinen

# The device, always: acquisition, fusion, the control core and the service
# that owns it. This set builds and runs with no field-bus personality at all.
set(app_srcs
    "src/main.c"
    "src/board.c"
    "src/ext_probe.c"
    "src/sensor_bus.c"
    "src/sensor_service.c"
    "src/sensor_fusion_service.cpp"
    "src/control_service.cpp"
    "src/protocol_registry.c"
    "src/device_config.c"
    "src/device_identity.cpp"
    "src/device_secret_core.cpp"
    "src/device_secret_esp.cpp"
)

# Field-bus personalities: the sources are conditional, the requirements are not.
#
# idf_component_register() is evaluated twice — an early expansion pass that
# builds the component dependency graph, and a real pass that compiles. CONFIG_*
# variables exist only in the second one, so a REQUIRES entry behind an
# if(CONFIG_...) is silently dropped from the graph and the component never
# becomes available. The requirements below are therefore unconditional.
#
# That costs build time for the unselected stacks and nothing else: no source in
# this component references them, so the linker pulls no object out of their
# archives. A KNX image still contains no radio code, which is the property that
# actually matters — see the build-time guard in src/protocol_registry.c.
if(CONFIG_HABINARI_PROTOCOL_KNX)
    list(APPEND app_srcs "src/knx_service.cpp")
endif()

if(CONFIG_HABINARI_PROTOCOL_MODBUS)
    list(APPEND app_srcs "src/mb_rtu_slave.c")
endif()

if(CONFIG_HABINARI_PROTOCOL_MQTT)
    list(APPEND app_srcs "src/mqtt_service.c" "src/mqtt_payload.c")
endif()

# The out-of-band service channel is not a personality (see oob_service.h), but
# it is conditional for the same reason they are: a KNX image must not link a
# radio stack. The source guards itself as well, so this is belt and braces
# rather than the only thing standing between KNX and the BLE controller.
if(CONFIG_HABINARI_OOB_BLE)
    list(APPEND app_srcs "src/oob_ble.c")
endif()

idf_component_register(SRCS ${app_srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES
                         # Explicit rather than transitive: these used to arrive
                         # through whichever protocol component happened to be
                         # linked, which stopped being true once the
                         # personalities became optional.
                         driver
                         esp_driver_gpio
                         esp_driver_i2c
                         esp_driver_uart
                         esp_timer
                         esp_app_format
                         nvs_flash
                         efuse
                         esp_security
                         bootloader_support
                         mbedtls
                         # Sensor drivers.
                         bme68x
                         bsec2
                         hdc302x
                         scd4x
                         # Protocol personalities — see the note above on why
                         # these cannot be conditional. esp-modbus and esp-mqtt
                         # are managed components and are already requirements
                         # of main through idf_component.yml.
                         KNstaX
                         esp_wifi
                         esp_netif
                         esp_event
                         # The BLE host, for the out-of-band service channel.
                         # Unconditional for the same reason as the above; with
                         # CONFIG_BT_ENABLED off the component contributes
                         # nothing to link against, and src/oob_ble.c — the only
                         # file that references it — is not compiled.
                         bt)

set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
