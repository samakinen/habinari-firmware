# Host tests for the application-level code in main/ that has no ESP-IDF
# dependency. The firmware itself is built with idf.py; this is a plain host
# build of the portable sources only, run the same way as the KNstaX suite:
#
#   cmake -S main/test -B build_host_tests && cmake --build build_host_tests
#   ctest --test-dir build_host_tests --output-on-failure
#
# Or via tools/run_main_host_tests.sh.
cmake_minimum_required(VERSION 3.16)
# C as well as C++: the protocol adapters and their wire formats are C, and
# compiling them as C++ would mangle the symbols the tests link against.
project(habinari_main_host_tests C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

set(MAIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(REPO_ROOT "${MAIN_DIR}/..")

# Reuse the KNstaX minimal Unity header so assertions behave identically in
# both suites (notably: abort-on-first-failure via throw).
add_library(unity INTERFACE)
target_include_directories(unity INTERFACE "${REPO_ROOT}/components/KNstaX/test/unity_mock")

find_library(MBEDCRYPTO_LIBRARY mbedcrypto)
if(NOT MBEDCRYPTO_LIBRARY)
    message(FATAL_ERROR "mbedcrypto not found. Install libmbedtls-dev to build the main/ host tests.")
endif()

add_executable(test_device_secret
    test_device_secret.cpp
    "${MAIN_DIR}/src/device_secret_core.cpp"
    "${MAIN_DIR}/src/device_identity.cpp"
)
target_include_directories(test_device_secret PRIVATE "${MAIN_DIR}/include")
target_link_libraries(test_device_secret PRIVATE unity ${MBEDCRYPTO_LIBRARY})
target_compile_options(test_device_secret PRIVATE -Wall -Wextra)

add_test(NAME test_device_secret COMMAND test_device_secret)

# The KNXnet/IP routing group as an installer types it. Portable C, so the
# validation that stands between a typo and a board that never rejoins the
# network is covered here rather than only on hardware.
add_executable(test_knx_ip_addr
    test_knx_ip_addr.cpp
    "${MAIN_DIR}/src/knx_ip_addr.c"
)
target_include_directories(test_knx_ip_addr PRIVATE "${MAIN_DIR}/include")
target_link_libraries(test_knx_ip_addr PRIVATE unity)
target_compile_options(test_knx_ip_addr PRIVATE -Wall -Wextra)

add_test(NAME test_knx_ip_addr COMMAND test_knx_ip_addr)

# Room-control logic: psychrometrics, dew point / slab moisture monitors,
# setpoint ladder, ventilation and thermostat behaviour. Header-only and
# platform-free, so no extra sources are needed.
add_executable(test_hvac_control test_hvac_control.cpp)
target_include_directories(test_hvac_control PRIVATE "${MAIN_DIR}/include")
target_link_libraries(test_hvac_control PRIVATE unity)
target_compile_options(test_hvac_control PRIVATE -Wall -Wextra)

add_test(NAME test_hvac_control COMMAND test_hvac_control)

# Sensor conditioning, redundancy/cross-validation and the event detectors
# (fire, occupancy, open window). Header-only and platform-free, and driven at
# simulated time so the multi-minute behaviours run in milliseconds here.
add_executable(test_sensor_fusion test_sensor_fusion.cpp)
target_include_directories(test_sensor_fusion PRIVATE "${MAIN_DIR}/include")
target_link_libraries(test_sensor_fusion PRIVATE unity)
target_compile_options(test_sensor_fusion PRIVATE -Wall -Wextra)

add_test(NAME test_sensor_fusion COMMAND test_sensor_fusion)

# The fusion glue: which sensor backs which measurand, what each cycle exports
# into sensor_data_t, the health masks and the alarm acknowledge. The service
# around it only adds the FreeRTOS lock, so the pipeline is built on its own;
# host_stubs supplies esp_err.h for the C header it shares with the service.
add_executable(test_sensor_fusion_service
    test_sensor_fusion_service.cpp
    "${MAIN_DIR}/src/sensor_fusion_pipeline.cpp"
)
target_include_directories(test_sensor_fusion_service PRIVATE
    "${MAIN_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/host_stubs"
)
target_link_libraries(test_sensor_fusion_service PRIVATE unity)
target_compile_options(test_sensor_fusion_service PRIVATE -Wall -Wextra)

add_test(NAME test_sensor_fusion_service COMMAND test_sensor_fusion_service)

# Modbus register map: scaling, "no reading" sentinels and the register
# addresses that docs/modbus-register-map.md publishes as the wire contract.
add_executable(test_modbus_registers test_modbus_registers.cpp)
target_include_directories(test_modbus_registers PRIVATE "${MAIN_DIR}/include")
target_link_libraries(test_modbus_registers PRIVATE unity)
target_compile_options(test_modbus_registers PRIVATE -Wall -Wextra)

add_test(NAME test_modbus_registers COMMAND test_modbus_registers)

# The Modbus slave's write/read-back bookkeeping (mb_sync.h): the logic that
# lost master writes landing between the snapshot and the refresh, and ran each
# commit twice. Pulled out of mb_rtu_slave.c so those interleavings can be
# placed exactly here instead of hoped for on a bench.
add_executable(test_modbus_sync test_modbus_sync.cpp)
target_include_directories(test_modbus_sync PRIVATE
    "${MAIN_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/host_stubs"
)
target_link_libraries(test_modbus_sync PRIVATE unity)
target_compile_options(test_modbus_sync PRIVATE -Wall -Wextra)

add_test(NAME test_modbus_sync COMMAND test_modbus_sync)

# The room controller itself, driven at simulated time. This is the test that
# the multiprotocol refactor exists to make possible: before it, the control
# model lived inside knx_service.cpp behind a FreeRTOS task and a KNX stack, and
# none of it could be reached from a host build.
add_executable(test_control_core test_control_core.cpp)
target_include_directories(test_control_core PRIVATE "${MAIN_DIR}/include")
target_link_libraries(test_control_core PRIVATE unity)
target_compile_options(test_control_core PRIVATE -Wall -Wextra)

add_test(NAME test_control_core COMMAND test_control_core)

# The command gate behind control_state_write(): every adapter's writes pass
# through it, so a NaN, an infinity or an undefined code point is stopped here
# or nowhere. host_stubs supplies esp_err.h for the return codes.
add_executable(test_control_command test_control_command.cpp)
target_include_directories(test_control_command PRIVATE
    "${MAIN_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/host_stubs"
)
target_link_libraries(test_control_command PRIVATE unity)
target_compile_options(test_control_command PRIVATE -Wall -Wextra)

add_test(NAME test_control_command COMMAND test_control_command)

# MQTT wire format: the state document and the command mapping. host_stubs
# supplies esp_err.h, which is all that control_state.h needs from ESP-IDF —
# anything more would mean the code under test has a platform dependency it
# should not have.
add_executable(test_mqtt_payload
    test_mqtt_payload.cpp
    "${MAIN_DIR}/src/mqtt_payload.c"
)
target_include_directories(test_mqtt_payload PRIVATE
    "${MAIN_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/host_stubs"
)
target_link_libraries(test_mqtt_payload PRIVATE unity)
target_compile_options(test_mqtt_payload PRIVATE -Wall -Wextra)

add_test(NAME test_mqtt_payload COMMAND test_mqtt_payload)

# The out-of-band settings registry: range enforcement, the text encoding both
# ends of a service channel agree on, and secret redaction. Portable C by
# design — no NVS, no FreeRTOS, no radio — which is exactly why the properties
# every channel depends on can be pinned here rather than on a bench.
add_executable(test_device_config
    test_device_config.cpp
    "${MAIN_DIR}/src/device_config.c"
)
target_include_directories(test_device_config PRIVATE
    "${MAIN_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/host_stubs"
)
target_link_libraries(test_device_config PRIVATE unity)
target_compile_options(test_device_config PRIVATE -Wall -Wextra)

add_test(NAME test_device_config COMMAND test_device_config)

# The pure helpers the Wi-Fi and KNX services lean on: copying a credential into
# the unterminated station-config fields, and recognising the DPT 9 "invalid
# data" code. The latter is checked against KNstaX's own DPT codec, the one
# every inbound group write is decoded by, rather than a copy of its formula.
add_executable(test_service_helpers
    test_service_helpers.cpp
    "${REPO_ROOT}/components/KNstaX/src/application/dpt.cpp"
)
target_include_directories(test_service_helpers PRIVATE
    "${MAIN_DIR}/include"
    "${REPO_ROOT}/components/KNstaX/include"
)
target_link_libraries(test_service_helpers PRIVATE unity)
target_compile_options(test_service_helpers PRIVATE -Wall -Wextra)

add_test(NAME test_service_helpers COMMAND test_service_helpers)

# The browser commissioning client (docs/webui/index.html) decodes the same GATT
# characteristics by hand, in JavaScript, at fixed offsets — and nothing in any
# build links the two, so a field inserted into oob_status_t would leave it
# reading the wrong bytes with no error anywhere. This compares the two sides.
#
# It is a Python script rather than a C++ test because the thing that has to be
# read is the page, and it needs no compiler or browser to do it.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME test_webui_parity
        COMMAND "${Python3_EXECUTABLE}" "${REPO_ROOT}/tools/check_webui_parity.py")
else()
    message(WARNING "Python 3 not found: skipping test_webui_parity, which is the "
                    "only thing checking docs/webui/ against the firmware.")
endif()
