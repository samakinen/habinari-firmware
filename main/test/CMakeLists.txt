# Host tests for the application-level code in main/ that has no ESP-IDF
# dependency. The firmware itself is built with idf.py; this is a plain host
# build of the portable sources only, run the same way as the KNstaX suite:
#
#   cmake -S main/test -B build_host_tests && cmake --build build_host_tests
#   ctest --test-dir build_host_tests --output-on-failure
#
# Or via tools/run_main_host_tests.sh.
cmake_minimum_required(VERSION 3.16)
project(sensor_board_main_host_tests CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# Modbus register map: scaling, "no reading" sentinels and the register
# addresses that docs/modbus-register-map.md publishes as the wire contract.
add_executable(test_modbus_registers test_modbus_registers.cpp)
target_include_directories(test_modbus_registers PRIVATE "${MAIN_DIR}/include")
target_link_libraries(test_modbus_registers PRIVATE unity)
target_compile_options(test_modbus_registers PRIVATE -Wall -Wextra)

add_test(NAME test_modbus_registers COMMAND test_modbus_registers)
