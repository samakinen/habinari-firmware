idf_component_register(
    SRCS "bsec_integration.c"
    INCLUDE_DIRS "include"
    REQUIRES bme68x driver
    PRIV_REQUIRES nvs_flash esp_timer
)

# The proprietary Bosch BSEC library is only linked when explicitly enabled. It
# is NOT redistributable and is therefore not committed to this repository — the
# user drops it into components/bsec2/bosch/ (see README.md). When disabled the
# component builds as inert stubs so the firmware still compiles with no external
# dependency.
if(CONFIG_BME688_USE_BSEC)
    set(BSEC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bosch")
    # ESP32-C6 has no official BSEC archive; the esp32c3 build (same RISC-V
    # RV32IMAC, soft-float) is ABI-compatible and is the community-verified
    # choice. Point BSEC_DIR/lib at that libalgobsec.a.
    if(NOT EXISTS "${BSEC_DIR}/lib/libalgobsec.a")
        message(FATAL_ERROR
            "CONFIG_BME688_USE_BSEC is ON but the BSEC library was not found at "
            "${BSEC_DIR}/lib/libalgobsec.a. See components/bsec2/README.md for how "
            "to obtain and install it (it is not committed — Bosch forbids "
            "redistribution).")
    endif()
    target_include_directories(${COMPONENT_LIB} PRIVATE "${BSEC_DIR}/include")
    target_link_libraries(${COMPONENT_LIB} PRIVATE "${BSEC_DIR}/lib/libalgobsec.a")
endif()
