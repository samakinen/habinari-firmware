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

    # The BSEC tuning blob (also non-redistributable, also dropped in locally).
    # Detected here rather than with __has_include in the source: ccache cannot
    # know that a previously absent file has appeared, so it would keep serving
    # the object compiled without the blob. A compile definition changes the
    # command line, which ccache does hash. CONFIGURE_DEPENDS re-runs the glob
    # on every build, so adding the blob later triggers a reconfigure by itself.
    file(GLOB BSEC_CONFIG_BLOB CONFIGURE_DEPENDS "${BSEC_DIR}/config/bsec_config.inc")
    if(BSEC_CONFIG_BLOB)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE BSEC_HAVE_CONFIG=1)
    else()
        message(STATUS
            "bsec2: no configuration blob at ${BSEC_DIR}/config/bsec_config.inc — "
            "IAQ/CO2-eq/VOC-eq will stay at zero (see components/bsec2/README.md).")
    endif()
endif()
