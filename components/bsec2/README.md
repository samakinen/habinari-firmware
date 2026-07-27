# bsec2 — BME688 air-quality via Bosch BSEC

Turns the BME688's raw gas resistance into meaningful **IAQ**, **CO2-equivalent**
and **breath-VOC-equivalent** outputs using Bosch's BSEC fusion library.

The raw gas resistance on its own is an uncompensated, drifting ohms reading and
is **not** an air-quality metric — BSEC does the baseline tracking, temperature/
humidity compensation and calibration that make it usable.

## Why the library is not in this repo

`libalgobsec.a` is a **proprietary Bosch binary** covered by a license that
**forbids redistribution**. It is download-gated behind Bosch's license
agreement, so it cannot be committed here. `components/bsec2/bosch/` is
git-ignored; you install the files locally.

When `CONFIG_BME688_USE_BSEC` is **off** (the default), this component compiles
as inert stubs and the firmware builds with no external dependency and no
air-quality output.

## Installing the BSEC library (one-time, local)

1. Download **BSEC 2.x** (tested against v2.6.1.0) from Bosch Sensortec after
   accepting their license agreement.
2. The ESP32-C6 has **no official BSEC archive**. Use the **esp32c3** build —
   the C6 and C3 are both RISC-V RV32IMAC / soft-float and the archive is
   ABI-compatible (community-verified). Inside the BSEC package find the
   `esp32c3` (or generic RISC-V soft-float) `libalgobsec.a`.
3. Lay the files out like this (paths are what `CMakeLists.txt` expects):

   ```
   components/bsec2/bosch/
     lib/libalgobsec.a          <- the esp32c3 RISC-V archive
     include/bsec_interface.h
     include/bsec_datatypes.h
     include/bsec_interface_multi.h   (if present in your package)
   ```

4. Enable it:

   ```
   idf.py menuconfig
     -> Component config -> BME688 air quality (BSEC)
        [*] Enable Bosch BSEC air-quality fusion for BME688
   ```

   (or set `CONFIG_BME688_USE_BSEC=y` in `sdkconfig.defaults`).

5. `idf.py build`. If the library is missing while the option is on, the build
   fails fast with a message pointing back here.

## Config options

- **Sample rate** — LP (~3 s, responsive, default) vs ULP (~300 s, battery).
- **State save interval** — how often BSEC calibration state is persisted to NVS
  so the baseline and accuracy survive reboots (0 = never).

## Notes / things to validate against your BSEC package

The real integration in `bsec_integration.c` (the `CONFIG_BME688_USE_BSEC`
branch) is written against the **documented BSEC 2.x API** and Bosch's reference
`bsec_integration` control loop. Confirm against the exact headers in your
package:

- output IDs (`BSEC_OUTPUT_IAQ`, `..._CO2_EQUIVALENT`, `..._BREATH_VOC_EQUIVALENT`,
  `..._SENSOR_HEAT_COMPENSATED_TEMPERATURE/HUMIDITY`),
- `bsec_bme_settings_t` / `bsec_input_t` / `bsec_output_t` field names,
- `BSEC_MAX_STATE_BLOB_SIZE`, `BSEC_MAX_WORKBUFFER_SIZE`, `BSEC_NUMBER_OUTPUTS`,
  `BSEC_MAX_PHYSICAL_SENSOR`, `BSEC_SAMPLE_RATE_LP/ULP`,
- whether your version wants a `bsec_set_configuration()` blob (for a specific
  IAQ/voltage/duration profile) before subscription.
