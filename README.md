# Sensor Board Firmware

This repository contains the firmware for the room air quality sensor based on the ESP32-C6.

**Note:** This project is a work in progress and is not yet fully functional. Contributions and feedback are welcome.

## Features (not all implemented yet)

### Sensors
- SCD4 (CO<sub>2</sub>)
- HDC3020 (temperature, humidity)
- BME688 (barometric pressure, gas analysis)
- Support for an external I2C probe with an SHT4x sensor (temperature, humidity)

### Communication Interfaces
- ESP32-C6 (Wi-Fi, BLE, Thread, Zigbee, USB)
- STKNX (KNX)
- SP3485EN (RS-485, MODBUS RTU)

## ETS product export

`main/include/knx_product.hpp` is the single source of truth for both the KNX
runtime and the ETS catalogue entry. Every `idf.py build` regenerates
`ets_export/sensor_board_tp1_ets.knxprod.xml` (plus the `.json` intermediate)
from it, so the two can never drift. The directory is gitignored — the export is
a build artefact, not a checked-in file.

Skip the regeneration with `idf.py -DSENSOR_BOARD_ETS_EXPORT=OFF build`, or run
it on its own with `cmake -S tools/ets_export -B build/ets_export && cmake --build build/ets_export`.

To get an actual `.knxprod` for ETS import, run the XML through OpenKNXproducer
on a host that has it installed: `tools/ets_export/package_with_openknxproducer.ps1`.

## Device root secret and the KNX FDSK

The KNX Data Secure FDSK is not stored anywhere. It is derived on every boot
from a 256-bit root secret in a read-protected eFuse key block:

```
FDSK = HMAC-SHA256(root, "KNstaX/KNX-FDSK/v1" || serial[6])[0..15]
```

The root secret is used through the HMAC peripheral, so software can never read
it back. That makes the FDSK survive `knx_service_reset_nvm()`, a full
`erase-flash` and OTA, while the same firmware image still gives every board a
different key — see `main/include/device_secret.hpp` for the reasoning.

### Provisioning is a dry run by default

`CONFIG_SENSOR_BOARD_ROOT_SECRET_BURN` is **off**, so an unprovisioned device
runs the whole path on each boot and burns nothing:

```
idf.py flash monitor
```

Look for `device_secret` lines reporting the RNG sample statistics, the
candidate root secret, and the FDSK and ETS certificate it would derive. The
candidate is discarded and regenerated every boot until burning is enabled.
While unprovisioned the device falls back to the legacy NVS-stored random tool
key, which a factory reset still replaces.

### Burning for real

Set `CONFIG_SENSOR_BOARD_ROOT_SECRET_BURN=y` and reflash. The next boot writes
the secret into `BLOCK_KEY5` (configurable) permanently, then cross-checks the
peripheral's derivation against the software one and logs the final
certificate. Print that certificate on the device label — it stays valid for
the life of the chip.

The production alternative is to burn the block from the host during flashing,
which uses the host CSPRNG rather than the chip's ADC noise:

```
espefuse.py burn_key BLOCK_KEY5 root_secret.bin HMAC_UP
```

Either way the firmware detects the provisioned block and derives from it.

### Host tests

The portable parts (the KDF, the entropy gate, the certificate encoding) are
covered by host tests:

```
tools/run_main_host_tests.sh
```
