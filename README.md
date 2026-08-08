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
