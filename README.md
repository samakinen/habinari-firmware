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
