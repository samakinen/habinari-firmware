# Sensor Board Firmware

This repository contains the firmware for the room air quality sensor based on the ESP32-C6.

**Note:** This project is a work in progress and is not yet fully functional. Contributions and feedback are welcome.

## Features (not all implemented yet)

### Sensors
- SCD4 (CO<sub>2</sub>)
- HDC3020 (temperature, humidity)
- BME688 (barometric pressure, gas analysis)
- Support for an external I2C probe with an SHT4x sensor (temperature, humidity)

### Communication interfaces

The board carries three: STKNX (KNX TP1), SP3485EN (RS-485) and the ESP32-C6's
own 2.4 GHz radio and USB. Which of them the firmware speaks is a **build-time
choice** — see [Field-bus personalities](#field-bus-personalities) below and
[docs/protocol-variants.md](docs/protocol-variants.md).

| Personality | Transport | Power | Kconfig |
|---|---|---|---|
| KNX TP1 | STKNX, ETS-commissioned | KNX bus, no aux supply | `SENSOR_BOARD_PROTOCOL_KNX` |
| Modbus RTU | SP3485EN, RS-485 | either | `SENSOR_BOARD_PROTOCOL_MODBUS` |
| MQTT | Wi-Fi, Home Assistant discovery | 5–30 V aux supply | `SENSOR_BOARD_PROTOCOL_MQTT` |

Alongside them, and **not** one of them, is an optional out-of-band service
channel over BLE (`SENSOR_BOARD_OOB_BLE`). It exists for the settings that
decide whether a personality works at all and therefore cannot be written over
it — the Modbus address, the Wi-Fi credentials — and it is absent from every
KNX image, where ETS already does that job through the programming button. See
[docs/ble-commissioning.md](docs/ble-commissioning.md).

### One gate for every protocol

The board has a programming button and an LED, and they mean exactly one thing:
**this device is selected**. Every commissioning path is gated on that one
protocol-neutral state — the KNX individual-address write, BLE advertising, and
the Modbus address commit. A board with its LED lit is the board that can be
commissioned, whatever protocol you reach it with, and it lapses on its own
after 15 minutes so walking away is not the same as leaving the door open.

That is also why a factory-fresh board is **silent on RS-485**: it has no
address, so any number of them can share one pair without colliding, and the
button (or a broadcast write of the serial number on the label) is what gives
one of them a name. Modbus standardises no addressing mechanism, so this is the
board's own convention — [docs/modbus-register-map.md](docs/modbus-register-map.md)
explains it and why the alternatives are worse.

## Documentation

* [docs/hvac-controller-manual.md](docs/hvac-controller-manual.md) — the
  commissioning and integration manual: what the device does, every KNX
  parameter and group object explained for the integrator, worked control
  examples, and troubleshooting.
* [docs/modbus-register-map.md](docs/modbus-register-map.md) — the Modbus RTU
  register map, scaling conventions, worked master transactions, and how a
  board with no address and no DIP switches gets one.
* [docs/protocol-variants.md](docs/protocol-variants.md) — how the field-bus
  personalities are selected and built, why KNX and the radio cannot share an
  image, and how to add a fourth protocol.
* [docs/mqtt-integration.md](docs/mqtt-integration.md) — the MQTT topic tree,
  the state document, the command topics, and how to provision Wi-Fi and broker
  credentials.
* [docs/ble-commissioning.md](docs/ble-commissioning.md) — the out-of-band
  service channel: the settings registry behind it, the GATT contract, the
  programming-mode gate and the per-device pairing passkey, the `ble_config.py`
  client, and why a KNX image has none of it.

## Architecture

```
sensor_bus / ext_probe   raw I2C acquisition, one cycle every 5 s
        │
sensor_fusion(.hpp)      conditioning, redundancy, cross-validation, trends,
        │                event detection  (platform-free, host-tested)
        ▼
   sensor_data_t          one measurement record; every value carries its validity
        │
        ▼
control_service          OWNS the device: Settings, Inputs, Outputs, NVS and the
        │                1 Hz control tick. control_core.hpp holds the room
        │                controller itself and is platform-free and host-tested.
        │
        │  control_state.h / control_service.hpp
        ▼
  protocol adapters      mappings onto a wire format, and nothing else
        ├── knx_service    KNX TP1 group objects, ETS-commissioned
        ├── mb_rtu_slave   Modbus RTU holding/input/coil/discrete registers
        └── mqtt_service   JSON state + scalar command topics, HA discovery

device_config            the settings a field bus cannot configure — its own
        │                address, its own credentials — contributed by whoever
        │                owns them  (platform-free, host-tested)
        ▼
  oob_service            one channel that renders that registry, and owns none
        └── oob_ble        of it. BLE GATT today; never a personality.
```

The device is `control_service`; the field buses are its clients. Every adapter
is optional and none of them can see another, so a build with no adapter at all
still samples, still runs its control loops and still logs — it simply has
nobody to tell.

That ordering is the point. The control model used to live *inside*
`knx_service.cpp`, which meant KNX was not one way to reach the device, it **was**
the device: turning it off left no control loop, no parameters and no NVS, and
Modbus was a passenger on a KNX stack. `control_state.h` (narrow, C, for any
adapter) and `control_service.hpp` (typed, C++, for in-tree adapters) are the two
views of the one owner. A setpoint written over Modbus or MQTT takes the
identical path a KNX telegram takes, clamping and configured limits included.

### Field-bus personalities

Personalities are **alternatives, chosen at build time**, not layers:

```bash
idf.py build                       # KNX TP1 + Modbus RTU — the default
tools/build_variants.sh            # all of them, each with its own build dir and sdkconfig
tools/build_variants.sh modbus-ble # just one
```

Use the script rather than `idf.py -B` by hand: `idf.py` defaults to the
project-root `./sdkconfig` regardless of `-B`, so a second variant built without
`-DSDKCONFIG` silently inherits the first one's configuration.

KNX and a radio **cannot** share an image, and `main/src/protocol_registry.c`
refuses to compile the Wi-Fi personality or the Bluetooth controller alongside
KNX. The TP1 receiver bit-bangs a 104 µs bit cell and samples at 52 µs, against a
measured worst-case ISR jitter of 51 µs on this board — one microsecond of
margin, which a radio stack will spend. The two are not wanted together anyway:
KNX is the bus-powered personality and the radio needs the 5–30 V auxiliary
supply.

Adding a protocol is: write the mapping, define one `protocol_adapter_t`, add it
to `protocol_registry.c` behind a Kconfig symbol. Nothing in `main.c`, the
control core or any other adapter changes.

The board carries three sensors measuring room temperature and three measuring
humidity. The fusion layer uses that overlap for fallback (a dead sensor is
replaced silently), voting (three sources publish the median, so a drifting part
cannot pull the value) and cross-validation (sources that stop agreeing raise an
alarm). The same readings, sampled fast enough to have a slope, drive advisory
rapid-temperature-rise detection, CO₂-derived occupancy and open-window
detection.

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

## Device root secret, the KNX FDSK and the BLE passkey

The device's per-device credentials are not stored anywhere. They are derived on
every boot from a 256-bit root secret in a read-protected eFuse key block:

```
FDSK    = HMAC-SHA256(root, "KNstaX/KNX-FDSK/v1"         || serial[6])[0..15]
passkey = HMAC-SHA256(root, "sensorboard/BLE-passkey/v1" || serial[6])[0..3] mod 10^6
```

The root secret is used through the HMAC peripheral, so software can never read
it back. That makes both survive `knx_service_reset_nvm()`, a full `erase-flash`
and OTA, while the same firmware image still gives every board different keys —
see `main/include/device_secret.hpp` for the reasoning.

Two labels, one root. Domain separation is what lets a second printable
credential be added without either revealing the other, and it is why the BLE
commissioning passkey did not need any new key infrastructure: it is the same
problem the FDSK already solved.

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

That suite also covers the portable application logic: the room-control loops
and psychrometrics (`test_hvac_control`), the sensor conditioning, redundancy
and event detectors (`test_sensor_fusion`), the Modbus register map's scaling
and addresses (`test_modbus_registers`), and the out-of-band settings registry
(`test_device_config`) — including the two properties every service channel
depends on and none can enforce for itself: that an out-of-range value is
refused rather than clamped, and that a secret never comes back out.
