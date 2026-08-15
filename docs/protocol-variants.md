# Field-bus personalities

The board has three ways of talking to a building: KNX TP1 through the STKNX
transceiver, Modbus RTU through the SP3485EN, and MQTT over the ESP32-C6's
Wi-Fi radio. This document is about how you choose between them, why the choice
is made at build time, and what it takes to add a fourth.

---

## 1. The model

A **personality** is a mapping and nothing else. It turns `control_state_t` into
its wire format and turns wire traffic into `control_state_write()` commands. It
does not run control loops, own settings, decide anything, or know that the
other personalities exist.

Everything the device actually *is* lives one layer down:

| Layer | File | What it owns |
|---|---|---|
| Control core | `main/include/control_core.hpp` | The room controller. Platform-free, host-tested, no locking, no clock. |
| Control service | `main/src/control_service.cpp` | `Settings`, `Inputs`, `Outputs`, NVS, the mutex, the 1 Hz tick. |
| Adapters | `knx_service.cpp`, `mb_rtu_slave.c`, `mqtt_service.c` | One wire format each. |

The two contracts an adapter may use:

* **`control_state.h`** — narrow, C, flattened. Any adapter can use it, and it
  is the only thing an out-of-tree adapter should need.
* **`control_service.hpp`** — the typed `Settings`/`Inputs`/`Outputs` structs and
  the lock guard. For in-tree C++ adapters that would otherwise pay to flatten
  and re-inflate a hundred fields; the KNX adapter is the reason it exists.

A build with **no** personality at all is legal and does something sensible: it
samples, fuses, runs the control loops and logs. It just has nobody to tell.

One thing that looks like a fourth personality is not one: the **out-of-band
service channel** over BLE. It carries the settings that decide whether a
personality works at all — the Modbus address, the Wi-Fi credentials — and so it
cannot be one of them. It maps nothing, publishes nothing and is never how a
building talks to the room. See
[docs/ble-commissioning.md](ble-commissioning.md).

---

## 2. Choosing one

`menuconfig` → **Habinari protocols**, or the ready-made defaults files:

```bash
tools/build_variants.sh          # all of them
tools/build_variants.sh mqtt     # one of them
idf.py build                     # the default variant, in ./build as usual
```

The script exists because of a trap worth knowing about: **`idf.py` reads the
project-root `./sdkconfig` regardless of `-B`.** Building a second variant with
only `-B` and `-DSDKCONFIG_DEFAULTS` makes it inherit whatever the previous
variant left in that file, and the symptom is two "different" variants coming
out byte-for-byte the same size. Each variant needs its own `-DSDKCONFIG` too,
which is what the script does:

```bash
idf.py -B build_mqtt \
       -DSDKCONFIG=build_mqtt/sdkconfig \
       -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.mqtt" \
       build
```

| Symbol | Default | Transport | Power |
|---|---|---|---|
| `HABINARI_PROTOCOL_KNX` | y | STKNX, TP1, ETS-commissioned | KNX bus terminal, no aux supply |
| `HABINARI_PROTOCOL_MODBUS` | y | SP3485EN, RS-485 | either |
| `HABINARI_PROTOCOL_MQTT` | n | Wi-Fi station | **5–30 V auxiliary supply required** |

Personalities compose with the BLE service channel, which is an *overlay* rather
than a variant of its own — it is not a personality, and
[docs/ble-commissioning.md](ble-commissioning.md) says what it is instead:

```bash
tools/build_variants.sh modbus-ble   # sdkconfig.defaults.modbus + .ble
```

Measured image sizes against the 1920 kB OTA slot (`tools/build_variants.sh`,
ESP-IDF v6.1, `-Os`):

| Variant | `habinari.bin` | Slot used |
|---|---|---|
| Modbus only | 349 216 B | 17 % |
| Modbus + BLE | 776 496 B | 39 % |
| KNX TP1 + Modbus | 998 688 B | 50 % |
| MQTT + Modbus + BLE | 1 590 144 B | 80 % |

The Modbus-only image is the interesting one: at a third the size of the KNX
build it is the same device, with the same control loops and the same
`control_state_t`, and it contains no KNX stack whatsoever. That is the
structural change made visible.

The MQTT figure is the one to watch. At 80 % of the slot it fits and an OTA
still has somewhere to land, but there is no room beside it for another large
subsystem — which is a real constraint on the Matter option in §5.

---

## 3. Why KNX and the radio cannot share an image

Selecting both is a **compile error**, raised by `main/src/protocol_registry.c`,
which refuses a KNX image with `CONFIG_BT_ENABLED` set as well — the BLE service
channel is not a personality but it is a radio, and the argument below does not
care about the distinction. Two independent reasons, either of which would be
sufficient:

**Timing.** The TP1 receiver bit-bangs a 104 µs bit cell from a GPIO edge ISR
and a gptimer alarm, sampling at 52 µs into the cell. The comment in
`sdkconfig.defaults` records the measured worst-case ISR jitter on this board as
**51 µs** — one microsecond of margin before a late alarm lands past the next
cell's edge and steals its bit. That is why the gptimer ISR is IRAM-resident,
cache-safe and pinned to LEVEL3. The Wi-Fi, BLE and 802.15.4 stacks install
their own high-priority interrupts, take long critical sections, and have hard
MAC deadlines of their own. They will spend margin that is not there.

**Power.** KNX is the bus-powered personality: the device runs entirely off the
TP1 terminal, within a bus current budget of roughly 10 mA at 29 V. Wi-Fi
transmit peaks past 300 mA. The radio personalities need the auxiliary supply on
the secondary connector, which is the same connector the RS-485 installation
already uses.

Making it a build-time error rather than a runtime check is deliberate: a KNX
image does not link the radio stacks at all, so the timing guarantee is a
property of the binary rather than of a flag somebody could get wrong.

> **Note on `REQUIRES`.** `main/CMakeLists.txt` lists the protocol components
> unconditionally even though the sources are conditional.
> `idf_component_register()` is evaluated twice and `CONFIG_*` only exists in
> the second pass, so a requirement behind an `if(CONFIG_...)` is silently
> dropped from the dependency graph. The unselected stacks are therefore
> *built* but never *linked* — no source references them, so the linker pulls
> nothing out of their archives. The KNX image measured 968 kB either way.

---

## 4. Adding a personality

Three steps, and none of them touch `main.c`, the control core, or another
adapter.

**1. Write the mapping.** Read state with `control_state_get()`, write commands
with `control_state_write()`. Keep the wire format in its own translation unit
with no platform dependency, so it can be host-tested — `mqtt_payload.c` and
`main/test/test_mqtt_payload.cpp` are the worked example. `main/test/host_stubs/`
supplies the one ESP-IDF header (`esp_err.h`) that `control_state.h` needs.

**2. Define one descriptor:**

```c
const protocol_adapter_t myproto_protocol_adapter = {
    .name            = "myproto",
    .start           = myproto_start,        /* control service is already up */
    .on_control_tick = myproto_on_tick,      /* MUST NOT BLOCK — wake your task */
    .on_sensor_data  = NULL,                 /* optional */
    .identify_active = NULL,                 /* optional; drives the board LED */
    .required        = false,                /* true aborts the boot on failure */
};
```

**3. Register it** in `main/src/protocol_registry.c` behind a new Kconfig symbol
in `main/Kconfig.projbuild`, and add its sources to `main/CMakeLists.txt`.

### Rules the registry relies on

* `start()` is called after `control_service_start()`, so `control_state_get()`
  already answers. NVS is already initialised.
* `on_control_tick()` runs **on the control task**. Set a flag, notify your own
  task, return. Anything that blocks there delays the control loop.
* `on_sensor_data()` runs on the **sensor task**, with the same rule.
* An adapter with `required = false` that fails to start is logged and skipped;
  the rest of the device carries on. Only mark a personality required if a board
  wired for it is useless without it.

---

## 5. What would come next

Ranked by what the hardware and the market actually support, given the auxiliary
supply already exists.

> **Done since this list was written:** *BLE as a service channel.* It is not a
> personality and is not in this document's model at all — see
> [docs/ble-commissioning.md](ble-commissioning.md). The gap it filled was the
> circular one: the Modbus address and the Wi-Fi credentials live in NVS and
> were writable only over the bus they configure. A KNX image still has none of
> it and needs none, because ETS reaches the device through the programming
> button and never had the problem.

1. **Matter over Thread.** The highest strategic value — directly consumable by
   Apple, Google, Amazon and Home Assistant with no gateway — and the cluster
   model (Thermostat, TemperatureMeasurement, RelativeHumidityMeasurement,
   CarbonDioxideConcentrationMeasurement, AirQuality) maps cleanly onto
   `control_state_t`. Two real costs: flash, where Matter-over-Thread would be
   tight against a 1920 kB slot alongside BSEC, and per-device DAC certificates.
   The second is less daunting than it looks — the eFuse-backed per-device secret
   infrastructure in `device_secret.hpp`, built for the KNX FDSK, is the same
   shape of problem.
2. **Zigbee**, only on customer demand. Large installed base, but Thread is
   taking the direction of travel.
