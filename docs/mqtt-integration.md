# MQTT integration

The Wi-Fi/MQTT personality publishes the same room-controller model that the KNX
and Modbus personalities expose, as one JSON state document plus per-topic
scalar commands. Home Assistant discovery is published on connect, so the device
appears as a climate entity and a set of sensors with no YAML.

**This personality requires the 5–30 V auxiliary supply on the secondary
connector.** It cannot be built into the same image as the KNX personality — see
[protocol-variants.md](protocol-variants.md#3-why-knx-and-the-radio-cannot-share-an-image).

Build it with:

```bash
tools/build_variants.sh mqtt
```

---

## 1. Provisioning

One firmware image, many sites: credentials are **not** compiled in. They live in
the NVS namespace `netcfg`, and the adapter refuses to start without them
(logging why, and leaving the rest of the device running).

| Key | Type | Meaning |
|---|---|---|
| `ssid` | str | Wi-Fi SSID |
| `pass` | str | Wi-Fi passphrase (empty for an open network) |
| `broker` | str | Broker URI, e.g. `mqtt://10.0.0.5:1883`, `mqtts://broker.example:8883` or `wss://broker.example:443` |
| `mq_user` | str | MQTT username (optional) |
| `mq_pass` | str | MQTT password (optional) |

There are three ways to write them, and the first is the one for the field.

**Over BLE**, if the image was built with the out-of-band service channel — the
`mqtt` variant is. This is the only route that works on a board already mounted
on a wall, because every other one needs the device on a bench with a cable:

```bash
tools/ble_config.py -a AA:BB:CC:DD:EE:FF set net.ssid "Site Wi-Fi"
tools/ble_config.py -a AA:BB:CC:DD:EE:FF set net.pass "hunter2"
tools/ble_config.py -a AA:BB:CC:DD:EE:FF set net.broker "mqtt://10.0.0.5:1883"
tools/ble_config.py -a AA:BB:CC:DD:EE:FF commit
```

The passphrase is write-only: it reads back as `<set>` and cannot be recovered
from the device. See [ble-commissioning.md](ble-commissioning.md).

**From a CSV at manufacturing time:**

```csv
key,type,encoding,value
netcfg,namespace,,
ssid,data,string,MyNetwork
pass,data,string,hunter2
broker,data,string,mqtt://10.0.0.5:1883
```

```bash
python "$IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py" \
    generate netcfg.csv netcfg.bin 0x8000
esptool.py write_flash 0x9000 netcfg.bin
```

…or call `mqtt_service_provision()` **from the console** at bring-up; it stores
the settings and reboots.

---

## 2. Topics

`<base>` is `<prefix>/<device-id>`, where the prefix is
`CONFIG_HABINARI_MQTT_BASE_TOPIC` (default `habinari`) and the device id
is the last three bytes of the Wi-Fi MAC in hex. Several boards therefore
coexist under one prefix without configuration.

| Topic | Direction | QoS | Retained | Payload |
|---|---|---|---|---|
| `<base>/state` | out | 0 | yes | the JSON document below |
| `<base>/availability` | out | 1 | yes | `online` / `offline` |
| `<base>/cmd/<name>` | in | — | — | a bare scalar |

`availability` is a retained last will, so a device that drops off the network
shows as unavailable rather than frozen at its last reading.

`state` is retained and QoS 0: a late subscriber should see the current room
state immediately, and a dropped update is superseded a few seconds later —
redelivery would only ever deliver a stale reading.

Publishing is **send-on-change plus heartbeat**, the same model the KNX
transmit policy uses. The device rebuilds the document on every control tick
(1 Hz), compares it against the last one it sent, and publishes if anything
differs. If nothing has differed for `CONFIG_HABINARI_MQTT_STATE_INTERVAL_S`
seconds (default 10) it publishes anyway, so a subscriber can tell a quiet room
from a dead device. An accepted command always publishes immediately — a UI that
has just sent a setpoint is waiting to see it take effect, including the clamped
value when the device clamped it. A reconnect republishes unconditionally, since
the broker may have lost the retained document.

---

## 3. The state document

```json
{
  "temperature": 21.48, "humidity": 43.9, "co2": 812, "pressure": 100812,
  "iaq": 62, "co2_equivalent": 790, "voc_equivalent": 0.62,
  "probe_temperature": 23.10, "probe_humidity": 48.2, "air_quality_accuracy": 3,
  "dew_point": 8.71, "absolute_humidity": 8.24, "pressure_sea_level": 101325,
  "setpoint": 21.00, "setpoint_base": 21.00, "setpoint_shift": 0.00,
  "mode": "heat", "preset": "comfort", "action": "heating",
  "heating_percent": 34, "cooling_percent": 0,
  "ventilation_percent": 20, "ventilation_level": 1, "ventilation_mode": 0,
  "co2_setpoint": 900,
  "heating_request": true, "cooling_request": false,
  "ventilation_boost": false, "dehumidify_request": false,
  "window_open": false, "presence": true,
  "dew_point_alarm": false, "floor_moisture_alarm": false,
  "floor_limit_active": false, "free_cooling_available": false,
  "device_fault": false, "identify": false,
  "controller_status": 388, "air_quality_status": 0
}
```

**A measurement that is not valid is omitted, never published as zero.** The
redundancy layer can genuinely report "there is no room temperature", and a
subscriber that cannot tell that from 0 °C will heat the room to setpoint against
a fabricated reading. Absent keys are how JSON says "unknown" — which is why the
discovery templates below all use `|default('')`.

Derived values (`dew_point`, `absolute_humidity`) are omitted unless the room
temperature *and* humidity are both valid; `pressure_sea_level` unless the
pressure is.

`controller_status` is the KNX DPT 22.101 StatusRHCC word and `air_quality_status`
the IAQ bitset, both carried verbatim so the three personalities report the same
diagnostics. Their bit layouts are in `hvac_control.hpp`.

---

## 4. Commands

Published to `<base>/cmd/<name>` as a bare scalar. Surrounding whitespace is
stripped, so a broker or bridge that appends a newline does no harm.

| Topic | Payload | Effect |
|---|---|---|
| `cmd/setpoint` | number, °C | Comfort heating setpoint. Clamped to the configured min/max. |
| `cmd/setpoint_shift` | number, K | User offset. Clamped to the configured maximum shift. |
| `cmd/co2_setpoint` | number, ppm | Ventilation CO₂ setpoint. |
| `cmd/mode` | `auto` `heat` `cool` `off`, or 0–3 | Controller mode. |
| `cmd/preset` | `auto` `comfort` `standby` `eco` `away`, or 0–4 | Operating preset. |
| `cmd/ventilation_mode` | `auto` `manual` `off` `boost`, or 0–3 | Ventilation mode. |
| `cmd/power` | `ON`/`OFF`, `true`/`false`, `1`/`0` | Controller enable. |
| `cmd/window` | `open`/`closed`, or a boolean | Window contact input. |
| `cmd/presence` | boolean | Presence input. |
| `cmd/acknowledge_alarms` | anything, including empty | Clear latched alarms. |

Names are case-insensitive. Numeric code points are accepted everywhere a name
is, for clients that would rather not carry a vocabulary. A payload that parses
as neither is rejected and logged rather than silently applied as 0 — `21x` is a
mistake, and quietly setting 21 would hide it.

The names published in `state.mode` and `state.preset` are exactly the names
accepted on `cmd/mode` and `cmd/preset`; `test_mqtt_payload.cpp` asserts that
round trip, because publishing in one vocabulary and accepting another is the
classic silent integration failure.

### Worked example

```bash
mosquitto_sub -h 10.0.0.5 -t 'habinari/+/state' -v
mosquitto_pub -h 10.0.0.5 -t 'habinari/a4f2c8/cmd/setpoint' -m '21.5'
mosquitto_pub -h 10.0.0.5 -t 'habinari/a4f2c8/cmd/preset'   -m 'eco'
```

---

## 5. Home Assistant

Discovery configuration is published retained under
`CONFIG_HABINARI_MQTT_DISCOVERY_PREFIX` (default `homeassistant`) whenever
the broker session comes up:

* `homeassistant/climate/<id>/thermostat/config` — the climate entity: current
  temperature and humidity, target temperature, hvac modes
  (`off`/`heat`/`cool`/`auto`), hvac action, and presets.
* `homeassistant/sensor/<id>/<object>/config` — room temperature, humidity, CO₂,
  pressure, IAQ, dew point, floor temperature, heating demand and ventilation
  demand.

Every entity carries the same `dev` block, so Home Assistant groups them under
one device card, and the same `avty_t` availability topic.

Set the discovery prefix to an empty string to publish no discovery
configuration at all — useful when the topics are consumed by something that is
not Home Assistant.

---

## 6. TLS brokers

`mqtts://` and `wss://` are verified against the certificate bundle built into
the firmware (`CONFIG_MBEDTLS_CERTIFICATE_BUNDLE`, Mozilla's root set), which
covers any broker with a certificate from a publicly-trusted CA — Let's
Encrypt included. There is currently no NVS key for pinning a private CA, so a
broker on a self-signed or internal-CA certificate cannot be reached over TLS
yet; put it behind `mqtt://` on the local network instead, or terminate TLS
with a public-CA certificate in front of it.

## 7. Behaviour worth knowing

* **Wi-Fi reconnects forever**, not after N attempts. This is a wall-mounted
  device with no UI; an access point down for an hour must not leave it
  permanently offline.
* **The control loops never stop.** Losing Wi-Fi, the broker, or both changes
  nothing about heating, cooling or ventilation — the field bus is a way of
  reaching the device, not the thing that runs it. A board with Modbus also
  built in stays fully controllable over RS-485 throughout.
* **Modem sleep is off** (`WIFI_PS_NONE`). The board is on the auxiliary supply
  and has to answer a broker promptly; the power saved is not worth the latency.
* **The adapter is optional.** An unprovisioned board logs the reason and boots
  normally.
