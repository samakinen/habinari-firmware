# Modbus RTU Register Map

The board is a Modbus RTU slave on the RS-485 port (SP3485EN transceiver) and
exposes the **same device model as the KNX side**: the measurements, the derived
values, the controller outputs and the writable control inputs. A Modbus master
can therefore read everything a KNX installation can see, and drive the room
controller as far as a KNX installation can drive it.

Both protocols write into one shared control state, so a setpoint written over
Modbus is clamped by the same ETS limits, resolved by the same setpoint ladder
and reported back on both buses. Neither protocol is a second implementation of
the control logic.

---

## Line settings

| Setting | Default | Notes |
|---|---|---|
| Slave address | 1 | 1–247, held in NVS |
| Baud rate | 19200 | 9600 / 19200 / 38400 / 57600 / 115200, held in NVS |
| Frame | 8N1 | fixed |
| Mode | RTU, half duplex | direction control on the UART's RTS pin |

Address and baud rate are writable over Modbus — see
[Changing the line settings](#changing-the-line-settings).

## Conventions

**Addresses are zero-based register offsets** as they appear on the wire. A
master that uses 4xxxx-style entity numbers should add its usual offset.

**Values are 16-bit scaled integers, not floats.** A float spans two registers
and vendors disagree about the word order, so a float map cannot be read
reliably without knowing which end wrote it. Every scaled quantity here is ×10
(0.1 °C, 0.1 %RH, 0.1 hPa, 0.1 g/m³, 0.1 K/h), which is finer than any sensor on
the board is accurate.

**Unavailable readings have a sentinel, not zero.** The board has redundant
sensors and will report a measurement as genuinely unavailable rather than
guess:

| Register type | "No reading" value |
|---|---|
| Signed (int16) | `0x8000` (−32768) |
| Unsigned (uint16) | `0xFFFF` (65535) |

Saturated values are clamped one short of the sentinel, so out-of-range can
never be mistaken for missing. **A master must check for the sentinel before
using a value** — treating `0x8000` as −3276.8 °C will drive a control loop to
its limit.

---

## Input registers — function 04, read-only

### Identity and health (0–9)

| Addr | Name | Type | Description |
|---|---|---|---|
| 0 | Device ID | u16 | `0x5B01`. Confirm before trusting the rest of the map. |
| 1 | Map version | u16 | Currently `2`. Bumped when register meanings change. |
| 2 | Firmware version | u16 | major << 8 \| minor |
| 3 | Uptime hours | u16 | |
| 4 | Uptime seconds | u16 | seconds within the current hour |
| 5 | Sensor health mask | u16 | bit per sensor package currently delivering |
| 6 | Sensor present mask | u16 | bit per package that has ever answered (i.e. is fitted) |
| 7 | Sensor suspect mask | u16 | bit per package failing the cross-check |
| 8 | Sample count | u16 | sampling cycles completed, wraps |
| 9 | Error count | u16 | cycles with at least one bus error |

Sensor mask bits: `0` HDC3020, `1` BME688, `2` SCD4x, `3` SHT4x probe.

### Room air measurements (10–19)

| Addr | Name | Type | Unit | Description |
|---|---|---|---|---|
| 10 | Room temperature | i16 | 0.1 °C | fused across every healthy source |
| 11 | Room humidity | i16 | 0.1 %RH | fused across every healthy source |
| 12 | Air pressure | u16 | 0.1 hPa | station pressure as measured |
| 13 | Air pressure (sea level) | u16 | 0.1 hPa | altitude-reduced, comparable with a forecast |
| 14 | CO₂ | u16 | ppm | SCD4x, true CO₂ |
| 15 | Air quality index | u16 | — | BSEC IAQ, 0–500, lower is cleaner |
| 16 | CO₂ equivalent | u16 | ppm | BSEC, VOC-derived — not a CO₂ measurement |
| 17 | VOC equivalent | u16 | 0.1 ppm | BSEC breath-VOC equivalent |
| 18 | Air quality accuracy | u16 | 0–3 | BSEC calibration status; 0 = warming up |
| 19 | *reserved* | | | |

### Derived values and trends (20–29)

| Addr | Name | Type | Unit | Description |
|---|---|---|---|---|
| 20 | Room dew point | i16 | 0.1 °C | |
| 21 | Room absolute humidity | u16 | 0.1 g/m³ | |
| 22 | Probe temperature | i16 | 0.1 °C | external / in-slab probe |
| 23 | Probe humidity | i16 | 0.1 %RH | |
| 24 | Probe absolute humidity | u16 | 0.1 g/m³ | comparable with register 21 |
| 25 | Dew point margin | i16 | 0.1 K | surface temperature − dew point |
| 26 | Temperature trend | i16 | 0.1 K/h | least-squares slope over the trend window |
| 27 | CO₂ baseline | u16 | ppm | tracked fresh-air level the excess is measured against |
| 28 | Estimated occupants | u16 | — | from the CO₂ excess; indication only |
| 29 | *reserved* | | | |

### Controller state (30–44)

| Addr | Name | Type | Unit | Description |
|---|---|---|---|---|
| 30 | Active setpoint | i16 | 0.1 °C | what the controller is working to now |
| 31 | Heating setpoint | i16 | 0.1 °C | effective, after mode and shift |
| 32 | Cooling setpoint | i16 | 0.1 °C | effective, after mode and shift |
| 33 | Setpoint shift | i16 | 0.1 K | effective shift, after clamping |
| 34 | Heating control value | u16 | % | 0–100 |
| 35 | Cooling control value | u16 | % | 0–100 |
| 36 | Ventilation demand | u16 | % | 0–100 |
| 37 | Ventilation stage | u16 | — | 0 Off, 1 Low, 2 Medium, 3 High, 4 Boost |
| 38 | HVAC mode (active) | u16 | — | resolved preset: 1 Comfort, 2 Standby, 3 Economy, 4 Building protection |
| 39 | Controller mode (active) | u16 | — | resolved KNX HVACContrMode code point |
| 40 | Controller status | u16 | — | KNX DPT 22.101 StatusRHCC word |
| 41 | Air quality status | u16 | — | bitset: 0 humidity boost, 1 CO₂ boost, 2 VOC boost, 3 sensor fault, 4 dehumidify |
| 42 | Room sensor status | u16 | — | KNX DPT 21.001 StatusGen octet |
| 43 | Floor probe status | u16 | — | StatusGen octet |
| 44 | Air quality sensor status | u16 | — | StatusGen octet |

The "active" mode registers report what the controller **resolved to**, which an
open window or an absent room can move away from what was requested.

### Provenance (45–47)

| Addr | Name | Type | Description |
|---|---|---|---|
| 45 | Temperature source | u16 | sensor package backing register 10 |
| 46 | Humidity source | u16 | sensor package backing register 11 |
| 47 | Fire reason mask | u16 | bit 0 rate of rise, bit 1 over-temperature, bit 2 air-quality rise |

A source register that is not `0` (HDC3020) means the reference sensor has
failed or been voted out and a fallback is carrying the measurement. The reading
is still good; the board needs service.

---

## Discrete inputs — function 02, read-only

| Addr | Name | Description |
|---|---|---|
| 0 | Service running | |
| 1 | Device fault | room air sensing lost, or a confirmed fire alarm |
| 2 | Probe present | external probe fitted |
| 3 | Heating request | binary heat demand |
| 4 | Cooling request | binary cool demand |
| 5 | Heat/cool mode | 1 = heating, 0 = cooling |
| 6 | Enable heat | heating not blocked |
| 7 | Enable cool | cooling not blocked |
| 8 | Dew point alarm | condensation risk |
| 9 | Floor moisture alarm | damp or leak in the slab |
| 10 | Floor limit active | maximum floor temperature engaged |
| 11 | Floor comfort active | minimum floor temperature engaged |
| 12 | Free cooling available | outside air can cool this room |
| 13 | Free drying available | outside air is drier than inside |
| 14 | Ventilation boost request | |
| 15 | Dehumidify request | |
| 16 | **Fire alarm** | confirmed rapid rise or over-temperature — advisory, see below |
| 17 | Fire pre-alarm | condition present, confirmation time not yet elapsed |
| 18 | Occupancy detected | inferred from the CO₂ signal |
| 19 | Open window detected | inferred from a ventilating temperature fall |
| 20 | Sensor disagreement | redundant sensors no longer agree; one has drifted |
| 21 | Programming mode | KNX programming mode is active |

> **The fire alarm is advisory and is not a substitute for a certified fire
> detector.** It is a corroborated early hint on a bus that is already wired —
> a runaway heater, a hob left on, a fault in the underfloor circuit this board
> controls. It is neither positioned nor certified for life safety.

---

## Coils — function 01/05, read/write

| Addr | Name | Description |
|---|---|---|
| 0 | Identify LED | lights the board's LED, for finding it in an installation |
| 1 | Controller on/off | same effect as the KNX Controller On/Off object |
| 2 | Window status | report a window contact to the controller |
| 3 | Presence | report occupancy to the controller |
| 4 | Alarm acknowledge | self-clearing; clears the latched fire alarm |

Coils 1–3 read back the device's current state, whichever bus last set it.

While KNX programming mode is active it owns the LED and coil 0 has no visible
effect — that is deliberate, since the LED is what an installer looks for.

---

## Holding registers — function 03/06/16, read/write

### Device configuration (0–3)

| Addr | Name | Type | Description |
|---|---|---|---|
| 0 | Slave address | u16 | 1–247, staged |
| 1 | Baud rate code | u16 | 0 = 9600, 1 = 19200, 2 = 38400, 3 = 57600, 4 = 115200, staged |
| 2 | Config commit | u16 | write 1 to persist and apply; self-clears |
| 3 | *reserved* | | |

### Control inputs (4–11)

| Addr | Name | Type | Unit | Description |
|---|---|---|---|---|
| 4 | Comfort setpoint | i16 | 0.1 °C | the comfort heating anchor; standby/economy/cooling move with it |
| 5 | Setpoint shift | i16 | 0.1 K | user offset from the comfort setpoint |
| 6 | HVAC mode | u16 | — | 1 Comfort, 2 Standby, 3 Economy, 4 Building protection |
| 7 | Controller mode | u16 | — | KNX HVACContrMode code point |
| 8 | Ventilation mode | u16 | — | 0 Auto, 1 Manual, 2 Off, 3 Boost |
| 9 | CO₂ setpoint | u16 | ppm | ventilation setpoint |
| 10–11 | *reserved* | | | |

Every write goes through the same handling as the equivalent KNX telegram,
including the ETS-configured limits. **Read back after writing:** a setpoint
outside the configured minimum/maximum comes back clamped, and a change made
over KNX appears here too.

### Changing the line settings

Writing register 0 or 1 alone does nothing. Stage the new values, then write `1`
to register 2:

1. Write the new address to register 0 and/or the baud code to register 1.
2. Write `1` to register 2 (config commit).
3. The board acknowledges, persists the settings, waits ~100 ms and restarts its
   Modbus stack on the new settings.
4. Reconnect at the new address and rate.

The commit is a separate step because changing the address of the device you are
mid-transaction with would otherwise lose the response to the very write that
requested it. Invalid values are rejected and logged, and the previous settings
stay in force.

---

## Worked examples

**Read the room temperature and check it is real** — function 04, address 10,
1 register:

```
request : 01 04 000A 0001
response: 01 04 02 00D7      -> 0x00D7 = 215 = 21.5 °C
sentinel: 01 04 02 8000      -> no healthy temperature source
```

**Set the comfort setpoint to 22.5 °C** — function 06, address 4:

```
request : 01 06 0004 00E1    -> 225 = 22.5 °C
```

Then read register 4 back: if the ETS maximum setpoint is 22.0 °C the device
reports `220`, not `225`.

**Move the device to address 7 at 38400 baud:**

```
01 06 0000 0007    ; stage address 7
01 06 0001 0002    ; stage baud code 2 (38400)
01 06 0002 0001    ; commit
                   ; reconnect as 07 @ 38400
```

---

## Relationship to the KNX object model

| Modbus | KNX equivalent |
|---|---|
| Input registers 10–29 | Room air, derived and trend group objects |
| Input registers 30–44 | Controller output objects (FB RTC / RTSM) |
| Discrete inputs | Alarm and status objects |
| Coils 1–3 | Controller On/Off, Window Status, Presence Status inputs |
| Holding registers 4–9 | Setpoint, mode and CO₂ setpoint objects |

Configuration that shapes the control behaviour — PID gains, mode reductions,
floor limits, dew-point margins, fusion and detection thresholds — is **ETS
parameter territory and is not writable over Modbus**. Those settings are part
of the commissioned application, and a plant controller changing them behind
ETS's back would leave the project file describing a device that no longer
exists. See [hvac-controller-manual.md](hvac-controller-manual.md).
