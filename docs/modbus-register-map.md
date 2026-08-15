# Modbus RTU Register Map

The board is a Modbus RTU slave on the RS-485 port (SP3485EN transceiver) and
exposes the **same device model as the KNX side**: the measurements, the derived
values, the controller outputs and the writable control inputs. A Modbus master
can therefore read everything a KNX installation can see, and drive the room
controller as far as a KNX installation can drive it.

Every protocol writes into one shared control state, owned by `control_service`,
so a setpoint written over Modbus is clamped by the same limits, resolved by the
same setpoint ladder and reported back on every bus. No protocol is a second
implementation of the control logic.

Modbus is one of the board's field-bus **personalities**, selected at build time
(`SENSOR_BOARD_PROTOCOL_MODBUS`, on by default). It combines with any other —
RS-485 is a UART with a direction pin and competes with nothing — and it is the
only personality that works on both KNX bus power and the 5–30 V auxiliary
supply. A Modbus-only image carries no KNX stack at all and still controls the
room. See [protocol-variants.md](protocol-variants.md).

---

## Line settings

| Setting | Default | Notes |
|---|---|---|
| Slave address | **none** | 1–246 once assigned; 0 means unassigned, held in NVS |
| Baud rate | 19200 | 9600 / 19200 / 38400 / 57600 / 115200, held in NVS |
| Frame | 8N1 | fixed |
| Mode | RTU, half duplex | direction control on the UART's RTS pin |

**A board out of the box has no address and is silent.** It answers nothing
until it has been commissioned. That is not an inconvenience to work around, it
is the point — see [Commissioning](#commissioning) below, which is also how the
address gets written.

Address 247 is reserved as the commissioning address and is never assignable.

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
| 1 | Map version | u16 | Currently `3`. Bumped when register meanings change. |
| 2 | Firmware version | u16 | major << 8 \| minor |
| 3 | Uptime hours | u16 | |
| 4 | Uptime seconds | u16 | seconds within the current hour |
| 5 | Sensor health mask | u16 | bit per sensor package currently delivering |
| 6 | Sensor present mask | u16 | bit per package that has ever answered (i.e. is fitted) |
| 7 | Sensor suspect mask | u16 | bit per package failing the cross-check |
| 8 | Sample count | u16 | sampling cycles completed, wraps |
| 9 | Error count | u16 | cycles with at least one bus error |

Sensor mask bits: `0` HDC3020, `1` BME688, `2` SCD4x, `3` SHT4x probe.

> **Map version 3** added the serial number at 48–50, the serial-select
> registers at 12–14, and the unassigned-address convention in
> [Commissioning](#commissioning). Nothing was renumbered, so an integration
> built against version 2 keeps working — but a device out of the box no longer
> answers at address 1, because it no longer has one.

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

### Serial number (48–51)

| Addr | Name | Type | Description |
|---|---|---|---|
| 48 | Serial 0 | u16 | `serial[0] << 8 \| serial[1]` |
| 49 | Serial 1 | u16 | `serial[2] << 8 \| serial[3]` |
| 50 | Serial 2 | u16 | `serial[4] << 8 \| serial[5]` |
| 51 | *reserved* | | |

The six bytes are the device's base MAC — the same identity the KNX device
object reports, the BLE channel advertises, and the box label carries. Readable
rather than only printable because a master that has just found a board on the
commissioning address needs to know which one it is before naming it.

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
| 0 | Slave address | u16 | 1–246, or 0 to unassign; staged |
| 1 | Baud rate code | u16 | 0 = 9600, 1 = 19200, 2 = 38400, 3 = 57600, 4 = 115200, staged |
| 2 | Config commit | u16 | write 1 to persist and apply; self-clears |
| 3 | *reserved* | | |

A commit is **refused unless the device is selected** — see
[Commissioning](#commissioning).

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

### Serial select (12–15)

| Addr | Name | Type | Description |
|---|---|---|---|
| 12 | Serial select 0 | u16 | `serial[0] << 8 \| serial[1]` |
| 13 | Serial select 1 | u16 | `serial[2] << 8 \| serial[3]` |
| 14 | Serial select 2 | u16 | `serial[4] << 8 \| serial[5]` |
| 15 | *reserved* | | |

Write a device's serial number here to select it for commissioning. The
selection arms for **30 seconds**, during which that device — and only that
device — accepts a config commit. Writing zeros releases it, as does the commit
itself.

Selection is detected as a *change* to these registers, because a memory-mapped
register area has no write callback and rewriting the same value looks like
nothing happening. To re-arm an expired selection, write zeros and then the
serial again.

Every write goes through the same handling as the equivalent KNX telegram,
including the ETS-configured limits. **Read back after writing:** a setpoint
outside the configured minimum/maximum comes back clamped, and a change made
over KNX appears here too.

---

## Commissioning

**Modbus standardises no way to give a device an address.** DALI has randomised
binary-search arbitration, M-Bus has secondary addressing over fabrication
numbers, CANopen has LSS, KNX has the programming button. Modbus has DIP
switches and vendor conventions, and this is ours.

The spec offers exactly two things worth building on, and both are used here:

* **address 0 is broadcast** — every slave processes the frame, none reply, so a
  write to it can never collide however many devices hear it;
* **247 is by convention a service address**, and 248–255 are reserved.

### The three states

| Assigned address | Programming mode | Answers to | Accepts a commit |
|---|---|---|---|
| none | no | *nothing* (listens on address 0) | no |
| none | yes | 247, the commissioning address | yes |
| 1–246 | no | its own address | only if serial-selected |
| 1–246 | yes | its own address | yes |

"Silent" means the slave is configured with address 0. It matches only frames
addressed to 0, and the spec forbids answering those — so it **hears every
broadcast and can never transmit**. That distinction is load-bearing: a device
that had simply stopped its UART would be unreachable by any means at all,
including the one mechanism meant to reach devices nobody can get to.

This is what lets any number of factory-fresh boards hang on one pair from the
first day. Two devices sharing an address both reply to a request for it, the
frames overlap on the wire, and the master gets a CRC error from a bus it cannot
talk its way out of. Devices that never transmit cannot do that to each other.

An **already-addressed** device keeps its address even while selected. Moving it
to 247 would drop it off a live bus for as long as somebody leant on the button,
and the master already knows where to find it.

### Selecting a device

Two ways, and they exist for two different situations.

**The programming button**, when somebody can reach the device. Hold it for a
second; the LED lights. This is the same state, button and LED that KNX
individual addressing and the BLE service channel use — a board with its LED lit
is the board that can be commissioned, whatever protocol you reach it with. It
lapses on its own after 15 minutes.

**The serial number**, when nobody can — a board already above a ceiling. Write
its serial to registers 12–14 over **broadcast**; only the matching device arms.
Because a broadcast needs no reply, this reaches a device that is silent for want
of an address, and no two devices can collide answering it.

### Assigning an address

Writing register 0 or 1 alone does nothing. Stage the values, then commit:

1. Select the device (button, or a serial-select write).
2. Write the new address to register 0 and/or the baud code to register 1.
3. Write `1` to register 2 (config commit).
4. The board persists, waits ~100 ms and restarts its Modbus stack on the new
   settings.
5. Reconnect at the new address and rate.

The commit is a separate step because changing the address of the device you are
mid-transaction with would otherwise lose the response to the very write that
requested it. Invalid values are rejected and logged, and the previous settings
stay in force.

A commit from an unselected device is **refused**. Reachable is not the same as
selected: without that rule a stray or mistargeted master write could re-address
a device in a running building, and the first anyone would know is a BMS point
going dark.

Writing address `0` unassigns the device and returns it to the silent state —
which is what to do with a board being pulled out of one installation and into
another.

### On a KNX build

The Modbus address is a Modbus problem, so it is solved here, but a KNX+Modbus
image has the same button and the same programming mode driving both. Pressing
it once selects the device for ETS *and* for RS-485 addressing.

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

**Commission a fresh board with the button** — hold the programming button until
the LED lights, then talk to the commissioning address:

```
F7 04 0030 0003    ; read serial from 247, registers 48-50
F7 04 06 84F7 03A1 B2C3
                   ; -> 84:F7:03:A1:B2:C3, the serial on the box label
F7 06 0000 0007    ; stage address 7
F7 06 0001 0002    ; stage baud code 2 (38400)
F7 06 0002 0001    ; commit
                   ; reconnect as 07 @ 38400
```

**Commission a board nobody can reach**, by serial, over broadcast. Nothing
replies to any of these — confirmation is the read at the end:

```
00 10 000C 0003 06 84F7 03A1 B2C3   ; select 84:F7:03:A1:B2:C3
00 10 0000 0003 06 0007 0002 0001   ; address 7, 38400, commit
                                    ; every other board ignores both frames
07 04 0030 0003                     ; confirm: read the serial back at address 7
```

**Move an addressed device to another address** — it is already reachable, so
select it by its own serial rather than walking to it:

```
01 10 000C 0003 06 84F7 03A1 B2C3   ; select this device
01 06 0000 0007                     ; stage address 7
01 06 0002 0001                     ; commit
                                    ; reconnect as 07
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
