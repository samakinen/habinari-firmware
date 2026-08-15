# Room HVAC Sensor/Controller TP1 — Commissioning and Integration Manual

**Order number:** SBTP1 · **Application program:** number 21, version 4 ·
**Manufacturer ID:** 0x00FA · **Medium:** KNX TP1 · **Group objects:** 61 (No. 0–60)

This manual describes the **KNX TP1 personality** of the board, which is the
default firmware variant and the only one that runs on KNX bus power alone. The
same device model is also available over Modbus RTU
([modbus-register-map.md](modbus-register-map.md)) and over MQTT
([mqtt-integration.md](mqtt-integration.md)); which personalities an image
contains is a build-time choice, described in
[protocol-variants.md](protocol-variants.md).

This manual is written for the person **commissioning** the device in ETS and
wiring it into an installation. It describes what the device does, what every
parameter and every group object means, and how to link the device into typical
KNX installations. Firmware internals are only mentioned where they change what
you see on the bus.

---

## Table of contents

1. [The device in one page](#1-the-device-in-one-page)
2. [Physical installation and first power-up](#2-physical-installation-and-first-power-up)
3. [Getting the product into ETS](#3-getting-the-product-into-ets)
4. [How the device works](#4-how-the-device-works)
5. [Group object reference](#5-group-object-reference)
6. [Parameter reference](#6-parameter-reference)
7. [Control behaviour in detail](#7-control-behaviour-in-detail)
8. [Integration examples](#8-integration-examples)
9. [Recommended group address structure](#9-recommended-group-address-structure)
10. [Diagnostics and troubleshooting](#10-diagnostics-and-troubleshooting)
11. [Commissioning checklist](#11-commissioning-checklist)
12. [Appendix A — default values](#appendix-a--default-values)
13. [Appendix B — pitfalls worth knowing before you start](#appendix-b--pitfalls-worth-knowing-before-you-start)

---

## 1. The device in one page

The board is a **room HVAC sensor and single-room controller** for KNX TP1. It
combines, in one device on one bus connection:

**Measurement (sensor side)**

| Quantity | Sensor | Notes |
|---|---|---|
| Room air temperature | HDC3020 | Correctable offset (self-heating) |
| Room relative humidity | HDC3020 | Correctable offset |
| CO₂ | SCD4x | True NDIR CO₂, not an estimate |
| Barometric pressure | BME688 | Published as measured *and* reduced to sea level |
| Air quality (IAQ, CO₂-equivalent, breath-VOC-equivalent, accuracy) | BME688 + BSEC | Only on firmware built with BSEC |
| Floor / slab temperature and humidity | External SHT4x probe | Optional, plugs into the probe connector |
| Dew point, absolute humidity (room and slab) | derived on board | Real psychrometrics, not a lookup |

**Control (controller side)**

* Full room temperature controller: heating and/or cooling, continuous PI or
  two-point, with the standard KNX operating modes (Comfort / Standby / Economy
  / Building protection) and the KNX setpoint ladder.
* Floor temperature limitation (protects the floor covering) and optional floor
  comfort tempering (a bathroom floor that must not feel cold).
* Condensation (dew point) protection for floor cooling and chilled ceilings,
  using the room air *and* a real surface temperature.
* Slab moisture / leak detection from the in-slab probe.
* Demand-controlled ventilation from CO₂, humidity and VOC, with mode override,
  stage output and a separate dehumidification request.
* Diagnostics: per-sensor status octets, a roll-up device fault, and the
  standard DPT 22.101 room-controller status word.

**Redundancy and inference (what the overlapping sensors buy)**

* Three sensors measure room temperature and three measure humidity. A failed
  sensor is replaced automatically; three healthy ones outvote a drifting one;
  and sensors that stop agreeing raise a cross-check alarm — a fault no
  single-sensor device can see. See [4.4](#44-redundancy-and-derived-events).
* Advisory rapid-temperature-rise / over-temperature detection, optionally
  corroborated by the gas sensor. **Not a certified fire detector** — read
  [5.11](#511-derived-events) before using it.
* Occupancy from CO₂ and open-window detection from air change, each used as the
  controller's presence/window input only where no real contact is linked.

In KNX terms the device implements the S-Mode HVAC Functional Blocks of KNX
Volume 7/19/20: **RTS** (room temperature sensor), **RRHS** (room humidity),
**RAQS** (air quality), **FTS** (floor temperature), **DPS** (dew point status),
**RTSM** (setpoint manager) and **RTC** (room temperature controller). It has no
local HMI — every input is a group object, so scheduling, presence, window
contacts and setpoint adjustment come from wherever your project puts them.

**Other interfaces.** The board also runs a Modbus RTU slave on RS-485 (default
address 1, 19200 baud 8N1) exposing the **same device model**: the same
measurements, derived values and controller outputs, and the same writable
setpoints, modes and inputs. Both protocols drive one shared control state, so
neither is a second implementation. It needs no ETS configuration — ignore it if
you do not use it. Wire-level details: [modbus-register-map.md](modbus-register-map.md).

---

## 2. Physical installation and first power-up

### Connections

| Connection | Purpose |
|---|---|
| KNX TP1 bus terminal | Power and communication. No auxiliary supply needed. |
| Probe connector (I²C) | Optional SHT4x floor/slab probe, powered by the board. |
| RS-485 terminal | Optional Modbus RTU. |
| USB-C | Console/diagnostics and firmware update. |

### Mounting notes that affect measurement quality

* Mount at the usual room-sensor height, away from direct sun, draughts, and
  heat sources. The board dissipates a little power itself, so the reading tends
  to run **above** the true room temperature — that is what the *Room
  temperature correction* parameter is for (see [§6.1](#61-measurements-and-sending-behaviour)).
* The floor probe belongs in a conduit **inside the slab**, in the heated zone,
  not under the covering edge. Its humidity channel is what gives you slab
  moisture and leak detection.
* Compare the board against a calibrated reference **after at least an hour** of
  bus power, then set the correction. Correcting before the device has reached
  thermal equilibrium bakes the warm-up transient into the offset.

### Button and LED

The device has one button (PROG) and one LED:

| Action | Result |
|---|---|
| Press and hold ≥ 1 s | Toggle KNX programming mode (LED on while active) |
| Press and hold ≥ 5 s | **Factory reset**: erases all stored KNX state (individual address, group addresses, parameters, security keys) and reboots |

The LED is lit whenever programming mode is active.

### First power-up

On the first boot the device is uncommissioned: it holds the initial individual
address (15.15.255), publishes nothing to the bus, and waits for ETS. Sensors
start sampling immediately (every 30 s) and the control loops run internally, so
by the time you finish downloading, the device has valid readings to send.

To spread the load after a bus-wide power failure, the device **holds its first
publish for a random delay of up to 8 s** and limits itself to 5 unsolicited
telegrams per second thereafter. A short quiet period after power-up is normal
and not a fault.

---

## 3. Getting the product into ETS

### 3.1 The product file

The ETS catalogue entry is generated from the firmware, so it can never drift
from what the device actually does. Each firmware build regenerates
`ets_export/sensor_board_tp1_ets.knxprod.xml`. To obtain an importable
`.knxprod`, run that XML through OpenKNXproducer on a host that has it
installed:

```powershell
tools/ets_export/package_with_openknxproducer.ps1
```

Then in ETS: **Catalog → Import…** and select the resulting `.knxprod`.

> Always import the product file that belongs to the firmware on the device.
> ETS cross-checks the hardware/application identity before it allows a
> download; a mismatch shows up as a refused or failing download rather than as
> misbehaviour later.

### 3.2 Data Secure

**The shipped firmware runs with KNX Data Secure enabled.** Security Mode is on
from the first boot, which means all management (configuration) access must be
secured. In practice:

* In ETS, the device must be added to the project **with its device
  certificate**. Without it, ETS cannot write the individual address, the
  parameters or the group addresses — and because unauthorised management writes
  are silently discarded (as the KNX specification requires), the symptom in ETS
  is a **timeout**, not an explicit "access denied".
* The device certificate is printed on the USB console at every boot, e.g.

  ```
  KNX Data Secure device certificate (enter in ETS): XXXXXX-XXXXXX-XXXXXX-XXXXXX-XXXXXX-XXXXXX
  ```

  together with the KNX serial number. Enter it in ETS under
  **Add device certificate** (or scan it if your board carries a printed label).
* Group communication itself is **not** forced to be secure: the product
  declares no per-object security requirement, so you can leave group
  communication plain or switch it on per group address, as your project
  requires.

If a device has already been commissioned once, ETS installs its own tool key
and the factory certificate is no longer the key in force. A factory reset (5 s
button press) returns the device to its factory key.

### 3.3 Download

1. Put the device into programming mode (long-press the button, LED lights) or
   use ETS's own mechanism.
2. Download the individual address, then the application.
3. After the download the device becomes operational and starts publishing.

**After every download** the device logs, on the console, which transmitting
objects your project left unlinked:

```
CO #43 "Cooling Control Value" has no group address - not transmitted
7 transmitting communication object(s) unlinked in this project
```

This is the fastest way to tell "deliberately unused" from "forgotten link".

---

## 4. How the device works

### 4.1 Signal flow

```
  HDC3020  ─┐                                  ┌─► Room T / RH / dew point / abs. humidity
  SCD4x    ─┤  sample every 5 s                ├─► CO₂, IAQ, CO₂-eq, VOC-eq, pressure
  BME688   ─┤   → fusion: filter, redundancy,  │
  SHT4x    ─┘     cross-check, trends →        ├─► Floor T / RH / abs. humidity, moisture alarm
                        │      → offsets →     ├─► Fire / occupancy / open-window events
                        ▼                      │
   bus inputs ──►  ┌─────────────────────┐     │
   (mode, presence,│  control tick, 1 Hz │─────┤
    window, shift, │                     │     │
    outside T/RH,  │ • preset resolution │     ├─► Heating / cooling control value (0–100 %)
    flow T,        │ • setpoint ladder   │     ├─► Heating / cooling demand (on/off)
    changeover,    │ • PI / two-point    │     ├─► Heat/cool mode, enable states, StatusRHCC
    enables,       │ • floor limit       │     ├─► Dew point alarm + margin
    dew point in)  │ • dew point monitor │     ├─► Ventilation demand / stage / boost
                   │ • slab moisture     │     ├─► Dehumidification request
                   │ • ventilation       │     └─► Device fault + per-sensor status
                   └─────────────────────┘
                        │
                        ▼
              transmit policy (COV / min. interval / heartbeat) ──► bus
```

* **Sensors are sampled every 5 s.** The control loops run **once per second**
  on the most recent readings. Sampling fast costs I2C traffic and nothing on
  the bus: what reaches KNX is decided much later by the transmit policy
  ([4.2](#42-when-the-device-sends-the-transmit-model)). It is what makes
  averaging, rate-of-change and fault detection work at all.
* **Readings are conditioned before anything sees them** — plausibility window,
  spike rejection, and exponential smoothing over the *measurement smoothing
  time constant*. This is the oversampling: noise is averaged out here rather
  than by publishing raw values less often, so a 0.2 K send-on-change threshold
  responds to the room instead of to sensor noise.
* Correction offsets are applied **once**, at the source, so the controller, the
  derived values and the published measurements all use the same number.
* ETS parameter changes take effect within one control tick — no restart needed.

### 4.2 When the device sends (the transmit model)

Every transmitting object obeys the KNX sensor Functional Block sending model,
configured by three parameters in *Measurements and sending behaviour*:

| Mechanism | Parameter | Default | Behaviour |
|---|---|---|---|
| Send on change (COV) | the per-measurand *"… change to send"* parameters | varies | An analogue value is sent when it has moved by at least this much since the last send. Discrete values (alarms, modes, status words, control values) are sent on **any** change. |
| Minimum interval | *Minimum time between transmissions* | 10 s | No object sends more often than this, however fast the value moves. |
| Heartbeat | *Cyclic retransmission (heartbeat)* | 900 s | Every object re-sends its current value at least this often, so a watchdog or a visualisation restart always recovers. 0 disables cyclic sending. |

In addition, the device limits itself to **5 unsolicited telegrams per second**
across all objects, and delays its first publish after power-up by a random
0–8 s. This keeps a whole floor of devices from talking at once after a bus
restart.

**Read requests** are answered by every transmitting object (the R flag is set),
from the same values the control loops use, so a read and a spontaneous send can
never disagree.

### 4.3 What survives a restart, and what does not

| State | Survives power cycle? |
|---|---|
| Individual address, group addresses, parameters, security keys | **Yes** |
| Base setpoint, CO₂ setpoint written over the bus | **No** — re-seeded from the ETS parameters at boot |
| Controller On/Off, HVAC operating mode, controller mode | **No** — re-seeded from the *"… after download"* parameters |
| Setpoint shift, ventilation mode, window/presence/outside/flow inputs | **No** — start from their defaults, inputs stay "unknown" until a telegram arrives |

Design your project so those runtime values are re-established after a restart:
a scheduler with cyclic sending, a visualisation that re-sends on reconnect, or
sending devices (window contacts, presence detectors, weather station)
configured to transmit cyclically. The device does **not** issue read requests of
its own at startup.

### 4.4 Redundancy and derived events

Three of the four sensors measure room temperature and humidity. The device uses
that overlap in three ways — see [5.12](#512-redundancy-and-cross-validation)
for the objects:

| Behaviour | What it does | What it costs |
|---|---|---|
| **Fallback** | A failed sensor is replaced by the next healthy one, silently, within the staleness window | Nothing. The room keeps its measurement; object 61 shows what died |
| **Voting** | With three healthy sources the median is published, so a drifting part cannot pull the value | Nothing |
| **Cross-check** | Sources that stop agreeing raise object 62 | A tolerance set too tight nags on healthy parts |

The same readings, sampled fast enough to have a slope, also support three
inferences (objects 63–68):

* **Rapid temperature rise** — advisory heat detection, confirmed over time and
  optionally corroborated by the BME688 gas signal. Read the warning in
  [5.11](#511-derived-events) before binding it to anything.
* **Occupancy from CO₂** — people are the only meaningful indoor CO₂ source. If
  no presence detector is linked, this drives the controller's presence input;
  a linked PIR always wins.
* **Open window from air change** — a temperature fall corroborated by CO₂
  falling too. If no window contact is linked, this drives the window input;
  a linked contact always wins.

Both inferences defer to a real bus object the moment one arrives, because an
installation that paid for contacts should get the contacts' answer.

---

## 5. Group object reference

Flags shown as ETS uses them: **C** communication, **R** read, **W** write,
**T** transmit, **U** update.

### 5.1 Room air measurements (FB RTS / RRHS / RAQS)

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 0 | Room Air Temperature | 9.001 | C R T | Measured room temperature, with the correction offset applied. This is also the controller's process value. |
| 1 | Room Relative Humidity | 9.007 | C R T | Room RH with correction applied. |
| 2 | Room CO₂ (SCD4x) | 9.008 | C R T | True measured CO₂ in ppm. Prefer this over object 6 wherever both exist. |
| 3 | Air Pressure (station) | 9.006 | C R T | Barometric pressure as measured at the installation. |
| 4 | Air Pressure (sea level) | 9.006 | C R T | Pressure reduced to mean sea level using the *Installation altitude* parameter. Use this one for weather-style visualisation and for comparing sites. |
| 5 | Air Quality Index (BSEC IAQ 0…500) | 7.001 | C R T | BSEC indoor air quality index; **lower is cleaner** (≈ 0–50 excellent, 100 moderate, > 200 poor). |
| 6 | CO₂ Equivalent (BSEC) | 9.008 | C R T | *Estimated* CO₂-equivalent from the VOC sensor. Not a CO₂ measurement. |
| 7 | Breath-VOC Equivalent (BSEC) | 9.008 | C R T | Estimated breath-VOC concentration. |
| 8 | Air Quality Accuracy (BSEC 0…3) | 5.010 | C R T | BSEC calibration state: 0 = not calibrated (ignore IAQ), 1 = low, 2 = medium, 3 = high. Rises over days of operation. |

### 5.2 Derived room air values

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 9 | Room Dew Point | 9.001 | C R T | Temperature at which room air starts to condense. The reference for any cooled surface in the room. |
| 10 | Room Absolute Humidity | 9.029 | C R T | Moisture **content** in g/m³. Comparable between air at different temperatures, which relative humidity is not. |

### 5.3 Floor probe and slab moisture (FB FTS)

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 11 | Floor Temperature | 9.001 | C R T | Slab temperature from the external probe (with correction). |
| 12 | Floor Slab Relative Humidity | 9.007 | C R T | RH inside the slab conduit. |
| 13 | Floor Slab Absolute Humidity | 9.029 | C R T | Moisture content in the slab, directly comparable with object 10. |
| 14 | Floor Slab Moisture Alarm | 1.005 | C R T | 1 = the slab is damp: either the RH threshold was exceeded, or the slab holds measurably more moisture than the room (a leak in progress). |
| 15 | Max Floor Temperature Limit Active | 1.011 | C R T | 1 = heating is currently inhibited because the floor reached its maximum temperature. |
| 16 | Min Floor Temperature Active | 1.011 | C R T | 1 = the controller is heating to keep the floor comfortably warm, independently of room demand. |

### 5.4 Condensation protection (FB DPS)

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 17 | Dew Point Alarm | 1.005 | C R T | 1 = a monitored surface is at or near the room dew point. Blocks cooling if configured to. |
| 18 | Dew Point Margin (surface − dew point) | 9.002 | C R T | Headroom in K before condensation. Positive is safe. Reads 0 when there is no surface reference at all (no floor probe and no flow temperature) — that is "unknown", not "0 K left". |
| 19 | Dew Point Alarm Input (external sensor) | 1.005 | C W U | Accepts a condensation alarm from a system-wide dew point sensor. Treated exactly like the locally derived alarm. |

### 5.5 System and neighbour inputs

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 20 | Outside Temperature Input | 9.001 | C W U | From a weather station. Used only for the free-cooling flag. |
| 21 | Outside Relative Humidity Input | 9.007 | C W U | From a weather station. Used only for the free-drying flag. |
| 22 | Cooling Flow Temperature Input | 9.001 | C W U | Flow (or coldest wetted surface) temperature of the cooling system. Feeds the dew point monitor. Link this if you cool with a chilled ceiling or fan coil. |
| 23 | Free Cooling Available (outside air) | 1.011 | C R T | 1 = outside air is more than 1 K cooler than the room **and** the room is above its cooling setpoint. Drive window/airing signalling or an AHU free-cooling mode. |
| 24 | Free Drying Available (outside air) | 1.011 | C R T | 1 = outside air holds measurably less moisture than the room (compared as absolute humidity, so it is honest in winter). |

### 5.6 Mode and setpoints (FB RTSM)

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 25 | Controller On/Off | 1.001 | C R W T U | Master enable. 0 switches all heating/cooling outputs off (measurements keep running). |
| 26 | HVAC Operating Mode Input | 20.102 | C W U | Requested mode: 0 Auto, 1 Comfort, 2 Standby, 3 Economy, 4 Building protection. This is the object a scheduler or central mode manager writes. |
| 27 | HVAC Operating Mode Status | 20.102 | C R T | The mode the controller actually **resolved** to, after window, presence and protection priorities. Never reports Auto. |
| 28 | Controller Mode Input | 20.105 | C W U | Requested direction: 0 Auto, 1 Heat, 3 Cool, 6 Off (other code points are treated as Auto). |
| 29 | Controller Mode Status | 20.105 | C R T | The direction the controller is actually working in. |
| 30 | Controller Mode for Secondary Controller | 20.105 | C R T | Same value as 29, meant to be linked to a *second* controller's mode input in the same room so the two can never fight (KNX main/secondary RTC). |
| 31 | Base Setpoint (Comfort heating) | 9.001 | C R W T U | The comfort heating setpoint — the anchor of the whole setpoint ladder. Writing it moves standby, economy and the cooling setpoints with it. This is the object a visualisation's "target temperature" belongs on. Clamped to the min/max setpoint parameters. |
| 32 | Setpoint Shift Input | 9.002 | C W U | Manual adjustment in K, applied on top of whatever preset is active. Clamped to ± *Maximum manual setpoint shift*. |
| 33 | Setpoint Shift Status | 9.002 | C R T | The shift actually applied after clamping — link this to a wall unit so it can show the truth when the user runs into the limit. |
| 34 | Active Setpoint Status | 9.001 | C R T | The setpoint the controller is working to right now (heating or cooling side, whichever is active). |
| 35 | Effective Heating Setpoint | 9.001 | C R T | Current heating setpoint, published unconditionally. |
| 36 | Effective Cooling Setpoint | 9.001 | C R T | Current cooling setpoint. Together with 35 this lets a visualisation draw the dead band. |

### 5.7 Room inputs (FB WOS / PRD / WCOS)

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 37 | Window Contact Input | 1.019 | C W U | 1 = window open. Behaviour is set by the *Behaviour with window open* parameter. Unknown (never received) is treated as closed. |
| 38 | Presence Input | 1.018 | C W U | 1 = occupied. Once the **first** telegram has been received, presence **overrides** the operating mode input: occupied → Comfort, unoccupied → Standby or Economy per parameter. Do not link this if you want the scheduler alone to decide. |
| 39 | Heating Enable Input | 1.003 | C W U | 0 disables the heating sequence entirely (e.g. central summer shutdown). Defaults to enabled if unlinked. |
| 40 | Cooling Enable Input | 1.003 | C W U | 0 disables the cooling sequence entirely (e.g. central winter shutdown). |
| 41 | Water Changeover Input (1 = Heat) | 1.100 | C W U | Tells the room which medium the two-pipe system currently circulates. Only used when *Heat/cool changeover* = "Follow changeover input"; polarity is configurable. |

### 5.8 Controller outputs (FB RTC)

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 42 | Heating Control Value | 5.001 | C R T | 0–100 % for a continuous valve actuator or a PWM/heating-actuator input. |
| 43 | Cooling Control Value | 5.001 | C R T | 0–100 % for the cooling actuator. |
| 44 | Heating Demand | 1.001 | C R T | Binary heat demand: for a thermoelectric valve driven on/off, or as a boiler/heat-pump demand signal. |
| 45 | Cooling Demand | 1.001 | C R T | Binary cool demand — chiller enable, changeover request, and so on. |
| 46 | Heat/Cool Mode Status (1 = Heat) | 1.100 | C R T | 0 only while actively cooling; neutral and heating both report 1 (as DPT 22.101 defines). |
| 47 | Heating Enabled Status | 1.003 | C R T | 0 while heating is blocked (window, floor limit, mode, changeover delay, off). |
| 48 | Cooling Enabled Status | 1.003 | C R T | 0 while cooling is blocked (dew point alarm, window, mode, changeover delay, off). |
| 49 | Controller Status (StatusRHCC) | 22.101 | C R T | The standard 16-bit room-controller status word. See [§10.2](#102-decoding-the-controller-status-word-object-49). |

### 5.9 Ventilation and air quality

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 50 | CO₂ Setpoint | 9.008 | C R W T U | The CO₂ level at which ventilation demand starts. Writable at runtime; falls back to the parameter value after a restart. |
| 51 | Ventilation Demand | 5.001 | C R T | 0–100 % continuous demand — for a damper actuator or an AHU with a percentage input. |
| 52 | Ventilation Stage | 5.010 | C R T | Bucketed stage for staged fans: 0 = Off, 1 = Low (1–33 %), 2 = Medium (34–66 %), 3 = High (67–99 %), 4 = Boost (100 %). |
| 53 | Ventilation Mode | 5.010 | C R W T U | 0 = Auto (demand-controlled), 1 = Manual (fixed percentage from the parameter), 2 = Off, 3 = Boost (100 %). Writable from a wall switch or visualisation. |
| 54 | Ventilation Boost Request | 1.001 | C R T | 1 while demand is at 100 %. Convenient one-bit signal for a simple extract fan. |
| 55 | Dehumidification Request | 1.001 | C R T | 1 while the humidity channel alone is calling for action. A distinct service from air change — this is what a dehumidifier or a reversible heat pump should follow. |
| 56 | Air Quality Status (bitset) | 7.001 | C R T | Why ventilation is running. Bit 0 humidity, bit 1 CO₂, bit 2 sensor fault, bit 3 VOC, bit 4 dehumidification requested. |

### 5.10 Diagnostics

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 57 | Device Fault | 1.005 | C R T | Roll-up alarm: 1 when the room air sensor (the one the control loops depend on) is not delivering valid readings. A missing floor probe is **not** a device fault. |
| 58 | Room Sensor Status | 21.001 | C R T | DPT_StatusGen octet for the HDC3020 package. |
| 59 | Floor Probe Status | 21.001 | C R T | StatusGen for the external probe. Bit 0 (out of service) = probe not fitted; bit 1 (fault) = fitted but not reading; bit 3 (in alarm) = slab moisture alarm. |
| 60 | Air Quality Sensor Status | 21.001 | C R T | StatusGen for the SCD4x/BME688 package. |
| 61 | Sensor Package Health (bitmask) | 5.010 | C R T | One bit per physical sensor currently delivering readings: bit 0 HDC3020, bit 1 BME688, bit 2 SCD4x, bit 3 SHT4x probe. The StatusGen objects say whether a *measurement* is healthy; this says which *part* is, which is what a service visit needs. |
| 62 | Sensor Cross-Check Alarm | 1.005 | C R T | 1 = sensors measuring the same quantity no longer agree, so one of them has drifted. See [5.12](#512-redundancy-and-cross-validation). |

### 5.11 Derived events

These are **inferences from the measurements, not measurements**. Each can be
wrong, so each says so in its name. Bind them where a wrong answer is
recoverable, and read [4.4](#44-redundancy-and-derived-events) first.

| No. | Object | DPT | Flags | Description |
|---|---|---|---|---|
| 63 | Rapid Temperature Rise / Fire Alarm (advisory) | 1.005 | C R T | 1 = a confirmed rapid temperature rise or over-temperature. **Advisory only — not a certified fire detector** (see below). Latched; cleared by object 69 or after the *auto-clear* period. |
| 64 | Rapid Temperature Rise Pre-Alarm | 1.005 | C R T | 1 = the condition is present but the confirmation time has not yet elapsed. Useful for logging and for tuning the thresholds; not worth waking anyone. |
| 65 | Room Temperature Trend (K/h) | 9.002 | C R T | Least-squares slope of the room temperature. Negative = falling. A heating loop that cannot move this is oversized, undersized or fighting an open window. |
| 66 | Occupancy Detected (from CO₂) | 1.018 | C R T | 1 = the CO₂ signal says the room is occupied. Unlike a PIR this does not miss someone sitting still. |
| 67 | Estimated Occupants (indication only) | 5.010 | C R T | Rough headcount from the CO₂ excess. Never used for control; for visualisation. |
| 68 | Open Window Detected (from air change) | 1.019 | C R T | 1 = a ventilating temperature fall, corroborated by CO₂ falling at the same time. |
| 69 | Alarm Acknowledge | 1.016 | C W | Write 1 to clear the latched fire alarm. |

> **The fire alarm is advisory.** An air-quality board in a room corner is in
> the wrong place and has the wrong response time for life safety, and nothing
> here is certified to EN 54. What it is worth is an early, corroborated hint on
> a bus that is already wired — a runaway heater, a hob left on, a fault in the
> underfloor circuit this board controls — minutes before anyone smells it. Do
> not let it replace a smoke detector, and do not bind it to anything that would
> be dangerous if it fired spuriously.

### 5.12 Redundancy and cross-validation

The board carries **three sensors that measure room temperature** (HDC3020,
BME688, SCD4x) and three that measure humidity. Objects 0 and 1 report the fused
result:

* **Fallback.** If the HDC3020 stops answering, the BME688 takes over, then the
  SCD4x. The room keeps its measurement and its control loop; object 61 shows
  which parts are alive.
* **Voting.** With three healthy sources the **median** is published, so a
  single drifting part stops affecting the value at all rather than being
  averaged into it.
* **Cross-check.** Sources differing by more than the configured limit raise
  object 62. With two sources there is no way to tell which one is lying, so the
  preferred one is kept and the disagreement is reported.

The external probe is deliberately **not** a room-air fallback: it measures a
different place, and using the slab temperature to control the room would be a
plausible-looking number of the wrong quantity.

---

## 6. Parameter reference

Parameters are grouped exactly as they appear in the ETS parameter view. Some
parameters and whole sections are hidden until a related setting enables them —
noted below.

### 6.1 Measurements and sending behaviour

| Parameter | Default | Range | What it does |
|---|---|---|---|
| Cyclic retransmission (heartbeat) | 900 s | 0–65535 | How often every transmitting object re-sends regardless of change. 0 = no cyclic sending. Shorten it if a visualisation or logic module needs frequent refreshes; lengthen it on a busy line. |
| Minimum time between transmissions | 10 s | 0–3600 | Floor on the sending rate of each object. Raise it if a noisy measurement produces bursts. |
| Room temperature correction | 0 K | −10…+10 | Added to the measured room temperature **before** everything else. Set it from a settled comparison against a reference instrument; on a wall-mounted board that also powers a TP1 transceiver, a small negative correction is normal. |
| Room temperature change to send | 0.2 K | 0–10 | COV threshold for objects 0 (and the setpoint status objects). |
| Room humidity correction | 0 % | −20…+20 | Added to the measured room RH. |
| Room humidity change to send | 2 % | 0–50 | COV threshold for object 1. |
| CO₂ change to send | 25 ppm | 0–5000 | COV threshold for objects 2, 6, 7 and 50. |
| Air pressure change to send | 50 Pa | 0–10000 | COV threshold for objects 3 and 4. |
| Air quality index change to send | 5 | 0–500 | COV threshold for object 5. |
| Floor temperature correction | 0 K | −10…+10 | Offset for the probe channel. |
| Floor temperature change to send | 0.2 K | 0–10 | COV threshold for object 11. |
| Floor humidity change to send | 2 % | 0–50 | COV threshold for object 12. |
| Dew point / absolute humidity change to send | 0.2 | 0–10 | COV threshold for the derived objects 9, 10, 13 and 18. |
| Installation altitude above sea level | 0 m | 0–4000 | Used to reduce station pressure to sea level (object 4). At 0 m, object 4 equals object 3. |

### 6.2 Room control — general

| Parameter | Default | Options | What it does |
|---|---|---|---|
| Controller state after download | On | Off / On | Initial value of object 25 after a download or a power cycle. |
| HVAC operating mode after download | Comfort | Auto / Comfort / Standby / Economy / Building protection | Initial value of the operating mode until a scheduler writes object 26. |
| Controller mode after download | Auto | Auto / Heat / Cool / Off | Initial value of the direction until object 28 is written. |
| Heating sequence | Used | Not used / Used | When "Not used", the heating loop never runs and the *Heating control* section is hidden. |
| Cooling sequence | **Not used** | Not used / Used | Cooling is **off by default**. Enable it before you expect anything on objects 43/45, and to reveal the *Cooling control* section and the cooling setpoint parameters. |
| Heat/cool changeover | Automatic (internal dead band) | Automatic / Follow changeover input / Fixed heating only / Fixed cooling only | How the device decides direction when the controller mode is Auto. "Automatic" lets the dead band between the heating and cooling setpoints separate the two. "Follow changeover input" is the right choice on a **two-pipe** system where a central valve decides what is in the pipe. |
| Changeover input polarity | 1 = heating | 1 = heating / 1 = cooling | Only visible with "Follow changeover input". Set it to match the sending device. |
| Minimum delay between heating and cooling | 300 s | 0–65535 | After running one direction, the other is held off for this long. Prevents a room from oscillating between heating and cooling, and protects reversible plant. |
| Behaviour with window open | Switch outputs off | Ignore / Building protection setpoint / Switch outputs off | "Switch outputs off" is the usual choice for radiators and floor loops; "Building protection setpoint" keeps frost protection working in cold climates with long airing periods. |
| Mode when the room is unoccupied | Standby | Standby / Economy | Which preset presence = 0 falls back to. Standby for rooms re-entered often, Economy for rooms empty for hours. |
| Behaviour on sensor failure | Switch outputs off | Hold last outputs / Switch outputs off / Keep controlling on last reading | What happens if the room sensor stops delivering. This same setting also governs the **floor probe** — see the warning in [Appendix B](#appendix-b--pitfalls-worth-knowing-before-you-start). |

### 6.3 Setpoints

The device follows the KNX setpoint model: **Comfort is the absolute anchor**,
Standby and Economy are shifts away from it, and only building protection is
absolute again. One write to object 31 therefore moves the whole ladder
coherently.

| Parameter | Default | Range | What it does |
|---|---|---|---|
| Comfort heating setpoint | 21 °C | 5–40 | The anchor. Also the power-up value of object 31. |
| Standby: reduction below comfort | 2 K | 0–15 | Heating setpoint in Standby = comfort − this. |
| Economy: reduction below comfort | 4 K | 0–20 | Heating setpoint in Economy = comfort − this. |
| Building protection: frost setpoint | 7 °C | 3–15 | Absolute frost protection setpoint. |
| Dead band between heating and cooling | 2 K | 0–10 | Comfort cooling setpoint = comfort heating + this. Never set it to 0 on a system that can do both: the dead band is what stops heating and cooling chasing each other. |
| Standby: increase above comfort cooling | 2 K | 0–15 | Visible when cooling is used. |
| Economy: increase above comfort cooling | 4 K | 0–20 | Visible when cooling is used. |
| Building protection: heat setpoint | 35 °C | 25–45 | Absolute overheating protection setpoint. Visible when cooling is used. |
| Lowest allowed setpoint | 7 °C | 3–25 | Hard clamp on any resulting setpoint, including bus writes and shifts. |
| Highest allowed setpoint | 35 °C | 15–45 | Hard clamp, as above. |
| Maximum manual setpoint shift | 3 K | 0–10 | Limits how far a user can move the setpoint with object 32. |

**Worked example** with the defaults and cooling enabled:

| Preset | Heating setpoint | Cooling setpoint |
|---|---|---|
| Comfort | 21.0 °C | 23.0 °C |
| Standby | 19.0 °C | 25.0 °C |
| Economy | 17.0 °C | 27.0 °C |
| Building protection | 7.0 °C | 35.0 °C |

Write 22 °C to object 31 and every row moves with it (Comfort 22/24, Standby
20/26, Economy 18/28); building protection stays where it is, which is what you
want.

### 6.4 Heating control *(visible when the heating sequence is used)*

| Parameter | Default | Range | What it does |
|---|---|---|---|
| Heating control algorithm | Continuous PI | Two-point (on/off) / Continuous PI | PI for continuous or PWM actuators; two-point for a simple on/off valve driven directly by object 44. |
| Heating proportional gain | 25 %/K | 1–200 | Expressed as %/K: gain = 100 / proportional band. 25 %/K = a 4 K band. Visible for PI only. |
| Heating reset time (0 = P only) | 9000 s | 0–65535 | Integral time. Long values suit heavy systems (a heated slab); shorter values suit fast ones (radiators, fan coils). |
| Heating derivative time (0 = off) | 0 s | 0–65535 | Usually left at 0 for room temperature. |
| Heating minimum control value | 0 % | 0–100 | When the output is above 0 it is lifted to at least this value — useful for actuators with a minimum useful opening. 0 means no minimum. |
| Heating maximum control value | 100 % | 0–100 | Caps the output, e.g. to limit an oversized emitter. |

**Starting points** (adjust after observation, one parameter at a time):

| System | Gain | Reset time |
|---|---|---|
| Underfloor heating (screed) | 12–25 %/K (4–8 K band) | 6000–12000 s |
| Radiators, thermoelectric valves | 25–50 %/K (2–4 K band) | 1800–3600 s |
| Fan coil / air heating | 50–100 %/K (1–2 K band) | 600–1200 s |

### 6.5 Cooling control *(visible when the cooling sequence is used)*

Identical in shape to the heating section: algorithm, gain, reset time,
derivative time, minimum and maximum control value. The cooling loop is
reverse-acting — output rises as the room rises above the cooling setpoint.
Chilled ceilings behave like floor heating (slow, long reset time); fan coils
are fast.

### 6.6 Control loop behaviour

| Parameter | Default | Range | What it does |
|---|---|---|---|
| Two-point switching hysteresis | 0.5 K | 0–5 | Heating switches on at setpoint − h and off at setpoint + h (mirrored for cooling). Used by the two-point algorithm and by the hysteresis-based demand objects. Too small = valve chatter; too large = noticeable temperature swing. |
| Binary demand derived from | Temperature hysteresis | Temperature hysteresis / Control value threshold | How objects 44/45 are produced. Use "Control value threshold" when the binary demand is really a *plant* request (boiler/chiller enable) that should follow the continuous control value. |
| Binary demand control-value threshold | 1 % | 1–100 | Visible with the threshold strategy: demand is 1 while the control value is at or above this. |
| Frost alarm below (0 = off) | 5 °C | 0–20 | Sets bit 13 of the status word (object 49). |
| Overheat alarm above (0 = off) | 35 °C | 0–60 | Sets bit 14 of the status word. |

### 6.7 Floor temperature

| Parameter | Default | Range | What it does |
|---|---|---|---|
| Maximum floor temperature (0 = off) | 28 °C | 0–45 | Heating is inhibited while the probe reads at or above this, and released again one hysteresis below it. This protects the floor covering — parquet and some vinyls typically want 27–29 °C. **Set to 0 if no floor probe is fitted** (see [Appendix B](#appendix-b--pitfalls-worth-knowing-before-you-start)). |
| Minimum floor temperature (0 = off) | 0 °C (off) | 0–35 | Floor comfort tempering: heat to keep the slab at least this warm even when the room does not ask for it. Typical bathroom value 22–24 °C. |
| Floor temperature hysteresis | 1 K | 0–10 | Applies to both the maximum and the minimum limit. |
| Control value while holding minimum floor temperature | 30 % | 0–100 | The output used while tempering the floor (unless room demand is already higher). |

### 6.8 Condensation protection

| Parameter | Default | Options / range | What it does |
|---|---|---|---|
| Surface temperature used for condensation check | Coldest available | Off / Floor probe / Flow temperature input / Coldest available | Which surface is compared against the room dew point. Floor cooling condenses on the slab; a chilled ceiling or fan coil condenses on whatever the flow temperature (object 22) describes. "Coldest available" uses whichever of the two is present and lower. |
| Alarm when surface is within … of dew point | 2 K | 0–10 | Alarm threshold. |
| Dew point alarm hysteresis | 1 K | 0–10 | The alarm clears once the margin exceeds threshold + hysteresis (default: alarm at ≤ 2 K, clear at ≥ 3 K). |
| Cooling during a dew point alarm | Block cooling | Continue (report only) / Block cooling | Keep "Block cooling" for any surface cooling system. "Continue" is only sensible where condensation cannot occur (e.g. air cooling with a dehumidified supply) and you only want the reporting. |

### 6.9 Slab moisture detection

Two independent detectors, because they catch different failures.

| Parameter | Default | Range | What it does |
|---|---|---|---|
| Slab humidity alarm above (0 = off) | 85 % | 0–100 | Absolute RH threshold inside the slab: catches slow, cumulative damp — a screed that never dried, or a leak that has already saturated the conduit. |
| Slab humidity alarm hysteresis | 5 % | 0–25 | Alarm at ≥ threshold, clears at ≤ threshold − hysteresis. |
| Alarm when slab is wetter than the room by (0 = off) | 2 g/m³ | 0–20 | Compares **absolute** humidity, so it works even though the slab is colder than the room. Liquid water evaporating into the conduit drives this up long before the relative reading looks alarming — this is the leak-in-progress detector. |

### 6.10 Ventilation and air quality

| Parameter | Default | Range | What it does |
|---|---|---|---|
| CO₂ setpoint (demand starts here) | 900 ppm | 400–5000 | Demand begins above this value. Also the power-up value of object 50. |
| CO₂ band to full demand | 400 ppm | 0–5000 | Demand ramps linearly to 100 % over this band (default: 0 % at 900 ppm, 100 % at 1300 ppm). A band of 0 makes the channel act as a pure on/off boost. |
| Humidity setpoint (demand starts here) | 65 % | 0–100 | Above this, humidity produces ventilation demand **and** the dehumidification request (object 55). |
| Humidity band to full demand | 15 % | 0–50 | Default: 100 % demand at 80 % RH. |
| Air quality index setpoint (0 = off) | 150 | 0–500 | BSEC IAQ level at which the VOC channel starts to call for air. Set to 0 on boards without BSEC. |
| Air quality index band to full demand | 150 | 0–500 | Default: 100 % demand at IAQ 300. |
| Base ventilation while the room is occupied | 0 % | 0–100 | Minimum automatic demand whenever presence is 1. Set it where a minimum air change is required during occupancy. Needs object 38 linked. |
| Demand in manual ventilation mode | 50 % | 0–100 | The fixed demand used when object 53 = 1 (Manual). |

The three channels run in parallel and **the worst one wins** — any single
pollutant above its setpoint justifies air change.

### 6.11 Sensor fusion and detection

| Parameter | Default | Range | What it does |
|---|---|---|---|
| Measurement smoothing time constant | 30 s | 0–600 | Exponential filter on the room air measurements. This is the oversampling knob: the board samples every 5 s and averages here. Raise it on a draughty installation where the temperature flickers; lower it if a fast-reacting emitter needs the controller to see changes sooner. 0 = no smoothing. |
| Temperature sensor disagreement limit | 1.5 K | 0–20 | How far the three temperature sensors may differ before object 62 is raised. With three healthy sources the outlier is also voted out of object 0. 0 = no cross-checking. |
| Humidity sensor disagreement limit | 6 % | 0–50 | The same for the humidity sensors. Wider because humidity sensors legitimately disagree more. |
| Rapid rise alarm threshold | 4 K/min | 0–30 | Rate of temperature rise that raises the advisory fire alarm, once sustained for the confirmation time. 0 = the rate path is off. Only armed above 28 °C, so a morning warm-up cannot trigger it. |
| Over-temperature alarm threshold | 55 °C | 0–120 | Fixed-temperature path, independent of the rate. 0 = off. |
| Rapid rise confirmation time | 30 s | 5–600 | How long the condition must persist. This is what a hairdryer or a sunbeam crossing the sensor fails to survive. |
| Also require rising air pollution | No | No / Yes | When Yes, a rate-of-rise alarm additionally needs the BME688 gas signal to be climbing. Combustion produces volatiles; a fan heater does not. The strongest false-alarm defence here — but it needs a calibrated BSEC, which takes hours after a cold boot, so leave it off until air quality accuracy (object 8) is reading 2 or 3. |
| Derive occupancy from CO₂ | Enabled | Enabled / Disabled | Produces objects 66 and 67, and drives the presence input when no presence detector is linked. |
| Detect open windows from air change | Enabled | Enabled / Disabled | Produces object 68, and drives the window input when no window contact is linked. Disable it in a room with a permanently open interior door where the CO₂ corroboration cannot work. |

---

## 7. Control behaviour in detail

### 7.1 Which mode wins

Each control tick, the active preset is resolved in this order — the first
matching rule wins:

1. **Room sensor invalid** → the *Behaviour on sensor failure* parameter decides
   (default: outputs off, both directions reported blocked, temperature alarms
   suppressed because there is no trustworthy reading behind them).
2. **Window open** and behaviour = "Building protection setpoint" → Building
   protection.
3. **Building protection requested** on object 26 → Building protection.
4. **Presence known** (object 38 has been received at least once):
   occupied → Comfort; unoccupied → Standby or Economy per parameter.
5. **Operating mode input** (object 26), if it is not Auto.
6. Otherwise **Comfort**.

Independently of the preset, the outputs are forced off when: the controller is
off (object 25 = 0), the controller mode is Off, or a window is open and the
behaviour is "Switch outputs off".

> Note the consequence of rule 4: if you link a presence detector, it takes
> precedence over your scheduler. Link it only where that is what you want.

### 7.2 From setpoint to output

1. The ladder gives a heating and a cooling setpoint for the active preset.
2. The setpoint shift (object 32), clamped to ± the maximum shift, is added to both.
3. Both are clamped to the min/max setpoint parameters, and the dead band is
   re-opened if the clamp squeezed them together.
4. Direction gating: controller mode, changeover configuration, the heating and
   cooling enable inputs, the dew point block and the minimum changeover delay
   decide whether each direction may run at all.
5. The permitted loops compute a control value (PI or two-point).
6. Binary demands are derived (hysteresis or threshold strategy).
7. **Heating wins**: if heating is active at all, cooling is forced to 0 —
   the two can never be commanded together.
8. Floor comfort tempering may raise the heating output to its configured value.
9. The floor **limit** overrides everything on the heating side: output 0.
10. Minimum/maximum output clamps are applied, and the result is published.

### 7.3 Free cooling and free drying

* **Free cooling** (object 23) is 1 when the outside temperature is more than
  1 K below the room **and** the room is above its cooling setpoint — i.e.
  opening up would actually help.
* **Free drying** (object 24) is 1 when outside air holds at least 0.5 g/m³ less
  moisture than room air. Because the comparison is on absolute humidity, cold
  and "90 % RH" outside air is correctly recognised as dry.

Both require the outside temperature (and, for drying, humidity) inputs to be
linked and receiving.

### 7.4 Ventilation

Demand = the largest of the CO₂, humidity and VOC ramps, then raised to the base
demand if the room is occupied. Mode overrides the result: Off → 0 %, Manual →
the fixed value, Boost → 100 %. The stage output buckets the percentage for
staged fans, and the boost/dehumidify bits give simple one-bit consumers
something to follow.

---

## 8. Integration examples

The group addresses below are examples in a `Main/Middle/Sub` structure; use
whatever your project standard is.

### 8.1 Underfloor heating, one room, continuous valve actuator

The most common case: a heating actuator that takes a 1-byte control value, a
scheduler that sets the operating mode, a window contact and a visualisation.

| GA | Name | Device object | Partner |
|---|---|---|---|
| 3/1/1 | Living room / room temperature | 0 (T) | Visualisation, logic |
| 3/1/2 | Living room / base setpoint | 31 (R W T U) | Visualisation, wall unit |
| 3/1/3 | Living room / operating mode | 26 (W) | Scheduler / central mode |
| 3/1/4 | Living room / operating mode status | 27 (T) | Visualisation |
| 3/1/5 | Living room / heating control value | 42 (T) | Heating actuator, channel 1 |
| 3/1/6 | Living room / window | 37 (W) | Window contact |
| 3/1/7 | Living room / floor temperature | 11 (T) | Visualisation |
| 3/1/8 | Living room / controller status | 49 (T) | Visualisation, diagnostics |

Parameters:

* Heating sequence **Used**, cooling **Not used**.
* Heating algorithm **Continuous PI**, gain ≈ 12–25 %/K, reset time 6000–12000 s.
* Maximum floor temperature 27–29 °C (a probe **is** fitted here).
* Behaviour with window open: **Switch outputs off**.
* Cyclic retransmission 900 s so the actuator's own watchdog stays happy.

Do not link the presence input unless the room really has a detector — if you
do, it overrides the scheduler.

### 8.2 Heating and cooling with a two-pipe system and a chilled surface

A central changeover valve decides whether the pipes carry hot or cold water;
the room must follow it, and must never cool a surface below the dew point.

| GA | Name | Device object | Partner |
|---|---|---|---|
| 3/2/1 | Office / room temperature | 0 | — |
| 3/2/2 | Office / base setpoint | 31 | Wall unit |
| 3/2/5 | Office / heating control value | 42 | Actuator, heating channel |
| 3/2/6 | Office / cooling control value | 43 | Actuator, cooling channel |
| 3/2/7 | Office / changeover (1 = heat) | 41 (W) | Central changeover status |
| 3/2/8 | Office / flow temperature | 22 (W) | Flow sensor / plant controller |
| 3/2/9 | Office / dew point alarm | 17 (T) | Visualisation, plant controller |
| 3/2/10 | Office / dew point margin | 18 (T) | Visualisation |
| 3/2/11 | Office / cooling demand | 45 (T) | Chiller / plant enable |

Parameters:

* Heating sequence **Used**, cooling sequence **Used**.
* Heat/cool changeover: **Follow changeover input**; set the polarity to match
  the central status.
* Surface temperature for condensation check: **Flow temperature input**
  (or **Coldest available** if a slab probe is also fitted).
* Cooling during a dew point alarm: **Block cooling**.
* Minimum delay between heating and cooling: 300 s or more.

To share one dew point sensor across a building, link every room's object 19 to
the central sensor's alarm instead of (or in addition to) the local derivation.

### 8.3 Demand-controlled ventilation

| GA | Name | Device object | Partner |
|---|---|---|---|
| 4/1/1 | Meeting room / CO₂ | 2 (T) | Visualisation |
| 4/1/2 | Meeting room / ventilation demand | 51 (T) | AHU or damper actuator (0–100 %) |
| 4/1/3 | Meeting room / ventilation stage | 52 (T) | Staged extract fan |
| 4/1/4 | Meeting room / ventilation mode | 53 (R W T U) | Wall button, visualisation |
| 4/1/5 | Meeting room / boost | 54 (T) | Simple on/off fan |
| 4/1/6 | Meeting room / CO₂ setpoint | 50 (R W T U) | Visualisation |
| 4/1/7 | Meeting room / air quality status | 56 (T) | Diagnostics |
| 4/1/8 | Meeting room / dehumidify request | 55 (T) | Dehumidifier / heat pump |

Parameters: CO₂ setpoint 800–1000 ppm with a 400 ppm band is a good starting
point for a meeting room (100 % at 1200–1400 ppm). Set the humidity setpoint to
65 % for comfort humidity control, or to 100 % to disable the humidity channel.
Set the air quality index setpoint to 0 unless the firmware includes BSEC.

Use **either** object 51 (continuous) **or** object 52/54 (staged), matching the
ventilation unit — linking all of them is harmless but produces bus traffic
nobody reads.

### 8.4 Sensor-only installation

If the room's heating is controlled elsewhere (a central controller, a
thermostat, another device), commission the board as a pure sensor: link objects
0–18 and 57–60 and leave the controller objects unlinked. The controller keeps
running internally, but with nothing linked it publishes nothing — the unlinked
list in the console log after download confirms it.

Consider setting *Controller state after download* = **Off** to make the intent
explicit.

### 8.5 Two emitters in one room (main and secondary controller)

Where a room has, say, an underfloor circuit and a radiator (or a second board),
let one device decide the direction and the other follow it:

* Main device object **30** (Controller Mode for Secondary Controller) →
  secondary device object **28** (Controller Mode Input).
* Give both devices the same base setpoint group address (object 31 on both).
* Give each its own control value object and actuator channel.

The secondary then never heats while the main is cooling, and vice versa.

### 8.6 Central mode management, presence and window contacts

* **Scheduler / central mode**: one group address carrying DPT 20.102 to object
  26 of every room. Have the scheduler send cyclically (or on reconnect) — the
  device does not read the mode back after a restart.
* **Presence**: link object 38 only in rooms with real detection. Choose
  Standby (short absences) or Economy (long ones) with the unoccupied-mode
  parameter.
* **Window contacts**: object 37, DPT 1.019. With the default behaviour, outputs
  drop immediately when a window opens and resume when it closes.
* **Central summer/winter shutdown**: objects 39/40 (Heating/Cooling Enable) let
  a plant controller take a whole sequence out of service without touching each
  room's parameters.

### 8.7 Visualisation and Home Assistant

For a KNX climate entity, the natural mapping is:

| Function | Device object |
|---|---|
| Current temperature | 0 |
| Target temperature (setpoint) | 31 (write and status on the same GA) |
| Setpoint shift / status | 32 / 33 |
| Operating mode / status | 26 / 27 |
| Controller mode / status | 28 / 29 |
| On/off | 25 |
| Current valve position | 42 |
| Heat/cool status | 46 |

Object 49 is a DPT 22.101 status word; expose it as its own sensor rather than
trying to feed it into a controller-status input that expects a different type.
Check the exact option names against your visualisation's current documentation.

---

## 9. Recommended group address structure

A structure that keeps a multi-room project readable:

```
3/–/–   Room climate
   3/1/x   Room 1        (temperature, setpoint, mode, control values, status)
   3/2/x   Room 2
   …
4/–/–   Ventilation
   4/1/x   Room 1 ventilation objects
5/–/–   Central / shared
   5/0/1   Central operating mode          → object 26 of every room
   5/0/2   Central heating enable          → object 39
   5/0/3   Central cooling enable          → object 40
   5/0/4   Changeover status (1 = heat)    → object 41
   5/0/5   Outside temperature             → object 20
   5/0/6   Outside humidity                → object 21
   5/0/7   Central dew point alarm         → object 19
6/–/–   Diagnostics
   6/1/x   Device fault, sensor status, moisture alarms
```

Keep the same sub-address number for the same function in every room — it makes
a 40-room project navigable and makes mistakes visible.

---

## 10. Diagnostics and troubleshooting

### 10.1 What the device tells you

| Signal | Where | Meaning |
|---|---|---|
| Device Fault (57) | bus | The room sensor is not delivering valid readings; the controller is operating under the sensor-failure behaviour. |
| Room / Floor / Air quality Sensor Status (58/59/60) | bus | Per-package DPT 21.001 status: bit 0 out of service (not fitted), bit 1 fault (fitted, not reading), bit 3 in alarm. |
| Controller Status (49) | bus | See below. |
| Air Quality Status (56) | bus | Which channel is driving ventilation. |
| Unlinked object list | console after each download | Transmitting objects with no group address. |
| `KNX bus power lost` | console | The TP1 bus supply dropped; transmission is suspended until it returns. |

### 10.2 Decoding the controller status word (object 49)

DPT 22.101 (DPT_StatusRHCC). The device sources these bits:

| Bit | Name | Set when |
|---|---|---|
| 0 | Fault | Room sensor invalid |
| 2 | Flow temperature limitation | The **floor temperature limit** is engaged (its spec wording is exactly floor-heating protection) |
| 7 | Heating disabled | Heating is currently blocked, for any reason |
| 8 | Heat/cool mode | 1 = heating (also while neutral), 0 = actively cooling |
| 11 | Cooling disabled | Cooling is currently blocked, for any reason |
| 12 | Dew point status | Condensation risk (local or received on object 19) |
| 13 | Frost alarm | Room below the frost alarm threshold |
| 14 | Overheat alarm | Room above the overheat alarm threshold |

The remaining bits stay 0 (their specified default for functions this device
does not implement).

### 10.3 Common situations

| Symptom | Likely cause | Fix |
|---|---|---|
| ETS download times out on a factory-fresh device | The device certificate has not been added to the project; Data Secure is active, so unauthorised management writes are silently ignored | Add the device certificate (console output at boot), then download again |
| Heating never runs, control value stays 0 | A floor probe that *was* reporting has failed, while *Maximum floor temperature* is non-zero and the sensor-failure behaviour is "Switch outputs off" — the limit is held engaged for safety | Repair or unplug the probe. A device that has never seen a probe no longer does this (see [Appendix B](#appendix-b--pitfalls-worth-knowing-before-you-start)); object 15 reads 1 while the limit is engaged |
| Cooling objects never move | Cooling sequence is "Not used" (the default) | Enable the cooling sequence |
| Setpoint reverts after a power cut | Runtime setpoints are re-seeded from the ETS parameters at boot | Have the visualisation/scheduler re-send, or set the parameter to the desired value |
| The scheduler's mode is ignored | A presence detector is linked to object 38 and outranks the mode input | Unlink presence, or accept the priority and drive comfort/standby through presence |
| Room temperature reads high | Self-heating | Set the room temperature correction after ≥ 1 h of settled operation |
| IAQ (object 5) looks implausible | BSEC is still calibrating | Check object 8 — accuracy 0 means "do not trust the index yet"; it rises over days |
| Values update rarely | COV threshold too large, or the minimum interval too long | Lower the relevant "change to send" parameter |
| Value bursts / busy line | COV threshold too small | Raise the threshold or the minimum interval |
| Dew point margin reads exactly 0 | No surface reference at all (no floor probe, no flow temperature input) | Link object 22, fit a probe, or set the condensation source to "Off" |
| Heating and cooling seem to fight | Dead band too small, or changeover configured "Automatic" on a two-pipe system | Increase the dead band; use "Follow changeover input" where a central valve decides |

### 10.4 Factory reset

Hold the button for 5 s. This erases the individual address, group addresses,
parameters and any tool key installed by ETS, and reboots the device with its
factory identity. You will need to re-download from ETS afterwards.

---

## 11. Commissioning checklist

1. Import the `.knxprod` that matches the firmware on the device.
2. Add the device to the project **with its device certificate** (Data Secure).
3. Set the individual address; download the application.
4. Read the console (or the unlinked-object log) and confirm every object you
   intended to link is linked.
5. Configure, at minimum:
   * heating / cooling sequences (cooling is off by default),
   * the setpoint ladder,
   * *Maximum floor temperature* — **0 if no probe is fitted**,
   * the condensation source if you cool any surface,
   * the sending behaviour (heartbeat / minimum interval / COV) for your line load.
6. Verify on the bus: room temperature, setpoint status, control value.
7. Test each interlock: open a window, write Building protection, write
   controller Off, and confirm the outputs and the status word respond.
8. After ≥ 1 h of settled operation, set the temperature and humidity
   corrections against a reference.
9. Note the device certificate and serial number in the project documentation.

---

## Appendix A — default values

| Area | Setting | Default |
|---|---|---|
| Sending | Heartbeat / minimum interval | 900 s / 10 s |
| Sending | COV: temperature, humidity, CO₂, pressure, IAQ, derived | 0.2 K, 2 %, 25 ppm, 50 Pa, 5, 0.2 |
| Control | Controller after download / operating mode / controller mode | On / Comfort / Auto |
| Control | Heating / cooling sequence | Used / **Not used** |
| Control | Changeover / minimum changeover delay | Automatic / 300 s |
| Control | Window / unoccupied / sensor failure behaviour | Switch outputs off / Standby / Switch outputs off |
| Setpoints | Comfort heating, standby, economy, protection | 21 °C, −2 K, −4 K, 7 °C |
| Setpoints | Dead band, cooling standby/economy/protection | 2 K, +2 K, +4 K, 35 °C |
| Setpoints | Min / max / max shift | 7 °C / 35 °C / 3 K |
| Loops | Algorithm, gain, reset, derivative (both directions) | PI, 25 %/K, 9000 s, 0 s |
| Loops | Hysteresis, demand strategy, threshold | 0.5 K, temperature hysteresis, 1 % |
| Loops | Frost / overheat alarm | 5 °C / 35 °C |
| Floor | Max / min / hysteresis / comfort output | 28 °C / off / 1 K / 30 % |
| Dew point | Source / margin / hysteresis / cooling | Coldest available / 2 K / 1 K / Block |
| Moisture | RH threshold / hysteresis / excess | 85 % / 5 % / 2 g/m³ |
| Ventilation | CO₂ setpoint / band | 900 ppm / 400 ppm |
| Ventilation | Humidity setpoint / band | 65 % / 15 % |
| Ventilation | IAQ setpoint / band | 150 / 150 |
| Ventilation | Base (occupied) / manual demand | 0 % / 50 % |

---

## Appendix B — pitfalls worth knowing before you start

1. **The floor limit now distinguishes "no probe" from "broken probe".** This
   used to be a trap: the default *Maximum floor temperature* is 28 °C and the
   default *Behaviour on sensor failure* is "Switch outputs off", which the
   floor probe shares — so a device with no probe fitted treated the limit as
   engaged and held the heating output at 0 forever, and you had to set
   *Maximum floor temperature* to 0 by hand on every probe-less device.

   The firmware now applies the floor limit only once a probe has actually
   reported. A device that has never seen one has no floor to protect and no
   limit to apply, so **setting *Maximum floor temperature* to 0 is no longer
   required** (it remains the right way to disable the limit deliberately on a
   device that does have a probe). A probe that reports and *then* fails is
   still a genuine fault and still blocks heating under "Switch outputs off" —
   there is a real slab there and no way to know how hot it is. Object 15 (Max
   Floor Temperature Limit Active) reads 1 in that case.
2. **Cooling is off by default.** The cooling sequence must be enabled before
   objects 43/45 or the cooling parameters do anything.
3. **Presence outranks the scheduler.** Once object 38 has received its first
   telegram, presence decides comfort vs. standby/economy; the operating mode
   input is only consulted for building protection and when presence is unknown.
4. **Runtime values are volatile.** Setpoints, modes and the CO₂ setpoint
   written over the bus revert to their ETS parameter values after a power cycle
   or a download.
5. **The device never reads inputs at startup.** Window, presence, outside
   temperature/humidity, flow temperature and changeover stay unknown until the
   sending device transmits. Configure those devices to send cyclically or on
   restart.
6. **Data Secure is on.** Commission with the device certificate; an unsecured
   management attempt looks like a timeout, not a refusal.
7. **Object 5 is an index, not a concentration**, and object 6 is an *estimate*.
   Where true CO₂ matters, use object 2.
8. **Air quality accuracy 0 means "not calibrated yet."** BSEC needs days of
   operation to reach accuracy 3; treat IAQ as provisional until then.
