# TODO: Modern KNX Room Sensor / Thermostat Feature Set

This TODO is for a greenfield KNX room sensor with integrated thermostat logic, no local UI, and bus-based control via ETS, Home Assistant, or another KNX visualization/controller.

## Design assumptions

- The device measures air temperature, humidity, air pressure, gas resistance, CO2, optional probe temperature, and optional probe humidity.
- The device may operate as a full room temperature controller, not only as a passive sensor.
- The product should interoperate cleanly with ETS, KNX Secure projects, Home Assistant KNX climate entities, and common KNX heating/cooling actuators.
- KNX Data Secure should be supported at group-object level where needed; ETS generates and loads secured group-address keys during commissioning, while device-specific secure commissioning requires factory device credentials such as FDSK/certificate.

## Source cross-check notes

- KNX’s own HVAC concept separates user setpoint, actual/effective setpoint, HVAC mode, valve settings, heat/cool status, optional window status, and optional presence status into standardized HVAC functional blocks. This supports adding mode/state/status objects instead of exposing only raw heat/cool requests.
- KNX Data Secure is configured per Group Address; secured runtime communication requires secured commissioning, and ETS creates and downloads a unique security key per secured Group Address.
- Home Assistant’s KNX climate integration commonly maps KNX thermostat behavior using separate objects for current temperature, target temperature, operation/preset mode, controller mode or heat/cool mode, and state feedback. This makes explicit mode and feedback objects important for good UI behavior.

---

# 1. Stabilize public ABI / ETS layout

## 1.1 Split object IDs from internal implementation

- [ ] Treat the existing `SensorBoardPort` enum as a public KNX application ABI.
- [ ] Avoid reordering existing enum values if already used in knxprod or field devices.
- [ ] Append new group objects only at the end.
- [ ] If a breaking redesign is acceptable for greenfield only, create a new application version instead of silently changing semantics.

## 1.2 Split parameter storage versioning

- [ ] Add an explicit parameter-layout version field if not already present.
- [ ] Continue the rule: append parameters only, never reorder.
- [ ] Add migration/defaulting logic for missing newly appended parameters.
- [ ] Add commissioning-time sanity checks for invalid stored values.

---

# 2. Recommended group-object model

## 2.1 Measurement objects

Keep or implement these as sensor outputs:

- [x] Air temperature — DPT 9.001, degrees Celsius
- [x] Relative humidity — DPT 9.007, percent RH
- [x] Air pressure — DPT 9.006 or suitable pressure DPT depending on scaling
- [x] Gas resistance — likely vendor-specific or DPT 13/14 if exposed numerically
- [x] CO2 concentration — typically ppm, use appropriate 2-byte/4-byte numeric DPT consistently with ETS representation
- [x] Probe temperature — DPT 9.001
- [x] Probe humidity — DPT 9.007

Add quality/status objects:

- [ ] Sensor status bitset or fault object
- [ ] Temperature sensor valid/fault
- [ ] Humidity sensor valid/fault
- [ ] CO2 sensor valid/fault
- [ ] Probe sensor present/fault

Recommended behavior:

- [ ] Support read flag for all measurement objects.
- [ ] Support transmit-on-change with configurable delta thresholds.
- [ ] Support cyclic transmit with configurable interval.
- [ ] Avoid transmitting every raw sensor sample.

## 2.2 Core thermostat control objects

Keep existing outputs, but add missing mode and state objects.

Existing objects to keep:

- [x] Thermostat setpoint — DPT 9.001
- [x] ThermostatRequest / heating demand — 1-bit, DPT 1.001
- [x] CoolingRequest / cooling demand — 1-bit, DPT 1.001
- [x] HeatingControlValue — DPT 5.001, 0..100 percent
- [x] CoolingControlValue — DPT 5.001, 0..100 percent

Add these objects:

- [ ] Controller enable / disable — 1-bit
- [ ] HVAC operating mode / preset — DPT 20.102-style semantics: Auto, Comfort, Standby, Economy, Building Protection
- [ ] Controller mode — Auto / Heat / Cool / Off, preferably compatible with common KNX climate gateway expectations
- [ ] Heat/cool changeover input — 1-bit or enum, configurable polarity
- [ ] Heat/cool state output — heating / cooling / neutral
- [ ] Actual active setpoint feedback — DPT 9.001
- [ ] Setpoint shift input — optional, DPT 9.002 or suitable signed delta representation
- [ ] Setpoint shift feedback — optional
- [ ] Controller active feedback — optional 1-bit
- [ ] Heating/cooling blocked feedback — optional 1-bit or diagnostic enum

Minimum recommended thermostat object set for interoperability:

```cpp
enum class SensorBoardPort : uint16_t {
    AirTemperature = 0,
    AirHumidity = 1,
    AirPressure = 2,
    GasResistance = 3,
    Co2 = 4,
    ProbeTemperature = 5,
    ProbeHumidity = 6,
    ThermostatSetpoint = 7,
    VentilationSetpoint = 8,
    ThermostatRequest = 9,
    VentilationBoostRequest = 10,
    CoolingRequest = 11,
    HeatingControlValue = 12,
    CoolingControlValue = 13,
    FloorLimitActive = 14,

    // Append-only greenfield thermostat additions
    ControllerEnable = 15,
    HvacOperatingMode = 16,
    ControllerMode = 17,
    HeatCoolChangeover = 18,
    HeatCoolState = 19,
    ActiveSetpointFeedback = 20,
    SetpointShift = 21,
    SetpointShiftFeedback = 22,
    WindowOpen = 23,
    Presence = 24,
    OccupancyMode = 25,
    ControllerFault = 26,
    SensorFaultStatus = 27,
    HeatingBlocked = 28,
    CoolingBlocked = 29,
};
```

---

# 3. Operating modes and setpoint hierarchy

## 3.1 Implement a KNX-friendly setpoint model

- [ ] Define base setpoints for:
  - Comfort
  - Standby
  - Economy
  - Building Protection / frost-heat protection
- [ ] Define separate heating and cooling setpoints if the product supports true heat/cool operation.
- [ ] Define minimum and maximum allowed user setpoints.
- [ ] Define optional setpoint shift limits.
- [ ] Publish active/effective setpoint feedback whenever the calculated target changes.

Recommended internal model:

```text
User/Bus Mode + Presence + Window + Heat/Cool Mode + Limits
        -> effective operating mode
        -> active heating/cooling setpoint
        -> controller output
```

## 3.2 Mode priority rules

Implement deterministic priority order:

1. Device fault / critical sensor invalid
2. Controller disabled
3. Window open
4. Building protection
5. Manual heat/cool/off mode
6. Presence/occupancy mode
7. Scheduled or bus-requested HVAC operating mode
8. Default mode from ETS parameter

- [ ] Document these priorities in the product manual and ETS parameter descriptions.
- [ ] Make status objects report why output is blocked or limited.

## 3.3 Suggested mode semantics

- [ ] `Off`: no active heating/cooling except optionally frost/building protection if configured.
- [ ] `Heat`: only heating controller can generate output.
- [ ] `Cool`: only cooling controller can generate output.
- [ ] `Auto`: controller selects heating/cooling according to changeover mode, deadband, or external heat/cool state.
- [ ] `Comfort`: normal occupied setpoint.
- [ ] `Standby`: lightly reduced comfort.
- [ ] `Economy`: energy-saving setback.
- [ ] `BuildingProtection`: frost/overheat protection only.

---

# 4. Inputs expected in modern KNX rooms

Add and implement:

- [ ] Window open input
  - On window open: switch to protection mode or block heating/cooling, configurable.
- [ ] Presence input
  - On presence: Comfort or normal mode.
  - On absence: Standby or Economy, configurable.
- [ ] External heat/cool changeover input
  - Useful for central heat pump / district heating / seasonal changeover systems.
- [ ] Optional outside temperature input
  - Useful for automatic heat/cool decision or limiting.
- [ ] Optional dew point / condensation alarm input for cooling systems.
  - If active, block cooling and publish cooling-blocked state.

---

# 5. Heating and cooling outputs

## 5.1 Continuous control

- [ ] Keep DPT 5.001 0..100 percent heating control value.
- [ ] Keep DPT 5.001 0..100 percent cooling control value.
- [ ] Add minimum output threshold parameter if needed for thermal actuators.
- [ ] Add maximum output limit parameter.
- [ ] Add output ramping or slew-rate limiting if needed.

## 5.2 Binary demand

- [ ] Keep 1-bit heating demand.
- [ ] Keep 1-bit cooling demand.
- [ ] Define whether binary demand is derived from:
  - control value > threshold, or
  - pure two-point hysteresis controller.
- [ ] Add parameter to select binary output strategy.

## 5.3 Mutual exclusion

- [ ] Guarantee heating and cooling are not active simultaneously unless explicitly configured for a special application.
- [ ] Add deadband between heating and cooling setpoints.
- [ ] Add minimum changeover delay to prevent rapid heat/cool toggling.

---

# 6. Controller algorithm

## 6.1 Supported algorithms

Implement at least:

- [ ] Two-point heating controller with hysteresis.
- [ ] Two-point cooling controller with hysteresis.
- [ ] PI controller for heating.
- [ ] PI controller for cooling.

Optional:

- [ ] PID derivative term, disabled by default.
- [ ] Anti-windup for integrator.
- [ ] Output freeze during sensor fault.
- [ ] Bumpless transfer when switching modes.

Recommendation:

- Use PI as the default for continuous 0..100 percent valve outputs.
- Use two-point as the default for relay-style outputs.
- Keep derivative disabled by default for slow room-temperature control.

## 6.2 Parameters to add

Append parameters only:

```cpp
enum class SensorBoardParameter : uint16_t {
    DefaultThermostatSetpoint = 0,
    DefaultVentilationSetpoint = 1,
    ThermostatHysteresis = 2,
    VentilationHysteresis = 3,
    HeatingKp = 4,
    HeatingTiSeconds = 5,
    HeatingTdSeconds = 6,
    CoolingKp = 7,
    CoolingTiSeconds = 8,
    CoolingTdSeconds = 9,
    CoolingDeadband = 10,
    MaxFloorTemperature = 11,
    FloorHysteresis = 12,
    HumidityBoostThreshold = 13,
    HumidityBoostHysteresis = 14,

    // Append-only additions
    ParameterLayoutVersion = 15,
    ControllerDefaultEnable = 16,
    DefaultHvacOperatingMode = 17,
    DefaultControllerMode = 18,
    ComfortHeatingSetpoint = 19,
    StandbyHeatingSetpoint = 20,
    EconomyHeatingSetpoint = 21,
    ProtectionHeatingSetpoint = 22,
    ComfortCoolingSetpoint = 23,
    StandbyCoolingSetpoint = 24,
    EconomyCoolingSetpoint = 25,
    ProtectionCoolingSetpoint = 26,
    MinSetpoint = 27,
    MaxSetpoint = 28,
    MaxSetpointShift = 29,
    HeatingEnabled = 30,
    CoolingEnabled = 31,
    HeatCoolChangeoverMode = 32,
    HeatCoolChangeoverPolarity = 33,
    HeatingControlAlgorithm = 34,
    CoolingControlAlgorithm = 35,
    HeatingMinimumOutputPercent = 36,
    CoolingMinimumOutputPercent = 37,
    HeatingMaximumOutputPercent = 38,
    CoolingMaximumOutputPercent = 39,
    BinaryDemandThresholdPercent = 40,
    MinimumHeatCoolChangeoverSeconds = 41,
    WindowOpenBehavior = 42,
    PresenceBehavior = 43,
    SensorFaultBehavior = 44,
    CyclicStatusIntervalSeconds = 45,
};
```

---

# 7. Floor temperature limiting

Keep the existing feature and make it bus-visible.

- [x] Max floor temperature parameter
- [x] Floor hysteresis parameter
- [x] FloorLimitActive object
- [ ] Use probe temperature as floor temperature when configured.
- [ ] Add floor sensor fault handling.
- [ ] Add configurable behavior on floor sensor failure:
  - block heating,
  - limit heating to safe maximum,
  - ignore limit and report fault.
- [ ] Ensure floor limit clamps heating output, not just binary demand.

---

# 8. Ventilation / IAQ control

Your existing model is acceptable but can be made more interoperable.

Existing:

- [x] VentilationSetpoint
- [x] VentilationBoostRequest
- [x] HumidityBoostThreshold
- [x] HumidityBoostHysteresis

Add:

- [ ] CO2 ventilation threshold parameter.
- [ ] CO2 ventilation hysteresis parameter.
- [ ] Ventilation demand output, 0..100 percent, DPT 5.001.
- [ ] Ventilation level output: Off / Low / Medium / High / Boost.
- [ ] Ventilation mode input: Auto / Manual / Off / Boost.
- [ ] Ventilation active feedback.
- [ ] IAQ status enum or bitset: OK / humidity boost / CO2 boost / sensor fault.

Recommended logic:

```text
humidity demand = f(relative humidity, threshold, hysteresis)
CO2 demand      = f(CO2 ppm, threshold, hysteresis)
manual boost    = bus override
ventilation demand = max(humidity demand, CO2 demand, manual boost)
```

---

# 9. Home Assistant interoperability checklist

- [ ] Provide separate state-feedback addresses for writable objects where possible.
- [ ] Avoid having Home Assistant write a setpoint that is immediately overwritten without publishing clear active-setpoint feedback.
- [ ] Expose current temperature address.
- [ ] Expose target temperature write address.
- [ ] Expose target temperature state/feedback address.
- [ ] Expose operation/preset mode address and feedback.
- [ ] Expose controller mode or heat/cool mode and feedback.
- [ ] Expose on/off or controller-enable address and feedback.
- [ ] Expose heating/cooling action state if possible.

Recommended behavior:

- Bus writes update internal state immediately.
- Device publishes feedback after accepting or rejecting a write.
- If a write is clamped by min/max limits, publish the clamped value.
- If a write is ignored due to mode, publish the current authoritative state.

---

# 10. KNX Secure implementation TODO

- [ ] Decide which objects should support Data Secure.
- [ ] Mark security capability correctly in the application metadata/knxprod.
- [ ] Support secure and plain group communication according to ETS configuration.
- [ ] Provision unique FDSK/device certificate per physical device during manufacturing.
- [ ] Do not share factory secrets between devices.
- [ ] Store device-specific secrets in encrypted NVS, eFuse-backed storage, or secure element.
- [ ] Ensure master reset restores tool key behavior according to KNX Secure expectations.
- [ ] Keep common firmware image independent of per-device secrets.

Suggested policy:

- Secure-capable for thermostat control, setpoints, mode objects, and outputs.
- Measurement-only objects may be optionally securable but not necessarily secure by default.
- Follow ETS project choice rather than hardcoding all objects secure.

---

# 11. Runtime behavior and bus etiquette

- [ ] For all readable objects, respond to GroupValueRead with latest authoritative value.
- [ ] For sensor values, transmit on configured delta and cyclic interval.
- [ ] For state/mode/control outputs, transmit immediately on state change.
- [ ] For important status objects, transmit cyclically at a conservative interval.
- [ ] Rate-limit repeated output changes.
- [ ] Avoid sending unchanged feedback unless cyclic reporting is due.
- [ ] On bus voltage return or reboot, publish critical states after randomized startup delay.

---

# 12. Diagnostics and commissioning support

Add:

- [ ] Firmware version object or property.
- [ ] Hardware revision object or property.
- [ ] Serial number property.
- [ ] Uptime or reset reason diagnostic, optional.
- [ ] Sensor fault code object.
- [ ] Controller state diagnostic object.
- [ ] Last accepted operating mode.
- [ ] Last output-limiting reason.

ETS parameter UX:

- [ ] Group parameters by function: Measurements, Thermostat, Heating, Cooling, Floor limit, Ventilation, Security, Diagnostics.
- [ ] Hide advanced PID parameters unless advanced mode is enabled.
- [ ] Provide sane defaults for floor heating, radiator, and fan-coil use cases.

---

# 13. Recommended implementation order

## Phase 1 — ABI and metadata

- [ ] Freeze new group-object list.
- [ ] Freeze append-only parameter list.
- [ ] Update knxprod/application metadata.
- [ ] Add parameter layout versioning.

## Phase 2 — Core thermostat behavior

- [ ] Implement controller enable.
- [ ] Implement operating mode/preset model.
- [ ] Implement controller mode: Off / Heat / Cool / Auto.
- [ ] Implement active setpoint calculation.
- [ ] Implement active setpoint feedback.
- [ ] Implement heat/cool state feedback.

## Phase 3 — Control outputs

- [ ] Implement PI heating output.
- [ ] Implement PI cooling output.
- [ ] Implement two-point heating/cooling demand.
- [ ] Implement deadband and mutual exclusion.
- [ ] Implement minimum changeover delay.

## Phase 4 — Room inputs

- [ ] Implement window open handling.
- [ ] Implement presence handling.
- [ ] Implement external heat/cool changeover.
- [ ] Implement optional condensation/cooling block input if used.

## Phase 5 — Floor and ventilation

- [ ] Implement floor limiting as output clamp.
- [ ] Implement floor sensor fault behavior.
- [ ] Implement humidity-based ventilation boost.
- [ ] Implement CO2-based ventilation demand.
- [ ] Implement ventilation demand percent and/or level output.

## Phase 6 — Interoperability testing

- [ ] Test ETS commissioning with plain group communication.
- [ ] Test ETS commissioning with KNX Data Secure enabled on selected group addresses.
- [ ] Test Home Assistant climate entity mapping.
- [ ] Test read/write/feedback behavior for modes and setpoints.
- [ ] Test bus restart and device reboot behavior.
- [ ] Test with at least one KNX heating actuator expecting DPT 5.001.
- [ ] Test with binary relay-style actuator using 1-bit demand.

## Phase 7 — Production readiness

- [ ] Add manufacturing provisioning for serial number and FDSK.
- [ ] Add QR/device certificate label generation.
- [ ] Add end-of-line test for sensors and KNX communication.
- [ ] Add persistent calibration/offset storage.
- [ ] Add firmware update strategy that preserves commissioned parameters and secure credentials.

---

# 14. Opinionated greenfield recommendation

For a modern product, do **not** expose only:

```text
setpoint + heating demand + cooling demand + 0..100% outputs
```

Instead, expose a full but still compact thermostat model:

```text
measurements
+ active setpoint
+ operating preset
+ controller mode
+ enable
+ presence/window inputs
+ heat/cool state
+ heating/cooling demand
+ heating/cooling 0..100% outputs
+ floor limiting
+ ventilation demand
+ diagnostics
```

This makes the device behave like a real KNX room controller rather than a proprietary sensor with a PID loop attached.
