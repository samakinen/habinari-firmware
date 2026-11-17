// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include "knx/product/commissioned_product.hpp"

#include "control_defaults.hpp"

// The medium below is a Kconfig choice in a firmware build. The ETS export
// tool compiles this same header on the host, where there is no sdkconfig.h;
// it passes the same symbol on the command line instead, so the two builds
// cannot describe different products.
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif

#include <cstdint>

/**
 * @file knx_product.hpp
 * @brief ETS product model for the room HVAC sensor/controller board.
 *
 * The board is modelled as a KNX S-Mode room heating/cooling controller with an
 * integrated sensor package, following "The KNX Standard v3.0.0", Volume 7/19/20
 * "S-Mode HVAC FBs" (v01.03.01). The group objects below are grouped by the
 * standard Functional Blocks they realise, so the device binds to any
 * conformant partner without a product-specific integration note:
 *
 *   RTS   Room Temperature Sensor            clause 3.1   HDC3020
 *   RRHS  Room Relative Humidity Sensor      7/10/1 3.20  HDC3020
 *   RAQS  Room Air Quality Sensor            7/10/1 3.14  SCD4x / BME688
 *   FTS   Floor Temperature Sensor           clause 3.2   SHT4x in-slab probe
 *   DPS   Dew Point Status Sensor            clause 3.4   derived on board
 *   RTSM  Room Temperature Setpoint Manager  clause 5     (setpoint ladder)
 *   RTC   Room Temperature Controller        clause 6     (heat/cool loops)
 *
 * Two deliberate deviations from the standard FB model, both in the interest of
 * the "wide compatibility" goal rather than against it:
 *
 *   - The four per-mode setpoints are NOT published as DPT_TempRoomSetpSetF16[4]
 *     (275.100). That DPT is referenced by the S-Mode FB standard but is only
 *     defined in AN188, which is still a Draft for Voting and is absent from the
 *     ratified Datapoint Types chapter (3/7/2 v02.02.01) shipped in the same
 *     v3.0.0 release. Individual DPT 9.001 setpoint objects are what shipping
 *     room controllers and Home Assistant's KNX climate platform actually speak.
 *   - Fan/ventilation staging uses DPT 5.010 rather than DPT_FanStage (5.100),
 *     for the same reason: 5.100 is not in the ratified DPT list. The primary
 *     ventilation signal is a DPT 5.001 percentage, which every AHU accepts.
 *
 * Prototype/greenfield: port numbers and parameter ids are free to be renumbered
 * as the model evolves. The knxprod is regenerated from this header on every
 * build (see cmake/ets_export.cmake) and re-imported into ETS; no old-layout
 * migration is kept. The stack stores the parameters with a fingerprint of
 * their layout, so a device that gets firmware with another layout takes its
 * defaults and asks for a download; bump kApplicationVersion with every change
 * ETS has to know about.
 */

namespace habinari_knx {

// Names, not namespaces: a using-directive here would follow this header into
// every file that includes it. These are the stack's vocabulary for describing
// a product, and the only parts of it this file needs unqualified.
namespace application = knx::application;
namespace endpoint = knx::product::endpoint;
namespace semantics = knx::product::semantics;
using knx::ManufacturerId;
using knx::product::Dpt9Float;
using knx::product::makeCommissionedProduct;
namespace params = knx::product::params;
using knx::product::makeEndpointDefinition;
using knx::product::PersistencePolicy;
using knx::product::ProductIdentity;
using knx::product::SecurityPolicy;
using knx::product::SecurityRequirement;

// ---------------------------------------------------------------------------
// Which medium this image speaks
//
// One device model, two ETS catalogue entries. Everything below this block —
// every group object, every parameter, every default — is identical on both;
// what differs is only what ETS needs in order to place the device in a
// topology: the medium type it declares, and therefore the product and order
// numbers that identify it.
//
// They have to be separate entries rather than one. ETS derives what a device
// may be connected to from its declared medium, and 03/02/06 rule 4 makes the
// consequences real: a KNX IP device requires its subnetwork, and every
// subnetwork above it, to hold KNX IP devices only. A single catalogue entry
// claiming both media would let an integrator drop a TP1 board into an IP line
// and only find out on site.
//
// The persistence namespace differs with them, deliberately. It keys the NVS
// blobs holding the individual address, the group object table and the security
// key material — state that belongs to one commissioned identity. Reflashing a
// board from one medium to the other must not have it wake up holding an
// address that ETS assigned to the other product.
//
// The application program number differs too, and has to. ETS stores one
// application program per manufacturer + number + version and keys it by a hash
// of its content; two programs sharing that key but differing in content are
// rejected on import ("the product has a different hash than the existing
// product"). These two cannot be one program, because a program declares its
// mask version and the mask version is medium-dependent — 07B0h on TP1, 57B0h
// on KNX IP. So they are numbered apart rather than versioned apart: they are
// different programs, not successive revisions of one.
#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)

inline constexpr const char *kProductKey = "habinari_ip";
inline constexpr const char *kProductDisplayName = "Habinari Room HVAC Sensor/Controller IP";
inline constexpr endpoint::Medium kProductMedium = endpoint::Medium::IP_Routing;
inline constexpr const char *kOrderNumber = "HBIP1";
inline constexpr const char *kPersistenceNamespace = "habinari_ip";
inline constexpr uint16_t kHardwareSerialNumber = 2;
inline constexpr uint16_t kApplicationNumber = 22;

#else  // TP1, and the default for a build that names no medium at all

inline constexpr const char *kProductKey = "habinari_tp1";
inline constexpr const char *kProductDisplayName = "Habinari Room HVAC Sensor/Controller TP1";
inline constexpr endpoint::Medium kProductMedium = endpoint::Medium::TP1;
inline constexpr const char *kOrderNumber = "HBTP1";
inline constexpr const char *kPersistenceNamespace = "habinari_tp1";
inline constexpr uint16_t kHardwareSerialNumber = 1;
inline constexpr uint16_t kApplicationNumber = 21;

#endif

// Bump on any change to the application program's ETS-visible content —
// parameters, group objects, the download procedure. ETS refuses to replace a
// program of the same number and version whose content hash has moved, so a
// regenerated knxprod that keeps the old version will not import.
inline constexpr uint16_t kApplicationVersion = 7;

// ---------------------------------------------------------------------------
// Parameter defaults
//
// The numbers themselves live in control_defaults.hpp, which has no dependency
// on the KNX stack, so a build without the KNX adapter still has them. This
// file is one *view* of those defaults: the ETS catalogue entry.
//
// NOTE: integer-valued defaults on purpose. ETS parameter defaults are exported
// as text and some knxprod tooling parses them with the system locale, so
// separator-free integers parse identically everywhere. Fractional values stay
// fully settable by the integrator (DPT 9 parameters render as decimal editors).
// ---------------------------------------------------------------------------

namespace cfg = habinari::config;

// ---------------------------------------------------------------------------
// KNX group-object (logical port) identity.
// ---------------------------------------------------------------------------

enum class HabinariPort : uint16_t {
    // --- Room air sensors: FB RTS / RRHS / RAQS ---------------------------
    RoomTemperature = 0,          // RTS.TempRoom, DPT 9.001
    RoomHumidity = 1,             // RRHS.HumRelRoom, DPT 9.007
    RoomCo2 = 2,                  // RAQS.AQRoom (SCD4x true CO2), DPT 9.008
    RoomAirPressure = 3,          // station pressure as measured, DPT 9.006
    RoomAirPressureSeaLevel = 4,  // altitude-reduced, DPT 9.006
    RoomAirQualityIndex = 5,      // BME688/BSEC IAQ 0..500 (lower = cleaner)
    RoomCo2Equivalent = 6,        // BSEC CO2-equivalent, DPT 9.008
    RoomVocEquivalent = 7,        // BSEC breath-VOC-equivalent, DPT 9.008
    AirQualityAccuracy = 8,       // BSEC calibration status 0..3

    // --- Derived room air values ------------------------------------------
    RoomDewPoint = 9,             // DPT 9.001 — condensation reference
    RoomAbsoluteHumidity = 10,    // DPT 9.029 — moisture content, g/m³

    // --- Floor probe: FB FTS + slab moisture -------------------------------
    FloorTemperature = 11,        // FTS.TempFloor, DPT 9.001
    FloorHumidity = 12,           // in-conduit RH, DPT 9.007
    FloorAbsoluteHumidity = 13,   // DPT 9.029 — comparable with the room value
    FloorMoistureAlarm = 14,      // DPT 1.005 — damp/leak in the slab
    FloorLimitActive = 15,        // DPT 1.011 — max floor temperature engaged
    FloorComfortActive = 16,      // DPT 1.011 — min floor temperature engaged

    // --- Condensation protection: FB DPS -----------------------------------
    DewPointAlarm = 17,           // DPS.DewPointStatus out, DPT 1.005
    DewPointMargin = 18,          // surface − dew point, DPT 9.002
    DewPointStatusInput = 19,     // RTC.DewPointStatus in, DPT 1.005

    // --- System / neighbour inputs -----------------------------------------
    OutsideTemperature = 20,      // OTS.TempOutside in, DPT 9.001
    OutsideHumidity = 21,         // ORHS in, DPT 9.007
    FlowTemperature = 22,         // FWTS.TempFlowWater in, DPT 9.001
    FreeCoolingAvailable = 23,    // DPT 1.011 — outside air can cool this room
    FreeDryingAvailable = 24,     // DPT 1.011 — outside air is drier than inside

    // --- Mode and setpoints: FB RTSM ---------------------------------------
    ControllerOnOff = 25,         // DPT 1.001, in/out (HA on_off_address)
    HvacMode = 26,                // RTSM.HVACMode in, DPT 20.102
    HvacModeStatus = 27,          // RTC.HVACModeAct out, DPT 20.102
    ContrMode = 28,               // RTC.ContrMode in, DPT 20.105
    ContrModeStatus = 29,         // RTC.ContrModeAct out, DPT 20.105
    ContrModeSecondary = 30,      // RTC.ContrModeSecondary out, DPT 20.105
    SetpointBase = 31,            // RTSM.TempRoomSetpUserAbs, DPT 9.001, in/out
    SetpointShift = 32,           // RTSM.TempRoomSetpUserOffset in, DPT 9.002
    SetpointShiftStatus = 33,     // ...UserOffsetEff out, DPT 9.002
    SetpointStatus = 34,          // RTC.TempRoomSetpAct out, DPT 9.001
    SetpointHeatingStatus = 35,   // RTSM.TempRoomSetpHeatEff out, DPT 9.001
    SetpointCoolingStatus = 36,   // RTSM.TempRoomSetpCoolEff out, DPT 9.001

    // --- Room inputs: FB WOS / PRD / WCOS ----------------------------------
    WindowStatus = 37,            // WOS.WindowStatus in, DPT 1.019
    PresenceStatus = 38,          // PRD.PresenceStatus in, DPT 1.018
    SwitchHeat = 39,              // RTC.SwitchHeat in, DPT 1.003
    SwitchCool = 40,              // RTC.SwitchCool in, DPT 1.003
    ChangeOverStatus = 41,        // WCOS.ChangeOverStatus in, DPT 1.100

    // --- Controller outputs: FB RTC ----------------------------------------
    HeatingControlValue = 42,     // RTC.ActTargetPosHeatStageA, DPT 5.001
    CoolingControlValue = 43,     // RTC.ActTargetPosCoolStageA, DPT 5.001
    HeatingRequest = 44,          // binary heat demand, DPT 1.001
    CoolingRequest = 45,          // binary cool demand, DPT 1.001
    HeatCoolModeStatus = 46,      // RTC.HeatCoolModeAct out, DPT 1.100
    EnableHeatStatus = 47,        // RTC.EnableHeatEff out, DPT 1.003
    EnableCoolStatus = 48,        // RTC.EnableCoolEff out, DPT 1.003
    ControllerStatus = 49,        // RTC.CombinedStatus_RTC, DPT 22.101

    // --- Ventilation / air quality ------------------------------------------
    Co2Setpoint = 50,             // AQSetp, DPT 9.008, in/out
    VentilationDemand = 51,       // continuous demand, DPT 5.001
    VentilationStage = 52,        // 0=Off,1=Low,2=Medium,3=High,4=Boost
    VentilationMode = 53,         // 0=Auto,1=Manual,2=Off,3=Boost, in/out
    VentilationBoostRequest = 54, // DPT 1.001
    DehumidifyRequest = 55,       // DPT 1.001
    AirQualityStatus = 56,        // bitset, see IaqStatusBit

    // --- Device diagnostics --------------------------------------------------
    DeviceFault = 57,             // roll-up alarm, DPT 1.005
    RoomSensorStatus = 58,        // HDC3020 StatusGO, DPT 21.001
    FloorProbeStatus = 59,        // SHT4x StatusGO, DPT 21.001
    AirQualitySensorStatus = 60,  // SCD4x + BME688 StatusGO, DPT 21.001
    SensorHealthMask = 61,        // bit per sensor package, see sensor_source_t
    SensorDisagreementAlarm = 62, // DPT 1.005 — sources measuring the same
                                  // quantity no longer agree (drift/failure)

    // --- Derived events (see sensor_fusion.hpp) ------------------------------
    // These are inferences from the measurements, not measurements. Each says
    // so in its object name, because an integrator binding them to real
    // building functions needs to know they can be wrong.
    FireAlarm = 63,               // DPT 1.005 — confirmed rapid rise / over-temp
    FirePreAlarm = 64,            // DPT 1.005 — condition present, unconfirmed
    TemperatureTrend = 65,        // DPT 9.002, K/h
    OccupancyDetected = 66,       // DPT 1.018 — inferred from the CO2 signal
    EstimatedOccupants = 67,      // DPT 5.010 — indication only
    WindowOpenDetected = 68,      // DPT 1.019 — inferred from a ventilating fall
    AlarmAcknowledge = 69,        // DPT 1.016 in — clears the latched fire alarm
};

// ---------------------------------------------------------------------------
// ETS parameter identity. The order in kHabinariParameterBlock below, not the
// numbers here, defines the byte layout of the parameter image.
// ---------------------------------------------------------------------------

enum class HabinariParameter : uint16_t {
    // Measurements
    MeasurementHeartbeatSeconds = 1,
    MeasurementMinRepTimeSeconds = 2,
    RoomTemperatureOffset = 3,
    RoomTemperatureCov = 4,
    RoomHumidityOffset = 5,
    RoomHumidityCov = 6,
    Co2Cov = 7,
    PressureCov = 8,
    AirQualityCov = 9,
    FloorTemperatureOffset = 10,
    FloorTemperatureCov = 11,
    FloorHumidityCov = 12,
    DerivedValueCov = 13,
    AltitudeM = 14,

    // Room control: general
    ControllerDefaultEnable = 15,
    DefaultHvacOperatingMode = 16,
    DefaultControllerMode = 17,
    HeatingEnabled = 18,
    CoolingEnabled = 19,
    HeatCoolChangeoverMode = 20,
    HeatCoolChangeoverPolarity = 21,
    MinimumHeatCoolChangeoverSeconds = 22,
    WindowOpenBehavior = 23,
    PresenceBehavior = 24,
    SensorFaultBehavior = 25,

    // Setpoints
    ComfortHeatingSetpoint = 26,
    StandbyHeatingReduction = 27,
    EconomyHeatingReduction = 28,
    ProtectionHeatingSetpoint = 29,
    CoolingDeadband = 30,
    StandbyCoolingIncrease = 31,
    EconomyCoolingIncrease = 32,
    ProtectionCoolingSetpoint = 33,
    MinSetpoint = 34,
    MaxSetpoint = 35,
    MaxSetpointShift = 36,

    // Heating loop
    HeatingControlAlgorithm = 37,
    HeatingKp = 38,
    HeatingTiSeconds = 39,
    HeatingTdSeconds = 40,
    HeatingMinimumOutputPercent = 41,
    HeatingMaximumOutputPercent = 42,

    // Cooling loop
    CoolingControlAlgorithm = 43,
    CoolingKp = 44,
    CoolingTiSeconds = 45,
    CoolingTdSeconds = 46,
    CoolingMinimumOutputPercent = 47,
    CoolingMaximumOutputPercent = 48,

    // Shared loop behaviour
    ThermostatHysteresis = 49,
    BinaryDemandStrategy = 50,
    BinaryDemandThresholdPercent = 51,
    FrostAlarmTemperature = 52,
    OverheatAlarmTemperature = 53,

    // Floor temperature
    MaxFloorTemperature = 54,
    MinFloorTemperature = 55,
    FloorHysteresis = 56,
    FloorComfortOutputPercent = 57,

    // Condensation protection
    DewPointSurfaceSource = 58,
    DewPointMargin = 59,
    DewPointHysteresis = 60,
    BlockCoolingOnDewPointAlarm = 61,

    // Slab moisture
    FloorMoistureThreshold = 62,
    FloorMoistureHysteresis = 63,
    FloorMoistureExcess = 64,

    // Ventilation / air quality
    VentilationSetpoint = 65,
    VentilationBand = 66,
    HumidityBoostThreshold = 67,
    HumidityBoostBand = 68,
    VocBoostThreshold = 69,
    VocBoostBand = 70,
    VentilationBaseDemandPercent = 71,
    VentilationManualDemandPercent = 72,

    // Sensor fusion and derived events
    SensorFilterSeconds = 73,
    TemperatureCrossCheck = 74,
    HumidityCrossCheck = 75,
    FireRateOfRise = 76,
    FireAbsoluteTemperature = 77,
    FireConfirmSeconds = 78,
    FireRequireAirQuality = 79,
    Co2OccupancyEnabled = 80,
    WindowDetectEnabled = 81,
};

// ETS section headings. Named once so a typo cannot silently split a block.
inline constexpr std::string_view kGroupMeasurement = "Measurements and sending behaviour";
inline constexpr std::string_view kGroupControlGeneral = "Room control - general";
inline constexpr std::string_view kGroupSetpoints = "Setpoints";
inline constexpr std::string_view kGroupHeating = "Heating control";
inline constexpr std::string_view kGroupCooling = "Cooling control";
inline constexpr std::string_view kGroupLoop = "Control loop behaviour";
inline constexpr std::string_view kGroupFloor = "Floor temperature";
inline constexpr std::string_view kGroupDewPoint = "Condensation protection";
inline constexpr std::string_view kGroupMoisture = "Slab moisture detection";
inline constexpr std::string_view kGroupVentilation = "Ventilation and air quality";
inline constexpr std::string_view kGroupFusion = "Sensor fusion and detection";

// ---------------------------------------------------------------------------
// ETS parameters
//
// One block, with one section per ETS page. The order of the parameters is
// the order in the parameter image the device stores, and KNstaX stores the
// image with a fingerprint of that layout: after a firmware update that adds,
// removes, reorders or retypes a parameter the device takes its defaults and
// reports that it needs a download. A change of *meaning* with the same type
// (a unit, a scale) is not visible to the fingerprint; it needs a new
// kApplicationVersion like every other change ETS has to know about.
// ---------------------------------------------------------------------------

inline constexpr auto kControllerDefaultEnableOptions =
    params::options<uint8_t>(
    {{0, "Off"},
     {1, "On"}});

inline constexpr auto kDefaultHvacOperatingModeOptions =
    params::options<uint8_t>(
    {{0, "Auto"},
     {1, "Comfort"},
     {2, "Standby"},
     {3, "Economy"},
     {4, "Building protection"}});

inline constexpr auto kDefaultControllerModeOptions =
    params::options<uint8_t>(
    {{0, "Auto"},
     {1, "Heat"},
     {2, "Cool"},
     {3, "Off"}});

inline constexpr auto kHeatingEnabledOptions =
    params::options<uint8_t>(
    {{0, "Not used"},
     {1, "Used"}});

inline constexpr auto kHeatCoolChangeoverModeOptions =
    params::options<uint8_t>(
    {{0, "Automatic (internal dead band)"},
     {1, "Follow changeover input"},
     {2, "Fixed: heating only"},
     {3, "Fixed: cooling only"}});

inline constexpr auto kHeatCoolChangeoverPolarityOptions =
    params::options<uint8_t>(
    {{0, "1 = heating (DPT 1.100)"},
     {1, "1 = cooling (inverted)"}});

inline constexpr auto kWindowOpenBehaviorOptions =
    params::options<uint8_t>(
    {{0, "Ignore"},
     {1, "Building protection setpoint"},
     {2, "Switch outputs off"}});

inline constexpr auto kPresenceBehaviorOptions =
    params::options<uint8_t>(
    {{0, "Standby"},
     {1, "Economy"}});

inline constexpr auto kSensorFaultBehaviorOptions =
    params::options<uint8_t>(
    {{0, "Hold last outputs"},
     {1, "Switch outputs off"},
     {2, "Keep controlling on last reading"}});

inline constexpr auto kHeatingControlAlgorithmOptions =
    params::options<uint8_t>(
    {{0, "Two-point (on/off)"},
     {1, "Continuous PI"}});

inline constexpr auto kBinaryDemandStrategyOptions =
    params::options<uint8_t>(
    {{0, "Temperature hysteresis"},
     {1, "Control value threshold"}});

inline constexpr auto kDewPointSurfaceSourceOptions =
    params::options<uint8_t>(
    {{0, "Off (no local dew point alarm)"},
     {1, "Floor probe"},
     {2, "Flow temperature input"},
     {3, "Coldest available"}});

inline constexpr auto kBlockCoolingOnDewPointAlarmOptions =
    params::options<uint8_t>(
    {{0, "Continue (report only)"},
     {1, "Block cooling"}});

inline constexpr auto kFireRequireAirQualityOptions =
    params::options<uint8_t>(
    {{0, "No - temperature alone"},
     {1, "Yes - fewer false alarms"}});

inline constexpr auto kCo2OccupancyEnabledOptions =
    params::options<uint8_t>(
    {{0, "Disabled"},
     {1, "Enabled"}});

inline constexpr auto kHabinariParameterBlock = params::parameterBlock<HabinariParameter>(
    {.key = "habinari", .name = "Habinari"},
    params::section(
        {.name = kGroupMeasurement},
        // ---- Measurements and sending behaviour --------------------------
        // The KNX sensor FBs parameterise Heartbeat/MinRepTime per
        // Datapoint. Held device-wide here instead: sixty per-sensor timing
        // fields would bury the handful of settings an integrator actually
        // changes, and the per-measurand knob that does matter — the COV
        // delta, whose sensible value differs by orders of magnitude
        // between K, %RH and ppm — is kept individually.
        params::param<HabinariParameter::MeasurementHeartbeatSeconds, uint16_t>(
            {.name = "Cyclic retransmission (heartbeat)",
             .def = cfg::kDefaultMeasurementHeartbeatSeconds,
             .min = 0,
             .max = 65535,
             .unit = "s",
             .key = "measurement_heartbeat_seconds"}),
        params::param<HabinariParameter::MeasurementMinRepTimeSeconds, uint16_t>(
            {.name = "Minimum time between transmissions",
             .def = cfg::kDefaultMeasurementMinRepTimeSeconds,
             .min = 0,
             .max = 3600,
             .unit = "s",
             .key = "measurement_min_rep_time_seconds"}),
        // Self-heating is the dominant error on a wall-mounted board that
        // also powers a TP1 transceiver, so an offset is not optional here.
        params::param<HabinariParameter::RoomTemperatureOffset, Dpt9Float>(
            {.name = "Room temperature correction",
             .def = Dpt9Float{cfg::kDefaultRoomTemperatureOffsetK},
             .min = -10,
             .max = 10,
             .unit = "K",
             .key = "room_temperature_offset"}),
        params::param<HabinariParameter::RoomTemperatureCov, Dpt9Float>(
            {.name = "Room temperature change to send",
             .def = Dpt9Float{cfg::kDefaultRoomTemperatureCovK},
             .min = 0,
             .max = 10,
             .unit = "K",
             .key = "room_temperature_cov"}),
        params::param<HabinariParameter::RoomHumidityOffset, Dpt9Float>(
            {.name = "Room humidity correction",
             .def = Dpt9Float{cfg::kDefaultRoomHumidityOffsetPct},
             .min = -20,
             .max = 20,
             .unit = "%",
             .key = "room_humidity_offset"}),
        params::param<HabinariParameter::RoomHumidityCov, Dpt9Float>(
            {.name = "Room humidity change to send",
             .def = Dpt9Float{cfg::kDefaultRoomHumidityCovPct},
             .min = 0,
             .max = 50,
             .unit = "%",
             .key = "room_humidity_cov"}),
        params::param<HabinariParameter::Co2Cov, uint16_t>(
            {.name = "CO2 change to send",
             .def = cfg::kDefaultCo2CovPpm,
             .min = 0,
             .max = 5000,
             .unit = "ppm",
             .key = "co2_cov"}),
        params::param<HabinariParameter::PressureCov, uint16_t>(
            {.name = "Air pressure change to send",
             .def = cfg::kDefaultPressureCovPa,
             .min = 0,
             .max = 10000,
             .unit = "Pa",
             .key = "pressure_cov"}),
        params::param<HabinariParameter::AirQualityCov, uint16_t>(
            {.name = "Air quality index change to send",
             .def = cfg::kDefaultAirQualityCovIndex,
             .min = 0,
             .max = 500,
             .key = "air_quality_cov"}),
        params::param<HabinariParameter::FloorTemperatureOffset, Dpt9Float>(
            {.name = "Floor temperature correction",
             .def = Dpt9Float{cfg::kDefaultFloorTemperatureOffsetK},
             .min = -10,
             .max = 10,
             .unit = "K",
             .key = "floor_temperature_offset"}),
        params::param<HabinariParameter::FloorTemperatureCov, Dpt9Float>(
            {.name = "Floor temperature change to send",
             .def = Dpt9Float{cfg::kDefaultFloorTemperatureCovK},
             .min = 0,
             .max = 10,
             .unit = "K",
             .key = "floor_temperature_cov"}),
        params::param<HabinariParameter::FloorHumidityCov, Dpt9Float>(
            {.name = "Floor humidity change to send",
             .def = Dpt9Float{cfg::kDefaultFloorHumidityCovPct},
             .min = 0,
             .max = 50,
             .unit = "%",
             .key = "floor_humidity_cov"}),
        params::param<HabinariParameter::DerivedValueCov, Dpt9Float>(
            {.name = "Dew point / absolute humidity change to send",
             .def = Dpt9Float{cfg::kDefaultDerivedCovK},
             .min = 0,
             .max = 10,
             .key = "derived_value_cov"}),
        params::param<HabinariParameter::AltitudeM, uint16_t>(
            {.name = "Installation altitude above sea level",
             .def = cfg::kDefaultAltitudeM,
             .min = 0,
             .max = 4000,
             .unit = "m",
             .key = "altitude_m"})),

    params::section(
        {.name = kGroupControlGeneral},
        // ---- Room control: general ---------------------------------------
        params::param<HabinariParameter::ControllerDefaultEnable, uint8_t>(
            {.name = "Controller state after download",
             .def = cfg::kDefaultControllerEnable,
             .options = kControllerDefaultEnableOptions,
             .key = "controller_default_enable"}),
        params::param<HabinariParameter::DefaultHvacOperatingMode, uint8_t>(
            {.name = "HVAC operating mode after download",
             .def = cfg::kDefaultHvacOperatingMode,
             .options = kDefaultHvacOperatingModeOptions,
             .key = "default_hvac_operating_mode"}),
        params::param<HabinariParameter::DefaultControllerMode, uint8_t>(
            {.name = "Controller mode after download",
             .def = cfg::kDefaultControllerMode,
             .options = kDefaultControllerModeOptions,
             .key = "default_controller_mode"}),
        params::param<HabinariParameter::HeatingEnabled, uint8_t>(
            {.name = "Heating sequence",
             .def = cfg::kDefaultHeatingEnabled,
             .options = kHeatingEnabledOptions,
             .key = "heating_enabled"}),
        params::param<HabinariParameter::CoolingEnabled, uint8_t>(
            {.name = "Cooling sequence",
             .def = cfg::kDefaultCoolingEnabled,
             .options = kHeatingEnabledOptions,
             .key = "cooling_enabled"}),
        params::param<HabinariParameter::HeatCoolChangeoverMode, uint8_t>(
            {.name = "Heat/cool changeover",
             .def = cfg::kDefaultHeatCoolChangeoverMode,
             .options = kHeatCoolChangeoverModeOptions,
             .key = "heat_cool_changeover_mode"}),
        params::param<HabinariParameter::HeatCoolChangeoverPolarity, uint8_t>(
            {.name = "Changeover input polarity",
             .def = cfg::kDefaultHeatCoolChangeoverPolarity,
             .options = kHeatCoolChangeoverPolarityOptions,
             .visible = params::when<HabinariParameter::HeatCoolChangeoverMode>(1),
             .key = "heat_cool_changeover_polarity"}),
        params::param<HabinariParameter::MinimumHeatCoolChangeoverSeconds, uint16_t>(
            {.name = "Minimum delay between heating and cooling",
             .def = cfg::kDefaultMinimumHeatCoolChangeoverSeconds,
             .min = 0,
             .max = 65535,
             .unit = "s",
             .key = "minimum_heat_cool_changeover_seconds"}),
        params::param<HabinariParameter::WindowOpenBehavior, uint8_t>(
            {.name = "Behaviour with window open",
             .def = cfg::kDefaultWindowOpenBehavior,
             .options = kWindowOpenBehaviorOptions,
             .key = "window_open_behavior"}),
        params::param<HabinariParameter::PresenceBehavior, uint8_t>(
            {.name = "Mode when the room is unoccupied",
             .def = cfg::kDefaultPresenceBehavior,
             .options = kPresenceBehaviorOptions,
             .key = "presence_behavior"}),
        params::param<HabinariParameter::SensorFaultBehavior, uint8_t>(
            {.name = "Behaviour on sensor failure",
             .def = cfg::kDefaultSensorFaultBehavior,
             .options = kSensorFaultBehaviorOptions,
             .key = "sensor_fault_behavior"})),

    params::section(
        {.name = kGroupSetpoints},
        // ---- Setpoints ------------------------------------------------------
        // KNX RTSM ladder: Comfort is the absolute anchor, Standby/Economy
        // are shifts away from it, and only building protection is absolute
        // again. One setpoint write from an HMI or Home Assistant then moves
        // the whole ladder coherently.
        params::param<HabinariParameter::ComfortHeatingSetpoint, Dpt9Float>(
            {.name = "Comfort heating setpoint",
             .def = Dpt9Float{cfg::kDefaultComfortHeatingSetpointC},
             .min = 5,
             .max = 40,
             .unit = "°C",
             .key = "comfort_heating_setpoint"}),
        params::param<HabinariParameter::StandbyHeatingReduction, Dpt9Float>(
            {.name = "Standby: reduction below comfort",
             .def = Dpt9Float{cfg::kDefaultStandbyHeatingReductionK},
             .min = 0,
             .max = 15,
             .unit = "K",
             .key = "standby_heating_reduction"}),
        params::param<HabinariParameter::EconomyHeatingReduction, Dpt9Float>(
            {.name = "Economy: reduction below comfort",
             .def = Dpt9Float{cfg::kDefaultEconomyHeatingReductionK},
             .min = 0,
             .max = 20,
             .unit = "K",
             .key = "economy_heating_reduction"}),
        params::param<HabinariParameter::ProtectionHeatingSetpoint, Dpt9Float>(
            {.name = "Building protection: frost setpoint",
             .def = Dpt9Float{cfg::kDefaultProtectionHeatingSetpointC},
             .min = 3,
             .max = 15,
             .unit = "°C",
             .key = "protection_heating_setpoint"}),
        params::param<HabinariParameter::CoolingDeadband, Dpt9Float>(
            {.name = "Dead band between heating and cooling",
             .def = Dpt9Float{cfg::kDefaultCoolingDeadbandK},
             .min = 0,
             .max = 10,
             .unit = "K",
             .key = "cooling_deadband"}),
        params::param<HabinariParameter::StandbyCoolingIncrease, Dpt9Float>(
            {.name = "Standby: increase above comfort cooling",
             .def = Dpt9Float{cfg::kDefaultStandbyCoolingIncreaseK},
             .min = 0,
             .max = 15,
             .unit = "K",
             .visible = params::when<HabinariParameter::CoolingEnabled>(1),
             .key = "standby_cooling_increase"}),
        params::param<HabinariParameter::EconomyCoolingIncrease, Dpt9Float>(
            {.name = "Economy: increase above comfort cooling",
             .def = Dpt9Float{cfg::kDefaultEconomyCoolingIncreaseK},
             .min = 0,
             .max = 20,
             .unit = "K",
             .visible = params::when<HabinariParameter::CoolingEnabled>(1),
             .key = "economy_cooling_increase"}),
        params::param<HabinariParameter::ProtectionCoolingSetpoint, Dpt9Float>(
            {.name = "Building protection: heat setpoint",
             .def = Dpt9Float{cfg::kDefaultProtectionCoolingSetpointC},
             .min = 25,
             .max = 45,
             .unit = "°C",
             .visible = params::when<HabinariParameter::CoolingEnabled>(1),
             .key = "protection_cooling_setpoint"}),
        params::param<HabinariParameter::MinSetpoint, Dpt9Float>(
            {.name = "Lowest allowed setpoint",
             .def = Dpt9Float{cfg::kDefaultMinSetpointC},
             .min = 3,
             .max = 25,
             .unit = "°C",
             .key = "min_setpoint"}),
        params::param<HabinariParameter::MaxSetpoint, Dpt9Float>(
            {.name = "Highest allowed setpoint",
             .def = Dpt9Float{cfg::kDefaultMaxSetpointC},
             .min = 15,
             .max = 45,
             .unit = "°C",
             .key = "max_setpoint"}),
        params::param<HabinariParameter::MaxSetpointShift, Dpt9Float>(
            {.name = "Maximum manual setpoint shift",
             .def = Dpt9Float{cfg::kDefaultMaxSetpointShiftK},
             .min = 0,
             .max = 10,
             .unit = "K",
             .key = "max_setpoint_shift"})),

    params::section(
        {.name = kGroupHeating, .visible = params::when<HabinariParameter::HeatingEnabled>(1)},
        // ---- Heating control -------------------------------------------------
        // The whole section is gated on the heating sequence being in use, so
        // an integrator who switched heating off does not get a section of
        // settings that the controller ignores. Cooling below is deliberately
        // identical in shape: the two sequences differ only in which enable
        // and which algorithm parameter they follow.
        params::param<HabinariParameter::HeatingControlAlgorithm, uint8_t>(
            {.name = "Heating control algorithm",
             .def = cfg::kDefaultHeatingControlAlgorithm,
             .options = kHeatingControlAlgorithmOptions,
             .key = "heating_control_algorithm"}),
        params::param<HabinariParameter::HeatingKp, Dpt9Float>(
            {.name = "Heating proportional gain",
             .def = Dpt9Float{cfg::kDefaultHeatingKp},
             .min = 1,
             .max = 200,
             .unit = "%/K",
             .visible = params::when<HabinariParameter::HeatingControlAlgorithm>(1),
             .key = "heating_pid_kp"}),
        params::param<HabinariParameter::HeatingTiSeconds, uint16_t>(
            {.name = "Heating reset time (0 = P only)",
             .def = cfg::kDefaultHeatingTiSeconds,
             .min = 0,
             .max = 65535,
             .unit = "s",
             .visible = params::when<HabinariParameter::HeatingControlAlgorithm>(1),
             .key = "heating_pid_ti_seconds"}),
        params::param<HabinariParameter::HeatingTdSeconds, uint16_t>(
            {.name = "Heating derivative time (0 = off)",
             .def = cfg::kDefaultHeatingTdSeconds,
             .min = 0,
             .max = 65535,
             .unit = "s",
             .visible = params::when<HabinariParameter::HeatingControlAlgorithm>(1),
             .key = "heating_pid_td_seconds"}),
        params::param<HabinariParameter::HeatingMinimumOutputPercent, uint8_t>(
            {.name = "Heating minimum control value",
             .def = cfg::kDefaultHeatingMinimumOutputPercent,
             .min = 0,
             .max = 100,
             .unit = "%",
             .key = "heating_minimum_output_percent"}),
        params::param<HabinariParameter::HeatingMaximumOutputPercent, uint8_t>(
            {.name = "Heating maximum control value",
             .def = cfg::kDefaultHeatingMaximumOutputPercent,
             .min = 0,
             .max = 100,
             .unit = "%",
             .key = "heating_maximum_output_percent"})),

    params::section(
        {.name = kGroupCooling, .visible = params::when<HabinariParameter::CoolingEnabled>(1)},
        // ---- Cooling control -------------------------------------------------
        // Mirror image of the heating section above, gated on the cooling
        // sequence. The PI terms follow the cooling algorithm rather than the
        // cooling enable: gating them on the enable made them appear under a
        // two-point configuration, where they do nothing.
        params::param<HabinariParameter::CoolingControlAlgorithm, uint8_t>(
            {.name = "Cooling control algorithm",
             .def = cfg::kDefaultCoolingControlAlgorithm,
             .options = kHeatingControlAlgorithmOptions,
             .key = "cooling_control_algorithm"}),
        params::param<HabinariParameter::CoolingKp, Dpt9Float>(
            {.name = "Cooling proportional gain",
             .def = Dpt9Float{cfg::kDefaultCoolingKp},
             .min = 1,
             .max = 200,
             .unit = "%/K",
             .visible = params::when<HabinariParameter::CoolingControlAlgorithm>(1),
             .key = "cooling_pid_kp"}),
        params::param<HabinariParameter::CoolingTiSeconds, uint16_t>(
            {.name = "Cooling reset time (0 = P only)",
             .def = cfg::kDefaultCoolingTiSeconds,
             .min = 0,
             .max = 65535,
             .unit = "s",
             .visible = params::when<HabinariParameter::CoolingControlAlgorithm>(1),
             .key = "cooling_pid_ti_seconds"}),
        params::param<HabinariParameter::CoolingTdSeconds, uint16_t>(
            {.name = "Cooling derivative time (0 = off)",
             .def = cfg::kDefaultCoolingTdSeconds,
             .min = 0,
             .max = 65535,
             .unit = "s",
             .visible = params::when<HabinariParameter::CoolingControlAlgorithm>(1),
             .key = "cooling_pid_td_seconds"}),
        params::param<HabinariParameter::CoolingMinimumOutputPercent, uint8_t>(
            {.name = "Cooling minimum control value",
             .def = cfg::kDefaultCoolingMinimumOutputPercent,
             .min = 0,
             .max = 100,
             .unit = "%",
             .key = "cooling_minimum_output_percent"}),
        params::param<HabinariParameter::CoolingMaximumOutputPercent, uint8_t>(
            {.name = "Cooling maximum control value",
             .def = cfg::kDefaultCoolingMaximumOutputPercent,
             .min = 0,
             .max = 100,
             .unit = "%",
             .key = "cooling_maximum_output_percent"})),

    params::section(
        {.name = kGroupLoop},
        // ---- Control loop behaviour ------------------------------------------
        params::param<HabinariParameter::ThermostatHysteresis, Dpt9Float>(
            {.name = "Two-point switching hysteresis",
             .def = Dpt9Float{cfg::kDefaultThermostatHysteresisC},
             .min = 0,
             .max = 5,
             .unit = "K",
             .key = "thermostat_hysteresis"}),
        params::param<HabinariParameter::BinaryDemandStrategy, uint8_t>(
            {.name = "Binary demand derived from",
             .def = cfg::kDefaultBinaryDemandStrategy,
             .options = kBinaryDemandStrategyOptions,
             .key = "binary_demand_strategy"}),
        params::param<HabinariParameter::BinaryDemandThresholdPercent, uint8_t>(
            {.name = "Binary demand control-value threshold",
             .def = cfg::kDefaultBinaryDemandThresholdPercent,
             .min = 1,
             .max = 100,
             .unit = "%",
             .visible = params::when<HabinariParameter::BinaryDemandStrategy>(1),
             .key = "binary_demand_threshold_percent"}),
        params::param<HabinariParameter::FrostAlarmTemperature, Dpt9Float>(
            {.name = "Frost alarm below (0 = off)",
             .def = Dpt9Float{cfg::kDefaultFrostAlarmTemperatureC},
             .min = 0,
             .max = 20,
             .unit = "°C",
             .key = "frost_alarm_temperature"}),
        params::param<HabinariParameter::OverheatAlarmTemperature, Dpt9Float>(
            {.name = "Overheat alarm above (0 = off)",
             .def = Dpt9Float{cfg::kDefaultOverheatAlarmTemperatureC},
             .min = 0,
             .max = 60,
             .unit = "°C",
             .key = "overheat_alarm_temperature"})),

    params::section(
        {.name = kGroupFloor},
        // ---- Floor temperature -------------------------------------------------
        params::param<HabinariParameter::MaxFloorTemperature, Dpt9Float>(
            {.name = "Maximum floor temperature (0 = off)",
             .def = Dpt9Float{cfg::kDefaultMaxFloorTemperatureC},
             .min = 0,
             .max = 45,
             .unit = "°C",
             .key = "max_floor_temperature"}),
        params::param<HabinariParameter::MinFloorTemperature, Dpt9Float>(
            {.name = "Minimum floor temperature (0 = off)",
             .def = Dpt9Float{cfg::kDefaultMinFloorTemperatureC},
             .min = 0,
             .max = 35,
             .unit = "°C",
             .key = "min_floor_temperature"}),
        params::param<HabinariParameter::FloorHysteresis, Dpt9Float>(
            {.name = "Floor temperature hysteresis",
             .def = Dpt9Float{cfg::kDefaultFloorHysteresisK},
             .min = 0,
             .max = 10,
             .unit = "K",
             .key = "floor_hysteresis"}),
        params::param<HabinariParameter::FloorComfortOutputPercent, uint8_t>(
            {.name = "Control value while holding minimum floor temperature",
             .def = cfg::kDefaultFloorComfortOutputPercent,
             .min = 0,
             .max = 100,
             .unit = "%",
             .key = "floor_comfort_output_percent"})),

    params::section(
        {.name = kGroupDewPoint},
        // ---- Condensation protection -------------------------------------------
        params::param<HabinariParameter::DewPointSurfaceSource, uint8_t>(
            {.name = "Surface temperature used for condensation check",
             .def = cfg::kDefaultDewPointSurfaceSource,
             .options = kDewPointSurfaceSourceOptions,
             .key = "dew_point_surface_source"}),
        params::param<HabinariParameter::DewPointMargin, Dpt9Float>(
            {.name = "Alarm when surface is within",
             .def = Dpt9Float{cfg::kDefaultDewPointMarginK},
             .min = 0,
             .max = 10,
             .unit = "K of dew point",
             .key = "dew_point_margin"}),
        params::param<HabinariParameter::DewPointHysteresis, Dpt9Float>(
            {.name = "Dew point alarm hysteresis",
             .def = Dpt9Float{cfg::kDefaultDewPointHysteresisK},
             .min = 0,
             .max = 10,
             .unit = "K",
             .key = "dew_point_hysteresis"}),
        params::param<HabinariParameter::BlockCoolingOnDewPointAlarm, uint8_t>(
            {.name = "Cooling during a dew point alarm",
             .def = cfg::kDefaultBlockCoolingOnDewPointAlarm,
             .options = kBlockCoolingOnDewPointAlarmOptions,
             .key = "block_cooling_on_dew_point_alarm"})),

    params::section(
        {.name = kGroupMoisture},
        // ---- Slab moisture detection ---------------------------------------------
        params::param<HabinariParameter::FloorMoistureThreshold, Dpt9Float>(
            {.name = "Slab humidity alarm above (0 = off)",
             .def = Dpt9Float{cfg::kDefaultFloorMoistureThresholdPct},
             .min = 0,
             .max = 100,
             .unit = "%",
             .key = "floor_moisture_threshold"}),
        params::param<HabinariParameter::FloorMoistureHysteresis, Dpt9Float>(
            {.name = "Slab humidity alarm hysteresis",
             .def = Dpt9Float{cfg::kDefaultFloorMoistureHysteresisPct},
             .min = 0,
             .max = 25,
             .unit = "%",
             .key = "floor_moisture_hysteresis"}),
        // Absolute humidity is a moisture *content*, so comparing the slab
        // against the room stays meaningful even though the slab is the
        // colder of the two — which is exactly where a relative-humidity
        // comparison misleads. Detects evaporating liquid water long before
        // the relative reading looks alarming.
        params::param<HabinariParameter::FloorMoistureExcess, Dpt9Float>(
            {.name = "Alarm when slab is wetter than the room by (0 = off)",
             .def = Dpt9Float{cfg::kDefaultFloorMoistureExcessGm3},
             .min = 0,
             .max = 20,
             .unit = "g/m³",
             .key = "floor_moisture_excess"})),

    params::section(
        {.name = kGroupVentilation},
        // ---- Ventilation and air quality --------------------------------------------
        params::param<HabinariParameter::VentilationSetpoint, uint16_t>(
            {.name = "CO2 setpoint (demand starts here)",
             .def = cfg::kDefaultVentilationSetpointPpm,
             .min = 400,
             .max = 5000,
             .unit = "ppm",
             .key = "ventilation_setpoint"}),
        params::param<HabinariParameter::VentilationBand, uint16_t>(
            {.name = "CO2 band to full demand",
             .def = cfg::kDefaultVentilationBandPpm,
             .min = 0,
             .max = 5000,
             .unit = "ppm",
             .key = "ventilation_band"}),
        params::param<HabinariParameter::HumidityBoostThreshold, Dpt9Float>(
            {.name = "Humidity setpoint (demand starts here)",
             .def = Dpt9Float{cfg::kDefaultHumidityBoostPct},
             .min = 0,
             .max = 100,
             .unit = "%",
             .key = "humidity_boost_threshold"}),
        params::param<HabinariParameter::HumidityBoostBand, Dpt9Float>(
            {.name = "Humidity band to full demand",
             .def = Dpt9Float{cfg::kDefaultHumidityBandPct},
             .min = 0,
             .max = 50,
             .unit = "%",
             .key = "humidity_boost_band"}),
        params::param<HabinariParameter::VocBoostThreshold, uint16_t>(
            {.name = "Air quality index setpoint (0 = off)",
             .def = cfg::kDefaultVocThresholdIndex,
             .min = 0,
             .max = 500,
             .key = "voc_boost_threshold"}),
        params::param<HabinariParameter::VocBoostBand, uint16_t>(
            {.name = "Air quality index band to full demand",
             .def = cfg::kDefaultVocBandIndex,
             .min = 0,
             .max = 500,
             .key = "voc_boost_band"}),
        params::param<HabinariParameter::VentilationBaseDemandPercent, uint8_t>(
            {.name = "Base ventilation while the room is occupied",
             .def = cfg::kDefaultVentilationBaseDemandPercent,
             .min = 0,
             .max = 100,
             .unit = "%",
             .key = "ventilation_base_demand_percent"}),
        params::param<HabinariParameter::VentilationManualDemandPercent, uint8_t>(
            {.name = "Demand in manual ventilation mode",
             .def = cfg::kDefaultVentilationManualDemandPercent,
             .min = 0,
             .max = 100,
             .unit = "%",
             .key = "ventilation_manual_demand_percent"})),

    params::section(
        {.name = kGroupFusion},
        // ---- Sensor fusion and detection ---------------------------------
        params::param<HabinariParameter::SensorFilterSeconds, uint16_t>(
            {.name = "Measurement smoothing time constant",
             .def = cfg::kDefaultSensorFilterSeconds,
             .min = 0,
             .max = 600,
             .unit = "s",
             .key = "sensor_filter_seconds"}),
        // The board measures room temperature three times over (HDC3020,
        // BME688, SCD4x). Beyond this difference they are not measuring the
        // same thing any more, which means one of them has drifted: with
        // three sources the outlier is voted out of the published value,
        // with two it is only reported.
        params::param<HabinariParameter::TemperatureCrossCheck, Dpt9Float>(
            {.name = "Temperature sensor disagreement limit (0 = off)",
             .def = Dpt9Float{cfg::kDefaultTemperatureCrossCheckK},
             .min = 0,
             .max = 20,
             .unit = "K",
             .key = "temperature_cross_check"}),
        params::param<HabinariParameter::HumidityCrossCheck, Dpt9Float>(
            {.name = "Humidity sensor disagreement limit (0 = off)",
             .def = Dpt9Float{cfg::kDefaultHumidityCrossCheckPct},
             .min = 0,
             .max = 50,
             .unit = "%",
             .key = "humidity_cross_check"}),
        // Advisory heat detection. Not a substitute for a certified fire
        // detector — see the FireAlarm object's description and the manual.
        params::param<HabinariParameter::FireRateOfRise, Dpt9Float>(
            {.name = "Rapid rise alarm threshold (0 = off)",
             .def = Dpt9Float{cfg::kDefaultFireRateOfRiseKPerMin},
             .min = 0,
             .max = 30,
             .unit = "K/min",
             .key = "fire_rate_of_rise"}),
        params::param<HabinariParameter::FireAbsoluteTemperature, Dpt9Float>(
            {.name = "Over-temperature alarm threshold (0 = off)",
             .def = Dpt9Float{cfg::kDefaultFireAbsoluteTemperatureC},
             .min = 0,
             .max = 120,
             .unit = "C",
             .key = "fire_absolute_temperature"}),
        params::param<HabinariParameter::FireConfirmSeconds, uint16_t>(
            {.name = "Rapid rise confirmation time",
             .def = cfg::kDefaultFireConfirmSeconds,
             .min = 5,
             .max = 600,
             .unit = "s",
             .key = "fire_confirm_seconds"}),
        // A fan heater raises the temperature fast and burns nothing. Only
        // combustion also drives the BME688 gas signal, so requiring both
        // is the strongest false-alarm defence available here — at the cost
        // of needing a calibrated BSEC, which takes hours after a cold boot.
        params::param<HabinariParameter::FireRequireAirQuality, uint8_t>(
            {.name = "Also require rising air pollution",
             .def = cfg::kDefaultFireRequireAirQuality,
             .options = kFireRequireAirQualityOptions,
             .key = "fire_require_air_quality"}),
        params::param<HabinariParameter::Co2OccupancyEnabled, uint8_t>(
            {.name = "Derive occupancy from CO2",
             .def = cfg::kDefaultCo2OccupancyEnabled,
             .options = kCo2OccupancyEnabledOptions,
             .key = "co2_occupancy_enabled"}),
        params::param<HabinariParameter::WindowDetectEnabled, uint8_t>(
            {.name = "Detect open windows from air change",
             .def = cfg::kDefaultWindowDetectEnabled,
             .options = kCo2OccupancyEnabledOptions,
             .key = "window_detect_enabled"})));

inline constexpr auto kHabinariParameters = params::parameterLayout(kHabinariParameterBlock);

inline constexpr auto kHabinariProduct =
    makeCommissionedProduct<kHabinariParameters>(
        makeEndpointDefinition<
            HabinariPort,
            // ---- FB RTS / RRHS / RAQS: room air measurements ---------------
            endpoint::semantics::TemperatureState<HabinariPort::RoomTemperature,
                                                  "room_temperature",
                                                  "Room Air Temperature",
                                                  false>,
            endpoint::semantics::HumidityState<HabinariPort::RoomHumidity,
                                               "room_humidity",
                                               "Room Relative Humidity",
                                               false>,
            endpoint::semantics::Co2State<HabinariPort::RoomCo2,
                                          "room_co2",
                                          "Room CO2 (SCD4x)",
                                          false>,
            endpoint::StatePort<HabinariPort::RoomAirPressure,
                                float,
                                "room_air_pressure",
                                "Air Pressure (station)",
                                application::dptids::Pressure,
                                false>,
            // Only the sea-level-reduced value is comparable with a forecast or
            // with another site, which is what a visualisation actually wants.
            endpoint::StatePort<HabinariPort::RoomAirPressureSeaLevel,
                                float,
                                "room_air_pressure_sea_level",
                                "Air Pressure (sea level)",
                                application::dptids::Pressure,
                                false>,
            endpoint::StatePort<HabinariPort::RoomAirQualityIndex,
                                uint16_t,
                                "room_air_quality_index",
                                "Air Quality Index (BSEC IAQ 0..500)",
                                application::dptids::Counter16,
                                false>,
            endpoint::StatePort<HabinariPort::RoomCo2Equivalent,
                                float,
                                "room_co2_equivalent",
                                "CO2 Equivalent (BSEC)",
                                application::dptids::CO2,
                                false>,
            endpoint::StatePort<HabinariPort::RoomVocEquivalent,
                                float,
                                "room_voc_equivalent",
                                "Breath-VOC Equivalent (BSEC)",
                                application::dptids::CO2,
                                false>,
            endpoint::StatePort<HabinariPort::AirQualityAccuracy,
                                uint8_t,
                                "air_quality_accuracy",
                                "Air Quality Accuracy (BSEC 0..3)",
                                application::dptids::Level,
                                false>,

            // ---- Derived room air values -----------------------------------
            endpoint::semantics::TemperatureState<HabinariPort::RoomDewPoint,
                                                  "room_dew_point",
                                                  "Room Dew Point",
                                                  false>,
            endpoint::StatePort<HabinariPort::RoomAbsoluteHumidity,
                                float,
                                "room_absolute_humidity",
                                "Room Absolute Humidity",
                                application::dptids::AbsoluteHumidity,
                                false>,

            // ---- FB FTS + slab moisture -------------------------------------
            endpoint::semantics::TemperatureState<HabinariPort::FloorTemperature,
                                                  "floor_temperature",
                                                  "Floor Temperature",
                                                  false>,
            endpoint::semantics::HumidityState<HabinariPort::FloorHumidity,
                                               "floor_humidity",
                                               "Floor Slab Relative Humidity",
                                               false>,
            endpoint::StatePort<HabinariPort::FloorAbsoluteHumidity,
                                float,
                                "floor_absolute_humidity",
                                "Floor Slab Absolute Humidity",
                                application::dptids::AbsoluteHumidity,
                                false>,
            endpoint::semantics::AlarmState<HabinariPort::FloorMoistureAlarm,
                                            "floor_moisture_alarm",
                                            "Floor Slab Moisture Alarm",
                                            false>,
            endpoint::StatePort<HabinariPort::FloorLimitActive,
                                bool,
                                "floor_limit_active",
                                "Max Floor Temperature Limit Active",
                                application::dptids::State,
                                false>,
            endpoint::StatePort<HabinariPort::FloorComfortActive,
                                bool,
                                "floor_comfort_active",
                                "Min Floor Temperature Active",
                                application::dptids::State,
                                false>,

            // ---- FB DPS: condensation protection ----------------------------
            endpoint::semantics::AlarmState<HabinariPort::DewPointAlarm,
                                            "dew_point_alarm",
                                            "Dew Point Alarm",
                                            false>,
            endpoint::StatePort<HabinariPort::DewPointMargin,
                                float,
                                "dew_point_margin",
                                "Dew Point Margin (surface - dew point)",
                                application::dptids::TemperatureDelta,
                                false>,
            endpoint::CommandPort<HabinariPort::DewPointStatusInput,
                                  bool,
                                  "dew_point_status_input",
                                  "Dew Point Alarm Input (external sensor)",
                                  application::dptids::Alarm,
                                  false>,

            // ---- System / neighbour inputs -----------------------------------
            endpoint::CommandPort<HabinariPort::OutsideTemperature,
                                  float,
                                  "outside_temperature",
                                  "Outside Temperature Input",
                                  application::dptids::Temperature,
                                  false>,
            endpoint::CommandPort<HabinariPort::OutsideHumidity,
                                  float,
                                  "outside_humidity",
                                  "Outside Relative Humidity Input",
                                  application::dptids::Humidity,
                                  false>,
            endpoint::CommandPort<HabinariPort::FlowTemperature,
                                  float,
                                  "flow_temperature",
                                  "Cooling Flow Temperature Input",
                                  application::dptids::Temperature,
                                  false>,
            endpoint::StatePort<HabinariPort::FreeCoolingAvailable,
                                bool,
                                "free_cooling_available",
                                "Free Cooling Available (outside air)",
                                application::dptids::State,
                                false>,
            endpoint::StatePort<HabinariPort::FreeDryingAvailable,
                                bool,
                                "free_drying_available",
                                "Free Drying Available (outside air)",
                                application::dptids::State,
                                false>,

            // ---- FB RTSM: mode and setpoints ---------------------------------
            endpoint::semantics::SwitchStateInOut<HabinariPort::ControllerOnOff,
                                                  "controller_on_off",
                                                  "Controller On/Off",
                                                  true>,
            // Split input/status pair: the requested mode and the mode the
            // controller actually resolved to can legitimately differ (window
            // open, presence, building protection), and a single in/out object
            // cannot express that without lying to whichever end reads it.
            endpoint::CommandPort<HabinariPort::HvacMode,
                                  application::Dpt20Mode,
                                  "hvac_mode",
                                  "HVAC Operating Mode Input",
                                  application::dptids::HvacModeComfort,
                                  true>,
            endpoint::StatePort<HabinariPort::HvacModeStatus,
                                application::Dpt20Mode,
                                "hvac_mode_status",
                                "HVAC Operating Mode Status",
                                application::dptids::HvacModeComfort,
                                true>,
            endpoint::CommandPort<HabinariPort::ContrMode,
                                  uint8_t,
                                  "contr_mode",
                                  "Controller Mode Input (0=Auto 1=Heat 3=Cool 6=Off)",
                                  application::dptids::HvacContrMode,
                                  true>,
            endpoint::StatePort<HabinariPort::ContrModeStatus,
                                uint8_t,
                                "contr_mode_status",
                                "Controller Mode Status",
                                application::dptids::HvacContrMode,
                                true>,
            // Drives a second RTC in the same room (e.g. radiator alongside the
            // underfloor circuit) so the two can never fight — KNX Vol 7/19/20
            // clause 6.3.4.2, Main/Secondary RTC.
            endpoint::StatePort<HabinariPort::ContrModeSecondary,
                                uint8_t,
                                "contr_mode_secondary",
                                "Controller Mode for Secondary Controller",
                                application::dptids::HvacContrMode,
                                false>,
            endpoint::StateInOutPort<HabinariPort::SetpointBase,
                                     float,
                                     "setpoint_base",
                                     "Base Setpoint (Comfort heating)",
                                     application::dptids::Temperature,
                                     true>,
            endpoint::CommandPort<HabinariPort::SetpointShift,
                                  float,
                                  "setpoint_shift",
                                  "Setpoint Shift Input",
                                  application::dptids::TemperatureDelta,
                                  false>,
            endpoint::StatePort<HabinariPort::SetpointShiftStatus,
                                float,
                                "setpoint_shift_status",
                                "Setpoint Shift Status",
                                application::dptids::TemperatureDelta,
                                false>,
            endpoint::semantics::TemperatureState<HabinariPort::SetpointStatus,
                                                  "setpoint_status",
                                                  "Active Setpoint Status",
                                                  false>,
            endpoint::semantics::TemperatureState<HabinariPort::SetpointHeatingStatus,
                                                  "setpoint_heating_status",
                                                  "Effective Heating Setpoint",
                                                  false>,
            endpoint::semantics::TemperatureState<HabinariPort::SetpointCoolingStatus,
                                                  "setpoint_cooling_status",
                                                  "Effective Cooling Setpoint",
                                                  false>,

            // ---- FB WOS / PRD / WCOS: room inputs ----------------------------
            endpoint::CommandPort<HabinariPort::WindowStatus,
                                  bool,
                                  "window_status",
                                  "Window Contact Input",
                                  application::dptids::WindowDoor,
                                  false>,
            endpoint::CommandPort<HabinariPort::PresenceStatus,
                                  bool,
                                  "presence_status",
                                  "Presence Input",
                                  application::dptids::Occupancy,
                                  false>,
            endpoint::CommandPort<HabinariPort::SwitchHeat,
                                  bool,
                                  "switch_heat",
                                  "Heating Enable Input",
                                  application::dptids::Enable,
                                  false>,
            endpoint::CommandPort<HabinariPort::SwitchCool,
                                  bool,
                                  "switch_cool",
                                  "Cooling Enable Input",
                                  application::dptids::Enable,
                                  false>,
            endpoint::CommandPort<HabinariPort::ChangeOverStatus,
                                  bool,
                                  "change_over_status",
                                  "Water Changeover Input (1=Heat)",
                                  application::dptids::HeatCool,
                                  false>,

            // ---- FB RTC: controller outputs ----------------------------------
            endpoint::semantics::PercentState<HabinariPort::HeatingControlValue,
                                              "heating_control_value",
                                              "Heating Control Value",
                                              false>,
            endpoint::semantics::PercentState<HabinariPort::CoolingControlValue,
                                              "cooling_control_value",
                                              "Cooling Control Value",
                                              false>,
            endpoint::semantics::SwitchState<HabinariPort::HeatingRequest,
                                             "heating_request",
                                             "Heating Demand",
                                             false>,
            endpoint::semantics::SwitchState<HabinariPort::CoolingRequest,
                                             "cooling_request",
                                             "Cooling Demand",
                                             false>,
            endpoint::StatePort<HabinariPort::HeatCoolModeStatus,
                                bool,
                                "heat_cool_mode_status",
                                "Heat/Cool Mode Status (1=Heat)",
                                application::dptids::HeatCool,
                                false>,
            endpoint::StatePort<HabinariPort::EnableHeatStatus,
                                bool,
                                "enable_heat_status",
                                "Heating Enabled Status",
                                application::dptids::Enable,
                                false>,
            endpoint::StatePort<HabinariPort::EnableCoolStatus,
                                bool,
                                "enable_cool_status",
                                "Cooling Enabled Status",
                                application::dptids::Enable,
                                false>,
            endpoint::StatePort<HabinariPort::ControllerStatus,
                                uint16_t,
                                "controller_status",
                                "Controller Status (DPT 22.101 StatusRHCC)",
                                application::dptids::StatusRHCC,
                                false>,

            // ---- Ventilation / air quality -------------------------------------
            endpoint::StateInOutPort<HabinariPort::Co2Setpoint,
                                     float,
                                     "co2_setpoint",
                                     "CO2 Setpoint",
                                     application::dptids::CO2,
                                     true>,
            endpoint::semantics::PercentState<HabinariPort::VentilationDemand,
                                              "ventilation_demand",
                                              "Ventilation Demand",
                                              false>,
            endpoint::StatePort<HabinariPort::VentilationStage,
                                uint8_t,
                                "ventilation_stage",
                                "Ventilation Stage (0=Off..4=Boost)",
                                application::dptids::Level,
                                false>,
            endpoint::StateInOutPort<HabinariPort::VentilationMode,
                                     uint8_t,
                                     "ventilation_mode",
                                     "Ventilation Mode (0=Auto 1=Manual 2=Off 3=Boost)",
                                     application::dptids::Level,
                                     true>,
            endpoint::semantics::SwitchState<HabinariPort::VentilationBoostRequest,
                                             "ventilation_boost_request",
                                             "Ventilation Boost Request",
                                             false>,
            endpoint::semantics::SwitchState<HabinariPort::DehumidifyRequest,
                                             "dehumidify_request",
                                             "Dehumidification Request",
                                             false>,
            endpoint::StatePort<HabinariPort::AirQualityStatus,
                                uint16_t,
                                "air_quality_status",
                                "Air Quality Status (bitset)",
                                application::dptids::Counter16,
                                false>,

            // ---- Device diagnostics ---------------------------------------------
            endpoint::semantics::AlarmState<HabinariPort::DeviceFault,
                                            "device_fault",
                                            "Device Fault",
                                            false>,
            // KNX FB sensor status octets (DPT 21.001 StatusGen), one per
            // physical sensor package rather than per measurand: a failure is a
            // property of the part, and this keeps the object count honest.
            endpoint::StatePort<HabinariPort::RoomSensorStatus,
                                uint8_t,
                                "room_sensor_status",
                                "Room Sensor Status (DPT 21.001)",
                                application::dptids::StatusGen,
                                false>,
            endpoint::StatePort<HabinariPort::FloorProbeStatus,
                                uint8_t,
                                "floor_probe_status",
                                "Floor Probe Status (DPT 21.001)",
                                application::dptids::StatusGen,
                                false>,
            endpoint::StatePort<HabinariPort::AirQualitySensorStatus,
                                uint8_t,
                                "air_quality_sensor_status",
                                "Air Quality Sensor Status (DPT 21.001)",
                                application::dptids::StatusGen,
                                false>,
            // Which physical packages are fully delivering, one bit each
            // (HDC3020, BME688, SCD4x, SHT4x) — a part that has lost one of its
            // measurands while still producing the rest reads as degraded here.
            // The StatusGen octets above say whether a *measurement* is healthy;
            // this says which *part* is, which is what a service visit needs.
            endpoint::StatePort<HabinariPort::SensorHealthMask,
                                uint8_t,
                                "sensor_health_mask",
                                "Sensor Package Health (bitmask)",
                                application::dptids::Level,
                                false>,
            // The board carries three sensors that measure room temperature and
            // three that measure humidity. When they stop agreeing, one of them
            // has drifted — a fault that is invisible to any single-sensor
            // device and that silently controls the room to the wrong value.
            endpoint::semantics::AlarmState<HabinariPort::SensorDisagreementAlarm,
                                            "sensor_disagreement_alarm",
                                            "Sensor Cross-Check Alarm",
                                            false>,

            // ---- Derived events -------------------------------------------------
            endpoint::semantics::AlarmState<HabinariPort::FireAlarm,
                                            "fire_alarm",
                                            "Rapid Temperature Rise / Fire Alarm (advisory)",
                                            false>,
            endpoint::semantics::AlarmState<HabinariPort::FirePreAlarm,
                                            "fire_pre_alarm",
                                            "Rapid Temperature Rise Pre-Alarm",
                                            false>,
            endpoint::StatePort<HabinariPort::TemperatureTrend,
                                float,
                                "temperature_trend",
                                "Room Temperature Trend (K/h)",
                                application::dptids::TemperatureDelta,
                                false>,
            endpoint::semantics::OccupancyState<HabinariPort::OccupancyDetected,
                                                "occupancy_detected",
                                                "Occupancy Detected (from CO2)",
                                                false>,
            endpoint::StatePort<HabinariPort::EstimatedOccupants,
                                uint8_t,
                                "estimated_occupants",
                                "Estimated Occupants (indication only)",
                                application::dptids::Level,
                                false>,
            endpoint::semantics::WindowDoorState<HabinariPort::WindowOpenDetected,
                                                 "window_open_detected",
                                                 "Open Window Detected (from air change)",
                                                 false>,
            endpoint::semantics::AlarmAckCommand<HabinariPort::AlarmAcknowledge,
                                                 "alarm_acknowledge",
                                                 "Alarm Acknowledge",
                                                 false>>(
            ProductIdentity{
                .productKey = kProductKey,
                .productDisplayName = kProductDisplayName,
                // Development placeholder. A device distributed as a KNX
                // product needs a manufacturer ID assigned by the KNX
                // Association and a certified application program; replace
                // this with yours before shipping anything. See
                // THIRD-PARTY-NOTICES.md.
                .manufacturerId = ManufacturerId(0x00FA),
                .medium = kProductMedium,
                .applicationNumber = kApplicationNumber,
                .applicationVersion = kApplicationVersion,
                .firmwareRevision = 1,
                .maxApduLength = 254,
                // Feeds both the device's PID_HARDWARE_TYPE / PID_VERSION /
                // PID_ORDER_INFO and the generated knxprod's hardware entry,
                // which ETS cross-checks before allowing a download.
                .hardwareSerialNumber = kHardwareSerialNumber,
                .hardwareVersion = 1,
                .orderNumber = kOrderNumber,
            },
            PersistencePolicy{
                .namespacePrefix = kPersistenceNamespace,
                .schemaVersion = 2,
                .persistKnxState = true,
            },
            // Must stay in step with knx_data_secure.cpp, which always installs a
            // factory tool key, and with CONFIG_KNX_ENABLE_SECURITY, which
            // sdkconfig.defaults sets for both media (knx_data_secure.cpp
            // refuses to compile without it). This is only what the ETS
            // catalogue entry advertises, and a device that claims Data Secure
            // but cannot honour it fails commissioning. No per-object requirement, so enabling Data
            // Secure stays the integrator's choice and the board still
            // commissions into plain, non-secure installations.
            SecurityPolicy{
                .dataSecureCapable = true,
                .groupObjectRequirement = SecurityRequirement::None,
                .individualAddressEntries = 8,
                // 0 → one group-key slot per group object.
                .groupKeyTableEntries = 0,
                .p2pKeyTableEntries = 1,
            }));

} // namespace habinari_knx
