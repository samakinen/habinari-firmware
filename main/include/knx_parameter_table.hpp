// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include "control_core.hpp"
#include "knx_product.hpp"

#include <cstddef>
#include <tuple>
#include <type_traits>

/**
 * @file knx_parameter_table.hpp
 * @brief ETS parameters: where each one's value lands in the control state.
 *
 * One row per parameter. The same table is applied at start-up and after every
 * ETS download (onParametersChanged), so the two can never map a parameter
 * differently.
 *
 * A setter receives the value in the type knx_product.hpp declares for that
 * parameter — Dpt9Float (KNX 2-byte half-float, implicitly a float), uint16_t
 * for counts and seconds, uint8_t for enumerations and percentages. The target
 * it writes to is anything shaped like control::ServiceState — `hvac`
 * (Settings), `fusion` (sensor_fusion_config_t) and `fusionConfigDirty` — so
 * the table can be exercised on the host without FreeRTOS. In the firmware it
 * runs with the control mutex held.
 *
 * That every parameter of the product has exactly one row is checked at compile
 * time below, so a parameter added to knx_product.hpp without a destination
 * here is a build error rather than a setting ETS writes and nothing reads.
 */

namespace habinari_knx {

template <HabinariParameter Id, typename Apply>
struct ParameterRow {
    Apply apply;
};

template <HabinariParameter Id, typename Apply>
constexpr ParameterRow<Id, Apply> row(Apply apply)
{
    return ParameterRow<Id, Apply>{apply};
}

template <HabinariParameter Id>
using ParameterValue = typename std::remove_cvref_t<
    decltype(kHabinariProduct)>::template ParameterValueType<Id>;

// Fusion tuning lands in the fusion config rather than in Settings and is
// pushed down to the acquisition side by the control tick, hence the dirty flag.
inline constexpr auto kParameterTable = std::make_tuple(
    row<HabinariParameter::MeasurementHeartbeatSeconds>([](auto &g, auto v) {
        g.hvac.heartbeatSeconds = v;
    }),
    row<HabinariParameter::MeasurementMinRepTimeSeconds>([](auto &g, auto v) {
        g.hvac.minRepTimeSeconds = v;
    }),
    row<HabinariParameter::RoomTemperatureOffset>([](auto &g, auto v) {
        g.hvac.roomTemperatureOffsetK = v;
    }),
    row<HabinariParameter::RoomTemperatureCov>([](auto &g, auto v) {
        g.hvac.roomTemperatureCovK = v;
    }),
    row<HabinariParameter::RoomHumidityOffset>([](auto &g, auto v) {
        g.hvac.roomHumidityOffsetPct = v;
    }),
    row<HabinariParameter::RoomHumidityCov>([](auto &g, auto v) {
        g.hvac.roomHumidityCovPct = v;
    }),
    row<HabinariParameter::Co2Cov>([](auto &g, auto v) {
        g.hvac.co2CovPpm = static_cast<float>(v);
    }),
    row<HabinariParameter::PressureCov>([](auto &g, auto v) {
        g.hvac.pressureCovPa = static_cast<float>(v);
    }),
    row<HabinariParameter::AirQualityCov>([](auto &g, auto v) {
        g.hvac.airQualityCovIndex = static_cast<float>(v);
    }),
    row<HabinariParameter::FloorTemperatureOffset>([](auto &g, auto v) {
        g.hvac.floorTemperatureOffsetK = v;
    }),
    row<HabinariParameter::FloorTemperatureCov>([](auto &g, auto v) {
        g.hvac.floorTemperatureCovK = v;
    }),
    row<HabinariParameter::FloorHumidityCov>([](auto &g, auto v) {
        g.hvac.floorHumidityCovPct = v;
    }),
    row<HabinariParameter::DerivedValueCov>([](auto &g, auto v) {
        g.hvac.derivedCov = v;
    }),
    row<HabinariParameter::AltitudeM>([](auto &g, auto v) {
        g.hvac.altitudeM = static_cast<float>(v);
    }),
    row<HabinariParameter::ControllerDefaultEnable>([](auto &g, auto v) {
        g.hvac.controllerDefaultEnable = (v != 0);
    }),
    row<HabinariParameter::DefaultHvacOperatingMode>([](auto &g, auto v) {
        g.hvac.defaultHvacOperatingMode = static_cast<habinari::hvac::OperatingPreset>(v);
    }),
    row<HabinariParameter::DefaultControllerMode>([](auto &g, auto v) {
        g.hvac.defaultControllerMode = static_cast<habinari::hvac::ControllerMode>(v);
    }),
    row<HabinariParameter::HeatingEnabled>([](auto &g, auto v) {
        g.hvac.heatingEnabled = (v != 0);
    }),
    row<HabinariParameter::CoolingEnabled>([](auto &g, auto v) {
        g.hvac.coolingEnabled = (v != 0);
    }),
    row<HabinariParameter::HeatCoolChangeoverMode>([](auto &g, auto v) {
        g.hvac.heatCoolChangeoverMode = static_cast<habinari::hvac::HeatCoolChangeoverMode>(v);
    }),
    row<HabinariParameter::HeatCoolChangeoverPolarity>([](auto &g, auto v) {
        g.hvac.heatCoolChangeoverPolarityInverted = (v != 0);
    }),
    row<HabinariParameter::MinimumHeatCoolChangeoverSeconds>([](auto &g, auto v) {
        g.hvac.minimumHeatCoolChangeoverSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::WindowOpenBehavior>([](auto &g, auto v) {
        g.hvac.windowOpenBehavior = static_cast<habinari::hvac::WindowOpenBehavior>(v);
    }),
    row<HabinariParameter::PresenceBehavior>([](auto &g, auto v) {
        g.hvac.presenceBehavior = static_cast<habinari::hvac::PresenceBehavior>(v);
    }),
    row<HabinariParameter::SensorFaultBehavior>([](auto &g, auto v) {
        g.hvac.sensorFaultBehavior = static_cast<habinari::hvac::SensorFaultBehavior>(v);
    }),
    row<HabinariParameter::ComfortHeatingSetpoint>([](auto &g, auto v) {
        g.hvac.comfortHeatingSetpointC = v;
    }),
    row<HabinariParameter::StandbyHeatingReduction>([](auto &g, auto v) {
        g.hvac.standbyHeatingReductionK = v;
    }),
    row<HabinariParameter::EconomyHeatingReduction>([](auto &g, auto v) {
        g.hvac.economyHeatingReductionK = v;
    }),
    row<HabinariParameter::ProtectionHeatingSetpoint>([](auto &g, auto v) {
        g.hvac.protectionHeatingSetpointC = v;
    }),
    row<HabinariParameter::CoolingDeadband>([](auto &g, auto v) {
        g.hvac.coolingDeadbandK = v;
    }),
    row<HabinariParameter::StandbyCoolingIncrease>([](auto &g, auto v) {
        g.hvac.standbyCoolingIncreaseK = v;
    }),
    row<HabinariParameter::EconomyCoolingIncrease>([](auto &g, auto v) {
        g.hvac.economyCoolingIncreaseK = v;
    }),
    row<HabinariParameter::ProtectionCoolingSetpoint>([](auto &g, auto v) {
        g.hvac.protectionCoolingSetpointC = v;
    }),
    row<HabinariParameter::MinSetpoint>([](auto &g, auto v) {
        g.hvac.minSetpointC = v;
    }),
    row<HabinariParameter::MaxSetpoint>([](auto &g, auto v) {
        g.hvac.maxSetpointC = v;
    }),
    row<HabinariParameter::MaxSetpointShift>([](auto &g, auto v) {
        g.hvac.maxSetpointShiftK = v;
    }),
    row<HabinariParameter::HeatingControlAlgorithm>([](auto &g, auto v) {
        g.hvac.heatingControlAlgorithm = static_cast<habinari::hvac::ControlAlgorithm>(v);
    }),
    row<HabinariParameter::HeatingKp>([](auto &g, auto v) {
        g.hvac.heatingKp = v;
    }),
    row<HabinariParameter::HeatingTiSeconds>([](auto &g, auto v) {
        g.hvac.heatingTiSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::HeatingTdSeconds>([](auto &g, auto v) {
        g.hvac.heatingTdSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::HeatingMinimumOutputPercent>([](auto &g, auto v) {
        g.hvac.heatingMinOutputPercent = v;
    }),
    row<HabinariParameter::HeatingMaximumOutputPercent>([](auto &g, auto v) {
        g.hvac.heatingMaxOutputPercent = v;
    }),
    row<HabinariParameter::CoolingControlAlgorithm>([](auto &g, auto v) {
        g.hvac.coolingControlAlgorithm = static_cast<habinari::hvac::ControlAlgorithm>(v);
    }),
    row<HabinariParameter::CoolingKp>([](auto &g, auto v) {
        g.hvac.coolingKp = v;
    }),
    row<HabinariParameter::CoolingTiSeconds>([](auto &g, auto v) {
        g.hvac.coolingTiSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::CoolingTdSeconds>([](auto &g, auto v) {
        g.hvac.coolingTdSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::CoolingMinimumOutputPercent>([](auto &g, auto v) {
        g.hvac.coolingMinOutputPercent = v;
    }),
    row<HabinariParameter::CoolingMaximumOutputPercent>([](auto &g, auto v) {
        g.hvac.coolingMaxOutputPercent = v;
    }),
    row<HabinariParameter::ThermostatHysteresis>([](auto &g, auto v) {
        g.hvac.thermostatHysteresisC = v;
    }),
    row<HabinariParameter::BinaryDemandStrategy>([](auto &g, auto v) {
        g.hvac.binaryDemandStrategy = static_cast<habinari::hvac::BinaryDemandStrategy>(v);
    }),
    row<HabinariParameter::BinaryDemandThresholdPercent>([](auto &g, auto v) {
        g.hvac.binaryDemandThresholdPercent = v;
    }),
    row<HabinariParameter::FrostAlarmTemperature>([](auto &g, auto v) {
        g.hvac.frostAlarmTemperatureC = v;
    }),
    row<HabinariParameter::OverheatAlarmTemperature>([](auto &g, auto v) {
        g.hvac.overheatAlarmTemperatureC = v;
    }),
    row<HabinariParameter::MaxFloorTemperature>([](auto &g, auto v) {
        g.hvac.maxFloorTemperatureC = v;
    }),
    row<HabinariParameter::MinFloorTemperature>([](auto &g, auto v) {
        g.hvac.minFloorTemperatureC = v;
    }),
    row<HabinariParameter::FloorHysteresis>([](auto &g, auto v) {
        g.hvac.floorHysteresisK = v;
    }),
    row<HabinariParameter::FloorComfortOutputPercent>([](auto &g, auto v) {
        g.hvac.floorComfortOutputPercent = v;
    }),
    row<HabinariParameter::DewPointSurfaceSource>([](auto &g, auto v) {
        g.hvac.dewPointSurfaceSource = static_cast<habinari::hvac::DewPointSurfaceSource>(v);
    }),
    row<HabinariParameter::DewPointMargin>([](auto &g, auto v) {
        g.hvac.dewPointMarginK = v;
    }),
    row<HabinariParameter::DewPointHysteresis>([](auto &g, auto v) {
        g.hvac.dewPointHysteresisK = v;
    }),
    row<HabinariParameter::BlockCoolingOnDewPointAlarm>([](auto &g, auto v) {
        g.hvac.blockCoolingOnDewPointAlarm = (v != 0);
    }),
    row<HabinariParameter::FloorMoistureThreshold>([](auto &g, auto v) {
        g.hvac.floorMoistureThresholdPct = v;
    }),
    row<HabinariParameter::FloorMoistureHysteresis>([](auto &g, auto v) {
        g.hvac.floorMoistureHysteresisPct = v;
    }),
    row<HabinariParameter::FloorMoistureExcess>([](auto &g, auto v) {
        g.hvac.floorMoistureExcessGm3 = v;
    }),
    row<HabinariParameter::VentilationSetpoint>([](auto &g, auto v) {
        g.hvac.ventilationSetpointPpm = static_cast<float>(v);
    }),
    row<HabinariParameter::VentilationBand>([](auto &g, auto v) {
        g.hvac.ventilationBandPpm = static_cast<float>(v);
    }),
    row<HabinariParameter::HumidityBoostThreshold>([](auto &g, auto v) {
        g.hvac.humidityBoostPct = v;
    }),
    row<HabinariParameter::HumidityBoostBand>([](auto &g, auto v) {
        g.hvac.humidityBandPct = v;
    }),
    row<HabinariParameter::VocBoostThreshold>([](auto &g, auto v) {
        g.hvac.vocThresholdIndex = static_cast<float>(v);
    }),
    row<HabinariParameter::VocBoostBand>([](auto &g, auto v) {
        g.hvac.vocBandIndex = static_cast<float>(v);
    }),
    row<HabinariParameter::VentilationBaseDemandPercent>([](auto &g, auto v) {
        g.hvac.ventilationBaseDemandPercent = v;
    }),
    row<HabinariParameter::VentilationManualDemandPercent>([](auto &g, auto v) {
        g.hvac.ventilationManualDemandPercent = v;
    }),
    row<HabinariParameter::SensorFilterSeconds>([](auto &g, auto v) {
        g.fusion.filter_tau_seconds = static_cast<float>(v);
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::TemperatureCrossCheck>([](auto &g, auto v) {
        g.fusion.temperature_cross_check_k = v;
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::HumidityCrossCheck>([](auto &g, auto v) {
        g.fusion.humidity_cross_check_pct = v;
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::FireRateOfRise>([](auto &g, auto v) {
        g.fusion.fire_rate_of_rise_k_per_min = v;
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::FireAbsoluteTemperature>([](auto &g, auto v) {
        g.fusion.fire_absolute_alarm_c = v;
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::FireConfirmSeconds>([](auto &g, auto v) {
        g.fusion.fire_confirm_seconds = static_cast<float>(v);
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::FireRequireAirQuality>([](auto &g, auto v) {
        g.fusion.fire_require_air_quality = (v != 0);
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::Co2OccupancyEnabled>([](auto &g, auto v) {
        g.fusion.co2_occupancy_enabled = (v != 0);
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::WindowDetectEnabled>([](auto &g, auto v) {
        g.fusion.window_detect_enabled = (v != 0);
        g.fusionConfigDirty = true;
    })
);

namespace detail {

template <typename ParametersT, typename TargetT, HabinariParameter Id, typename Apply>
void loadParameter(const ParametersT &params, TargetT &target, const ParameterRow<Id, Apply> &)
{
    Apply{}(target, params.template get<Id>());
}

}  // namespace detail

/// Apply every row to @p target, each with the value @p params holds for it.
/// `params` is anything with `get<Id>()`: the block view of the product's
/// parameters, `parameters().block<HabinariParameter>()`.
template <typename ParametersT, typename TargetT>
void loadParameters(const ParametersT &params, TargetT &target)
{
    std::apply([&params, &target](const auto &...rows) {
        (detail::loadParameter(params, target, rows), ...);
    }, kParameterTable);
}

namespace detail {

template <HabinariParameter Id, typename Apply>
constexpr HabinariParameter rowId(const ParameterRow<Id, Apply> &)
{
    return Id;
}

template <HabinariParameter Id>
constexpr std::size_t rowsFor()
{
    return std::apply(
        [](const auto &...rows) { return (std::size_t{0} + ... + (rowId(rows) == Id ? 1u : 0u)); },
        kParameterTable);
}

template <typename BlockT>
struct EveryParameterHasOneRow;

template <typename... Parameters>
struct EveryParameterHasOneRow<knx::product::params::ParameterBlock<HabinariParameter, Parameters...>> {
    static constexpr bool value = ((rowsFor<Parameters::id>() == 1u) && ...);
};

}  // namespace detail

static_assert(
    detail::EveryParameterHasOneRow<std::remove_cvref_t<decltype(kHabinariParameterBlock)>>::value,
    "kParameterTable must have exactly one row per ETS parameter");

}  // namespace habinari_knx
