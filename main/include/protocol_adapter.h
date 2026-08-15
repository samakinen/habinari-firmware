#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file protocol_adapter.h
 * @brief What a field-bus personality has to provide, and nothing more.
 *
 * The board can speak KNX TP1, Modbus RTU or MQTT-over-Wi-Fi. They are
 * alternatives, not layers: which ones are compiled in is a build-time choice
 * (see Kconfig, menu "Sensor board protocols"), because the radio and the
 * bit-banged KNX TP1 driver cannot share a CPU — see the build-time guard in
 * protocol_registry.c for why that is enforced rather than documented.
 *
 * An adapter is a *mapping*, never an owner. It does not run control loops, own
 * settings or decide anything: it turns control_state_t into its wire format and
 * turns wire traffic into control_state_write() commands. Everything an adapter
 * needs is in control_state.h (C) or control_service.hpp (C++ in-tree).
 *
 * Adding a protocol is therefore: write the mapping, define one descriptor of
 * this type, add it to protocol_registry.c behind a Kconfig symbol. Nothing in
 * main.c, the control core or any other adapter changes.
 */
typedef struct protocol_adapter {
    /// Short lowercase identifier, used in logs and in the boot banner.
    const char *name;

    /// Bring the personality up. Called once, from app_main, after the control
    /// service is running so control_state_get() is already answerable.
    esp_err_t (*start)(void);

    /// Called after every control tick, from the control task. MUST NOT BLOCK —
    /// wake your own task and return. `generation` increments once per tick, so
    /// an adapter that missed ticks can tell.
    void (*on_control_tick)(uint32_t generation);

    /// Called when a new fused measurement record lands, from the sensor task.
    /// Optional; the same data is reachable via control_state_get(). Must not
    /// block. Use it when the protocol wants to report promptly on new data
    /// rather than at the next tick.
    void (*on_sensor_data)(const sensor_data_t *data);

    /// Does this adapter currently want the board LED lit? KNX programming mode
    /// and a Modbus master's LED coil both answer yes here. Optional.
    bool (*identify_active)(void);

    /**
     * True if this adapter, not the control service, owns programming mode.
     *
     * Programming mode is the board's one protocol-neutral "this device is
     * selected" state: the button raises it, the LED shows it, and BLE pairing
     * and Modbus addressing are both gated on it. Normally the control service
     * owns the flag and toggles it itself.
     *
     * KNX is the exception, and has to be. Its stack holds programming mode as
     * part of the device object, ETS can set it over the bus, and the stack is
     * only safe to touch from the KNX task — so the service must not toggle the
     * flag behind its back. An adapter that sets this consumes
     * takeIdentifyToggleRequest() on its own task and reports the result back
     * through control_service_set_identify_active().
     *
     * At most one adapter may claim it; with none, the service does the work,
     * which is why a Modbus-only build blinks its LED at all.
     */
    bool owns_programming_mode;

    /// If start() fails: true aborts the boot, false logs and carries on. Only
    /// the personality the device is actually wired to should be required, and
    /// even then it is usually better to boot and complain — a device that
    /// refuses to start is a device that cannot tell you why.
    bool required;
} protocol_adapter_t;

/// The compiled-in adapters, in start order. Never NULL; `*count` may be 0.
const protocol_adapter_t *const *protocol_adapters(size_t *count);

/// Start every adapter. Returns the first error from an adapter marked
/// `required`; failures of optional adapters are logged and skipped.
esp_err_t protocol_adapters_start_all(void);

/// Fan-outs. Safe to call with no adapters compiled in.
void protocol_adapters_notify_tick(uint32_t generation);
void protocol_adapters_notify_sensor_data(const sensor_data_t *data);

/// True if any adapter wants the LED lit.
bool protocol_adapters_identify_active(void);

/// True if some adapter owns programming mode, so the control service must
/// leave the toggle request alone. See protocol_adapter_t::owns_programming_mode.
bool protocol_adapters_own_programming_mode(void);

#ifdef __cplusplus
}
#endif
