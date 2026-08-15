/*
 * Which field-bus personalities this image contains.
 *
 * An explicit array behind Kconfig guards, rather than linker-section magic:
 * the set is small, it is decided at build time, and "what is in this image?"
 * should be answerable by reading one file rather than by dumping sections.
 */
#include "protocol_adapter.h"

#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "protocols";

#if CONFIG_SENSOR_BOARD_PROTOCOL_KNX
extern const protocol_adapter_t knx_protocol_adapter;
#endif
#if CONFIG_SENSOR_BOARD_PROTOCOL_MODBUS
extern const protocol_adapter_t modbus_protocol_adapter;
#endif
#if CONFIG_SENSOR_BOARD_PROTOCOL_MQTT
extern const protocol_adapter_t mqtt_protocol_adapter;
#endif

/*
 * The KNX TP1 driver bit-bangs a 104 us bit cell from a GPIO edge ISR and a
 * gptimer alarm, sampling at 52 us. The comment in sdkconfig.defaults records
 * the measured worst-case ISR jitter on this board as 51 us — one microsecond
 * of margin before a late alarm lands past the next cell's edge and steals its
 * bit. The Wi-Fi / BLE / 802.15.4 stacks install their own high-priority
 * interrupts and take long critical sections, and the radio MAC has hard
 * deadlines of its own; they will spend margin that is not there.
 *
 * The board is not asked to do both anyway — KNX is bus-powered and the radio
 * personalities need the 5-30 V auxiliary supply. So this is a build-time error
 * rather than a note somebody has to remember, and a KNX image does not link the
 * radio stacks at all: the timing guarantee is a property of the binary, not of
 * a runtime flag someone could get wrong.
 */
#if CONFIG_SENSOR_BOARD_PROTOCOL_KNX && CONFIG_SENSOR_BOARD_PROTOCOL_MQTT
#error "KNX TP1 and the Wi-Fi/MQTT personality cannot share an image: the bit-banged TP1 receiver has ~1 us of timing margin and the radio stack will consume it. Pick one in menuconfig -> Sensor board protocols."
#endif

/*
 * The same argument, applied to the BLE service channel — and it is worth being
 * explicit rather than leaving it to Kconfig, because CONFIG_BT_ENABLED is not
 * ours to gate. CONFIG_SENSOR_BOARD_OOB_BLE already depends on !KNX and would be
 * silently dropped from a KNX configuration; this catches the other route in,
 * where somebody enables the Bluetooth controller directly and expects the TP1
 * receiver to survive it.
 *
 * There is no loss here. A KNX device does not need an out-of-band channel: ETS
 * reaches it through the programming button and the individual address, which is
 * the same circularity-breaking trick, standardised and already in the box.
 */
#if CONFIG_SENSOR_BOARD_PROTOCOL_KNX && CONFIG_BT_ENABLED
#error "KNX TP1 and the Bluetooth controller cannot share an image, for the same reason as the Wi-Fi personality: the bit-banged TP1 receiver has ~1 us of ISR-jitter margin and the BLE MAC has hard deadlines of its own. A KNX device is configured from ETS and needs no out-of-band channel."
#endif

static const protocol_adapter_t *const s_adapters[] = {
#if CONFIG_SENSOR_BOARD_PROTOCOL_KNX
    &knx_protocol_adapter,
#endif
#if CONFIG_SENSOR_BOARD_PROTOCOL_MODBUS
    &modbus_protocol_adapter,
#endif
#if CONFIG_SENSOR_BOARD_PROTOCOL_MQTT
    &mqtt_protocol_adapter,
#endif
    NULL,  /* keeps the array well-formed when nothing is enabled */
};

#define ADAPTER_COUNT ((sizeof(s_adapters) / sizeof(s_adapters[0])) - 1u)

const protocol_adapter_t *const *protocol_adapters(size_t *count)
{
    if (count != NULL) {
        *count = ADAPTER_COUNT;
    }
    return s_adapters;
}

esp_err_t protocol_adapters_start_all(void)
{
    if (ADAPTER_COUNT == 0u) {
        ESP_LOGW(TAG, "No protocol adapter compiled in — sensing and control run, "
                      "but nothing is published");
        return ESP_OK;
    }

    esp_err_t first_required_error = ESP_OK;
    for (size_t i = 0; i < ADAPTER_COUNT; ++i) {
        const protocol_adapter_t *adapter = s_adapters[i];
        if (adapter->start == NULL) {
            continue;
        }

        const esp_err_t err = adapter->start();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "%s: started", adapter->name);
            continue;
        }

        /* One personality failing must not take the others down: a board with a
         * disconnected RS-485 pair is still a working KNX device. */
        if (adapter->required) {
            ESP_LOGE(TAG, "%s: failed to start: %s", adapter->name, esp_err_to_name(err));
            if (first_required_error == ESP_OK) {
                first_required_error = err;
            }
        } else {
            ESP_LOGW(TAG, "%s: failed to start (optional): %s", adapter->name,
                     esp_err_to_name(err));
        }
    }
    return first_required_error;
}

void protocol_adapters_notify_tick(uint32_t generation)
{
    for (size_t i = 0; i < ADAPTER_COUNT; ++i) {
        if (s_adapters[i]->on_control_tick != NULL) {
            s_adapters[i]->on_control_tick(generation);
        }
    }
}

void protocol_adapters_notify_sensor_data(const sensor_data_t *data)
{
    for (size_t i = 0; i < ADAPTER_COUNT; ++i) {
        if (s_adapters[i]->on_sensor_data != NULL) {
            s_adapters[i]->on_sensor_data(data);
        }
    }
}

bool protocol_adapters_identify_active(void)
{
    for (size_t i = 0; i < ADAPTER_COUNT; ++i) {
        if (s_adapters[i]->identify_active != NULL && s_adapters[i]->identify_active()) {
            return true;
        }
    }
    return false;
}

bool protocol_adapters_own_programming_mode(void)
{
    for (size_t i = 0; i < ADAPTER_COUNT; ++i) {
        if (s_adapters[i]->owns_programming_mode) {
            return true;
        }
    }
    return false;
}
