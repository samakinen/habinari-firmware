/*
 * The out-of-band settings registry.
 *
 * Portable C on purpose: no NVS, no FreeRTOS, no radio. Storage belongs to
 * whoever registered the item, and transport belongs to whoever renders the
 * registry. What is left here — the table, the type system, and the text
 * encoding both ends agree on — is exactly the part that is worth testing on a
 * host, and main/test/test_device_config.cpp does.
 */
#include "device_config.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Pointers, not copies: an owner's descriptor array is static and outlives
 * everything, and copying 32 items would cost 2 kB of RAM to duplicate data
 * that is already in flash. */
static const device_config_item_t *s_items[DEVICE_CONFIG_MAX_ITEMS];
static size_t s_count;
static bool s_reboot_pending;

/* --- Helpers -------------------------------------------------------------- */

static bool str_equal_ci(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0') {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == *b;
}

static size_t string_limit(const device_config_item_t *item)
{
    const size_t hard = DEVICE_CONFIG_STRING_MAX - 1u;
    if (item->max <= 0.0f) {
        return hard;
    }
    const size_t configured = (size_t)item->max;
    return configured < hard ? configured : hard;
}

static bool item_is_writable(const device_config_item_t *item)
{
    return item->set != NULL && (item->flags & DEVICE_CONFIG_FLAG_READ_ONLY) == 0u;
}

/* Bounds are floats, which is exact for every integer up to 2^24. Every item in
 * this firmware is an address, a baud code, a port or a percentage, so that is
 * three orders of magnitude of headroom; an item that needed more would need a
 * different bound type, not a wider cast here. */
static bool within_bounds(const device_config_item_t *item, double value)
{
    if (item->min == 0.0f && item->max == 0.0f) {
        return true; /* unbounded */
    }
    return value >= (double)item->min && value <= (double)item->max;
}

static esp_err_t validate(const device_config_item_t *item, const device_config_value_t *value)
{
    if (value->type != item->type) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (item->type) {
    case DEVICE_CONFIG_TYPE_BOOL:
        return ESP_OK;

    case DEVICE_CONFIG_TYPE_UINT:
        return within_bounds(item, (double)value->num.u) ? ESP_OK : ESP_ERR_INVALID_ARG;

    case DEVICE_CONFIG_TYPE_INT:
        return within_bounds(item, (double)value->num.i) ? ESP_OK : ESP_ERR_INVALID_ARG;

    case DEVICE_CONFIG_TYPE_FLOAT:
        if (isnan(value->num.f) || isinf(value->num.f)) {
            return ESP_ERR_INVALID_ARG;
        }
        return within_bounds(item, (double)value->num.f) ? ESP_OK : ESP_ERR_INVALID_ARG;

    case DEVICE_CONFIG_TYPE_ENUM:
        /* Labels are the bound when there are labels: an enum with five names
         * has five legal values, and repeating that in min/max would be a
         * second place to get it wrong. */
        if (item->enum_count > 0u) {
            return value->num.u < item->enum_count ? ESP_OK : ESP_ERR_INVALID_ARG;
        }
        return within_bounds(item, (double)value->num.u) ? ESP_OK : ESP_ERR_INVALID_ARG;

    case DEVICE_CONFIG_TYPE_STRING:
        return strnlen(value->str, DEVICE_CONFIG_STRING_MAX) <= string_limit(item)
                   ? ESP_OK
                   : ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_INVALID_ARG;
}

/* --- Registry ------------------------------------------------------------- */

static bool item_well_formed(const device_config_item_t *item)
{
    if (item->key == NULL || item->get == NULL) {
        return false;
    }
    const size_t key_len = strnlen(item->key, DEVICE_CONFIG_KEY_MAX);
    if (key_len == 0u || key_len >= DEVICE_CONFIG_KEY_MAX) {
        return false;
    }
    if (item->type == DEVICE_CONFIG_TYPE_ENUM && item->enum_count > 0u
        && item->enum_labels == NULL) {
        return false;
    }
    return true;
}

esp_err_t device_config_register(const device_config_item_t *items, size_t count)
{
    if (items == NULL || count == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_count + count > DEVICE_CONFIG_MAX_ITEMS) {
        return ESP_ERR_NO_MEM;
    }

    /* Validate the whole batch before accepting any of it. Half an owner's
     * settings showing up is worse than none of them: it looks like a working
     * device with a missing knob rather than like the programming error it is. */
    for (size_t i = 0; i < count; ++i) {
        if (!item_well_formed(&items[i])) {
            return ESP_ERR_INVALID_ARG;
        }
        if (device_config_find(items[i].key) != NULL) {
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t j = 0; j < i; ++j) {
            if (strncmp(items[i].key, items[j].key, DEVICE_CONFIG_KEY_MAX) == 0) {
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    for (size_t i = 0; i < count; ++i) {
        s_items[s_count++] = &items[i];
    }
    return ESP_OK;
}

void device_config_reset(void)
{
    s_count = 0;
    s_reboot_pending = false;
    memset(s_items, 0, sizeof(s_items));
}

size_t device_config_count(void)
{
    return s_count;
}

const device_config_item_t *device_config_at(size_t index)
{
    return index < s_count ? s_items[index] : NULL;
}

const device_config_item_t *device_config_find(const char *key)
{
    if (key == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < s_count; ++i) {
        if (strncmp(s_items[i]->key, key, DEVICE_CONFIG_KEY_MAX) == 0) {
            return s_items[i];
        }
    }
    return NULL;
}

size_t device_config_index_of(const device_config_item_t *item)
{
    for (size_t i = 0; i < s_count; ++i) {
        if (s_items[i] == item) {
            return i;
        }
    }
    return SIZE_MAX;
}

bool device_config_reboot_pending(void)
{
    return s_reboot_pending;
}

/* --- Access --------------------------------------------------------------- */

esp_err_t device_config_get(const device_config_item_t *item, device_config_value_t *out)
{
    if (item == NULL || out == NULL || item->get == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->type = item->type;
    return item->get(item, out);
}

esp_err_t device_config_set(const device_config_item_t *item, const device_config_value_t *value)
{
    if (item == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!item_is_writable(item)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    const esp_err_t valid = validate(item, value);
    if (valid != ESP_OK) {
        return valid;
    }

    const esp_err_t err = item->set(item, value);
    if (err == ESP_OK && (item->flags & DEVICE_CONFIG_FLAG_REBOOT) != 0u) {
        s_reboot_pending = true;
    }
    return err;
}

esp_err_t device_config_set_text(const device_config_item_t *item, const char *text)
{
    device_config_value_t value;
    const esp_err_t err = device_config_parse(item, text, &value);
    if (err != ESP_OK) {
        return err;
    }
    return device_config_set(item, &value);
}

/* --- Text encoding -------------------------------------------------------- */

static esp_err_t parse_bool(const char *text, bool *out)
{
    static const char *const truthy[] = {"1", "true", "on", "yes"};
    static const char *const falsy[] = {"0", "false", "off", "no"};

    for (size_t i = 0; i < sizeof(truthy) / sizeof(truthy[0]); ++i) {
        if (str_equal_ci(text, truthy[i])) {
            *out = true;
            return ESP_OK;
        }
    }
    for (size_t i = 0; i < sizeof(falsy) / sizeof(falsy[0]); ++i) {
        if (str_equal_ci(text, falsy[i])) {
            *out = false;
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_ARG;
}

/* strtoul happily accepts "12abc" and "-1". A configuration channel must not:
 * a typo has to come back as an error the installer can see, not as a value
 * that is almost right. */
static esp_err_t parse_u32(const char *text, uint32_t *out)
{
    if (*text == '\0' || *text == '-' || isspace((unsigned char)*text)) {
        return ESP_ERR_INVALID_ARG;
    }
    char *end = NULL;
    errno = 0;
    const unsigned long parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = (uint32_t)parsed;
    return ESP_OK;
}

static esp_err_t parse_i32(const char *text, int32_t *out)
{
    if (*text == '\0' || isspace((unsigned char)*text)) {
        return ESP_ERR_INVALID_ARG;
    }
    char *end = NULL;
    errno = 0;
    const long parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > INT32_MAX || parsed < INT32_MIN) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = (int32_t)parsed;
    return ESP_OK;
}

static esp_err_t parse_f32(const char *text, float *out)
{
    if (*text == '\0' || isspace((unsigned char)*text)) {
        return ESP_ERR_INVALID_ARG;
    }
    char *end = NULL;
    errno = 0;
    const float parsed = strtof(text, &end);
    if (end == text || *end != '\0' || isnan(parsed) || isinf(parsed)) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = parsed;
    return ESP_OK;
}

esp_err_t device_config_parse(const device_config_item_t *item,
                              const char *text,
                              device_config_value_t *out)
{
    if (item == NULL || text == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->type = item->type;

    esp_err_t err = ESP_ERR_INVALID_ARG;
    switch (item->type) {
    case DEVICE_CONFIG_TYPE_BOOL:
        err = parse_bool(text, &out->num.b);
        break;

    case DEVICE_CONFIG_TYPE_UINT:
        err = parse_u32(text, &out->num.u);
        break;

    case DEVICE_CONFIG_TYPE_INT:
        err = parse_i32(text, &out->num.i);
        break;

    case DEVICE_CONFIG_TYPE_FLOAT:
        err = parse_f32(text, &out->num.f);
        break;

    case DEVICE_CONFIG_TYPE_ENUM:
        /* Labels first, so a technician can write "38400" as "38400" and a
         * script can write the code point. A label that looks like a number is
         * still matched as a label, which is what the author of the label meant. */
        err = ESP_ERR_INVALID_ARG;
        for (size_t i = 0; i < item->enum_count; ++i) {
            if (item->enum_labels[i] != NULL && str_equal_ci(text, item->enum_labels[i])) {
                out->num.u = (uint32_t)i;
                err = ESP_OK;
                break;
            }
        }
        if (err != ESP_OK) {
            err = parse_u32(text, &out->num.u);
        }
        break;

    case DEVICE_CONFIG_TYPE_STRING:
        if (strnlen(text, DEVICE_CONFIG_STRING_MAX) >= DEVICE_CONFIG_STRING_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
        /* Bounded above, so the copy cannot truncate silently. */
        strncpy(out->str, text, DEVICE_CONFIG_STRING_MAX - 1u);
        err = ESP_OK;
        break;
    }

    if (err != ESP_OK) {
        return err;
    }
    return validate(item, out);
}

int device_config_format(const device_config_item_t *item,
                         const device_config_value_t *value,
                         char *out,
                         size_t out_len)
{
    if (item == NULL || value == NULL || out == NULL || out_len == 0u) {
        return -1;
    }

    int written;
    if ((item->flags & DEVICE_CONFIG_FLAG_SECRET) != 0u) {
        /* Whatever the hook returned, the only thing that leaves the device is
         * whether there is something there. The check lives here rather than in
         * each channel so that adding a channel cannot reopen the hole. */
        const bool set = (item->type == DEVICE_CONFIG_TYPE_STRING) ? (value->str[0] != '\0')
                                                                  : (value->num.u != 0u);
        written = snprintf(out, out_len, "%s", set ? "<set>" : "<unset>");
    } else {
        switch (item->type) {
        case DEVICE_CONFIG_TYPE_BOOL:
            written = snprintf(out, out_len, "%s", value->num.b ? "on" : "off");
            break;
        case DEVICE_CONFIG_TYPE_UINT:
            written = snprintf(out, out_len, "%u", (unsigned)value->num.u);
            break;
        case DEVICE_CONFIG_TYPE_INT:
            written = snprintf(out, out_len, "%d", (int)value->num.i);
            break;
        case DEVICE_CONFIG_TYPE_FLOAT:
            written = snprintf(out, out_len, "%g", (double)value->num.f);
            break;
        case DEVICE_CONFIG_TYPE_ENUM:
            if (value->num.u < item->enum_count && item->enum_labels[value->num.u] != NULL) {
                written = snprintf(out, out_len, "%s", item->enum_labels[value->num.u]);
            } else {
                written = snprintf(out, out_len, "%u", (unsigned)value->num.u);
            }
            break;
        case DEVICE_CONFIG_TYPE_STRING:
            written = snprintf(out, out_len, "%s", value->str);
            break;
        default:
            return -1;
        }
    }

    if (written < 0 || (size_t)written >= out_len) {
        out[0] = '\0';
        return -1;
    }
    return written;
}

int device_config_get_text(const device_config_item_t *item, char *out, size_t out_len)
{
    device_config_value_t value;
    if (device_config_get(item, &value) != ESP_OK) {
        return -1;
    }
    return device_config_format(item, &value, out, out_len);
}

const char *device_config_type_name(device_config_type_t type)
{
    switch (type) {
    case DEVICE_CONFIG_TYPE_BOOL:
        return "bool";
    case DEVICE_CONFIG_TYPE_UINT:
        return "uint";
    case DEVICE_CONFIG_TYPE_INT:
        return "int";
    case DEVICE_CONFIG_TYPE_FLOAT:
        return "float";
    case DEVICE_CONFIG_TYPE_ENUM:
        return "enum";
    case DEVICE_CONFIG_TYPE_STRING:
        return "string";
    }
    return "?";
}
