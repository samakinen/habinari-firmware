// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/*
 * The out-of-band settings registry.
 *
 * What is worth pinning here is not the table — it is the two properties every
 * channel rendering the registry depends on and none of them can enforce for
 * itself: that a bad value is refused rather than clamped, and that a secret
 * never comes back out. Both live in device_config.c precisely so that adding a
 * fourth transport cannot get either of them wrong.
 */
#include "unity.h"

extern "C" {
#include "device_config.h"
}

#include <cstring>
#include <string>

namespace {

// --- A fake owner ----------------------------------------------------------
// Storage that stands in for NVS, so the tests exercise the registry rather
// than the flash driver.

uint32_t g_address = 1;
uint32_t g_baud_code = 1;
float g_offset = 0.0f;
bool g_enabled = true;
std::string g_name = "";
std::string g_secret = "";
int g_set_calls = 0;

esp_err_t getAddress(const device_config_item_t *, device_config_value_t *out)
{
    out->num.u = g_address;
    return ESP_OK;
}

esp_err_t setAddress(const device_config_item_t *, const device_config_value_t *value)
{
    ++g_set_calls;
    g_address = value->num.u;
    return ESP_OK;
}

esp_err_t getBaud(const device_config_item_t *, device_config_value_t *out)
{
    out->num.u = g_baud_code;
    return ESP_OK;
}

esp_err_t setBaud(const device_config_item_t *, const device_config_value_t *value)
{
    g_baud_code = value->num.u;
    return ESP_OK;
}

esp_err_t getOffset(const device_config_item_t *, device_config_value_t *out)
{
    out->num.f = g_offset;
    return ESP_OK;
}

esp_err_t setOffset(const device_config_item_t *, const device_config_value_t *value)
{
    g_offset = value->num.f;
    return ESP_OK;
}

esp_err_t getEnabled(const device_config_item_t *, device_config_value_t *out)
{
    out->num.b = g_enabled;
    return ESP_OK;
}

esp_err_t setEnabled(const device_config_item_t *, const device_config_value_t *value)
{
    g_enabled = value->num.b;
    return ESP_OK;
}

esp_err_t getName(const device_config_item_t *, device_config_value_t *out)
{
    std::snprintf(out->str, sizeof(out->str), "%s", g_name.c_str());
    return ESP_OK;
}

esp_err_t setName(const device_config_item_t *, const device_config_value_t *value)
{
    g_name = value->str;
    return ESP_OK;
}

/// A well-behaved secret hook: reports presence, never the value.
esp_err_t getSecret(const device_config_item_t *, device_config_value_t *out)
{
    out->str[0] = g_secret.empty() ? '\0' : 'x';
    out->str[1] = '\0';
    return ESP_OK;
}

/// A badly behaved one, which hands the registry the real passphrase. Used to
/// prove that redaction does not depend on the owner getting this right.
esp_err_t getSecretLeaky(const device_config_item_t *, device_config_value_t *out)
{
    std::snprintf(out->str, sizeof(out->str), "%s", g_secret.c_str());
    return ESP_OK;
}

esp_err_t setSecret(const device_config_item_t *, const device_config_value_t *value)
{
    g_secret = value->str;
    return ESP_OK;
}

esp_err_t getUptime(const device_config_item_t *, device_config_value_t *out)
{
    out->num.u = 42;
    return ESP_OK;
}

const char *const kBaudLabels[] = {"9600", "19200", "38400"};

const device_config_item_t kItems[] = {
    {
        .key = "mb.addr",
        .label = "Modbus slave address",
        .unit = nullptr,
        .type = DEVICE_CONFIG_TYPE_UINT,
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .min = 1.0f,
        .max = 247.0f,
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = getAddress,
        .set = setAddress,
        .ctx = nullptr,
    },
    {
        .key = "mb.baud",
        .label = "Baud rate",
        .unit = "bit/s",
        .type = DEVICE_CONFIG_TYPE_ENUM,
        .flags = 0,
        .min = 0.0f,
        .max = 0.0f,
        .enum_labels = kBaudLabels,
        .enum_count = 3,
        .get = getBaud,
        .set = setBaud,
        .ctx = nullptr,
    },
    {
        .key = "cal.offset",
        .label = "Temperature offset",
        .unit = "K",
        .type = DEVICE_CONFIG_TYPE_FLOAT,
        .flags = 0,
        .min = -5.0f,
        .max = 5.0f,
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = getOffset,
        .set = setOffset,
        .ctx = nullptr,
    },
    {
        .key = "dev.enabled",
        .label = "Enabled",
        .unit = nullptr,
        .type = DEVICE_CONFIG_TYPE_BOOL,
        .flags = 0,
        .min = 0.0f,
        .max = 0.0f,
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = getEnabled,
        .set = setEnabled,
        .ctx = nullptr,
    },
    {
        .key = "dev.name",
        .label = "Device name",
        .unit = nullptr,
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = 0,
        .min = 0.0f,
        .max = 8.0f,
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = getName,
        .set = setName,
        .ctx = nullptr,
    },
    {
        .key = "net.pass",
        .label = "Passphrase",
        .unit = nullptr,
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = DEVICE_CONFIG_FLAG_SECRET,
        .min = 0.0f,
        .max = 63.0f,
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = getSecret,
        .set = setSecret,
        .ctx = nullptr,
    },
    {
        .key = "dev.uptime",
        .label = "Uptime",
        .unit = "s",
        .type = DEVICE_CONFIG_TYPE_UINT,
        .flags = DEVICE_CONFIG_FLAG_READ_ONLY,
        .min = 0.0f,
        .max = 0.0f,
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = getUptime,
        .set = nullptr,
        .ctx = nullptr,
    },
};

void loadFixture()
{
    device_config_reset();
    g_address = 1;
    g_baud_code = 1;
    g_offset = 0.0f;
    g_enabled = true;
    g_name.clear();
    g_secret.clear();
    g_set_calls = 0;
    TEST_ASSERT_EQUAL(ESP_OK,
                      device_config_register(kItems, sizeof(kItems) / sizeof(kItems[0])));
}

const device_config_item_t *find(const char *key)
{
    const device_config_item_t *item = device_config_find(key);
    TEST_ASSERT_NOT_NULL(item);
    return item;
}

std::string textOf(const char *key)
{
    char buffer[DEVICE_CONFIG_STRING_MAX + 16] = {};
    const int written = device_config_get_text(find(key), buffer, sizeof(buffer));
    TEST_ASSERT_GREATER_OR_EQUAL(0, written);
    return std::string(buffer);
}

// --- Registration ----------------------------------------------------------

void test_registration_preserves_order_and_finds_by_key()
{
    loadFixture();

    TEST_ASSERT_EQUAL(sizeof(kItems) / sizeof(kItems[0]), device_config_count());
    TEST_ASSERT_EQUAL_STRING("mb.addr", device_config_at(0)->key);
    TEST_ASSERT_EQUAL_STRING("mb.baud", device_config_at(1)->key);
    TEST_ASSERT_NULL(device_config_at(device_config_count()));
    TEST_ASSERT_NULL(device_config_find("nope"));

    // The index is the handle a channel puts on the wire; it has to agree with
    // what a lookup by key returns or a client will read one item and write
    // another.
    TEST_ASSERT_EQUAL(2u, device_config_index_of(find("cal.offset")));
}

void test_duplicate_key_registers_nothing()
{
    loadFixture();

    const size_t before = device_config_count();
    static const device_config_item_t clash[] = {
        {.key = "zzz.new",
         .label = nullptr,
         .unit = nullptr,
         .type = DEVICE_CONFIG_TYPE_UINT,
         .flags = 0,
         .min = 0.0f,
         .max = 0.0f,
         .enum_labels = nullptr,
         .enum_count = 0,
         .get = getUptime,
         .set = nullptr,
         .ctx = nullptr},
        {.key = "mb.addr", /* already registered */
         .label = nullptr,
         .unit = nullptr,
         .type = DEVICE_CONFIG_TYPE_UINT,
         .flags = 0,
         .min = 0.0f,
         .max = 0.0f,
         .enum_labels = nullptr,
         .enum_count = 0,
         .get = getUptime,
         .set = nullptr,
         .ctx = nullptr},
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_register(clash, 2));
    // All or nothing: a half-registered owner looks like a working device with
    // a missing knob rather than like the programming error it is.
    TEST_ASSERT_EQUAL(before, device_config_count());
    TEST_ASSERT_NULL(device_config_find("zzz.new"));
}

// --- Range enforcement -----------------------------------------------------

void test_out_of_range_is_refused_not_clamped()
{
    loadFixture();

    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("mb.addr"), "247"));
    TEST_ASSERT_EQUAL(247u, g_address);

    // 248 must not silently become 247. A device that quietly rounds its own
    // address into range is a device nobody can find on the bus.
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("mb.addr"), "248"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("mb.addr"), "0"));
    TEST_ASSERT_EQUAL(247u, g_address);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("cal.offset"), "5.5"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("cal.offset"), "-5.5"));
    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("cal.offset"), "-1.25"));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.25f, g_offset);
}

void test_malformed_numbers_are_refused()
{
    loadFixture();

    // strtoul would accept every one of these. A typo has to come back as an
    // error the installer can see, not as a value that is almost right.
    const char *const bad[] = {"12abc", "", " 12", "-1", "0x10", "1.5", "1e3"};
    for (const char *text : bad) {
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("mb.addr"), text));
    }
    TEST_ASSERT_EQUAL(0, g_set_calls);
    TEST_ASSERT_EQUAL(1u, g_address);
}

void test_string_length_is_bounded_by_the_item()
{
    loadFixture();

    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("dev.name"), "Room 214"));
    TEST_ASSERT_EQUAL_STRING("Room 214", g_name.c_str());

    // Nine characters against a limit of eight. Truncating would store a name
    // that is not the one anybody typed.
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("dev.name"), "Room 2145"));
    TEST_ASSERT_EQUAL_STRING("Room 214", g_name.c_str());
}

void test_read_only_items_refuse_writes()
{
    loadFixture();

    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, device_config_set_text(find("dev.uptime"), "7"));
    TEST_ASSERT_EQUAL_STRING("42", textOf("dev.uptime").c_str());
}

// --- Text encoding ---------------------------------------------------------

void test_bool_accepts_the_words_people_type()
{
    loadFixture();

    for (const char *text : {"0", "off", "false", "no", "OFF"}) {
        g_enabled = true;
        TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("dev.enabled"), text));
        TEST_ASSERT_FALSE(g_enabled);
    }
    for (const char *text : {"1", "on", "true", "yes", "True"}) {
        g_enabled = false;
        TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("dev.enabled"), text));
        TEST_ASSERT_TRUE(g_enabled);
    }
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("dev.enabled"), "maybe"));
    TEST_ASSERT_EQUAL_STRING("on", textOf("dev.enabled").c_str());
}

void test_enum_round_trips_through_its_label()
{
    loadFixture();

    // What is read back has to be writable again unchanged, or a client that
    // dumps the configuration cannot restore it.
    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("mb.baud"), "38400"));
    TEST_ASSERT_EQUAL(2u, g_baud_code);
    TEST_ASSERT_EQUAL_STRING("38400", textOf("mb.baud").c_str());

    // The code point works too, for a script that does not want to know the
    // labels.
    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("mb.baud"), "0"));
    TEST_ASSERT_EQUAL(0u, g_baud_code);

    // Labels are the bound when there are labels.
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("mb.baud"), "3"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("mb.baud"), "76800"));
    TEST_ASSERT_EQUAL(0u, g_baud_code);
}

void test_numeric_round_trip()
{
    loadFixture();

    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("mb.addr"), "34"));
    TEST_ASSERT_EQUAL_STRING("34", textOf("mb.addr").c_str());

    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("cal.offset"), "0.5"));
    TEST_ASSERT_EQUAL_STRING("0.5", textOf("cal.offset").c_str());
}

void test_format_reports_a_buffer_that_is_too_small()
{
    loadFixture();
    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("dev.name"), "Room 214"));

    char tiny[4] = {};
    // Silently returning "Roo" would put a wrong value on the wire.
    TEST_ASSERT_EQUAL(-1, device_config_get_text(find("dev.name"), tiny, sizeof(tiny)));
    TEST_ASSERT_EQUAL_STRING("", tiny);
}

// --- Secrets ---------------------------------------------------------------

void test_secret_reads_report_presence_only()
{
    loadFixture();

    TEST_ASSERT_EQUAL_STRING("<unset>", textOf("net.pass").c_str());
    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("net.pass"), "correct horse battery"));
    TEST_ASSERT_EQUAL_STRING("correct horse battery", g_secret.c_str());
    TEST_ASSERT_EQUAL_STRING("<set>", textOf("net.pass").c_str());
}

void test_secret_redaction_does_not_trust_the_owner()
{
    device_config_reset();
    g_secret = "correct horse battery";

    // Same item, but with a hook that hands back the real passphrase. Redaction
    // is the registry's job precisely so that one careless owner — or one
    // transport that forgets to check the flag — cannot leak it.
    static const device_config_item_t leaky[] = {
        {.key = "net.pass",
         .label = "Passphrase",
         .unit = nullptr,
         .type = DEVICE_CONFIG_TYPE_STRING,
         .flags = DEVICE_CONFIG_FLAG_SECRET,
         .min = 0.0f,
         .max = 63.0f,
         .enum_labels = nullptr,
         .enum_count = 0,
         .get = getSecretLeaky,
         .set = setSecret,
         .ctx = nullptr},
    };
    TEST_ASSERT_EQUAL(ESP_OK, device_config_register(leaky, 1));

    TEST_ASSERT_EQUAL_STRING("<set>", textOf("net.pass").c_str());
}

// --- Reboot marking --------------------------------------------------------

void test_reboot_pending_is_raised_only_by_a_successful_write()
{
    loadFixture();
    TEST_ASSERT_FALSE(device_config_reboot_pending());

    // Not flagged: writing it changes behaviour now.
    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("mb.baud"), "9600"));
    TEST_ASSERT_FALSE(device_config_reboot_pending());

    // Flagged, but refused — nothing was stored, so nothing needs a restart.
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, device_config_set_text(find("mb.addr"), "300"));
    TEST_ASSERT_FALSE(device_config_reboot_pending());

    TEST_ASSERT_EQUAL(ESP_OK, device_config_set_text(find("mb.addr"), "17"));
    TEST_ASSERT_TRUE(device_config_reboot_pending());
}

} // namespace

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_registration_preserves_order_and_finds_by_key);
    RUN_TEST(test_duplicate_key_registers_nothing);

    RUN_TEST(test_out_of_range_is_refused_not_clamped);
    RUN_TEST(test_malformed_numbers_are_refused);
    RUN_TEST(test_string_length_is_bounded_by_the_item);
    RUN_TEST(test_read_only_items_refuse_writes);

    RUN_TEST(test_bool_accepts_the_words_people_type);
    RUN_TEST(test_enum_round_trips_through_its_label);
    RUN_TEST(test_numeric_round_trip);
    RUN_TEST(test_format_reports_a_buffer_that_is_too_small);

    RUN_TEST(test_secret_reads_report_presence_only);
    RUN_TEST(test_secret_redaction_does_not_trust_the_owner);

    RUN_TEST(test_reboot_pending_is_raised_only_by_a_successful_write);

    return UNITY_END();
}
