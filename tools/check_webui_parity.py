#!/usr/bin/env python3
"""
Confirm that the browser commissioning client still speaks the firmware's wire
contract.

`docs/webui/index.html` decodes the GATT characteristics in
`main/include/oob_service.h` by hand: fixed offsets into a packed struct, opcode
constants, flag bits, and the type and flag enumerations from
`main/include/device_config.h`. Nothing in the build links the two, so a field
inserted into `oob_status_t` or a renumbered `device_config_type_t` would leave
the page reading the wrong bytes with no error anywhere — the readings would
simply be wrong, which is the worst way for a commissioning tool to fail.

This is the link. It parses both sides and compares them:

    tools/check_webui_parity.py

It needs no compiler and no browser, so it runs anywhere the host tests do, and
it is registered with CTest as `test_webui_parity`.

`tools/ble_config.py` is checked for the service UUID only. It uses `struct`
format strings rather than hand-written offsets, which are readable enough to
review directly, and it is the reference client — if the two clients ever
disagree, this file is not the place that finds out.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OOB_HEADER = ROOT / "main/include/oob_service.h"
OOB_SOURCE = ROOT / "main/src/oob_ble.c"
CONFIG_HEADER = ROOT / "main/include/device_config.h"
PAGE = ROOT / "docs/webui/index.html"
CLI = ROOT / "tools/ble_config.py"

problems = []


def check(what, got, want, hint=""):
    if got != want:
        problems.append(
            f"{what}\n"
            f"      firmware: {want!r}\n"
            f"      page:     {got!r}"
            + (f"\n      {hint}" if hint else ""))


def read(path):
    return path.read_text(encoding="utf-8")


# --- The C side ------------------------------------------------------------

C_TYPE_SIZES = {
    "uint8_t": 1, "int8_t": 1, "char": 1,
    "uint16_t": 2, "int16_t": 2,
    "uint32_t": 4, "int32_t": 4, "float": 4,
}

FIELD = re.compile(r"^\s*(\w+)\s+(\w+)\s*(?:\[(\d+)\])?\s*;")


def packed_struct(source, typedef_name):
    """
    Field offsets of a `typedef struct __attribute__((packed)) {...} name;`.

    Packed means no padding, so the offsets are just a running sum — which is
    the only reason a text parse can be trusted here. A struct that lost its
    packed attribute would be a firmware bug in its own right, so it is checked
    rather than accommodated.
    """
    # `[^{}]*` rather than a lazy `.*?`: the latter would happily start at an
    # earlier struct and swallow everything up to this one's closing brace.
    match = re.search(
        r"typedef struct\s+__attribute__\(\(packed\)\)\s*\{([^{}]*)\}\s*" + typedef_name + r"\s*;",
        source, re.S)
    if not match:
        problems.append(
            f"{typedef_name}: no packed struct of that name in {OOB_HEADER.name}. "
            f"If it stopped being packed, the page's fixed offsets are wrong.")
        return {}, 0

    offsets, offset = {}, 0
    for line in match.group(1).splitlines():
        line = line.split("///")[0].split("//")[0]
        field = FIELD.match(line)
        if not field:
            continue
        ctype, name, count = field.group(1), field.group(2), field.group(3)
        if ctype not in C_TYPE_SIZES:
            problems.append(f"{typedef_name}.{name}: unhandled C type {ctype!r}; teach this script about it.")
            continue
        offsets[name] = (offset, ctype)
        offset += C_TYPE_SIZES[ctype] * (int(count) if count else 1)
    return offsets, offset


def c_define(source, name, cast=str):
    match = re.search(rf"^#define\s+{name}\s+(.+)$", source, re.M)
    if not match:
        problems.append(f"{name}: not found in the firmware headers.")
        return None
    return cast(match.group(1).strip())


def strip_comments(source):
    """Doc comments name the very constants being parsed; drop them first."""
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.S)
    return re.sub(r"//[^\n]*", "", source)


def c_enum_values(source, prefix):
    """
    `NAME = <expr>,` for every enumerator starting with `prefix`, plus implicit
    successors — enough for the `1u << n` flag enums and the plain counters.
    """
    source = strip_comments(source)
    values, previous = {}, -1
    for name, expr in re.findall(rf"({prefix}\w*)\s*(?:=\s*([^,\n}}]+))?", source):
        if expr.strip():
            text = expr.strip().replace("u", "").replace("U", "")
            try:
                value = eval(text, {"__builtins__": {}}, {})  # noqa: S307 - constant arithmetic only
            except Exception:
                continue
        else:
            value = previous + 1
        values[name] = previous = value
    return values


def uuid_base():
    """Rebuild the UUID from the little-endian byte list in the OOB_UUID128 macro."""
    macro = re.search(r"#define OOB_UUID128\(nn\).*?BLE_UUID128_INIT\((.*?)\)", read(OOB_SOURCE), re.S)
    if not macro:
        problems.append("OOB_UUID128: macro not found in oob_ble.c.")
        return None
    body = macro.group(1).replace("\\", " ")
    # 16 bytes in all, the first of which — the low half of the per-characteristic
    # suffix — is the macro's `0x##nn` parameter and so is not a literal.
    literals = [int(b, 16) for b in re.findall(r"0x([0-9a-fA-F]{2})\b", body)]
    if len(literals) != 15:
        problems.append(f"OOB_UUID128: expected 15 literal bytes beside the suffix, found {len(literals)}.")
        return None
    # NimBLE takes the bytes least-significant first, so reversing them gives
    # the UUID as written; the parameterised byte lands last, and is 00 for the
    # service itself.
    hexed = "".join(f"{b:02x}" for b in reversed(literals)) + "00"
    return f"{hexed[0:8]}-{hexed[8:12]}-{hexed[12:16]}-{hexed[16:20]}-{hexed[20:32]}"


# --- The page --------------------------------------------------------------

def js_body(source, function_name):
    start = source.index(f"function {function_name}(")
    opening = source.index("{", start)
    depth = 0
    for i in range(opening, len(source)):
        depth += (source[i] == "{") - (source[i] == "}")
        if depth == 0:
            return source[opening:i]
    raise AssertionError(f"unterminated function {function_name}")


ACCESSOR_FOR = {
    "uint8_t": "getUint8", "char": "getUint8",
    "uint16_t": "getUint16", "int16_t": "getInt16",
    "uint32_t": "getUint32", "int32_t": "getInt32",
    "float": "getFloat32",
}


def js_getters(body):
    """`field: view.getT(N, ...)` and `field: scaled(view.getT(N, ...))`."""
    return {
        field: (accessor, int(offset))
        for field, accessor, offset in
        re.findall(r"(\w+):\s*(?:scaled\()?view\.(get\w+)\((\d+)", body)
    }


def js_const(source, pattern, cast=str):
    match = re.search(pattern, source)
    if not match:
        problems.append(f"the page has no constant matching {pattern!r}.")
        return None
    return cast(match.group(1))


def js_object(source, name):
    """`const NAME = { A: 1, B: 2 };` -> {'A': 1, 'B': 2}."""
    match = re.search(rf"const {name} = \{{([^}}]*)\}}", source)
    if not match:
        problems.append(f"the page has no `const {name} = {{...}}`.")
        return {}
    # The shift alternative has to come first, or `1 << 3` is read as the
    # literal 1.
    return {
        key: (1 << int(shift)) if shift else int(value, 0)
        for key, shift, value in
        re.findall(r"(\w+):\s*(?:1 << (\d+)|(0x[0-9a-fA-F]+|\d+))", match.group(1))
    }


# --- The comparison --------------------------------------------------------

def main():
    header, config, page = read(OOB_HEADER), read(CONFIG_HEADER), read(PAGE)

    # 1. Wire version. A bump means the page needs revisiting, whatever else
    #    this script says, because the meaning of a field may have changed
    #    without its offset moving.
    check("OOB_WIRE_VERSION", js_const(page, r"WIRE_VERSION = (\d+)", int),
          c_define(header, "OOB_WIRE_VERSION", int),
          "Bump WIRE_VERSION in the page only after checking every field it decodes.")

    # 2. Struct offsets and accessor widths.
    info, info_size = packed_struct(header, "oob_device_info_t")
    status, status_size = packed_struct(header, "oob_status_t")

    info_body = js_body(page, "parseInfo")
    info_js = js_getters(info_body)
    for c_name, js_name in [("item_count", "itemCount"), ("flags", "flags")]:
        if c_name in info and js_name in info_js:
            accessor, offset = info_js[js_name]
            check(f"oob_device_info_t.{c_name} offset", offset, info[c_name][0])
            check(f"oob_device_info_t.{c_name} accessor", accessor, ACCESSOR_FOR[info[c_name][1]])

    version_read = re.search(r"getUint8\((\d+)\);\s*\n\s*checkVersion", info_body)
    check("oob_device_info_t.version offset", int(version_read.group(1)) if version_read else None,
          info.get("version", (None,))[0])

    serial = re.search(r"subarray\((\d+),\s*(\d+)\)", info_body)
    check("oob_device_info_t.serial offset", int(serial.group(1)) if serial else None,
          info.get("serial", (None,))[0])
    check("oob_device_info_t.serial length", int(serial.group(2)) - int(serial.group(1)) if serial else None, 6)

    text = re.search(r"subarray\((\d+)\), 2\)", info_body)
    check("oob_device_info_t.text offset", int(text.group(1)) if text else None,
          info.get("text", (None,))[0])
    check("oob_device_info_t size", info_size, 74,
          "The page reads the trailing text field to the end of the value; "
          "a resized struct is worth an eye even when the offsets still line up.")

    status_js = js_getters(js_body(page, "parseStatus"))
    version_read = re.search(r"checkVersion\(view\.getUint8\((\d+)\)", js_body(page, "parseStatus"))
    check("oob_status_t.version offset", int(version_read.group(1)) if version_read else None,
          status.get("version", (None,))[0])
    for c_name, js_name in [
            ("flags", "flags"),
            ("temperature_c_x100", "temperature"),
            ("humidity_pct_x100", "humidity"),
            ("co2_ppm", "co2"),
            ("active_setpoint_c_x100", "setpoint"),
            ("heating_percent", "heating"),
            ("cooling_percent", "cooling"),
            ("ventilation_percent", "ventilation"),
            ("hvac_mode", "hvacMode"),
            ("controller_mode", "controllerMode"),
            ("sensor_health_mask", "healthMask"),
            ("uptime_s", "uptime")]:
        if c_name not in status:
            problems.append(f"oob_status_t.{c_name}: gone from the firmware, but the page still reads it.")
            continue
        if js_name not in status_js:
            problems.append(f"oob_status_t.{c_name}: the page no longer decodes it (expected `{js_name}:`).")
            continue
        accessor, offset = status_js[js_name]
        check(f"oob_status_t.{c_name} offset", offset, status[c_name][0])
        check(f"oob_status_t.{c_name} accessor", accessor, ACCESSOR_FOR[status[c_name][1]])

    unread = set(status) - {c for c, _ in [
        ("version", 0), ("flags", 0), ("temperature_c_x100", 0), ("humidity_pct_x100", 0),
        ("co2_ppm", 0), ("active_setpoint_c_x100", 0), ("heating_percent", 0),
        ("cooling_percent", 0), ("ventilation_percent", 0), ("hvac_mode", 0),
        ("controller_mode", 0), ("sensor_health_mask", 0), ("uptime_s", 0)]}
    if unread:
        problems.append(
            f"oob_status_t has field(s) the page does not show: {', '.join(sorted(unread))}. "
            f"Add them to docs/webui/index.html (parseStatus and renderStatus) or to this list.")
    check("oob_status_t size", status_size, 20)

    # 3. The item descriptor. Built by hand in build_descriptor(), so the only
    #    firmware-side constant to hold on to is the fixed header length.
    descriptor_body = re.search(r"static int build_descriptor\(.*?\n\}", read(OOB_SOURCE), re.S)
    if descriptor_body:
        guard = re.search(r"if \(cap < (\d+)u?\)", descriptor_body.group(0))
        strings_at = js_const(page, r"subarray\((\d+)\), 3 \+ enumCount\)", int)
        check("descriptor header length", strings_at, int(guard.group(1)) if guard else None,
              "build_descriptor() writes a fixed header, then key/label/unit and the enum labels.")
        order = re.findall(r"item->(min|max)|buf\[offset\+\+\] = (?:\(uint8_t\)\()?(?:index|item->type|item->flags|enum_count)",
                           descriptor_body.group(0))
        check("descriptor field order", [o for o in order if o], ["min", "max"],
              "min must still be written before max; the page reads them at 4 and 8.")

    descriptor_js = js_getters(js_body(page, "parseDescriptor"))
    for field, offset, accessor in [("index", 0, "getUint16"), ("type", 2, "getUint8"),
                                    ("flags", 3, "getUint8"), ("min", 4, "getFloat32"),
                                    ("max", 8, "getFloat32")]:
        check(f"descriptor.{field}", descriptor_js.get(field), (accessor, offset))

    # 4. Opcodes, flags and the magic word.
    cmds = c_enum_values(header, "OOB_CMD_")
    page_cmds = js_object(page, "CMD")
    for c_name, js_name in [("OOB_CMD_COMMIT", "COMMIT"), ("OOB_CMD_REBOOT", "REBOOT"),
                            ("OOB_CMD_IDENTIFY", "IDENTIFY"),
                            ("OOB_CMD_FACTORY_RESET", "FACTORY_RESET"),
                            ("OOB_CMD_END_COMMISSIONING", "END")]:
        check(c_name, page_cmds.get(js_name), cmds.get(c_name))
    missing = set(cmds) - {"OOB_CMD_COMMIT", "OOB_CMD_REBOOT", "OOB_CMD_IDENTIFY",
                           "OOB_CMD_FACTORY_RESET", "OOB_CMD_END_COMMISSIONING"}
    if missing:
        problems.append(f"new control opcode(s) the page cannot send: {', '.join(sorted(missing))}.")

    check("OOB_FACTORY_RESET_MAGIC",
          js_const(page, r'FACTORY_RESET_MAGIC = "([^"]+)"'),
          c_define(header, "OOB_FACTORY_RESET_MAGIC", lambda s: s.strip('"')))

    check("OOB_STATUS_INVALID_SIGNED", js_const(page, r"INVALID_SIGNED = (-?\d+)", int), -32768,
          "0x8000 as a signed 16-bit value.")

    info_flags = c_enum_values(header, "OOB_INFO_FLAG_")
    page_info_flags = js_object(page, "INFO_FLAG")
    for c_name, js_name in [("OOB_INFO_FLAG_PROVISIONED", "PROVISIONED"),
                            ("OOB_INFO_FLAG_REBOOT_PENDING", "REBOOT_PENDING"),
                            ("OOB_INFO_FLAG_SECRET_FROM_EFUSE", "SECRET_FROM_EFUSE")]:
        check(c_name, page_info_flags.get(js_name), info_flags.get(c_name))

    status_flags = c_enum_values(header, "OOB_STATUS_FLAG_")
    page_status_flags = js_object(page, "STATUS_FLAG")
    for c_name, js_name in [("OOB_STATUS_FLAG_HAS_SENSOR_DATA", "HAS_SENSOR_DATA"),
                            ("OOB_STATUS_FLAG_IDENTIFY", "IDENTIFY"),
                            ("OOB_STATUS_FLAG_DEVICE_FAULT", "DEVICE_FAULT"),
                            ("OOB_STATUS_FLAG_REBOOT_PENDING", "REBOOT_PENDING")]:
        check(c_name, page_status_flags.get(js_name), status_flags.get(c_name))

    # 5. The registry: type numbering, item flags, string bound. The page turns
    #    the type code into a form control and the flags into behaviour, so a
    #    renumbering here would silently render the wrong widget.
    types = c_enum_values(config, "DEVICE_CONFIG_TYPE_")
    page_types = js_object(page, "TYPE")
    for c_name, js_name in [("DEVICE_CONFIG_TYPE_BOOL", "BOOL"), ("DEVICE_CONFIG_TYPE_UINT", "UINT"),
                            ("DEVICE_CONFIG_TYPE_INT", "INT"), ("DEVICE_CONFIG_TYPE_FLOAT", "FLOAT"),
                            ("DEVICE_CONFIG_TYPE_ENUM", "ENUM"), ("DEVICE_CONFIG_TYPE_STRING", "STRING")]:
        check(c_name, page_types.get(js_name), types.get(c_name))
    check("device_config_type_t count", len(page_types),
          len([t for t in types if t != "DEVICE_CONFIG_TYPE_COUNT"]),
          "A new setting type needs a form control in buildInput() and a rule in validate().")

    item_flags = c_enum_values(config, "DEVICE_CONFIG_FLAG_")
    page_item_flags = js_object(page, "ITEM_FLAG")
    for c_name, js_name in [("DEVICE_CONFIG_FLAG_SECRET", "SECRET"),
                            ("DEVICE_CONFIG_FLAG_READ_ONLY", "READ_ONLY"),
                            ("DEVICE_CONFIG_FLAG_REBOOT", "REBOOT")]:
        check(c_name, page_item_flags.get(js_name), item_flags.get(c_name))

    # DEVICE_CONFIG_STRING_MAX counts the NUL; the page bounds the text itself.
    check("longest string value", js_const(page, r"STRING_MAX = (\d+)", int),
          c_define(config, "DEVICE_CONFIG_STRING_MAX", int) - 1)

    # 6. UUIDs, rebuilt from the firmware's own macro.
    base = uuid_base()
    if base:
        check("service UUID", js_const(page, r'`([0-9a-f-]+)\$\{suffix\}`') + "0000", base)
        check("service UUID (tools/ble_config.py)",
              re.search(r'BASE = "([0-9a-f-]+)\{:04x\}"', read(CLI)).group(1) + "0000", base)

    # --- Verdict ---
    if problems:
        print(f"{PAGE.relative_to(ROOT)} no longer matches the firmware:\n", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        print(f"\n{len(problems)} mismatch(es). The wire contract is documented in "
              f"docs/ble-commissioning.md §4.", file=sys.stderr)
        return 1

    print("docs/webui/index.html matches the firmware wire contract.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
