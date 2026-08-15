#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Sami Mäkinen
"""
Commission a Habinari device over its BLE service channel.

This is the reference client for the GATT contract in
docs/ble-commissioning.md — small enough to read in one sitting, which is the
point: the contract is what matters, and anyone can write their own app against
it. It is also the thing to reach for when something on a bench is not
behaving, because it prints exactly what the device reports.

    pip install bleak

    tools/ble_config.py scan
    tools/ble_config.py --address AA:BB:CC:DD:EE:FF show
    tools/ble_config.py --address AA:BB:CC:DD:EE:FF get mb.addr
    tools/ble_config.py --address AA:BB:CC:DD:EE:FF set mb.addr 17
    tools/ble_config.py --address AA:BB:CC:DD:EE:FF set net.ssid "Site Wi-Fi"
    tools/ble_config.py --address AA:BB:CC:DD:EE:FF commit
    tools/ble_config.py --address AA:BB:CC:DD:EE:FF watch
    tools/ble_config.py --address AA:BB:CC:DD:EE:FF identify on

The device advertises only while it is in programming mode: hold the programming
button for a second (the LED lights), or power up a board that has never been
commissioned. Outside programming mode `scan` finds nothing, and that is correct
behaviour rather than a fault. Programming mode lapses on its own after
CONFIG_HABINARI_PROGRAMMING_MODE_TIMEOUT_S — 15 minutes by default.

That is the same state, the same button and the same LED that KNX individual
addressing and Modbus re-addressing use. A board with its LED lit is the board
that can be commissioned, whatever protocol you reach it with.

## Pairing

Every characteristic needs an encrypted, authenticated link, so the first
connection has to pair with the six-digit passkey printed on the device (or
logged by the firmware on the bench). Bleak does not implement a pairing agent
on Linux; BlueZ does, so pair once with bluetoothctl and this tool reuses the
bond afterwards:

    bluetoothctl
    > agent KeyboardDisplay
    > default-agent
    > pair AA:BB:CC:DD:EE:FF
    [enter the six digits]
    > trust AA:BB:CC:DD:EE:FF
    > quit

On Windows and macOS the OS raises its own passkey prompt on the first
protected read, and `bleak` drives it for you.
"""

import argparse
import asyncio
import contextlib
import struct
import sys

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover - a missing dependency, not a bug
    sys.exit("bleak is not installed. Run: pip install bleak")

BASE = "1d298b96-21f7-4109-87a0-17de9955{:04x}"
UUID_SERVICE = BASE.format(0x0000)
UUID_INFO = BASE.format(0x0001)
UUID_SELECT = BASE.format(0x0002)
UUID_DESCRIPTOR = BASE.format(0x0003)
UUID_VALUE = BASE.format(0x0004)
UUID_CONTROL = BASE.format(0x0005)
UUID_STATUS = BASE.format(0x0006)

WIRE_VERSION = 1

CMD_COMMIT = 0x01
CMD_REBOOT = 0x02
CMD_IDENTIFY = 0x03
CMD_FACTORY_RESET = 0x04
CMD_END_COMMISSIONING = 0x05

TYPE_NAMES = ["bool", "uint", "int", "float", "enum", "string"]

FLAG_SECRET = 1 << 0
FLAG_READ_ONLY = 1 << 1
FLAG_REBOOT = 1 << 2

INFO_PROVISIONED = 1 << 0
INFO_REBOOT_PENDING = 1 << 1
INFO_SECRET_FROM_EFUSE = 1 << 2

STATUS_HAS_SENSOR_DATA = 1 << 0
STATUS_IDENTIFY = 1 << 1
STATUS_DEVICE_FAULT = 1 << 2
STATUS_REBOOT_PENDING = 1 << 3

INVALID_SIGNED = -32768


def check_version(version, what):
    if version != WIRE_VERSION:
        sys.exit(
            f"{what} reports wire version {version}, this tool speaks {WIRE_VERSION}. "
            "Update whichever is older."
        )


def split_strings(blob, count=None):
    """NUL-separated strings, as the descriptor and info blobs carry them."""
    parts = blob.split(b"\x00")
    decoded = [p.decode("utf-8", "replace") for p in parts]
    if count is not None:
        decoded += [""] * (count - len(decoded))
        return decoded[:count]
    return decoded


class DeviceInfo:
    def __init__(self, raw):
        version, item_count, flags, _reserved = struct.unpack_from("<BBBB", raw, 0)
        check_version(version, "Device")
        self.item_count = item_count
        self.flags = flags
        self.serial = raw[4:10]
        text = raw[10:]
        self.firmware, self.personalities = split_strings(text, 2)

    @property
    def serial_text(self):
        return ":".join(f"{b:02X}" for b in self.serial)

    def describe(self):
        state = "commissioned" if self.flags & INFO_PROVISIONED else "NOT commissioned"
        if self.flags & INFO_REBOOT_PENDING:
            state += ", restart pending"
        if not self.flags & INFO_SECRET_FROM_EFUSE:
            state += ", NO DEVICE ROOT SECRET (development passkey)"
        personalities = self.personalities or "(none)"
        return (
            f"serial {self.serial_text}  firmware {self.firmware}\n"
            f"personalities: {personalities}\n"
            f"state: {state}\n"
            f"settings: {self.item_count}"
        )


class Item:
    def __init__(self, raw):
        index, type_code, flags, minimum, maximum, enum_count = struct.unpack_from(
            "<HBBffB", raw, 0
        )
        self.index = index
        self.type = TYPE_NAMES[type_code] if type_code < len(TYPE_NAMES) else "?"
        self.flags = flags
        self.min = minimum
        self.max = maximum
        strings = split_strings(raw[13:], 3 + enum_count)
        self.key, self.label, self.unit = strings[0], strings[1], strings[2]
        self.enum_labels = strings[3 : 3 + enum_count]

    def range_text(self):
        if self.enum_labels:
            return "|".join(self.enum_labels)
        if self.type == "string":
            return f"max {int(self.max)} chars"
        if self.min == 0.0 and self.max == 0.0:
            return ""
        return f"{self.min:g}..{self.max:g}"

    def notes(self):
        marks = []
        if self.flags & FLAG_READ_ONLY:
            marks.append("read-only")
        if self.flags & FLAG_SECRET:
            marks.append("write-only")
        if self.flags & FLAG_REBOOT:
            marks.append("needs restart")
        return ", ".join(marks)


class Status:
    def __init__(self, raw):
        (
            version,
            self.flags,
            temperature,
            humidity,
            co2,
            setpoint,
            self.heating,
            self.cooling,
            self.ventilation,
            self.hvac_mode,
            self.controller_mode,
            self.health_mask,
            self.uptime_s,
        ) = struct.unpack_from("<BBhhhhBBBBBBI", raw, 0)
        check_version(version, "Status")
        self.temperature = self._scaled(temperature, 100.0)
        self.humidity = self._scaled(humidity, 100.0)
        self.co2 = self._scaled(co2, 1.0)
        self.setpoint = self._scaled(setpoint, 100.0)

    @staticmethod
    def _scaled(value, scale):
        return None if value == INVALID_SIGNED else value / scale

    def describe(self):
        def number(value, unit, digits=2):
            return "--" if value is None else f"{value:.{digits}f} {unit}"

        marks = []
        if self.flags & STATUS_IDENTIFY:
            marks.append("identify")
        if self.flags & STATUS_DEVICE_FAULT:
            marks.append("FAULT")
        if self.flags & STATUS_REBOOT_PENDING:
            marks.append("restart pending")
        if not self.flags & STATUS_HAS_SENSOR_DATA:
            marks.append("no readings yet")

        return (
            f"T {number(self.temperature, 'C')}  RH {number(self.humidity, '%')}  "
            f"CO2 {number(self.co2, 'ppm', 0)}  setpoint {number(self.setpoint, 'C')}  "
            f"heat {self.heating}% cool {self.cooling}% vent {self.ventilation}%  "
            f"sensors 0x{self.health_mask:02x}  up {self.uptime_s} s"
            + (f"  [{', '.join(marks)}]" if marks else "")
        )


class Board:
    """One connected device. Selection is stateful, so reads are serialised."""

    def __init__(self, client):
        self.client = client
        self._lock = asyncio.Lock()

    async def info(self):
        return DeviceInfo(await self.client.read_gatt_char(UUID_INFO))

    async def _select(self, index):
        await self.client.write_gatt_char(
            UUID_SELECT, struct.pack("<H", index), response=True
        )

    async def item(self, index):
        async with self._lock:
            await self._select(index)
            return Item(await self.client.read_gatt_char(UUID_DESCRIPTOR))

    async def read_value(self, index):
        async with self._lock:
            await self._select(index)
            raw = await self.client.read_gatt_char(UUID_VALUE)
            return raw.decode("utf-8", "replace")

    async def write_value(self, index, text):
        async with self._lock:
            await self._select(index)
            await self.client.write_gatt_char(
                UUID_VALUE, text.encode("utf-8"), response=True
            )

    async def items(self):
        info = await self.info()
        return [await self.item(i) for i in range(info.item_count)]

    async def find(self, key):
        for item in await self.items():
            if item.key == key:
                return item
        sys.exit(f"No setting named '{key}'. Run `show` to list them.")

    async def status(self):
        return Status(await self.client.read_gatt_char(UUID_STATUS))

    async def control(self, opcode, payload=b""):
        await self.client.write_gatt_char(
            UUID_CONTROL, bytes([opcode]) + payload, response=True
        )


@contextlib.asynccontextmanager
async def connect(address):
    client = BleakClient(address)
    try:
        await client.connect()
    except Exception as exc:  # noqa: BLE001 - the message is the useful part
        sys.exit(
            f"Could not connect to {address}: {exc}\n"
            "Is the board in programming mode? Hold the programming button for a\n"
            "second; the LED lights while it is."
        )
    try:
        yield client
    finally:
        await client.disconnect()


# --- Commands --------------------------------------------------------------


async def cmd_scan(args):
    print(f"Scanning {args.timeout:.0f} s for boards advertising the service...")

    # No service_uuids filter: the service UUID rides in the scan response
    # rather than the advertisement — the name and a 128-bit UUID do not both
    # fit in 31 bytes — and not every backend applies that filter to scan
    # responses. Matching on the returned advertisement data is portable.
    found = await BleakScanner.discover(timeout=args.timeout, return_adv=True)
    matches = [
        (device, adv)
        for device, adv in found.values()
        if UUID_SERVICE in [u.lower() for u in adv.service_uuids]
    ]
    if not matches:
        print(
            "Nothing found. The device advertises only while it is in programming\n"
            "mode — hold the programming button for a second (the LED lights) and\n"
            "try again."
        )
        return
    for device, adv in matches:
        rssi = f"{adv.rssi} dBm" if adv.rssi is not None else ""
        print(f"  {device.address}  {adv.local_name or device.name or '(unnamed)'}  {rssi}")


async def cmd_show(args):
    async with connect(args.address) as client:
        board = Board(client)
        info = await board.info()
        print(info.describe())
        print()
        for item in await board.items():
            value = await board.read_value(item.index)
            notes = item.notes()
            unit = f" {item.unit}" if item.unit else ""
            print(f"  [{item.index}] {item.key:<12} = {value}{unit}")
            details = [item.type, item.range_text(), notes]
            print(f"       {item.label}  ({', '.join(d for d in details if d)})")
        print()
        print(f"  status: {(await board.status()).describe()}")


async def cmd_get(args):
    async with connect(args.address) as client:
        board = Board(client)
        item = await board.find(args.key)
        print(await board.read_value(item.index))


async def cmd_set(args):
    async with connect(args.address) as client:
        board = Board(client)
        item = await board.find(args.key)
        try:
            await board.write_value(item.index, args.value)
        except Exception as exc:  # noqa: BLE001
            # The device refuses out-of-range values rather than clamping them,
            # so this is the normal way a typo comes back.
            sys.exit(f"Refused: {exc}\n  {args.key} accepts {item.range_text()}")
        readback = await board.read_value(item.index)
        print(f"{item.key} = {readback}")
        if item.flags & FLAG_REBOOT:
            print("  takes effect after a restart — run `commit` when finished")


async def cmd_commit(args):
    async with connect(args.address) as client:
        await Board(client).control(CMD_COMMIT)
    print("Committed. The device is marking itself commissioned and restarting.")


async def cmd_reboot(args):
    async with connect(args.address) as client:
        await Board(client).control(CMD_REBOOT)
    print("Restarting.")


async def cmd_identify(args):
    async with connect(args.address) as client:
        await Board(client).control(
            CMD_IDENTIFY, bytes([1 if args.state == "on" else 0])
        )
    print(f"Identify {args.state}.")


async def cmd_factory_reset(args):
    if not args.yes:
        answer = input(
            "This erases all persisted state, including commissioning and BLE\n"
            "bonds, and reboots. Type 'erase' to continue: "
        )
        if answer.strip() != "erase":
            sys.exit("Cancelled.")
    async with connect(args.address) as client:
        await Board(client).control(CMD_FACTORY_RESET, b"RST!")
    print("Factory reset requested.")


async def cmd_end(args):
    async with connect(args.address) as client:
        await Board(client).control(CMD_END_COMMISSIONING)
    print("Programming mode ended: the board is silent again.")


async def cmd_watch(args):
    stop = asyncio.Event()

    def on_status(_handle, data):
        print(Status(data).describe())

    async with connect(args.address) as client:
        await client.start_notify(UUID_STATUS, on_status)
        print("Subscribed to live status. Ctrl-C to stop.")
        try:
            await stop.wait()
        except asyncio.CancelledError:
            pass
        finally:
            await client.stop_notify(UUID_STATUS)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--address", "-a", help="device BLE address")
    sub = parser.add_subparsers(dest="command", required=True)

    scan = sub.add_parser("scan", help="find boards with the window open")
    scan.add_argument("--timeout", type=float, default=8.0)
    scan.set_defaults(func=cmd_scan, needs_address=False)

    show = sub.add_parser("show", help="device info, every setting, live status")
    show.set_defaults(func=cmd_show, needs_address=True)

    get = sub.add_parser("get", help="read one setting")
    get.add_argument("key")
    get.set_defaults(func=cmd_get, needs_address=True)

    setter = sub.add_parser("set", help="write one setting")
    setter.add_argument("key")
    setter.add_argument("value")
    setter.set_defaults(func=cmd_set, needs_address=True)

    commit = sub.add_parser("commit", help="mark commissioned, close window, restart")
    commit.set_defaults(func=cmd_commit, needs_address=True)

    reboot = sub.add_parser("reboot", help="restart without marking anything")
    reboot.set_defaults(func=cmd_reboot, needs_address=True)

    identify = sub.add_parser("identify", help="drive the board LED")
    identify.add_argument("state", choices=["on", "off"])
    identify.set_defaults(func=cmd_identify, needs_address=True)

    reset = sub.add_parser("factory-reset", help="erase everything and reboot")
    reset.add_argument("--yes", action="store_true", help="skip the confirmation")
    reset.set_defaults(func=cmd_factory_reset, needs_address=True)

    end = sub.add_parser("end", help="leave programming mode now")
    end.set_defaults(func=cmd_end, needs_address=True)

    watch = sub.add_parser("watch", help="stream live readings")
    watch.set_defaults(func=cmd_watch, needs_address=True)

    args = parser.parse_args()
    if args.needs_address and not args.address:
        parser.error("--address is required (run `scan` to find one)")

    try:
        asyncio.run(args.func(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
