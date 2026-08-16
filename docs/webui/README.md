# The browser commissioning client

`index.html` is a second client for the out-of-band service channel, speaking
the same GATT contract as `tools/ble_config.py` — wire version 1, described in
[../ble-commissioning.md](../ble-commissioning.md). It exists so that a
commissioning laptop or an Android phone needs nothing installed: no Python, no
`bleak`, no signing, no app store.

`ble_config.py` remains the reference client and the thing to reach for on a
bench, because it prints exactly what the device reports. This is the one an
installer uses.

Four files, no build step, no dependencies:

| File | What it is |
|---|---|
| `index.html` | the whole client — wire decoding, the form, the actions |
| `sw.js` | offline cache, so a plant room with no signal still has a tool |
| `manifest.webmanifest` | lets Chrome add it to a home screen |
| `icon.svg` | the icon that goes with it |

---

## Where it has to be served from

Web Bluetooth is only available in a **secure context**: HTTPS, or `localhost`.
A file opened straight from disk gets no `navigator.bluetooth` at all — the page
detects that and says so rather than failing obscurely, but there is no way
around it.

### GitHub Pages, from `/docs` (recommended)

Repository → Settings → Pages → Source: *Deploy from a branch*, branch `main`,
folder `/docs`. The client is then at

```
https://samakinen.github.io/habinari-firmware/webui/
```

and the rest of `docs/` — the HVAC controller manual, the Modbus register map,
the MQTT integration notes — is published alongside it. That is the intent: the
documentation is publishable, the firmware is not. Pages serves only the folder
it is pointed at, so `main/`, `components/` and `tools/` stay private with the
repository.

Nothing else changes, and no workflow file is needed. `docs/.nojekyll` makes
Pages serve the folder as-is rather than running it through Jekyll.

**One prerequisite worth checking first.** GitHub Pages is available for private
repositories only on a paid plan (Pro, Team or Enterprise). On GitHub Free,
Pages works for public repositories only, so a private repository cannot publish
at all — see the fallback below.

### If the repository is private on GitHub Free

Push this directory, and nothing else, to a small **public** repository of its
own:

```bash
git subtree push --prefix docs/webui <public-remote> main
```

The firmware stays where it is. Re-run it after every change to `index.html`.

### Locally, on a bench

```bash
python3 -m http.server 8000 --directory docs/webui
```

then open `http://localhost:8000`. `localhost` counts as a secure context even
over plain HTTP, so this works with no certificate and no hosting at all — and
it is the quickest way to try a change before publishing it.

### What being published means

The page becomes public whichever route is taken, and that part is fine: it
contains no secrets and no device data. Everything it knows comes from a BLE
link the browser opens on the installer's own machine, and that link is useless
without physical access to a board in programming mode and its per-device
passkey.

`.nojekyll` is present in both this directory and `docs/`, so either route
serves the files as-is.

### Offline

Load the page once with a network. The service worker caches it, and it works
afterwards with no connectivity at all. On Android, Chrome's *Add to Home
screen* turns it into a launcher icon that opens without the address bar.

After editing `index.html`, **bump `CACHE_VERSION` in `sw.js`** or returning
users keep the old page.

---

## What it does

* Reads device info: serial, firmware, compiled-in personalities, whether the
  board has been commissioned, whether a restart is pending, and whether the
  passkey came from the eFuse root secret or the development fallback — the last
  of these raises a warning, because a board in that state has the same passkey
  as every other unprovisioned board.
* Renders every registered setting as a form built from the device's own
  descriptors — a `<select>` for enums and bools, a bounded number field for the
  numeric types, a password field for secrets. Nothing about the settings is
  compiled into this page; adding one to the firmware makes it appear here.
* Validates before writing, using the bounds in the descriptor. This is not
  belt-and-braces: Web Bluetooth flattens every ATT error into one generic
  `DOMException`, so `0x13 value not allowed` and `0x03 write not permitted`
  arrive indistinguishable. The device still refuses bad values — that is where
  the real check lives — but the message an installer reads comes from here.
* Streams live status while connected: readings, controller outputs, mode,
  sensor health, uptime, plus the identify toggle.
* Commit, restart, leave programming mode, and factory reset behind a typed
  confirmation.

Values are never clamped, secrets are never displayed, and the page holds no
state of its own — everything it shows was read back from the device.

---

## Keeping it in step with the firmware

`index.html` decodes the GATT characteristics by hand — fixed offsets into a
packed struct, opcode constants, the type and flag enumerations from
`device_config.h`. Nothing in any build links the two, so a field inserted into
`oob_status_t` would leave the page reading the wrong bytes with no error
anywhere: the readings would simply be wrong, which is the worst way for a
commissioning tool to fail.

`tools/check_webui_parity.py` is the link. It parses both sides and compares
them — offsets, accessor widths, opcodes, flag bits, the setting types, the
string bound, and the service UUID rebuilt from the `OOB_UUID128` macro:

```bash
tools/check_webui_parity.py
```

It needs no compiler and no browser, and it runs with the host suite as
`test_webui_parity`:

```bash
tools/run_main_host_tests.sh
```

When it fails it names the constant, prints both values and says which side is
the firmware. The page is what changes; the firmware is the authority.

Two things it deliberately does not check, because it cannot:

* **Meaning.** A field that keeps its offset but changes what it means passes.
  That is what `OOB_WIRE_VERSION` is for — bump it, and the check fails until
  the page is revisited, which is the point.
* **The text encoding.** `on`/`off`, enum labels, `<set>`/`<unset>` and the
  bounds rules in `validate()` are transcribed from `device_config.c`. Its own
  host test (`test_device_config`) pins the device's behaviour; nothing pins the
  transcription. If you change the encoding, read `validate()` and
  `buildInput()` on the way past.

---

## Platform support

| Platform | Status |
|---|---|
| Chrome / Edge on Windows, macOS, Linux, ChromeOS | supported |
| Chrome on Android | supported |
| Safari, Firefox (any platform) | **no Web Bluetooth**, and neither vendor intends to add it |
| iOS / iPadOS | **not supported** — every iOS browser uses Safari's engine |

iOS is the real gap. If it becomes necessary, the options are a third-party
WebBLE browser from the App Store or a native app — both of which reintroduce
the install this page exists to avoid. Until then, iOS commissions with
`ble_config.py` from a laptop.

---

## Pairing

There is no pairing API in Web Bluetooth; the operating system owns it, exactly
as it does for `bleak`. Every characteristic on the device demands an encrypted
**and authenticated** link, so the first protected read triggers the platform's
passkey prompt.

* **Android, Windows, macOS** — a system dialog appears. Type the six digits.
* **Linux** — Chrome ships no pairing agent. Pair once with `bluetoothctl`
  (`agent KeyboardDisplay`, `default-agent`, `pair <address>`, the six digits,
  `trust <address>`); the bond is reused afterwards. This is the same caveat
  `ble_config.py` has always carried.

A factory reset erases bonds along with everything else, so the host must be
told to forget the device before pairing again.

---

## Troubleshooting

**The chooser is empty.** Almost always a board that is not in programming mode.
Hold the programming button for a second — the LED lights while the window is
open, and it lapses after 15 minutes by default. `scan` finding nothing is
correct behaviour, not a fault.

**The chooser is empty and the LED *is* lit.** The service UUID rides in the
scan response rather than the advertisement, because a name and a 128-bit UUID
do not both fit in 31 bytes. Chrome scans actively and merges the two, but if a
platform does not, the filtered chooser will be empty. The *Board not in the
list?* link reopens the chooser unfiltered; pick the board by name (`<prefix>
XXYYZZ` — `CONFIG_HABINARI_DEVICE_NAME_PREFIX` plus the last three MAC bytes,
`HVAC Controller XXYYZZ` by default — or whatever `dev.name` was set to).

**Nothing is found on Android.** Android requires location services to be
switched on for BLE scanning, and Chrome needs the *Nearby devices* permission.
Neither is about location in any meaningful sense, but the scan silently returns
nothing without them.

**Connects, then fails on the first read.** That is the pairing step. See above.

**"The board refused this value".** The device refuses out-of-range values
rather than clamping them — a device that quietly turned Modbus address 300 into
247 would be a device nobody could find. The accepted range is printed under the
field.

**The page says it reports a different wire version.** The firmware and this
page disagree about the GATT contract. Update whichever is older; the version
byte is `OOB_WIRE_VERSION` in `main/include/oob_service.h`.
