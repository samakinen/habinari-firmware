# The out-of-band service channel

Some settings decide whether a field bus works at all, so they cannot be
written over it. This document is about the channel that carries those, the
registry behind it, and the GATT contract a client speaks.

---

## 1. The problem, precisely

It is circularity, not convenience.

**Modbus.** The slave address and baud rate live in NVS and were writable only
over Modbus itself. That worked exactly until it did not: two boards out of the
box were both address 1 on the same twisted pair, and a master could not address
either of them to move one.

Modbus now solves this on its own bus — an unaddressed board is silent, and the
programming button or a broadcast serial-select names one at a time. See
[modbus-register-map.md](modbus-register-map.md#commissioning). This channel is
still the pleasanter way to do it from a phone, and it is what reaches settings
that no compiled-in personality exposes.

**MQTT.** Worse. The SSID, passphrase and broker URI decide whether the device
has a network. Nothing on the network can write them, because there is no
network until they are already right. A misprovisioned board is unreachable by
every means the firmware has.

**KNX.** Does not have the problem. ETS commissions over the same twisted pair,
but it reaches the device through the *programming button* and the individual
address rather than through an address the device already has to be given. The
standard broke the loop in 1990 and everything else — parameters, group
objects, diagnostics — rides on the application program. This is why a KNX
image has no BLE in it and needs none; see §6.

---

## 2. The shape of the solution

Two layers, and the split is the whole design:

| Layer | File | What it is |
|---|---|---|
| Registry | `main/include/device_config.h` | Typed settings, contributed by whoever owns them. Portable C: no NVS, no FreeRTOS, no radio. Host-tested. |
| Channel | `main/include/oob_service.h`, `main/src/oob_ble.c` | One transport that renders the registry. Owns no settings. |

Owners register their own items during start-up:

```
control_service.cpp   dev.name
mb_rtu_slave.c        mb.addr, mb.baud
mqtt_service.c        net.ssid, net.pass, net.broker, net.user, net.mqpass
```

and the channel renders whatever it finds, with no idea what any of it means.
Replacing BLE with a console or a USB device is replacing `oob_ble.c` and
nothing else — which is the test of whether the split earned its keep.

Two properties belong to the registry rather than to any channel, because a
channel that had to remember them would eventually forget:

* **Out-of-range values are refused, never clamped.** A device that quietly
  turns Modbus address 300 into 247 is a device nobody can find on the bus.
* **Secrets never come back out.** `DEVICE_CONFIG_FLAG_SECRET` items render as
  `<set>` or `<unset>` whatever their own `get()` hook returns, so one careless
  owner cannot leak a passphrase and neither can a second transport added
  later. `main/test/test_device_config.cpp` pins both with a deliberately leaky
  hook.

---

## 3. Access model: one gate for every protocol

The channel advertises exactly while the board is in **programming mode**, and
nothing else opens it. There is no separate BLE window and no second gesture.

Programming mode is the board's one protocol-neutral "this device is selected"
state, and every commissioning path is gated on it:

| Protocol | What programming mode unlocks |
|---|---|
| KNX TP1 | the individual-address write, as the standard has always done |
| BLE | advertising at all |
| Modbus RTU | the address/baud commit, and the commissioning address for an unaddressed board |

It is raised by holding the programming button for a second, by ETS on a KNX
build, or by the board itself at first boot when nothing has ever been committed
— there being no other way in at that point. The LED is lit for exactly as long
as it lasts. **A board with its LED lit is the board that can be commissioned**,
whatever protocol you reach it with, and that is the whole mental model an
installer needs.

It lapses on its own after `CONFIG_HABINARI_PROGRAMMING_MODE_TIMEOUT_S`
(default 900 s), because a device left selected is a device anybody within reach
can re-address or pair with. The timeout is delivered as a synthetic button
press, so a KNX build leaves programming mode through the same path ETS and the
button already use rather than through a third mechanism nobody has reasoned
about. It cannot interrupt an ETS download: programming mode is needed only for
the individual-address write, and everything after that is normally addressed.

Outside programming mode the radio is silent and the device cannot be found at
all — `scan` returning nothing is correct behaviour, not a fault.

> **Fixed along the way.** Before this, the button raised a toggle request that
> only the KNX adapter consumed. On a Modbus-only or MQTT-only build nothing
> ever picked it up, so the LED never lit and identify did nothing at all. The
> control service now owns the flag whenever no adapter claims it — see
> `protocol_adapter_t::owns_programming_mode`.


### Pairing

LE Secure Connections with passkey entry, bonded. Every characteristic requires
an encrypted **and authenticated** link with a 16-byte key — authenticated is
the one that matters, because encryption alone is satisfied by Just Works
pairing, which any passer-by can complete.

The six-digit passkey is derived per device from the eFuse root secret:

```
passkey = HMAC-SHA256(root, "habinari/BLE-passkey/v1" || serial[6])[0..3] mod 10^6
```

This is the same root the KNX FDSK comes from, under a different
domain-separation label, so the passkey inherits every property that was built
for the FDSK: unique per device, unpredictable from public data, unreadable by
software (only the HMAC peripheral can use it), and surviving `nvs_flash_erase`,
a full `erase-flash`, a reflash and OTA. A label printed at manufacture stays
valid for the life of the chip. See `main/include/device_secret.hpp`.

On a device with no root secret burned, the firmware falls back to passkey
`123456` and logs an error on every boot. It is the same on every unprovisioned
board on purpose — a value that looked unique but was not would invite somebody
to print it on a label.

**What this buys and what it does not.** A static passkey is weaker than a
random per-session one: an attacker who observes a pairing learns it for good.
It is the standard trade for a headless device, and the mitigations are the ones
that fit the threat — the passkey is on the device (physical possession), the
programming-mode window is minutes long and gated behind a button press, and
bonding means the common case pairs once in a plant room rather than repeatedly
in a corridor.
A factory reset erases NVS, which revokes every bond along with everything else.

---

## 4. The GATT contract

One vendor primary service, six characteristics. All values little-endian.

```
service                       1d298b96-21f7-4109-87a0-17de99550000
  device info      read       ...0001
  select           read/write ...0002
  descriptor       read       ...0003
  value            read/write ...0004
  control          write      ...0005
  status           read/notify...0006
```

`select` / `descriptor` / `value` are an indexed cursor: write an item index to
`select`, then read `descriptor` to learn what it is and read or write `value`
to use it. The pattern costs one extra round trip and works with any MTU, which
matters on a channel where the client is somebody's phone.

Indices are stable for the life of a boot — registration only happens during
start-up — but not across firmware versions. Look items up by key.

### Device info (`...0001`, read)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | wire version (currently 1) |
| 1 | u8 | number of settings |
| 2 | u8 | flags |
| 3 | u8 | reserved |
| 4 | u8[6] | serial number (the base MAC; the same six bytes KNX uses) |
| 10 | char[64] | `"<firmware>\0<personality,personality>\0"` |

Flags: `0x01` committed at least once, `0x02` a stored setting needs a restart,
`0x04` the passkey came from the eFuse root secret rather than the development
fallback.

### Select (`...0002`, read/write)

u16 item index. Writing an index past the end is refused.

### Descriptor (`...0003`, read)

Describes the selected item.

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | index |
| 2 | u8 | type: 0 bool, 1 uint, 2 int, 3 float, 4 enum, 5 string |
| 3 | u8 | flags: `0x01` write-only/secret, `0x02` read-only, `0x04` needs restart |
| 4 | f32 | min (for `string`, unused) |
| 8 | f32 | max (for `string`, the maximum length in characters) |
| 12 | u8 | number of enum labels |
| 13 | … | NUL-terminated: key, label, unit, then one label per enum code point |

### Value (`...0004`, read/write)

UTF-8 text, not NUL-terminated — the ATT length delimits it. Text is the
interchange form because it is self-describing, it survives any MTU for every
type the registry has, and it is what a console channel would need anyway.

* `bool` accepts `1/0`, `true/false`, `on/off`, `yes/no`, case-insensitively,
  and reads back as `on`/`off`.
* `enum` accepts either a label or a code point, and reads back as the label —
  so a dumped configuration can be written back unchanged.
* Secrets read back as `<set>` or `<unset>`.

Errors come back as ATT error codes: `0x13` (value not allowed) for a value out
of range or malformed, `0x03` (write not permitted) for a read-only item, `0x0D`
(invalid length) for an over-long write.

### Control (`...0005`, write)

One opcode byte, plus a payload where noted.

| Opcode | Action |
|---|---|
| `0x01` | commit: mark commissioned, leave programming mode, restart |
| `0x02` | restart, marking nothing |
| `0x03` | identify blink — payload `u8`, non-zero lights the LED. Local to this link and does not affect programming mode: it stays connected either way, so it is safe to flip off without ending the session (use `0x05` for that). |
| `0x04` | factory reset — payload must be the four bytes `RST!` |
| `0x05` | leave programming mode now |

The magic word on the factory reset is not paranoia about the link. An
authenticated link means the client is who it says; it says nothing about
whether a fat finger hit `0x04` instead of `0x03`.

### Status (`...0006`, read and notify)

20 bytes of live readings, notified every
`CONFIG_HABINARI_OOB_BLE_STATUS_PERIOD_MS` (default 2 s) while a client is
subscribed, and not at all otherwise.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | wire version |
| 1 | u8 | flags: `0x01` readings valid, `0x02` identify blink (opcode `0x03`, not programming mode — a connected client is in programming mode by definition), `0x04` fault, `0x08` restart pending |
| 2 | i16 | temperature, °C × 100 |
| 4 | i16 | humidity, %RH × 100 |
| 6 | i16 | CO₂, ppm |
| 8 | i16 | active setpoint, °C × 100 |
| 10 | u8 | heating % |
| 11 | u8 | cooling % |
| 12 | u8 | ventilation % |
| 13 | u8 | HVAC operating mode |
| 14 | u8 | controller mode |
| 15 | u8 | sensor health bitmask |
| 16 | u32 | uptime, seconds |

`0x8000` in a signed field means "no reading", the same sentinel and the same
scaling the Modbus register map uses, so the two views of this device cannot
disagree about what an absent sensor looks like.

---

## 5. Using it

There are two clients, speaking the same contract. `tools/ble_config.py` is the
reference one; `docs/webui/` is the one an installer uses.

### From a browser

`docs/webui/index.html` is a single self-contained page that renders the same
registry over Web Bluetooth — no Python, no `bleak`, nothing installed on the
commissioning machine. It is served from GitHub Pages; see
[webui/README.md](webui/README.md) for how, and for why it is published from a
branch of its own rather than out of `docs/`.

It builds its form out of the descriptors the device hands over, so a setting
added to the registry appears there without the page changing. Three things
about it belong here rather than in its own README, because they are properties
of the channel rather than of the page:

* **It cannot be opened from disk.** Web Bluetooth is only available in a secure
  context, so the page has to come over HTTPS or from `localhost`.
* **Chrome and Edge only, and no iOS at all.** Safari and Firefox have not
  implemented Web Bluetooth, and every iOS browser uses Safari's engine. Android
  is supported and is the mobile target; an iPhone commissions with the CLI from
  a laptop.
* **ATT error codes do not survive the browser.** Web Bluetooth collapses `0x13`,
  `0x03` and `0x0D` into one generic `DOMException`, so the page validates
  against the descriptor's own bounds before writing and treats the device's
  refusal as the backstop. The device is still the authority — §2's rule that
  values are refused and never clamped is unaffected — but the message the
  installer reads is composed client-side.

Offline works after one load: a service worker caches the page, and Chrome on
Android will add it to a home screen.

The page decodes §4 by hand at fixed offsets, and nothing in the build links the
two, so `tools/check_webui_parity.py` compares them — offsets, accessor widths,
opcodes, flag bits, the setting types and the service UUID. It runs with the
host suite as `test_webui_parity` and needs no compiler or browser. A change to
`oob_service.h` or `device_config.h` that the page has not followed fails there
rather than on a bench.

### From a terminal

`tools/ble_config.py` is the reference client — small enough to read in one
sitting, which is the point. It is what to reach for on a bench, because it
prints exactly what the device reports.

```bash
pip install bleak

tools/ble_config.py scan
tools/ble_config.py -a AA:BB:CC:DD:EE:FF show
tools/ble_config.py -a AA:BB:CC:DD:EE:FF set mb.addr 17
tools/ble_config.py -a AA:BB:CC:DD:EE:FF set net.ssid "Site Wi-Fi"
tools/ble_config.py -a AA:BB:CC:DD:EE:FF set net.pass "correct horse battery staple"
tools/ble_config.py -a AA:BB:CC:DD:EE:FF set net.broker "mqtt://10.0.0.5:1883"
tools/ble_config.py -a AA:BB:CC:DD:EE:FF commit
```

`commit` marks the device commissioned, leaves programming mode and restarts
into the new settings. After that the device advertises only when the button is
held again.

`watch` streams the live status, which is the fastest way to confirm a board is
mounted in the right room before leaving the building.

On Linux, `bleak` has no pairing agent — BlueZ does. Pair once with
`bluetoothctl` (`agent KeyboardDisplay`, `default-agent`, `pair <address>`,
enter the six digits, `trust <address>`) and the bond is reused afterwards.
Windows and macOS raise their own passkey prompt on the first protected read.

### Building an image that has it

```bash
tools/build_variants.sh modbus-ble    # RS-485 + BLE commissioning
tools/build_variants.sh mqtt          # Wi-Fi/MQTT + BLE commissioning
```

`sdkconfig.defaults.ble` is an overlay rather than a variant, because the
channel is not a personality — it composes with one.

| Variant | Image | OTA slot |
|---|---|---|
| Modbus only | 349 216 B | 17 % |
| Modbus + BLE | 776 496 B | 39 % |
| KNX TP1 + Modbus | 998 688 B | 50 % |
| MQTT + Modbus + BLE | 1 590 144 B | 80 % |

BLE costs roughly 430 kB of flash. That is most of the difference between the
Modbus images, and it is why the channel is `default n`: an RS-485 site can
commission a board entirely over Modbus now, so the small image is a real
product rather than a compromise.

The MQTT variant at 80 % of the OTA slot is the one to watch. It still fits, and
an OTA still has somewhere to land, but there is not room for another large
subsystem beside it — Matter over Thread, if it comes, will need this measured
again.

---

## 6. Why a KNX image has none of this

`CONFIG_HABINARI_OOB_BLE` depends on `!HABINARI_PROTOCOL_KNX`, and
`main/src/protocol_registry.c` refuses to compile a KNX image with
`CONFIG_BT_ENABLED` set by any other route. Three independent reasons, any one
of which would be enough:

**There is nothing for it to do.** ETS is already a complete out-of-band
configuration channel, and a better one: it is standardised, every integrator
already owns it, and it carries the whole application program rather than a
handful of bootstrap settings. The circularity in §1 never arises, because the
programming button and the individual address are exactly the mechanism this
channel reinvents. Adding BLE to a KNX build would mean two ways to configure
one device — one of them worse, and both able to disagree.

**Timing.** The TP1 receiver bit-bangs a 104 µs bit cell from a GPIO edge ISR
and a gptimer alarm, sampling at 52 µs. The measured worst-case ISR jitter on
this board is 51 µs: one microsecond of margin before a late alarm lands past
the next cell's edge and steals its bit. The BLE controller installs its own
high-priority interrupts, takes long critical sections and has hard MAC
deadlines of its own. It spends margin that is not there.

**Power.** KNX is the bus-powered personality, within roughly 10 mA at 29 V from
the bus terminal. A BLE transmit burst is not in that budget. Every other
personality needs the 5–30 V auxiliary supply anyway, which is where the radio
becomes affordable.

The Modbus settings are still registered on a KNX+Modbus image — the registry
does not care whether anything is rendering it — so exposing the RS-485 address
as an ETS parameter later is a small change, and the natural one. It would bump
the parameter layout version and require an ETS re-import, which is why it is
not being done as a side effect of this work.

---

## 7. Adding a setting

Three steps, and none of them touch the channel.

1. Write a `get`/`set` pair over wherever the value lives.
2. Add a `device_config_item_t` with a key, a type and honest bounds.
3. `device_config_register()` it from your start path — **before** anything that
   can fail, so a subsystem that could not start is still reconfigurable. That
   is the case that matters most.

```c
static const device_config_item_t my_items[] = {
    {
        .key   = "mb.addr",              /* owner.name, <= 15 chars, stable forever */
        .label = "Modbus slave address",
        .type  = DEVICE_CONFIG_TYPE_UINT,
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .min = 1.0f, .max = 247.0f,      /* refused outside, never clamped */
        .get = my_get, .set = my_set,
    },
};
device_config_register(my_items, 1);
```

Keys are on the wire and in scripts: pick one and keep it. Bounds are `float`,
which is exact for every integer below 2²⁴ — three orders of magnitude more than
any address, port or percentage needs.

## 8. Adding a channel

Adding a *client* is a different and much smaller thing: §4 is the whole
contract, and `tools/ble_config.py` and `docs/webui/index.html` are two
independent readings of it. Neither knows what any setting means. A third would
want its own entry in `tools/check_webui_parity.py`, for the reason given
there — a client that decodes offsets by hand has nothing else holding it to
this file.

`oob_service.h` is the contract: `start`, `set_programming_mode`, `advertising`,
`client_connected`. Write a second implementation, guard it with
its own Kconfig symbol, and render `device_config_count()` / `device_config_at()`
/ `device_config_get_text()` / `device_config_set_text()` however your transport
prefers. `main.c` calls `oob_service_start()` unconditionally and does not know
which one it got, exactly as it does not know which personalities are compiled
in.

The obvious next one is a console channel over the existing USB Serial/JTAG:
zero flash cost, works on a KNX image, and useful on a bench. It would render
the same registry through the same text encoding, which is why the encoding is
text and why it is host-tested.
