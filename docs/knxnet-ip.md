<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
<!-- Copyright (C) 2025-2026 Sami Mäkinen -->

# KNX over KNXnet/IP

The same device, the same application program, the same ETS parameters — on IP
instead of twisted pair.

Build it with:

```bash
tools/build_variants.sh knxip
```

---

## 1. What this variant is

A **KNX IP device** in the sense of 03/02/06 "Communication Medium KNX IP": IP
is not a tunnel to a KNX network here, it *is* the KNX medium. The device joins
the KNXnet/IP routing multicast group and is a full KNX device on it — group
objects, parameters, individual address, ETS download — with no TP1 line
anywhere in the picture and no gateway in between.

That is worth separating from the two things it is often confused with:

| | What it is | Is this us? |
|---|---|---|
| KNX IP device | IP as the KNX medium; the device speaks routing multicast directly | **yes** |
| KNXnet/IP tunnelling client | A device that opens a unicast connection *out* to a gateway | no |
| KNXnet/IP router / interface | A device other tools connect *through* to reach a KNX line | no |

The consequences of that choice are in §5.

### What differs from the TP1 variant

Almost nothing, and deliberately. Every group object, every ETS parameter, every
default and the whole control model are shared source
(`main/include/knx_product.hpp`, `main/src/knx_service.cpp`). What changes:

* the transport — `IpRoutingOptions` instead of a TP1 physical layer;
* the ETS catalogue entry, because ETS derives a topology from the declared
  medium (§4);
* the power requirement — a KNX IP device needs the 5–30 V auxiliary supply,
  since there is no bus terminal to run from;
* the out-of-band channel — BLE is *available* here, and wanted, for the reason
  in §3.

---

## 2. Commissioning from ETS6

### Get the device onto the network

The device needs Wi-Fi before ETS can see it, and the credentials for it are the
circular problem the BLE service channel exists for: they decide whether the
device has a network at all, and nothing on the network can write them until
they are already right.

1. Press the programming button. The LED lights, and the device advertises over
   BLE — the same gate every other commissioning path on this board uses.
2. Write `net.ssid` and `net.pass`. See
   [ble-commissioning.md](ble-commissioning.md) and `tools/ble_config.py`.
3. The device reboots into the network. Both settings carry the reboot flag, so
   the tool tells you that rather than leaving you wondering.

The board still senses and still runs its control loops throughout — an
unprovisioned KNX IP device is a working room controller that nobody can reach,
not a dead one.

### Point ETS at the network

In **Bus → Connections**, choose a connection of type **KNXnet/IP Routing**
(ETS may label it "Multicast"), on the group the device uses — `224.0.23.12`
unless the project changed it. ETS then talks to the multicast group directly,
using the PC itself as the KNX IP endpoint. No router and no interface is
needed; that is the point of the medium.

The device also answers KNXnet/IP discovery — but do not go looking for it
under **Bus → Discovered Interfaces**. That list holds interfaces ETS can
connect *through*, and ETS builds it from the Tunnelling service family, which
this device deliberately does not advertise (§8). Its search response carries
Core and Routing and nothing else, which is what a KNX IP device is. A device
that never appears there is behaving correctly, and no amount of waiting will
change it.

Discovery is a bring-up tool here, not part of commissioning. It answers one
question — is the device on the network and talking? — and Wireshark on
`224.0.23.12`, or any KNXnet/IP scanner, is the place to ask it.

### Find it in programming mode

With the routing connection up and the programming button pressed, the device
appears in **Bus → Devices → Programming Mode**, at its uncommissioned address
`15.15.255`. That list is built from `A_IndividualAddress_Read` — an ordinary
KNX broadcast over the multicast group, not a KNXnet/IP service — and the
device answers it only while programming mode is on. Seeing it there means the
whole path works: routing in, the stack, and routing back out. It is the
milestone to aim for, and the one that says the next step is a download.

### Program it

From there it is an ordinary KNX download:

1. Import `ets_export/habinari_ip_ets.knxprod.xml` (via OpenKNXproducer — see
   the README) and add the device to the project.
2. Press the programming button. The LED lights, and the device answers the
   individual-address write.
3. Download the individual address, then the application program.

### Data Secure

Unchanged from TP1, including the eFuse-derived FDSK. The device logs its serial
number and factory tool key at every boot; enter both in ETS to commission
securely. See the README's device-root-secret section.

The KNX serial number is the board's base MAC, so it identifies the *board* and
not the personality flashed onto it. Reflashing a board from TP1 to KNXnet/IP
leaves the serial — and therefore the device certificate — unchanged, which is
correct: it is still one device. ETS enforces that too, and a project holding
both catalogue entries backed by the same physical board will reject the second
certificate as already in use. Test one medium at a time on a given board, or
use two boards.

---

## 3. Why BLE is available here and not on TP1

On TP1 the out-of-band channel is barred, and for three reasons that all stop
applying on IP:

* **ETS is already the out-of-band channel on TP1.** It reaches the device
  through the programming button and the individual address, over the same
  twisted pair. There is no circularity to break. On IP there very much is: ETS
  reaches the device over a Wi-Fi network that the SSID and passphrase create.
* **Timing.** The bit-banged TP1 receiver samples a 104 µs bit cell at 52 µs
  with about a microsecond of ISR-jitter margin, and the BLE controller spends
  it. A KNX IP device has no bit cell to protect — it is already on the radio.
* **Power.** TP1 is the bus-powered personality, inside about 10 mA at 29 V. A
  KNX IP device runs from the auxiliary supply and has the budget.

---

## 4. Two catalogue entries, one device

`habinari_tp1_ets` and `habinari_ip_ets` are separate ETS products. Every group
object, every parameter and every default is identical; what differs is what
ETS needs in order to place the device in a topology and to keep the two apart
in its catalogue.

They have to be separate. ETS derives what a device may be connected to from its
declared medium, and 03/02/06 rule 4 makes that consequential:

> If a KNX IP device is assigned to a Subnetwork as a simple device then that
> Subnetwork and any Subnetwork higher in the system structure shall contain
> KNX IP devices only.

A single entry claiming both media would let an integrator drop a TP1 board into
an IP line and find out on site. The build follows the image: a TP1 build
refreshes the TP1 entry, a KNXnet/IP build refreshes the IP one.

| | TP1 | KNXnet/IP |
|---|---|---|
| Medium (`MediumType/@Number`) | 0 — TP | 5 — IP |
| Mask version | `07B0h` (1968) | `57B0h` (22448) |
| `Hardware/@IsIPEnabled` | absent | `true` |
| Order number | `HBTP1` | `HBIP1` |
| Hardware serial number | 1 | 2 |
| Application program number | 21 | 22 |

Three of those rows are load-bearing in ways that are invisible from firmware:

* **The medium** is what rule 4 acts on. ETS resolves `MT-*` against its own
  master data by id, so the id and the number both have to say IP — writing
  `MT-0 Number="5"` silently leaves the product on TP1.
* **The mask version** is what the device reports in its Device Descriptor, and
  ETS compares the two before every download. It is medium-dependent because
  the medium is the high nibble of the mask.
* **The application program number** keeps the two entries apart in the ETS
  catalogue. ETS stores one program per manufacturer + number + version, keyed
  by a hash of its content, and refuses an import whose content has moved under
  an existing key — "the product has a different hash than the existing
  product". These two programs cannot share a key, because they declare
  different mask versions.

`main/include/knx_product.hpp` holds all six values, and both the running
firmware and the exported catalogue entry are generated from them.

### Re-importing after a change

ETS will not replace an application program of the same number and version whose
content hash has changed. Any change to the ETS-visible content — parameters,
group objects, the download procedure — therefore needs
`kApplicationVersion` bumped in `knx_product.hpp`, or the old product removed
from the ETS catalogue first.

### Persistence

The NVS namespace differs with the medium (`habinari_tp1` / `habinari_ip`). It
holds the individual address, the group object table and the security key
material — state belonging to one commissioned identity. Reflashing a board from
one medium to the other must not have it wake up holding an address ETS assigned
to the other product.

---

## 5. Topology, and what it costs

Rule 4 above is the price of the medium, and it is worth being explicit about:
**a KNX IP device cannot sit in a mixed line.** Its subnetwork, and every
subnetwork above it, must contain KNX IP devices only. A line with TP1 devices
on it needs a KNXnet/IP router, and then the Habinari on that line should be the
TP1 variant.

Where this variant fits well:

* an all-IP installation, or an all-IP line behind a router;
* a retrofit where pulling bus cable to the room is the expensive part;
* a bench or a demonstration, where a laptop and an access point are the whole
  installation.

Where it does not:

* a room already on a TP1 line — use the TP1 variant, which is also the one
  validated on real hardware.

---

## 6. Settings

Compile-time, under *Habinari protocols → KNXnet/IP medium*:

| Kconfig | Default | What it is |
|---|---|---|
| `HABINARI_KNXIP_MULTICAST_ADDR` | `224.0.23.12` | Routing group. The registered System Setup address. |
| `HABINARI_KNXIP_PORT` | `3671` | Fixed by 03/02/06 §2.1; configurable only for test benches. |
| `HABINARI_KNXIP_TTL` | `16` | Multicast hop limit. `1` confines KNX traffic to the local subnet. |
| `HABINARI_KNXIP_WIFI_WAIT_S` | `30` | How long to wait for an address before starting the stack anyway. |

Run-time, through the settings registry (BLE, or any future service channel):

| Key | What it is |
|---|---|
| `knx.mcast` | Routing group, overriding the compiled-in default. Takes effect on the next boot. |
| `net.ssid`, `net.pass` | Wi-Fi credentials. |

`knx.mcast` is validated on write: a value outside `224.0.0.0/4` is refused
rather than stored. The device joins the group at start-up, so accepting a bad
one produces a board that reboots and is never seen again — refusing the write
is the last point at which that is recoverable without a cable.
(`main/src/knx_ip_addr.c`, host-tested.)

---

## 7. KNX/IP Secure Routing

Distinct from Data Secure, and orthogonal to it. Data Secure protects the
*telegram* end to end; Secure Routing encrypts the *multicast group* so a device
on the LAN cannot read or inject KNX traffic at all.

ETS enables it by downloading a backbone key into the device — an ordinary
property write (`PID_BACKBONE_KEY`) over a secured management connection. The
device applies it immediately, without a restart: the routing socket starts
wrapping every datagram in a `SECURE_WRAPPER` from the next telegram onwards.
Clearing the key in the project turns it back off the same way.

### Do not start with a secured backbone

That property write has to arrive somehow, and on this device it cannot arrive
over a backbone that is already secure. Turning Security on for the IP line
before the first download produces a deadlock, not an error message:

* System broadcasts stay in the clear, so part of the commissioning dialogue
  works and the device looks reachable — ETS reads the serial number, the Data
  Secure sync completes, `A_DomainAddressSerialNumber_Write` lands.
* Everything else — every point-to-point connection, `DeviceDescriptor_Read`,
  `A_IndividualAddressSerialNumber_Read` — is normal traffic, so ETS wraps it,
  and a device with no backbone key drops every one. The download stalls
  part-way through with the device visible and mute.
* `PID_BACKBONE_KEY` is itself a property write over a point-to-point
  connection. It is in the half that is wrapped, so the key that would break
  the deadlock is the one thing that cannot get through it.

What breaks the cycle on a device that supports it is a secure *unicast*
KNXnet/IP session — Device Management or Tunnelling, opened against the
device's own control endpoint with credentials from its device certificate.
Unicast is the point: it reaches a device that cannot read the group. Both
services are in §8, and neither is implemented here, which is why KNX
documentation describes the backbone key as being downloaded to IP Secure
*couplers and interfaces* — the device classes that have them.

So commission with the IP line's Security **off**. Only once the device is
fully downloaded is there a device that could hold a backbone key at all.

The device certificate is not the missing piece, though expecting it to be is
the natural mistake. It does bootstrap this device, exactly as KNX Secure
intends — the Data Secure sync in the trace above is that bootstrap working,
over system broadcast, on an already-secured group. But it bootstraps the
*application layer*: it authorises telegrams whose payload ETS can then
protect. It cannot decrypt a `SECURE_WRAPPER`, which is a different layer with
a different key, and it cannot conjure a point-to-point channel to carry a
download. Two protections, stacked, commissioned by different means.

Two details worth knowing, both from the specification rather than choices made
here:

* **Discovery and system broadcasts stay in the clear** even on a secured group
  (03/02/06 §4.1.3). They carry their own protection, in the Security Control
  Field, and a device that tried to decrypt them would fail an unwrap they were
  never meant to pass.
* **Plain routing indications are dropped** once a group is secured. Accepting
  them would make the encryption advisory.

---

## 8. What is not implemented

Named because "not implemented" and "implemented but untested" are different
things, and both are worse to discover on site:

* **KNXnet/IP Tunnelling server.** ETS cannot use this device as an *interface*
  to reach other devices. It is an end device on the medium, not a gateway.
  Device Class C makes Tunnelling optional (03/08/01 Table 2).
* **Device Management server.** Mandatory for Device Class C, and absent. ETS
  configures this device the same way it configures a TP1 one — over the KNX
  network, not over a KNXnet/IP management connection — so nothing in the
  commissioning path above needs it.
* **IP Secure tunnelling sessions.** The client half exists in KNstaX but is
  compiled out here: a KNX IP device does not open tunnelling sessions, and the
  ECDH and PBKDF2 it needs use mbedTLS entry points that mbedTLS 4 — what recent
  ESP-IDF ships — no longer provides.
* **DHCP only.** No static address configuration, no AutoIP. The device reports
  DHCP as its assignment method and does not pretend otherwise.

### Validation status

**This variant has not been run against real ETS6 or on real hardware.** The
protocol work behind it is covered by host tests — the wire formats are checked
octet by octet against the specification's own layouts — and every build variant
compiles and links. That is not the same as a device on a network answering a
tool, and the TP1 variant, which *has* completed a full ETS6 download and secure
commissioning on hardware, remains the one to reach for where either will do.

`components/KNstaX/docs/reference/knx_conformance_status.md` tracks the stack
side of the same question.
