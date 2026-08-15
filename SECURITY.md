# Security policy

## Reporting a vulnerability

Please report security issues privately, not in the public issue tracker:

* Preferred: GitHub's **[Report a vulnerability](https://github.com/samakinen/habinari-firmware/security/advisories/new)**
  (Security → Advisories), which keeps the report private until a fix exists.
* Or email **sami.makinen@flou.io**.

Include what you did, what happened, and which variant and version you were
running. You will get an acknowledgement within a few days. This is a personal
prototype project, not a vendor with an on-call rota, so please be patient about
the fix itself — and say up front if you plan to disclose on a schedule.

## Supported versions

Only `main` is supported. There are no maintained release branches yet, and no
backports.

## What the security model actually promises

Habinari is **prototype firmware on prototype hardware**. Do not deploy it where
a failure or a compromise matters, and do not treat it as a certified KNX
product — it is not, and the manufacturer ID it ships with is a development
placeholder.

What is designed in:

* **Per-device credentials from an eFuse root secret.** A 256-bit root secret
  lives in a read- and write-protected eFuse key block and is only ever used
  through the HMAC peripheral, so software cannot read it back. The KNX FDSK and
  the BLE pairing passkey are derived from it with domain-separated labels, so
  the same firmware image gives every board different keys, and neither
  credential reveals the other. A factory reset and a full `erase-flash` do not
  change them.
* **One gate for commissioning.** Programming mode is the single
  protocol-neutral "this device is selected" state, it requires physical access
  to the button, and it lapses after 15 minutes. BLE advertises only while it is
  set; outside it the device cannot be found at all.
* **KNX Data Secure.** Supported through the Security Interface Object and
  advertised in the catalogue entry, with tool-key-secured management once
  Security Mode is on. Enabling it is the integrator's choice; the device also
  commissions into plain installations.
* **The out-of-band settings registry refuses rather than clamps** out-of-range
  values, and never reads a secret back out. Both properties are covered by host
  tests.

What is **not** promised:

* **No Secure Boot or flash encryption by default.** Someone with physical
  access and a flash programmer can read and replace the firmware image. The
  root secret in eFuse resists this; the rest of the device does not.
* **Modbus RTU and MQTT have no device authentication.** Modbus RTU has no
  security whatsoever — the physical RS-485 segment is the trust boundary. MQTT
  security is whatever your broker and TLS configuration provide.
* **BLE pairing protects the channel, not the phone.** The passkey is printed
  per device; anyone who has it and physical access during programming mode can
  write the settings that channel exposes.
* **No security review or certification has been performed** on this code by
  anyone.

## Please do not

Report findings against the **dry-run key logging**: with
`CONFIG_HABINARI_ROOT_SECRET_BURN` off, the firmware deliberately prints a
candidate root secret over the console. That candidate is discarded on every
boot and belongs to no device; the behaviour exists so values can be checked on
real hardware before a fuse is burned, and it is switchable with
`CONFIG_HABINARI_ROOT_SECRET_LOG_DRY_RUN_SECRET`.
