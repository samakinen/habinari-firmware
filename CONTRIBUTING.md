# Contributing to Habinari

Thanks for looking. This is a prototype that is being built in the open, so
issues, questions and patches are all useful — including "this document is
wrong" and "this did not build".

## Licensing of contributions

Habinari is **GPL-3.0-or-later**. By submitting a pull request you agree that
your contribution is licensed under the same terms and that you have the right
to submit it. There is no CLA and no copyright assignment; you keep your
copyright.

New files should carry the two-line header the rest of the tree uses:

```c
// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) <year> <your name>
```

Do **not** add GPL headers to third-party files. `components/bme68x/bme68x.c`
and its headers are Bosch's under BSD-3-Clause and stay that way; see
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).

Never commit Bosch BSEC material (`components/bsec2/bosch/`). It is
redistribution-forbidden and git-ignored for that reason.

## Getting set up

```bash
git clone --recurse-submodules https://github.com/samakinen/habinari-firmware.git
cd habinari-firmware
idf.py set-target esp32c6
idf.py build
```

ESP-IDF v5.3 or later; developed against v6.1. A devcontainer with the toolchain
is in [.devcontainer/](.devcontainer/).

## Before you open a pull request

1. **Run the host tests.** They need no hardware and no ESP-IDF:

   ```bash
   tools/run_main_host_tests.sh
   ```

2. **Build the variants your change can affect.** A change to shared code
   affects all of them, and the personalities are mutually exclusive, so a
   default build alone does not prove much:

   ```bash
   tools/build_variants.sh
   ```

3. **Keep the platform-free parts platform-free.** `control_core.hpp`,
   `sensor_fusion.hpp`, `hvac_control.hpp`, `device_config.c`,
   `modbus_registers.h` and `device_secret_core.cpp` are host-compiled and
   host-tested. No ESP-IDF headers, no FreeRTOS, no logging macros in them —
   that is what makes them testable, and it is enforced by the fact that the
   host build has no such headers.

4. **New behaviour in those parts needs a test in `main/test/`.**

## Things worth knowing before you change them

* **`main/include/knx_product.hpp` is the source of truth for the ETS catalogue
  entry as well as the runtime.** Adding, removing or reordering a group object
  or a parameter changes the application program, which means existing ETS
  projects must be re-imported and re-downloaded. Bump `applicationVersion` and
  `kParameterLayoutVersion` when you do, and say so in the pull request.
* **Adding a field bus** is: write the mapping, define one `protocol_adapter_t`,
  register it in `main/src/protocol_registry.c` behind a Kconfig symbol. If it
  needs a radio it cannot coexist with KNX — see
  [docs/protocol-variants.md](docs/protocol-variants.md).
* **Adapters must not own state.** Setpoints, modes and persisted configuration
  belong to `control_service`; an adapter maps them onto a wire format and does
  nothing else. A value that only one protocol can reach is a bug.
* **The TP1 receiver has about one microsecond of timing margin.** Anything that
  adds interrupt latency — a new high-priority ISR, a long critical section — can
  break KNX reception on hardware while every test still passes.

## Style

Match the file you are editing. Broadly: 4-space indent, no tabs, C++23 for the
`.cpp`/`.hpp` sources and C for the `.c` ones, comments that explain *why*
rather than restating the code.

## Reporting bugs

Include the firmware variant (which personality), the ESP-IDF version, and the
console log around the failure. For KNX problems, the ETS version and what ETS
reported are usually the two facts that identify the bug.

Security issues go to [SECURITY.md](SECURITY.md), not to the issue tracker.
