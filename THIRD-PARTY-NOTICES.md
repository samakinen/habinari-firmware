# Third-party notices

Habinari itself is licensed under **GPL-3.0-or-later** (see [LICENSE](LICENSE)).
This file records the third-party code it is built with, so that a binary image
can be accompanied by the notices those licenses require.

Nothing listed here is *modified* by this project unless the table says so.

## Vendored in this repository

| Path | Component | Copyright | License |
|---|---|---|---|
| `components/bme68x/bme68x.c`, `include/bme68x.h`, `include/bme68x_defs.h` | Bosch BME68x Sensor API | © 2023 Bosch Sensortec GmbH | BSD-3-Clause ([LICENSE](components/bme68x/LICENSE)) |
| `components/bme68x/bme68x_esp.[ch]`, `components/bme68x/CMakeLists.txt` | ESP-IDF glue for the above | © 2025-2026 Sami Mäkinen | GPL-3.0-or-later |
| `components/hdc302x/`, `components/scd4x/` | I2C drivers written for this project | © 2025-2026 Sami Mäkinen | GPL-3.0-or-later |
| `components/KNstaX/` (submodule) | KNstaX KNX System-7 stack | © 2025-2026 Sami Mäkinen | GPL-3.0-or-later |

BSD-3-Clause is compatible with the GPL; the Bosch notice must be reproduced in
documentation accompanying a binary distribution, which is what this file does.

## Fetched at build time (not distributed here)

The ESP-IDF component manager downloads these into `managed_components/`, which
is git-ignored. Versions are pinned in `dependencies.lock`.

| Component | License |
|---|---|
| ESP-IDF (v6.1) | Apache-2.0 |
| `espressif/esp-modbus` | Apache-2.0 |
| `espressif/mqtt` | Apache-2.0 |
| `espressif/libsodium` | ISC |
| `pedrominatel/sht4x` | Apache-2.0 |

Apache-2.0 is one-way compatible with GPLv3 (and **not** with GPLv2) — this is
why Habinari is GPL**v3**-or-later and cannot be relicensed to GPLv2.

## Bosch BSEC — proprietary, optional, not distributed

`components/bsec2/` contains this project's own integration code
(GPL-3.0-or-later). The library it integrates against — `libalgobsec.a` plus the
Bosch BSEC headers and configuration blobs — is **proprietary, redistribution is
forbidden by its license, and it is not present in this repository**. It is
git-ignored and must be downloaded from Bosch Sensortec under their license
agreement; see [components/bsec2/README.md](components/bsec2/README.md).

`CONFIG_BME688_USE_BSEC` is **off by default**. With it off, `components/bsec2`
compiles as inert stubs, and the resulting firmware contains no Bosch
proprietary code and is a pure GPL work.

Turning it on links a proprietary library into a GPL work, which makes the
resulting binary something you cannot convey under the GPL alone. That is fine
for building and running your own devices, and it is why the option is off by
default. If you want to distribute an image with air quality enabled, ask the
copyright holder — no blanket permission is granted here.

## KNX

KNX® is a registered trademark of the KNX Association cvba. This project is not
affiliated with, endorsed by, or certified by the KNX Association.

The KNX manufacturer ID used in the product definition (`0x00FA`) is a
**development placeholder**. A device sold or distributed as a KNX product needs
a manufacturer ID assigned to you by the KNX Association and a certified
application program; see `main/include/knx_product.hpp`.

No KNX Association specification documents are redistributed by this project.
