# Code review findings — 23 October 2026

Read-only review of the Habinari firmware (`main/`) and the KNstaX stack
(`components/KNstaX`), done at commit `0017af4` (KNstaX `7b1d8c7`) with
uncommitted edits in `main/src/knx_service.cpp`, `main/src/main.c` and KNstaX.
Line numbers refer to that state and will drift as fixes land.

**✔** = confirmed by reading the code during the review. Unmarked items come
from the review agents and were spot-checked but not traced line by line.
**?** = depends on spec details recalled from memory; check the PDFs in
`docs/The KNX Standard v3.0.0` before implementing.

Tick items off as they are fixed. Status as of 1 November 2026: phase 1 of
`docs/code-review-2026-10-fix-design.md` is done (uncommitted), and the items
still open are listed there in section 8. KNstaX paths are relative to
`components/KNstaX/`.

---

## Suggested order

1. Security: P1–P3, P7. Do not ship KNX IP Secure Routing until P1 and P2 are fixed.
2. Reliability: P4, P5, P8, P9, P10, with a host test reproducing each bug.
3. CI: restore KNstaX CI (T1), build the shipping knxip configuration, add
   `idf.py build` per variant, fix the log-level dependency (P11).
4. Remove dead code (section 2) and put client-side KNXnet/IP behind its own Kconfig option.
5. Refactor (section 4): split `knxServiceTask`, merge duplicated
   property/tunnelling code, add a `knx_add_test()` CMake helper.

---

## 1. Correctness and security (fix first)

- [x] **P1 ✔ Secure Routing reuses AES-CTR nonces (high).** When ETS downloads
  the backbone key, `IpRoutingPhysical::applySecureRouting(key)` builds
  `SecureRoutingSecurity` with `secureRoutingSerial_`/`secureRoutingTag_` still
  zero and `initialSeq` defaulting to 1. The nonce is `seq||serial||tag||FF00`,
  so it repeats across reboots and between devices sharing the key.
  `setSecureRouting` (which sets serial/tag) only runs when
  `options.secureRouting.enabled`, which `main/` never sets.
  - `src/physical/ip_routing_physical.cpp:241-290`, `include/knx/physical/ip_routing_physical.hpp:100-101, 215-216`
  - `include/knx/product/commissioned_product.hpp:1434-1466`, `src/netip/secure_wrapper.cpp:124-133`
  - Fix: use the real device serial and tag, keep the sequence across reboots
    (or derive it from the multicast timer), and never restart it under the same key.
- [x] **P2 ✔? Secure Routing has no freshness or replay check (high).**
  `SecureRoutingSecurity::unprotect` only verifies the MAC
  (`src/netip/secure_routing_security.cpp:54-60`). There is no TIMER_NOTIFY
  (0955h) handling and no latency-tolerance check, so captured telegrams can
  be replayed and real IP Secure routers may reject us. Implement the
  multicast timer and synchronisation per 03_08_09 / AN159.
  - Related (?): a plain ROUTING_BUSY is processed on a secured group
    (`src/netip/routing_endpoint.cpp:326-331` vs `:366-378`) and `waitTimeMs` of
    up to 65 535 ms is used as sent (`src/netip/routing_flow_control.cpp:124-137`).
    Require BUSY and LOST_MESSAGE to arrive wrapped when secure, and cap the wait time and busy counter.
- [x] **P3 ✔ Management services ignore the communication mode (high).**
  `ApplicationLayer::handleTransportRx`/`dispatchService`
  (`src/application/application_layer.cpp:1498-1745`) dispatch every APCI
  regardless of `destinationType`. A_Memory_Write or A_PropertyValue_Write to a
  group or broadcast address is executed. Add a per-service table of allowed
  modes (connected, connectionless, broadcast, system broadcast) and check it before dispatch.
- [x] **P4 ✔ TP1: own group/broadcast frames never wait for L2 ACK (high).**
  `txFrameExpectsAck()` returns false for group addresses
  (`include/knx/physical/bitbang_driver_timer_isr.hpp:513-538`; same in
  `src/physical/tp1_mac_physical.cpp:136-145`), so lost group telegrams are never repeated.
  03_02_02 §2.4.2 says L_Data is acknowledged for group frames too. Let the
  data-link repeat logic (`src/datalink/tp1_data_link_layer.cpp:291-318`) handle them.
- [x] **P5 ✔ TP1: the ISR ACKs frames with a bad check octet or length (high).**
  It only downgrades to NACK on parity or framing errors
  (`src/physical/bitbang_driver_timer_isr.cpp:803-806`). Keep a running XOR in
  `rxHandleStop()` and check it, plus the 8–23 / 9–263 octet length, in
  `rxHandleTelegramComplete()` before arming the ACK.
- [x] **P6 Master reset is a no-op (medium-high).** `handleRestart`
  (`src/application/application_layer.cpp:1105-1168`) ignores the erase code and
  channel, and the BAU just calls `_platform.restart()`
  (`src/bau/bau_management.cpp:870-881`), so a factory reset (0x02) wipes
  nothing and reports success. Implement erase codes 1–7 or reject them with
  "Unsupported Erase Code" / "Invalid Channel Number". Also check that
  reserved bits 1–4 of octet 7 are zero (03_03_07 §3.4.2.2).
- [x] **P7 ✔ KNstaX can erase the whole NVS partition (medium).** *Withdrawn on verification; the erase moved to the application anyway (A9).*
  `NvsPersistence::init` (`src/objects/object_persistence.cpp:119-124`) calls
  `nvs_flash_init()` and `ESP_ERROR_CHECK(nvs_flash_erase())`, which would wipe
  Wi-Fi credentials and device secrets. It is latent today only because
  `main/src/control_service.cpp:59` initialises NVS first. Move NVS
  initialisation and erasing out of the library.
- [x] **P8 ✔ Modbus loses master writes and runs commits twice (high).**
  - `mb_refresh_writables` ends with `slave->applied_holding = *h`
    (`main/src/mb_rtu_slave.c:521-522`). A master write that arrives between
    the apply snapshot (`:540`) and the refresh is skipped by `MB_REFRESH`, but
    is then recorded as already applied and dropped. Coils have the same problem.
  - `config_commit` is tested with `!= 0` (`:628`), not `MB_CHANGED`, so a
    commit is processed on two consecutive cycles.
  - Fix: record the `current` snapshot as `applied_*` inside
    `mb_apply_writes`, and clear `config_commit` under the lock when it is consumed.
- [x] **P9 ✔ `TrendMonitor` regression drifts with uptime (high).**
  `main/include/sensor_fusion.hpp:403-442`: `elapsed_` is a `float` that only
  resets on an invalid reading, and `n*sumTT - sumT²` loses precision through
  cancellation. It is about 1–2 % off after a day, meaningless after about a
  week, and after about 194 days a 1 s step no longer changes `elapsed_`. The
  slope drives window-open detection, the fire rate-of-rise alarm and occupancy.
  Regress on `t - t_oldest` and add a host test with a large time offset.
- [x] **P10 Commands accept NaN, ±inf and out-of-range enums (high).**
  - `main/src/mqtt_payload.c:330-338`: `strtod` accepts "nan" and "inf".
  - `main/src/control_service.cpp:538-554` stores `SETPOINT_SHIFT` and `CO2_SETPOINT` unchecked.
  - Casting an out-of-range float to `uint8_t` is undefined behaviour, and
    `clampf(NaN)` returns NaN into the PID.
  - KNX casts raw bus bytes to `OperatingPreset`/`VentilationMode`
    (`main/src/knx_service.cpp:1045-1048, 1184-1187`).
  - Fix: make `control_state_write` the single place that checks `isfinite`
    and enum ranges, returning `ESP_ERR_INVALID_ARG`. Callers check the result.
- [x] **P11 ✔ KNstaX log level is ignored in production builds (medium).**
  `KNX_LOG_LEVEL` depends on `KNX_DEBUG_ENABLED` (`Kconfig:535-537`), so with
  debug off `CONFIG_KNX_LOG_LEVEL` is undefined and `include/knx/util/log.hpp:20-24`
  falls back to 5, compiling in all logs. `sdkconfig.defaults.knxip:74`
  (`=3`) has no effect. Drop the `depends on`, or default to 2–3 on `ESP_PLATFORM`.

### Other correctness items

- [x] **Missing sensor → reboot loop.** `main/src/sensor_bus.c:117-147` returns an
  error if SCD4x or BME68x setup fails, and `main/src/main.c:206-207` wraps it in
  `ESP_ERROR_CHECK`. The `i2c_bus_init` result is unchecked and the error paths leak handles.
  Treat sensors as degradable.
- [x] **KNX start failure is silent.** `knxAdapterStart` returns `ESP_OK` once
  `xTaskCreate` succeeds (`main/src/knx_service.cpp:2196-2204`). If
  `startProduct` then fails, the task calls `vTaskDelete(nullptr)` (`:1632-1635`).
  Have `start()` wait for the task's result with a timeout.
- [x] **More than 16 group objects on one GA silently drops the telegram.**
  `src/bau/bau_group_runtime.cpp:709-716`, `src/objects/association_table_object.cpp:224`.
- [x] **Downloads are not saved atomically.** Each property is committed
  separately (`src/objects/interface_object_manager.cpp:410-430`,
  `object_persistence.cpp:171`). Save the tables as one blob, or write a commit marker last.
- [x] **Memory services don't conform to 03_03_07 §3.5.3.** A failed read sends
  no A_Memory_Response with number = 0, and a write always sends a read-back,
  ignoring Verify Mode (`src/application/memory_service.cpp`).
- [x] **Unsupported device descriptor type gets no reply.** It should answer
  with type 3Fh (`src/application/device_descriptor_service.cpp:41-44`).
- [x] **A_Authorize levels are stored but never enforced.** `getCurrentLevel`
  has no caller (`src/application/authorization_service.cpp:43-82`).
- [x] **Bit-bang `close()` deletes the RX task before clearing the ISR notify
  callback.** An edge in that window uses a freed TCB
  (`src/datalink/tp1_data_link_layer.cpp:144-166`).
- [x] **ISR ACK table holds 64 GAs while the data link allows 256.** Groups 65+
  are never ACKed (`tp1_data_link_layer.hpp:295`, `.cpp:588-591`).
- [x] **The device never sends BUSY.** `setLocalBusy` is never called, and a
  full RX queue drops already-ACKed frames (`src/physical/tp1_mac_physical.cpp:409-416`).
- [x] **Arbitration loss is reported as BUSY.** It uses up busy retries, and the
  4 ms retry wait is shorter than the ~15.6 ms (150 bit times) the spec
  requires (`bitbang_driver_timer_isr.cpp:949`, `tp1_data_link_layer.cpp:282`).
- [x] **Bus-monitor mode still ACKs** (`src/physical/bitbang_driver_timer_isr_espidf.cpp:217-220`).
- [x] **Transport layer deviates from 03_03_04.** The retransmit back-off
  doubles 3 s → 6 s instead of a fixed 3 s; exhausted retries don't send
  T_Disconnect; T_ACK/T_NAK with a wrong sequence number are ignored
  (`src/transport/connection_table.cpp:283-292`, `transport_layer_receive.cpp:537-566`).
- [x] **Filter table can't hold real ETS ranges.** It is a 64-entry first-match
  `std::vector`, so large ranges are refused, Clear(x) then Set(x) still blocks
  x, and the coupler ack policy is never applied
  (`src/network/filter_table.cpp`, `routing_table_control.cpp:150-165`,
  `two_port_coupler.cpp:127-130`). Use a 0x0000–0x6FFF bitmap.
- [x] **Unauthenticated ROUTING_BUSY.** See P2. Also, the randomised backoff is
  dead: `setRandomSource` stores a source nothing reads
  (`ip_routing_physical.cpp:237`, `routing_endpoint.cpp:373`).
- [x] **Races in `main`.**
  - `sensor_fusion_configure` vs `sensor_fusion_update`
    (`main/src/sensor_fusion_service.cpp:271-316`).
  - Wi-Fi callback/context set in two unsynchronised writes (`main/src/wifi_service.c:373-374`).
  - `taskHandle`, `controlTickPending` and `ipIdentityDirty` are plain or `volatile`
    (`main/src/knx_service.cpp:196-206`).
  - `publish_forced` has the same pattern (`main/src/mqtt_service.c:433-434`).
  - The Modbus handle can be recreated while in use (`main/src/mb_rtu_slave.c:874-884`).
- [x] **MQTT discovery JSON can overflow.** `n += snprintf` without a bounds
  check (`main/src/mqtt_service.c:223-235`).
- [x] **`ESP_ERROR_CHECK` in optional adapters.** It defeats the
  "optional adapter is skipped" design (`main/src/wifi_service.c:304-311`,
  `main/src/mqtt_service.c:540`). Unchecked `esp_mqtt_client_publish` results
  (`mqtt_service.c:235, 401`). A 32-character SSID is truncated (`wifi_service.c:300`).
- [x] **`InplaceFunction` copying a move-only callable is undefined behaviour**
  (`include/knx/util/inplace_function.hpp:61-65, 112-118`).
- [x] **`Esp32Platform::fatalError()` hangs silently** in `vTaskDelay`. Use
  `abort()` or `esp_restart()` (`src/platform/esp32_platform.cpp:50`).
- [x] **Security hygiene (low).**
  - Non-constant-time MAC compares (`secure_wrapper.cpp:262`,
    `secure_application_layer.cpp:177`, `data_secure.cpp:265`).
  - Unverified plaintext is left in the buffer on MAC failure.
  - `uint16_t` truncation before the bounds check in `SecureWrapper::wrap` (`:157`).
  - Key material is not zeroised, and `retiredRoutingSecurity_` grows by one
    key per download.
  - No all-zero X25519 shared-secret check.
  - Key material in `main/` is not zeroised.
- [x] **Client-side issues (only matter if tunnelling is enabled).** The secure
  tunnelling client never verifies the SESSION_RESPONSE MAC
  (`secure_session_bootstrap.cpp:53-60`) and has no replay check.
  `TunnelingServerEndpoint` CONNECT_RESPONSE lacks HPAI/CRD, and its sequence
  handling is wrong.

---

## 2. Unused or unneeded code

Unused anywhere, or only by tests:
- [x] `CMakeLists.standalone.txt`, which references files that don't exist.
- [x] `ManchesterCodec` (TP1 isn't Manchester-coded), `Tp1FrameCodec`, and
  `timer_gpio_hal_virtual.cpp`, which is compiled into the ESP image.
- [x] `RoutingTable` and the NetworkLayer forwarding path (which also gets hop count 7 wrong).
- [x] The async transport receive path (`transport_layer_receive.cpp:74-160, 247-326`),
  a diverging copy of `handleNumberedData`.
- [x] `DataSecureSession` crypto, and `KeyDerivation::deriveSessionKey`/`deriveBilateralKeys`.
- [ ] `application::PropertyStore`/`PropertyStoreManager`, a shadow property model
  that still drives some access decisions (`src/bau/bau.cpp:429-452`,
  `property_ext_services.cpp:135-148, 272-283`).
- [x] About 25 static `encode*`/`decode*` service codecs used only by tests.
  `RestartService` already disagrees with the production parser.
- [x] `Esp32Memory`, which allocates 4 KB eagerly, and the whole SPI abstraction
  (drags `esp_driver_spi` into REQUIRES).
- [ ] Headers included nowhere: `knx.hpp`, `enum_string.hpp` (doesn't compile),
  `allocation_tracker.hpp`.
- [x] Unused `ApplicationLayer` members: `sendToGroup`, `setPropertyWriteConsumer`,
  `defaultSendOptions`, `serviceResponseSendOptions`, `propertyStoreManager`,
  and the receive queue (about 2.2 KB of RAM).
- [x] `BusAccessUnit::beginRequestGroupObjectValue`;
  `GroupObjectTableObject::findByAddress`/`hasAddress` (tests only).
- [x] Kconfig options never read: `KNX_PRODUCTION_BUILD`, `KNX_USE_NVS`,
  `KNX_SECURITY_KEY_SIZE`, `KNX_IP_MULTICAST_ADDR`,
  `KNX_ENABLE_DEVICE_DESCRIPTOR`, `KNX_ENABLE_PROPERTY_SERVICES`.
  `CONFIG_KNX_MEDIUM_RF` is still offered; `CONFIG_KNX_VIRTUAL_BUS` has no entry.
- [x] About 20 unused `config.hpp` constants, including `ENABLE_ROUTING` and
  `RX_TASK_PRIORITY`, which Habinari sets for nothing.
- [x] `main/`: `sensor_fusion_reset`, `sensor_bus_deinit`,
  `control_service_generation`, `deriveBlePasskey`, `FilteredChannel::rawValue`,
  `TrendMonitor::ratePerHour`, `mqtt_service_connected`/`_provision`,
  `mb_rtu_slave_stop`, `oob_service_advertising`/`_client_connected`.
  Also `HABINARI_REQUIRES_AUX_POWER`, `MB_REG_OFFSET`, unused `board.h` pins,
  the constexpr-dead branches at `knx_service.cpp:1773-1775, 2141-2143`, and the
  empty `sdkconfig.ci`.

Compiled into the firmware but not needed:
- [x] Client-side KNXnet/IP: `gateway_discovery_client`, `tunneling_session_client`,
  `ip_tunneling_physical`, and about 2.5k lines of header-only device-management
  commissioner code. It is always in `KNX_SOURCES_NETIP`
  (`cmake/knx_sources.cmake:91-109`). Add a `KNX_SOURCES_NETIP_CLIENT` group
  keyed on `KNX_ENABLE_TUNNELING`, and check the linker map.
- [x] `knx/testing/mock_tp1_physical.hpp` is included unconditionally by
  `src/bau/bau.cpp:41` and `bau_stack_port.cpp:27, 1380`.
- [x] **The TPUART backend is broken, and it is the Kconfig default** (Habinari
  uses bit-bang) ✔?. `LDataCon=0x0B`, but the chip sends 0x8B; only `0x9x`
  control octets are accepted, so normal `0xBC` frames are dropped; TX framing
  is wrong (`src/physical/tpuart_physical.hpp:35-41`, `.cpp:181-204, 393-401`).
  Rewrite it against the NCN5120 datasheet, or remove it and make bit-bang the default.

---

## 3. Performance

- [x] AES is set up 2–3 times per secure packet (`aes128_cbc_mac.cpp:19`,
  `aes128_ctr.cpp:21`). Cache one cipher per security context.
- [x] Heap allocation per inbound IP routing datagram
  (`std::deque<std::vector>`, `ip_routing_physical.cpp:75`) and a cEMI → TP1
  round-trip. Use a fixed ring, or pass `LDataFrame`.
- [x] TP1 RX copies each frame three times and emits unused per-byte events;
  the 128-entry `Tp1RxEvent` queue is about 7 KB of RAM.
- [x] Linear address-table and association-table scans per group telegram.
  Use binary search, plus a GA-to-GO index built after download.
- [x] `pdMS_TO_TICKS` with `CONFIG_FREERTOS_HZ=100` turns timeouts under 10 ms
  into polls (`include/knx/platform/freertos_platform.hpp:117, 139`). Base
  `millis()` on `esp_timer`.
- [x] `takePublishSnapshot()` runs on every loop pass
  (`main/src/knx_service.cpp:2083`); move it inside `if (controlTickDue)`.
- [ ] The 32 KB KNX task stack is sized only for startup
  (`main/src/knx_service.cpp:75`). The header-only product runtime is about
  53 KB of code and its bindings builder uses about 8.9 KB of stack: move the
  parts that don't depend on the template out of `commissioned_product.hpp` /
  `endpoint_runtime.hpp`.
- [ ] `InterfaceObjectManager` nested `std::map`; mixed `std::function` and
  `InplaceFunction`; per-call `std::string` + `std::map` in `log.cpp`
  `resolveEffectiveLevel`.

---

## 4. Maintainability

- [ ] Split `knxServiceTask` (about 1,265 lines, `main/src/knx_service.cpp:882-2146`),
  where about 75 parameters are mapped twice (`1271-1608` and `1798-1919`). Use
  one `{param, setter}` table. Move the global log-level setup into `main`.
- [ ] `BusAccessUnit` has about 93 public methods; `ApplicationLayer` is 1.8k lines.
  Move the roughly 100-line property-write lambda in `bau.cpp:454-536` into
  named methods.
- [ ] Seven single-implementation `LowerStack*` interfaces
  (`bau_stack_port.cpp:120-204`); `createTp1StackPort` is also used for IP.
  The TP1 RX chain is six layers deep; fold `Tp1MacController` into `Tp1MacPhysical`.
- [x] Duplication:
  - classic vs extended property services;
  - `tunneling_session_client` vs `secure_tunneling_client`;
  - the 6-byte sequence encoder (three copies);
  - `handleNumberedData` (two copies);
  - `netcfg_*` in the MQTT and Wi-Fi services;
  - `build_device_name`, `clampf`, the four `publishPort` overloads;
  - sensor scaling done three ways (Modbus, BLE, MQTT).
- [ ] Too many error-result types (`Result`, `PersistenceResult`,
  `PropertyAccessResult`, `MemoryAccessResult`, `bool`), and many results
  discarded with `(void)`.
- [x] `FixedVector` never destroys elements and has no tests; `back()` on an
  empty vector indexes -1.
- [x] `CMakeLists.txt` is 2.4k lines, about 2,000 of them hand-written test
  targets. Add a `knx_add_test()` helper. Replace `CMAKE_SOURCE_DIR` with
  `CMAKE_CURRENT_SOURCE_DIR` (8 places); fix the undefined
  `${PYTHON_EXECUTABLE}` (`:640`); `BUILD_TESTS=OFF` still builds all tests.
- [x] Three conflicting version numbers: 0.3.0-alpha in `idf_component.yml`,
  0.2.0-alpha in `CMakeLists.txt`/`knx.hpp`, and `project(VERSION 1.0.0)`.
- [x] Add `-Wundef`, since the `#if` matrix covers 38 files.
- [x] `sdkconfig.defaults.*` overlays repeat the flash and partition settings;
  `.prod` is missing from `tools/build_variants.sh`.
- [x] `using namespace` in headers (`main/include/knx_product.hpp:59-61, 134`,
  `control_core.hpp:47`). Fix the stale comments listed in the review (e.g.
  `control_service.cpp:11, 148-150`, `control_state.h:28-34`). Comments that
  read as change history belong in commit messages.

---

## 5. Tests and CI

- [x] **T1** KNstaX CI was removed in `e7eeff1`, so none of its roughly 150 CTest
  targets, the interop pytest suite or `scripts/ci/knstax_size_budget.py` run
  automatically. The README (`:83, :231, :294, :317`) and
  `DEVELOPMENT_GUIDELINES.md:406` still claim CI and sanitiser runs.
- [x] Habinari CI (`.github/workflows/host-tests.yml`) runs only `main/test`;
  add `idf.py build` for each variant.
- [x] The standalone build can't produce the shipping knxip configuration
  (TP1 off, NETIP + SECURE on, tunnelling off): `CMakeLists.txt:350-352` forces
  secure tunnelling on.
- [x] Seven unregistered test files (about 3k lines): `test_linux_platform.cpp`,
  `test_linux_uart_spi.cpp`, `test_cemi_golden.cpp`, `test_ets_integration.cpp`,
  `test_linux_uart_spi_real.cpp`, `test/integration/test_transport_connection.cpp`
  (stale duplicate), `test_esp32_hardware_validation.cpp`. Also
  `tools/knxprod_exporter/test_exporter_medium.py`.
- [ ] Flaky virtual-time tests: real threads with wall-clock timeouts, re-run
  `until-fail:3` (`CMakeLists.txt:2181-2187`, `virtual_platform.cpp:209-244`).
  Move to a single-threaded scheduler.
- [x] Untested in `main`: `control_service` (including `control_state_write`
  validation), the Modbus apply/refresh logic, `sensor_fusion_service`, the
  KNX parameter mapping, and the `TrendMonitor` long-uptime case.
