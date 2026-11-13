# Fix design for the October 2026 code review

Companion to `docs/code-review-2026-10-23.md`. That file lists the findings;
this one says how each is to be fixed, what is deliberately deferred, and how
the fixes are tested. KNstaX paths are relative to `components/KNstaX/`.

## 0. Verification outcome (26 October 2026)

Every finding was re-checked against the code before designing fixes.
Changes to the review:

- **P7 is withdrawn.** `NvsPersistence::init` erases only on
  `ESP_ERR_NVS_NO_FREE_PAGES` / `ESP_ERR_NVS_NEW_VERSION_FOUND`, the standard
  ESP-IDF idiom that `main/` already runs first. Kept as a style item only: the
  library should not own NVS initialisation (see A9).
- **P10, KNX part, is withdrawn.** Casting a `uint8_t` to an
  `enum class : uint8_t` is well defined. A related real bug replaces it: the
  DPT 9 "invalid" value 0x7FFF decodes to 670 760 and is accepted as a CO₂ setpoint.
- **P9 is worse than stated.** Simulated with float32 rounding: slope error
  ~13 % at 6 h, ~43 % at 12 h, ~100 % (rate reads 0) after a day.
- **P8, double commit, happens on every commit**, not only in a race.
- New findings, added to the work below:
  - TP1: an own group telegram that loses arbitration is dropped and reported as success.
  - The KNXnet/IP parameter object (backbone key) is not persisted.
  - Transport: a connection timeout drops the connection without T_Disconnect;
    the T_NAK carries the expected instead of the received sequence number.
  - `PID_DEVICE_CONTROL = 13` in `constants.hpp` (the Device Object uses 14).
  - MQTT discovery inserts the device name without JSON escaping.
  - A 32-character SSID is truncated to 31 and can never be joined.

The ETS6 and Home Assistant tests passing is consistent with all of this:
nearly every finding needs frame loss, IP Secure, bad input, missing
hardware, long uptime or a race.

## 1. Principles

- **Do not break what works.** The ETS download and group-traffic paths are
  proven on hardware. Every change on those paths gets a host test that runs
  the exact ETS sequence it could affect (individual-address write in
  programming mode, connect, property/memory download, restart).
- **One test per bug.** Each fix lands with a host test that fails without it.
- **Spec first.** Where a fix depends on the standard, the code comment cites
  the clause (e.g. `03_08_09 §2.2.2.3`), and the clause was read, not recalled.
- **Prefer removing to fixing** for code nothing uses.
- Hardware-only behaviour (TP1 timing, IP Secure against a real router) can
  only be partly covered by host tests. Section 7 lists what needs a bench run.

## 2. Work packages

The packages are split by file ownership so they can be worked on in parallel.
Phase 1 is the fixes. Phase 2 removes dead code and refactors, after phase 1,
because it touches the same files.

### A — KNXnet/IP Secure Routing (P1, P2 and related)

Owns: `src/netip/**`, `src/physical/ip_routing_physical.cpp`,
`include/knx/physical/ip_routing_physical.hpp`, `include/knx/netip/**`,
`src/security/{aes*,x25519,secure_application_layer,data_secure}.cpp`, and the
secure-routing wiring in `commissioned_product.hpp` / `endpoint_runtime.hpp`.

- **A1 Multicast timer (P1, P2).** A new pure-logic `McTimerSync` class
  implements 03_08_09 §2.2.2.2–§2.2.2.3. It holds:
  - the 48-bit `mc_timer`, kept as an offset over a millisecond clock so it never decreases;
  - events E01–E11 and actions A0–A9;
  - the time-keeper and time-follower delay windows;
  - latency tolerance from `PID_MULTICAST_LATENCY_TOLERANCE` (default 2000 ms)
    and the sync fraction (default 10 %).

  `SecureRoutingSecurity` builds its sequence information from `mc_timer`
  rather than a counter. The message tag is random per frame, and two frames
  in the same millisecond get different tags. Serial = the device's KNX serial
  number, passed in by the product runtime (it is no longer zero).
  `unprotect` checks the MAC first, then classifies the timer (E05–E08) and
  discards outdated frames (E08).
- **A2 TIMER_NOTIFY (0955h).**
  - Encode and decode the frame (MAC over an empty payload, 03_08_09 §2.2.2.4).
  - Handle it in `RoutingEndpoint`.
  - Send it when `notify_timer` expires.
  - On a backbone-key change, reset `mc_timer` to 0 (E11 → A7).
  - The initial notify is sent at random(0, 10 s) after start.
- **A3 Persistence of `mc_timer`** (§2.2.4.2). A callback interface is
  implemented by the product runtime on top of the existing persistence:
  - persist interval D = 1 h of `mc_timer` time;
  - on load, add D to the stored value;
  - persist before sending, or on receiving, a value above T + D.
- **A4 Persist the KNXnet/IP parameter object**, so the backbone key survives
  a reboot. Coordinated with C2, which owns `interface_object_manager.cpp`.
- **A5 Secured group hygiene.** When secure routing is on, accept
  ROUTING_BUSY, ROUTING_LOST_MESSAGE and ROUTING_SYSTEM_BROADCAST only when
  they arrive wrapped. Cap `waitTimeMs` at 100 ms per BUSY frame and the
  accumulated wait at 1 s; the spec's typical BUSY wait is 20–100 ms.
- **A6 Random BUSY back-off.** Wire `setRandomSource` through to the
  `noteBusyReceived` call and give it a default source (esp_random / the
  platform RNG) so it is never dead.
- **A7 Crypto hygiene.**
  - Constant-time MAC compares.
  - Zero the plaintext buffer when the MAC check fails.
  - Check the length before narrowing to `uint16_t` in `SecureWrapper::wrap`.
  - Bound `retiredRoutingSecurity_` to one previous context, which covers
    frames in flight during a rekey.
  - Reject an all-zero X25519 shared secret.
  - Zero key buffers in destructors (helper `secureZero`).
- **A8 Performance.** Keep one AES context per security context instead of
  re-keying 2–3 times per packet. Replace the `std::deque<std::vector>`
  datagram queue with a fixed ring of fixed buffers.
- **A9 NVS ownership.** `NvsPersistence::init` stops erasing. If
  `nvs_flash_init` fails it returns an error and leaves the decision to the
  application.
- **A10 Client-side tunnelling** (secure session MAC, CONNECT_RESPONSE,
  sequence handling) is **not fixed**. The client is not used by any Habinari
  image; E3 takes it out of the build unless `KNX_ENABLE_TUNNELING` is on.
  Fixing it waits for someone who needs the client role.

### B — TP1 physical and data link (P4, P5 and related)

Owns: `src/physical/bitbang_*`, `include/knx/physical/bitbang_*`,
`src/physical/tp1_mac_*`, `src/datalink/tp1_*`, `include/knx/datalink/tp1_*`,
`src/physical/tpuart_*`.

- **B1 Group frames are acknowledged (P4).** The sender waits for the
  acknowledge window after every frame except the individual broadcast
  0xFFFF:
  - ACK means success.
  - NAK, BUSY or no answer is repeated by the existing data-link logic, with
    the repeat flag cleared.
  - For a group frame, no answer counts as success after the repeats, because
    a group nobody listens to is legitimate; NAK still means failure.
- **B2 Arbitration loss is its own outcome.** The ISR reports
  `ArbitrationLost`, not BUSY. The data link re-sends after the bus is idle
  (the ISR already waits 50 bit times), without using up the busy or NAK
  budget. A maximum of 10 tries keeps a jammed bus from looping forever.
  This also fixes the "group frame lost and reported OK" finding.
- **B3 BUSY wait.** After BUSY, wait at least 150 bit times (16 ms at
  9600 bit/s) before repeating (03_02_02 §2.4.2 / A.8).
- **B4 Check octet and length in the ISR (P5).** Keep a running XOR in
  `rxHandleStop()`. In `rxHandleTelegramComplete()`, before the armed ACK goes
  out, downgrade to NAK if the XOR is not 0xFF or the length doesn't match the
  length field (8–23 standard, 9–263 extended).
- **B5 Never ACK what will be dropped.**
  - Size the ISR acknowledge table to the data-link limit of 256 (sorted
    array, binary search in ISR context).
  - When the RX queue is full, the ISR answers BUSY instead of ACK, and the
    frame isn't queued.
  - Drop-oldest is removed.
- **B6 Bus-monitor mode** disables ACK generation in the ISR core.
- **B7 `close()` order.** Clear the ISR notify callback and wait for the ISR
  to see it, then delete the RX task.
- **B8 RX copies and events.** Stop queuing a per-byte `DataByte` event that
  nothing consumes, and shrink the event queue to 32 entries. Remove one frame
  copy by handing the frame buffer by reference to the MAC.
- **B9 TPUART.** It is not fixed here: it can't be tested without the chip.
  It is marked broken in Kconfig and in the source, and the default changes to
  bit-bang (Kconfig edit by E1). The backlog keeps "rewrite against the
  NCN5120 datasheet".

### C1 — Application layer and management (P3, P6 and related)

Owns: `src/application/**` (except `dpt.*`), `src/bau/bau_management.cpp`,
`src/bau/bau.cpp` (management paths), `include/knx/application/**`.

- **C1.1 Communication-mode check (P3).** A `constexpr` table maps each APCI
  to the modes it is allowed in, taken from 03_03_07 §3 (the service mapping
  tables):
  - connection-oriented point-to-point;
  - connectionless point-to-point;
  - broadcast;
  - system broadcast;
  - multicast (group).

  `dispatchService` checks the table first and silently discards a request in
  the wrong mode. **Guard rail:** a host test replays the full ETS
  commissioning sequence (IndividualAddress_Write in programming mode by
  broadcast, IndividualAddress_Read, DeviceDescriptor_Read, connect,
  Authorize, Memory/Property download, Restart) and must still pass.
- **C1.2 Master reset (P6).**
  - Ignore the request if reserved bits 1–4 of octet 7 aren't zero.
  - Channel ≠ 0 → "Invalid Channel Number".
  - Erase codes are implemented through a BAU hook into the object persistence:
    - 1: confirmed restart (existing behaviour);
    - 2: factory reset (erase all KNX persisted objects including Tool Key and
      backbone key, then restart);
    - 3: reset individual address to the default;
    - 4: unload the application program;
    - 5: reset parameter memory;
    - 6: reset address and association tables;
    - 7: factory reset keeping the individual address.
  - Anything else → "Unsupported Erase Code".
- **C1.3 Memory services** (03_03_07 §3.5.3/§3.5.4).
  - A failed read answers A_Memory_Response with number = 0.
  - Writes answer only when Verify Mode (`PID_DEVICE_CONTROL` bit 2) is set:
    with the read-back on success, number = 0 on failure.
  - `encodeResponse` accepts count 0.
- **C1.4 Device descriptor.** An unsupported type answers with type 3Fh and no data.
- **C1.5 A_Authorize.** Levels are enforced only where the stack has a
  defined required level. Memory write needs the configured write level, and
  property write needs the level in the property's descriptor. With no keys
  set (the default), every key yields level 0, so nothing changes for a device
  ETS hasn't keyed. On a Data Secure device the Tool Key already gates
  management, so this only matters for plain devices with A_Key_Write.
- **C1.6** Fix `PID_DEVICE_CONTROL` to 14 in `constants.hpp`.

### C2 — Transport, network, objects, utilities

Owns: `src/transport/**`, `src/network/**`, `src/objects/**`,
`src/util/**`, `include/knx/util/**`, `include/knx/objects/**`,
`src/bau/bau_group_runtime.cpp`, `src/platform/esp32_platform.cpp`.

- **C2.1 Transport** (03_03_04 state machine, Style 1 Rationalised).
  - Fixed 3 s acknowledge timeout.
  - Retries exhausted (E18), connection timeout (E16) and a wrong-sequence
    T_ACK in OPEN_WAIT (E09) each send T_Disconnect and close (A6).
  - T_NAK carries the received sequence number (A4).
  - A wrong-sequence T_NAK is ignored (A0, a defensible choice), with a
    comment saying so.
- **C2.2 More than 16 group objects per group address.** Iterate the
  association table directly instead of copying into a 16-entry buffer, so
  any number of group objects is delivered.
- **C2.3 Atomic download.** Persist through a generation marker:
  - write every changed property under the next generation's keys;
  - then write the marker last;
  - on load, use the generation the marker names and erase the other one.

  One NVS commit makes the switch atomic.
- **C2.4 Persist the KNXnet/IP parameter object** (for A4), and **the
  `mc_timer` record** if A uses the object store.
- **C2.5 Filter table.** Replace it with a 0x0000–0x6FFF bitmap
  (3.5 KB, allocated only when a coupler exists) and apply the coupler ack
  policy. Low priority: Habinari is not a coupler.
- **C2.6 `InplaceFunction`.** Copying a move-only callable is a compile-time
  error (`static_assert` in the copy constructor and copy assignment).
- **C2.7** `Esp32Platform::fatalError()` logs, then calls `abort()`.
- **C2.8 `FixedVector`.** Use raw aligned storage; `clear`, `pop_back` and
  `erase` destroy the elements they remove, and `back()` / `front()` assert
  the vector isn't empty. Add tests.
- **C2.9 Lookup speed.** Binary search on the (sorted) address table, plus a
  GA → group-object index rebuilt after each download.

### D1 — `main/`: control, Modbus, sensor fusion

Owns: `main/src/control_service.cpp`, `main/src/mb_rtu_slave.c`,
`main/include/sensor_fusion.hpp`, `main/src/sensor_fusion_service.cpp`,
`main/include/hvac_control.hpp`, `main/include/control_*.h*`, `main/test/**`.

- **D1.1 Modbus (P8).**
  - `mb_apply_writes` records the snapshot it acted on as `applied_*`.
  - `mb_refresh_writables` no longer overwrites `applied_*`.
  - `config_commit` is consumed (cleared in the register map) under the lock
    in the same critical section that reads it.
  - The handle is protected by a mutex (or the publish path checks a
    `stack_running` flag under the same lock), so `mb_rtu_slave_publish`
    never uses a deleted handle.
- **D1.2 TrendMonitor (P9).** Regress on `t − t_oldest` with times as
  `double` relative to the oldest sample, and keep `elapsed_` as a
  `uint32_t` millisecond counter. A host test feeds a 1 K/min ramp starting
  at 30 days of uptime and checks the slope within 1 %.
- **D1.3 Command validation (P10).** `control_state_write` is the single gate:
  - `isfinite` on every float;
  - range checks on setpoints, shift and CO₂ setpoint;
  - enum range checks before any cast;
  - returns `ESP_ERR_INVALID_ARG`.

  `clampf` returns the low bound for NaN as a last line of defence. Callers
  (MQTT, Modbus, KNX, BLE) check the result. KNX treats DPT 9 0x7FFF as
  "invalid" and ignores it. Host tests are added for `control_state_write`.
- **D1.4 Fusion races.** `sensor_fusion_update` holds the fusion mutex for
  the whole update. The mutex is created in `sensor_fusion_init`, not lazily.
  `acknowledge_alarms` takes the lock.

### D2 — `main/`: services and start-up

Owns: `main/src/main.c`, `main/src/sensor_bus.c`, `main/src/knx_service.cpp`,
`main/src/mqtt_service.c`, `main/src/mqtt_payload.c`, `main/src/wifi_service.c`,
`main/include/knx_product.hpp`, `main/Kconfig*`.

- **D2.1 Sensors are degradable.**
  - `sensor_bus_enable` returns OK with a per-sensor present mask.
  - A missing SCD4x or BME68x is logged and published as unavailable.
  - `i2c_bus_init` and `bme68x_init` results are checked, and the error paths
    release their handles.
  - `main.c` does not `ESP_ERROR_CHECK` it.
- **D2.2 KNX start is reported.** `knxAdapterStart` waits (5 s timeout) on a
  task notification carrying the `startProduct` result, and returns its error
  so the required-adapter logic refuses to boot and says why.
- **D2.3 Races.**
  - Wi-Fi callback and context are set as one pair under a spinlock.
  - `controlTickPending`, `ipIdentityDirty` and `publish_forced` become
    `std::atomic` / `_Atomic` with exchange-to-clear.
  - `taskHandle` is written before the task can be notified.
  - The KNX Wi-Fi lambda null-checks.
- **D2.4 MQTT.**
  - Discovery JSON is built with a bounded appender that stops and reports
    truncation; the topic is skipped with an error log instead of
    overflowing.
  - The device name is JSON-escaped.
  - Publish results are checked, and `last_payload` is updated only on success.
  - `parse_number` rejects non-finite values (with D1.3 as the real gate).
- **D2.5 Optional adapters don't abort.** `ESP_ERROR_CHECK` in the Wi-Fi and
  MQTT adapters becomes error returns.
- **D2.6 SSID.** Copy up to 32 bytes without requiring a terminator, as
  `wifi_sta_config_t` allows.
- **D2.7 KNX service.**
  - `takePublishSnapshot()` runs only when a control tick is due.
  - The 81 parameters are mapped once, in a `{param, setter}` table used by both
    `onParameterChanged` and the start-up `params.get` pass.
  - The constexpr-dead branches are removed.
  - The global log-level setup moves to `main`.
- **D2.8** `HABINARI_REQUIRES_AUX_POWER` is logged at boot as its help text
  promises. `using namespace` comes out of headers.

### E — Build, Kconfig, CI, documentation

Owns: `Kconfig` (KNstaX and top level), `CMakeLists*.txt`, `cmake/**`,
`.github/**`, `tools/build_variants.sh`, `sdkconfig.*`, README files,
`DEVELOPMENT_GUIDELINES.md`, `idf_component.yml`,
`include/knx/platform/freertos_platform.hpp`.

- **E1 Kconfig.**
  - Drop `depends on KNX_DEBUG_ENABLED` from `KNX_LOG_LEVEL`, default 3 on
    `ESP_PLATFORM` (P11).
  - Make bit-bang the TP1 default and mark TPUART "(broken, do not use)" (B9).
  - Remove the unread options.
  - Remove `KNX_MEDIUM_RF` (no implementation).
  - Add `KNX_VIRTUAL_BUS` (host builds only).
- **E2 Ticks.** `pdMS_TO_TICKS` rounds up and never turns a non-zero timeout
  into a poll (one helper used by every call site in `freertos_platform.hpp`).
- **E3 Client netip sources** move to `KNX_SOURCES_NETIP_CLIENT`, built only
  with `KNX_ENABLE_TUNNELING`. `mock_tp1_physical.hpp` and the mock
  constructor are guarded by `KNX_ENABLE_TEST_HOOKS`, which is off on ESP.
  `timer_gpio_hal_virtual.cpp` moves to `KNX_SOURCES_VIRTUAL`.
- **E4 CI.**
  - Restore a KNstaX workflow: host build plus CTest, with the ASan/UBSan job
    the docs promise, and the interop pytest suite.
  - The Habinari workflow gains a firmware job: `idf.py build` for every variant in
    `tools/build_variants.sh`, in `espressif/idf:release-v5.5`.
  - The standalone build gains a knxip-shaped configuration (TP1 off, NETIP
    and SECURE on, tunnelling off) instead of forcing secure tunnelling on.
- **E5 Housekeeping.**
  - One version number (0.3.0-alpha) generated from `idf_component.yml`.
  - `-Wundef`.
  - `CMAKE_CURRENT_SOURCE_DIR` in place of `CMAKE_SOURCE_DIR`.
  - `Python3_EXECUTABLE` in place of `${PYTHON_EXECUTABLE}`.
  - `BUILD_TESTS=OFF` really skips tests.
  - `.prod` added to `build_variants.sh`.
  - `sdkconfig.ci` either filled or deleted.
  - The overlays stop repeating the flash and partition settings.
- **E6 Docs.** Correct the README and guideline claims about CI to match E4.

### F — Phase 2: dead code and refactors (after phase 1)

- **F1 Remove** (each removal is checked by a full build of every variant and the test suite):
  - Files and build scripts: `CMakeLists.standalone.txt`, `enum_string.hpp`, `allocation_tracker.hpp`.
  - TP1 codecs: `Tp1FrameCodec`, and `ManchesterCodec`'s encode/decode (its
    timing constants stay).
  - Unused stack paths: the NetworkLayer forwarding path and the unused
    `RoutingTable` accessor (the coupler policy stays), and the async transport
    receive path.
  - Security: `DataSecureSession` crypto, and the unused `KeyDerivation` functions.
  - Test-only codecs: the static service codecs used only by tests (tests move
    to the production parsers).
  - SPI and memory: `Esp32Memory` and the SPI abstraction (drop `esp_driver_spi` from REQUIRES).
  - Dead members: unused `ApplicationLayer` members and the receive queue,
    `BusAccessUnit::beginRequestGroupObjectValue`, and
    `GroupObjectTableObject::findByAddress`/`hasAddress`.
  - Unused `config.hpp` constants.
  - Everything in `main/` the review lists, plus `MB_REG_OFFSET`, unused
    `board.h` pins and the empty `sdkconfig.ci`.
- **F2 `PropertyStore`.** Access decisions move to the descriptors in
  `InterfaceObjectManager` (the real model), then `PropertyStore` and
  `PropertyStoreManager` are deleted.
- **F3 Tests.**
  - Register or delete the seven orphaned test files.
  - Add `knx_add_test()` and convert the hand-written targets.
  - Move the flaky virtual-time tests to a single-threaded scheduler (or keep
    `until-fail` and label them, if the scheduler rewrite is too large).
- **F4 Small duplications:**
  - the 6-byte sequence encoder;
  - `clampf`, `build_device_name`, `netcfg_*`;
  - the four `publishPort` overloads;
  - sensor scaling (one table shared by Modbus, BLE and MQTT);
  - the property-write lambda in `bau.cpp` → named methods;
  - the `std::string` / `std::map` in `log.cpp` `resolveEffectiveLevel` →
    cached per tag.
- **F5 Comments.** Fix the stale comments the review lists; remove comments
  that only record change history.

## 3. Deferred, with reasons

| Item | Why not now |
|---|---|
| Split `BusAccessUnit` (93 methods) and `ApplicationLayer` (1.8k lines) | Large structural change on proven paths with no behaviour gain. Revisit when a feature needs it. |
| Collapse the seven `LowerStack*` interfaces and fold `Tp1MacController` into `Tp1MacPhysical` | Same reason. B8 already removes the copies that cost RAM. |
| One error-result type | Touches every API. Do it with the next API break, not in a bug-fix round. |
| Move template-independent code out of `commissioned_product.hpp` | Code-size only. The knxip image fits again once P11 is fixed. |
| 32 KB KNX task stack | Deliberate and measured (see the comment). Re-measure after F2. |
| `InterfaceObjectManager` nested `std::map`, mixed `std::function` and `InplaceFunction` | Not on a hot path. |
| TPUART rewrite | Needs the chip to test (B9 disables it instead). |
| Client-side tunnelling fixes | Not built into any image after E3 (A10). |
| Linux UART/SPI "real" tests, ESP32 hardware-validation test | Need hardware; registered under a `hardware` label that CI skips. |

## 4. Testing strategy

- **Host, KNstaX:** the full CTest suite (152 tests at baseline) plus the new tests:
  - A: `McTimerSync` state machine (every event in both states), replay and
    outdated discard, nonce uniqueness across a simulated reboot,
    TIMER_NOTIFY round trip, and ROUTING_BUSY ignored on a secured group.
  - B: virtual-bus tests for group repeat on NAK, arbitration loss retried, a
    bad checksum NAK'd, the 65th group address ACK'd, and BUSY when the RX
    queue is full.
  - C1: mode table (each management service by group and broadcast is
    ignored), the full ETS commissioning sequence, each master-reset erase
    code, and memory responses with Verify Mode on and off.
  - C2: transport disconnect cases, 17 group objects on one GA, a download
    interrupted between properties and before the marker, and `FixedVector`.
- **Host, `main/`:** the existing 9 tests, plus:
  - `control_state_write` validation;
  - Modbus apply/refresh with a write landing between snapshot and refresh;
  - commit processed once;
  - TrendMonitor at 30 days of uptime.
- **Firmware:** `idf.py build` of knx, knxip, modbus, modbus-ble and mqtt,
  with image sizes compared against the baseline (knxip must still fit its
  partition).
- **CI** runs all of the above (E4).

## 5. Order

1. Phase 1, in parallel: A, B, C1, C2, D1, D2, E.
2. Integration: full host suite, all firmware builds, and a review of the combined diff.
3. Phase 2 (F), then integration again.
4. Bench tests (section 7) by the device owner.

## 6. Risks

- **C1.1 could break commissioning** if the mode table is wrong. Mitigation:
  the ETS-sequence host test, plus checking the table against 03_03_07 and the
  services ETS actually sends (IndividualAddress_* and
  IndividualAddressSerialNumber_* are broadcast; DomainAddress* uses system
  broadcast on IP and RF).
- **B1/B2 change TP1 timing.** The virtual bus models the ISR, not the
  electrical bus. A bench check on a real TP1 line is required before release.
- **A1 changes what the device sends under IP Secure.** It has to be tried
  against ETS6 with Secure Routing enabled, and against a real IP Secure router
  or xknx.
- **C2.3 changes the NVS layout.** The loader must also read the current
  (unmarked) layout once, so devices in the field keep their download.

## 7. Bench checks for the device owner

1. TP1: an ETS6 download, group traffic from HA via an IP interface, and
   unplugging the only receiver of a group address while sending (expect
   three repeats, then no error).
2. knxip, plain: an ETS6 download and HA group traffic (regression check).
3. knxip, IP Secure: enable Secure Routing in ETS6, download, check that HA
   (xknx with the backbone key) sees the device's telegrams. Reboot and check again.
4. Factory reset from ETS ("Reset to factory defaults"): the device forgets
   its individual address and application.
5. Boot with the SCD4x unplugged: the device runs and reports CO₂ as unavailable.

## 8. Status (1 November 2026)

Phase 1 (A, B, C1, C2, D1, D2, E) is implemented in the working tree of both
repositories. It is not committed. The checklist in
`docs/code-review-2026-10-23.md` is ticked to match.

**Verified.** KNstaX host suite 155/155 with no build warnings, `main/` host
suite 12/12, and every firmware variant builds with no warnings and fits its
partition:

| Variant | Before | After phase 1 |
|---|---|---|
| knx | 989,232 B | 1,001,664 B |
| knxip | 1,687,312 B | 1,687,616 B |
| modbus | 347,216 B | 349,360 B |
| modbus-ble | — | 797,472 B |
| mqtt | 1,661,232 B | 1,666,432 B |

The CI workflows were only parsed locally; their first run on GitHub is the
real test (the firmware job uses `espressif/idf:v6.1`, not the
`release-v5.5` image named in E4).

**Follow-up review of the combined diff** found and fixed:

- The Modbus task refreshed the registers from a state snapshot taken before
  it applied the master's writes, so a master reading back straight after a
  write saw its old value for one 200 ms cycle. The state is now read again
  after the writes are applied.
- An access level granted by A_Authorize outlived its connection: nothing
  ever cleared it. 03/03/07 §3.5.7: "A current access level shall be valid
  until the connection is released". The connection table now reports every
  closed connection (T_Disconnect either way, timeout, repetitions used up),
  and the application layer clears that partner's level. Tests:
  `test_AccessLevelEndsWithTheConnection`,
  `test_ClosedCallbackSeesTimeoutAndPartnerDisconnect`.
- `RoutingEndpoint::flowControl_` was read and written from two tasks (the
  stack sends, the data-link receive task records ROUTING_BUSY) without a
  lock. It has its own mutex now.
- The eleven "unused variable" warnings in knxip came from log calls compiled
  out at level 3. A compiled-out level now expands to an `if (false)` call, so
  its arguments stay referenced and its format string checked; the optimiser
  drops it.
- Housekeeping: `knx.hpp` reported 0.2.0, the dead `CONFIG_KNX_MEDIUM_RF`
  branch in `config.hpp`, an unused `mock_tp1_physical.hpp` include in
  `bau_group_runtime.cpp`, and the empty `sdkconfig.ci` (deleted).

**Checked and left as they are:**

- The latency-tolerance poll runs on the receive task, but the property copy
  it makes is taken under the object's value mutex.
- No plain multicast leaves the device at boot before a stored backbone key is
  applied: the key is applied on the KNX task straight after `start()`, before
  that task's first `loop()`, which is where all sending happens.
- E11 for a changed routing multicast address cannot occur: the configured
  group is written into the KNXnet/IP Parameter Object at start, so a
  non-default multicast address from ETS is not supported at all. That is a
  missing feature, recorded here, not a timer bug.
- Verify Mode is a device property (`PID_DEVICE_CONTROL`), not connection
  state, so it is not reset on T_Disconnect.

**Still open:**

| Item | Why |
|---|---|
| C2.5 filter table | Habinari is not a coupler. |
| E3, client sources in knxip | Measured at about 12 KB (`tunneling_session_client` 8.4 KB, `ip_tunneling_physical` 3.4 KB). Removing them means guarding five files; knxip has 12 % headroom. |
| A_Key_Write | Not implemented. Access levels only bite once firmware calls `setKeys()`, which Habinari does not. |
| Tests for `sensor_fusion_service` | Not added in phase 1. |
| Phase 2 (F) | Not started. |

**New risk for the bench (section 6):** C2.1 closes a connection when a T_ACK
for a frame other than the outstanding one arrives without the repeat flag
(E09, as the standard requires). On a high-latency IP path, a late T_ACK for
a frame the device already repeated could now end an ETS session, where before
it was ignored. Watch for "T_ACK seq=… while waiting for seq=… - disconnecting"
in the log during downloads over KNXnet/IP.

### Phase 2 progress

**F1 done** (uncommitted): about 7,500 lines of dead code removed from KNstaX
and `main/`, plus the Kconfig options that only fed the removed constants
(and their lines in `sdkconfig.defaults*`). KNstaX host tests 147/147 (the 8
deleted targets only covered removed code), `main/` 12/12, no warnings. knx
976,464 B (−25 KB), knxip 1,667,456 B (−20 KB). `mb_rtu_slave_stop` went
with it, so nothing stops the Modbus task, as before.

**F2 found a real inconsistency** before anything was moved. Comparing the
shadow `PropertyStore` with `InterfaceObjectManager::describeProperty` over
every object and PID:

- The extended property services (`A_PropertyExt*`) run on the store, which
  orders properties by PID; the classic services use the live model's order.
  For the Device, Application Program, Security and KNXnet/IP Parameter
  objects, the two service families return different PIDs for the same
  property index.
- Group Object Table PID 9 (TableData) is read-only with the full element
  count in the manifest (and so in the store and the extended services), but
  writeable with one element in the live model (the classic services).

Deleting the store outright would change what ETS sees through the extended
services, so F2 is done in two steps: the classic services and the Data
Secure policy move off the store now, with no change on the wire; the
extended services stay on it until the index order and the TableData access
are decided against 03_03_07 / 03_05_01 and checked with ETS6 on the bench.

**F2 step one done:** the classic property services and the Data Secure
access check no longer read `PropertyStore`; only the extended services do
(see above). No change on the wire. knx 974,800 B, knxip 1,666,192 B.

**F3 done:** six orphaned test files deleted (stale copies, gtest leftovers,
code that no longer exists, one ESP32-only hardware app), `test_linux_platform`
rewritten for Unity and registered, `test_exporter_medium.py` registered.
Test targets go through `knx_add_test()` (cmake/knx_testing.cmake);
`CMakeLists.txt` 2,345 → 992 lines with every test's command, labels,
timeouts and compile/link lines identical before and after. Four integration
tests that never carried the `integration` label now do. The three
virtual-time tests keep `until-fail:3` and the `sdk-flake-candidate` label:
a deterministic fix needs fibres behind `createTask` or the stack's tasks as
step functions, which is a rewrite. One real race (VTRUN_002) fixed.

**F4/F5 done:** one 48-bit sequence encoder, one `clampf` (now NaN-safe in
the fusion code too), one `device_display_name()`, shared `netcfg_*` hooks,
one templated `publishPort`, named methods for the BAU property write, and a
per-tag cache in `resolveEffectiveLevel` (keyed by tag pointer: tags must be
static strings, as they are). Sensor scaling stays separate on purpose: Modbus
(×10, saturating), BLE (×100, invalid marker) and MQTT (plain floats) are
different wire contracts. Stale and history-only comments fixed. The fusion
logic moved into `habinari::fusion::Pipeline` so it is host-tested
(`test_sensor_fusion_service`, 14 cases).

**After phase 2:** KNstaX host 149/149, `main/` host 13/13, no warnings.

| Variant | Before phase 2 | After phase 2 |
|---|---|---|
| knx | 1,001,664 B | 975,040 B |
| knxip | 1,687,616 B | 1,666,224 B |
| modbus | 349,360 B | 348,992 B |
| modbus-ble | 797,472 B | 797,296 B |
| mqtt | 1,666,432 B | 1,665,808 B |

**Open after phase 2:** the extended property services on `PropertyStore`
(index order, Group Object Table TableData access), the filter table (C2.5),
client-side tunnelling, the three flaky virtual-time tests, the structural
items deferred in section 3, and the bench checks in section 7 plus the E09
risk above.

## 9. Phase 3: the remaining open items (12 November 2026)

Phases 1 and 2 are committed on branch `review-2026-10-fixes` (KNstaX
`2cda699`, firmware `c8e89ba`). The product is still greenfield, so the
items deferred earlier as "large change, no behaviour gain" are now in
scope: the aim is the right long-term design, not the smallest patch.

**Settled without code:** the E09 risk noted in section 8. The transport
layer as built is Style 3 of 03_03_04 §5.4.3 for the T_ACK/T_NAK events
(T_ACK in OPEN_IDLE ignored, wrong-sequence T_NAK ignored, wrong-sequence
T_ACK in OPEN_WAIT closes). Style 3 is a defined style, and a duplicate
T_ACK can only hit OPEN_WAIT if it arrives after the device has sent its
next frame, which ETS's request/response ordering makes unlikely.

Work, one package at a time, each with tests and a full build:

1. **One property model.** Settle the property order by index and the
   Group Object Table TableData access against 03_05_01 / 03_03_07, make
   `InterfaceObjectManager` the single source for every property service
   (classic and extended), and delete `PropertyStore` and the duplicate
   manifest data that disagrees with it.
2. **Remove the TPUART backend.** It is broken (section 2, B9), cannot be
   tested without the chip, and Habinari uses bit-bang. A correct NCN5120
   driver is new work for when a product needs one.
3. **Build the KNXnet/IP client role only with tunnelling enabled** (E3):
   guard the five call sites so knxip links no tunnelling client (about
   12 KB).
4. **Deterministic virtual time.** Replace the thread-per-task model in
   `VirtualPlatform` so the three flaky tests become deterministic and the
   `until-fail` repeat goes.
5. **Split `knxServiceTask`** in `main/src/knx_service.cpp` into units with
   one job each (product construction, parameter binding, group-object
   publication, diagnostics).
6. **Routing multicast address from ETS.** Apply a downloaded
   PID_ROUTING_MULTICAST_ADDRESS, and treat a change as E11 (mc_timer reset)
   as 03_08_09 requires.
7. **A_Key_Write** (03_03_07 §3.5.8), so access levels can be keyed by ETS.
8. **Structural KNstaX work** from section 3: fold the single-implementation
   `LowerStack*` interfaces, one error-result type, split `BusAccessUnit` and
   `ApplicationLayer`, re-measure the 32 KB KNX task stack, and the
   `InterfaceObjectManager` map and callable types. Planned in detail once
   1–7 are in, since they change the same code.
9. **Filter table** (C2.5) and the client-side tunnelling fixes (A10), both
   for the library's coupler and client roles, which no Habinari image uses.

### Phase 3 progress

**3.1 done.** One property model: `InterfaceObjectManager` serves the
classic and extended property services; `PropertyStore`, the core manifest
and its compliance tables are deleted. Property index = rank by PID
(PID_OBJECT_TYPE is index 0); Group Object Table PID_TABLE is PID 23,
PDT_GENERIC_02[], writeable through both families, and a length write (ETS
clearing the table) is accepted and not applied. knx 961,824 B (−13 KB),
knxip 1,656,848 B (−9 KB). **Bench:** property enumeration by index in ETS6
(order changed, index 0 now exists), an ETS6 download (GOT PID 23 now
exists), and a Data Secure download over the extended services.

**3.2 and 3.3 done.** TPUART backend removed (TP1 is bit-bang only; the
Kconfig choice leaves room for an NCN5120 driver). `KNX_IP_TUNNELING`
guards the KNXnet/IP client role, so knxip links no tunnelling client; CI
builds a tunnelling-off host configuration. knx 954,096 B, knxip
1,645,424 B.

**3.4 done.** The unused platform UART abstraction is removed (knx
950,640 B, knxip 1,643,856 B). `VirtualPlatform` runs tasks as fibres on a
single-threaded scheduler driven by virtual time: no wall-clock waits, no
`until-fail` repeat, no flake label (this supersedes the F3 note above).
2,000 runs of each former flake under load: 0 failures, 6–25× faster.
Follow-up: the ASan/UBSan host build reports about 460 `-Wconversion`
warnings in untouched headers, apparently GCC's UBSan instrumentation noise;
the sanitiser CI job does not use `-Werror`, but this should be confirmed
against a baseline and then silenced for that build only.

**3.5 done.** `knx_service.cpp` split into units with one job each (2,215 →
427 lines; the task is 101 lines); parameter table, group-write mapping
and publish schedule host-tested; ETS export byte-identical. Follow-up
fixes: group reads now answer exactly the published value (or DPT 9 7FFFh
with no valid reading), and the unlinked-object report runs on every entry
into Operational, including boot and the programming button.

**3.6 done.** PID_ROUTING_MULTICAST_ADDRESS written by ETS (or by
A_DomainAddressSerialNumber_Write) is validated (224.0.0.0/4), persisted
and becomes the routing group at the next start, as 03_08_03 §2.5.17
requires ("shall become active after a reset"); a different value is E11
(sync restarted), the same value E12. Factory reset 02h restores the
product default. This supersedes the section 8 note that E11 cannot occur.
Fixed on the way: the KNXnet/IP Parameter and Router Object seeds and the
backbone-key write observer were installed before the objects existed, so
a run-time key download only took effect after a reboot.

**3.7 done.** A_Key_Write (03_03_07 §3.5.8): one key per access level 0–14,
FFFFFFFFh = no key, persisted, reset by Master Reset 02h/07h for the
requester's level and below. A device with no keys keeps every partner at
level 0. This supersedes the section 8 note that A_Key_Write is missing.

**Bench (3.6/3.7):** ETS6 multicast-address change plus restart, seen by
xknx on the new group; backbone-key download taking effect without a
reboot; ETS reads of the KNXnet/IP Parameter Object; BCU key write and
re-authorisation; an unkeyed device staying at level 0 throughout a
download.
