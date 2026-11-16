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

### 3.8 plan (structural KNstaX work)

From a read-only survey at KNstaX `987a00c`. None of it may change what ETS6
or Home Assistant see: a test whose expected bytes change is a regression.

Latent bugs found: `InterfaceObjectManager`, `BusAccessUnit` and
`CommissionedProductRuntime` are movable although they hand out `this` and
self-pointers (nothing moves them today; delete the moves); the `knx_tp1_rx`
task stack (8 KB, top of its Kconfig range, runs Data Secure decryption and
management dispatch) has never been measured.

| # | Package |
|---|---|
| 3.8a | One concrete stack port instead of `BusAccessStackPort` + seven single-implementation `LowerStack*` interfaces; `TransmissionProgressState` = `util::OperationProgressState`; dead stubs |
| 3.8b | Fold `Tp1MacController` into `Tp1MacPhysical`; delete the unimplemented `Tp1PhysicalLayer` and update ARCHITECTURE/guidelines |
| 3.8c | `InterfaceObjectManager`: drop the test-only registered-instance path, `FixedVector` of kernel objects, delete the moves (and the BAU's) |
| 3.8d | One error type: `util::Result<T>` everywhere internally (`PersistenceResult`, `PropertyAccessResult`, `MemoryAccessResult` go); wire return codes stay value types with one mapping; audit discarded results on persistence/send paths |
| 3.8e/f | Split `BusAccessUnit` into `DeviceModel`, `GroupRuntime` and `DeviceManagement` behind the existing facets |
| 3.8g | Split `ApplicationLayer` into receive/dispatch, `ManagementServer` and `ASendPipeline`, with an explicit request context |
| 3.8h | Stack high-water logging for `knx_service` and `knx_tp1_rx`, a static stack report (`-fstack-usage`), and a bench procedure with a sizing rule |
| 3.8i | Delete the `CommissionedProductRuntime` move constructor; optionally move template-independent product code to a .cpp |

### 3.8h Stack sizing for the KNX tasks (13 November 2026)

**The tasks.** `knx_service` (32 KB, `kKnxServiceTaskStackSize`) runs
start-up, the stack's `loop()`, group-write events, publication and
persistence. `knx_tp1_rx` (`CONFIG_KNX_RX_TASK_STACK_SIZE` = 8192, the top of
its Kconfig range) is created by `Tp1DataLinkLayer` on **both** media: the
name is historical, KNXnet/IP routing runs through the same data link layer,
and on knxip that task blocks in `select()` on the routing socket. It runs
Secure Routing unwrap and TIMER_NOTIFY, the Data Secure RX/TX transforms,
management dispatch and the blocking response send. No other task runs KNX
code (lwIP's task only moves datagrams; the TP1 link re-poll is an
`esp_timer` callback that reads a GPIO).

**Runtime instrumentation** (`main/src/knx_stack_watch.cpp`). The service
task logs `uxTaskGetStackHighWaterMark` (bytes) for both tasks, looked up by
name, at the end of start-up, on every lifecycle change (a download or a
reset, signalled from the lifecycle callback and logged on the next service
pass) and every 10 minutes, with the worst value seen since boot. Under
1 KB free it logs a warning. The worst values are also the read-only
service-channel item `knx.stack` ("svc N rx N bytes free", -1 = not seen),
which knxip's BLE channel shows; the TP1 image has no service channel, so
there it is the log only.

**Static report.** A measurement build in a directory of its own adds
`-fstack-usage -fcallgraph-info=su` to every object (the shipped flags are
unchanged):

    idf.py -B build_su -DHABINARI_STACK_USAGE=ON build   # plus the image's SDKCONFIG_DEFAULTS
    tools/stack_report.py build_su knxServiceTask Tp1DataLinkLayer::rxTaskFunc \
        --edges tools/stack_report.edges

It prints the deepest resolved chain, the largest reachable frames and the
indirect calls where the chain stops. `tools/stack_report.edges` stitches in
the receive path's known indirect calls (physical-port function pointers,
the layers' receive callbacks, the Data Secure transforms, the routing
security, property providers). The totals are lower bounds. Results
(GCC 16.1, ESP-IDF 6.2; knx at its default `-Og`, knxip at `-Os`):

| | knx (TP1) | knxip |
|---|---|---|
| `knxServiceTask`, deepest resolved chain | 15,328 B | 18,480 B |
| of which `buildAndStartProduct` frame (the bindings builder) | 10,240 B | 13,696 B |
| `rxTaskFunc`, deepest resolved chain (edges stitched) | 5,408 B / 50 frames | 5,424 B / 52 frames |
| unresolved indirect sites reachable from the RX task | 121 in 75 functions | 128 in 79 functions |

Largest RX-task frames: `SecureApplicationLayer::ccmDecrypt` 720 B and
`ccmEncrypt` 704 B (knxip; Data Secure was not yet compiled into the TP1
image, see below and "Data Secure on the TP1 image"), `PropertyExtServices::handleValueWrite` 656–672 B,
`IpRoutingPhysical::beginTransmit` 624 B, the property and memory read
handlers 592–624 B, `ApplicationLayer::handleTransportRx` 576–624 B. The
deepest RX chain is a management request answered under Data Secure:
receive, `handleTransportRx`, `ManagementServer`, `ASendPipeline`, the
transport TX transform, `ccmEncrypt`, the AES driver, and the IDF lock/
malloc/log path below it. What it cannot see: the libc `vprintf` behind
`esp_log` (prebuilt, no frame data), the restart and Master Reset callbacks
into the BAU, the function-property and memory-region providers, and the
interface objects' kernel handler tables.

**Judgement.** Nothing is clearly undersized, so neither size changes. The
RX task resolves to 5.4 KB of 8 KB; with printf under a log call it is
plausibly near 6.5 KB, which is the bench threshold below, not an overflow.
The service task's static figure (15–18.5 KB) is well under 32 KB, but the
start-up path is mostly behind unresolved calls (the 34 KB overflow recorded
in `knx_service.cpp` came from frames the static graph does not join), so
32 KB stays until the bench gives a peak. knxip's builder frame is 13.7 KB,
more than the ~9 KB `knx_service.cpp` assumed; its comment now carries the
measured frames.

**Bench procedure** (both images unless noted; read the `Stack ...` log
lines, or `knx.stack` on knxip, after each step):

1. Cold boot, uncommissioned.
2. Boot after a full download with Data Secure keys in force.
3. ETS6 full Secure download over TP1, and over IP.
4. Property enumeration over the extended services (ETS6 device info /
   property browser).
5. Master Reset 02h from ETS.
6. Home Assistant reading every group object in one burst.
7. knxip: Secure Routing backbone-key download, then a TIMER_NOTIFY
   exchange with xknx.
8. A 24 h soak with normal traffic; record the periodic worst values.

**Sizing rule.** Size = peak × 1.25 + 1 KB, rounded up to 512 B. If the
service task's start-up peak is more than 1.5× its steady-state peak,
consider running start-up in a short-lived task and shrinking the service
task. If the RX task's peak exceeds 6.5 KB, raise the Kconfig range of
`KNX_RX_TASK_STACK_SIZE` (8192 is its maximum today) and its default.

**Found on the way, not changed:**

- The TP1 image is built without `CONFIG_KNX_ENABLE_SECURITY` (only
  `sdkconfig.defaults.knxip` sets it), so it links no Data Secure code,
  while `knx_product.hpp` advertises `dataSecureCapable = true` and
  `knx_data_secure.cpp` installs a tool key. Either the TP1 image should
  enable it (more flash, and the RX task then carries the CCM frames above)
  or the TP1 catalogue entry should not claim Data Secure. Decided: enable
  it; done, see "Data Secure on the TP1 image" below.
- `Dpt10` encodes time of day as `DDDHHHHH MMMMMMSS SSSS0000`, and
  `Dpt19::decode` reads the hour across octets 4 and 5. 03_07_02 has DPT 10
  as `NNNHHHHH 00MMMMMM 00SSSSSS` and the DPT 19 hour in the low five bits of
  its fourth octet. Habinari uses neither DPT; the -Wconversion casts below
  keep the current bytes.

**-Wconversion in the sanitiser build** (supersedes the 3.4 follow-up). The
warnings were real `int` → `uint8_t`/`uint16_t` narrowings in
shift-and-or expressions; GCC normally proves the range and stays quiet, and
UBSan's shift instrumentation hides the range from it. Each is now an
explicit cast of the whole expression, with the bytes unchanged: 36 sites in
`types.hpp`, `protocol/tpci.hpp`, `application/device_descriptor.hpp`,
`util/byte_stream.hpp`, `util/bit_ops.hpp`,
`physical/bitbang_driver_timer_isr.hpp`, `dpt.cpp`, `cemi.cpp`,
`device_management_codec.cpp`, `filter_table.cpp`,
`ets_format_validator.cpp` and `tp1_data_link_layer.cpp`. The ASan/UBSan
build is at 0 warnings and the CI sanitiser job now configures with
`-DKNX_WARNINGS_AS_ERRORS=ON`.

**3.8 done** (a–i): one concrete stack port; Tp1MacController folded into
Tp1MacPhysical; one error type; `BusAccessUnit` split into DeviceModel,
GroupRuntime and DeviceManagement; `ApplicationLayer` split into receive,
ManagementServer and ASendPipeline with an explicit request context;
non-movable runtime objects; stack instrumentation (above). Fixed on the
way: failed persistence saves and Data Secure sequence checkpoints are
retried, a failed erase no longer carries stale data into a new bank, and
a router role no longer logs errors at boot. The sanitizer build is
warning-free and CI enforces it.

**3.9 done.**
- DPT codecs checked against 03_07_02: DPT 10 and 19 layouts fixed, DPT 15
  rewritten (6 BCD digits + flags), DPT 243/244 replaced by the real types,
  DPT 9 maximum and rounding fixed (Habinari publishes DPT 9: at most 1 LSB
  differs at double-rounding edges), DPT 16/17/242 corrected.
- Filter table: the 03_05_01 §5.2/5.3 bitmap over the full group address
  space, owned by the coupler only; forwarding per 03_03_03 Figure 5 and
  the GROUP_IACK_ROUT acknowledge filter in the TP1 ISR
  (`CONFIG_KNX_ENABLE_COUPLER`). Memory-mapped (Type 3) loading is not done.
- KNXnet/IP tunnelling client and server per 03_08_04 / 03_08_09: secure
  session MAC with a Device Authentication Code, replay protection,
  CONNECT_RESPONSE HPAI/CRD, sequence and repetition rules, heartbeat.

**Open after phase 3:**
- ~~Five interop tests fail~~ — fixed, see "Interop suite" below.
- ~~The TP1↔IP interface bridge does not apply the tunnel individual address
  or produce L_Data.con~~ — done, see "Follow-ups (14 November 2026)" below.
- ~~Data Secure on the TP1 image~~ — done, see below.
- Bench: ETS filter-table download (and which load path ETS uses), DPT 243
  field order, a real gateway with DAC verification, xknx/ETS6 against TCP
  tunnelling, plus the earlier bench lists.

### Data Secure on the TP1 image (14 November 2026)

**What was wrong.** `CONFIG_KNX_ENABLE_SECURITY` was set only in
`sdkconfig.defaults.knxip`. The TP1 image is `sdkconfig.defaults` alone, so
`tools/build_variants.sh knx` produced an image with no CCM transforms while
the product (`SecurityPolicy::dataSecureCapable`) and the TP1 `.knxprod`
(`SecureEnabled="true"`) advertised Data Secure and `knx_data_secure.cpp`
installed a factory Tool Key and printed a certificate for it. The secure
ETS6 download recorded on TP1 hardware was most likely built from the
untracked project-root `sdkconfig` on the development machine, which has the
option on; the scripted variant build did not.

**Change.** `CONFIG_KNX_ENABLE_SECURITY=y` moves to `sdkconfig.defaults`
(restated in the knxip overlay). Only the AES block cipher comes from the IDF
(`esp_aes`); CCM is KNstaX's own, so no mbedTLS options change, and the
mbedTLS 4 restriction (tunnelling withdrawn with security on) does not touch
TP1, which has no tunnelling client. `knx_data_secure.cpp` now
`static_assert`s `KNX_SECURE_ENABLED`, so a KNX image without it no longer
builds. The non-KNX variants compile the option and link none of it (their
sizes are unchanged to the byte). The ETS export is built on the host from
`knx_product.hpp` and already said secure-capable for both media; nothing
changes there.

**Size.** knx 952,592 → 965,584 B (+12,992 B, 49 % of 0x1e0000). knxip,
modbus, modbus-ble and mqtt unchanged. 0 warnings, no unknown Kconfig
symbols.

**Stack.** Static report for the knx image with security on (`-Og`,
`HABINARI_STACK_USAGE=ON`, edges stitched): `rxTaskFunc` deepest resolved
chain 6,128 B over 54 frames (was 5,408 B without security). It is a
management response encrypted on the RX task: receive → `handleTransportRx`
(624) → `ManagementServer` → `ASendPipeline` → transport TX transform (336)
→ `processOutgoing` (224) → `ccmEncrypt` (784) → CBC-MAC → `esp_aes_crypt_ecb`,
then the IDF's lazily-created AES lock, heap, log and `__assert_func` tail.
5.1 KB of it is the stack down to the AES call. `ccmDecrypt` is 912 B, on a
shallower path. 125 unresolved indirect sites in 76 functions remain. The
`knxServiceTask` chain is unchanged at 15,328 B. Per the sizing rule the RX
stack stays at 8192 (a resolved lower bound under the 6.5 KB threshold);
`sdkconfig.defaults` carries the measurement. Bench steps 2 and 3 (boot with
keys in force; ETS6 full Secure download over TP1) give the peak that
decides it.

**Test.** `test_data_secure_virtual_tp1` (KNstaX): a whole device on
`createTp1StackPort()` / `Tp1MacPhysical` / the bit-bang ISR core on the
virtual bus, RX task on a VirtualPlatform fibre. A group write secured with
the group key reaches the group object (plain refused, replay refused as
SequenceNumberInvalid); an A_PropertyValue_Read secured with the Tool Key is
answered with a Tool-Access-secured A_PropertyValue_Response in an extended
frame, which a peer S-AL verifies and decrypts, and the plain request is
answered plain. The peer's request is CCM-protected with the public block and
cipher primitives, not the S-AL under test. The virtual bit-bang driver moved
to `test/common/virtual_bitbang_tp1_driver.hpp` and now forwards the queue
notify callback to the ISR core, as the ESP-IDF wrapper does.

### Interop suite (14 November 2026)

All five were harness faults; no KNstaX library code changed. xknx 4.1.1,
`ctest -R interop_pytest`: 32 passed, 0 skipped (was 26 passed, 5 failed,
1 skipped), stable over three runs.

- **The four Data Secure over UDP tunnelling cases**
  (`test_xknx_to_knstax_…_groupwrite`, `test_xknx_reads_from_knstax_…`,
  `test_knstax_to_xknx_…_groupwrite`, `test_data_secure_replay_dropped_or_flagged`).
  `knstax_tunnel_peer` set the group key, Security Mode and sequence numbers
  on the Security Object *before* `BusAccessUnit::init()`. Since init
  restores the interface objects and rebuilds the key maps from the
  persisted key tables (`applySecurityKeyTables`), the key was gone: inbound
  secured telegrams failed with "Unknown Key" and outbound ones went in
  plain, which xknx flagged and the replay test never saw as secure. The
  peer now configures security after init (and after the group object), as
  the product API already requires (`installFactoryToolKey`: "Must be called
  after start()").
- **`test_client_device_descriptor_read_response_udp_tunneling`** sent
  A_DeviceDescriptor_Read to group address 1/0/0. 03/03/07 Table 1 defines
  it point-to-point only, and since phase 1 the application layer ignores a
  service on a mode it is not defined for. The test now sends it
  connectionless to the peer's individual address 1.1.1, and gets the
  response.
- **Found on the way.** The parametrised Data Secure case of
  `test_knstax_interop_suite.py` was skipped on every run: it wrote an ad-hoc
  unsigned keyring that xknx refuses to load, and turned the load failure
  into a skip. It now uses the signed ETS-format generator of the dedicated
  Data Secure tests, fails instead of skipping on a load error, and passes.
  The test gateway's CONNECT_RESPONSE echoed the CRI as the CRD, which xknx
  (it takes `current_address` from the CRD) read as tunnel address 0.2.0; it
  now returns the assigned address (channel n = 1.1.n, so the peer is 1.1.1
  and xknx 1.1.2, the addresses the keyrings name). That was not what failed
  the five (they pass with either CRD) but xknx was sending from the wrong
  address.
- **CI.** `interop/python/requirements.txt` pinned `xknx` nothing, so CI took
  whatever was newest; it now pins `xknx==4.1.1` (also in
  `requirements-secure.txt`), the version verified here. The workflow
  installs from that file, so it needs no change.

**Configuration before init() (done).** One rule, documented on
`BusAccessUnit::init()` and in DEVELOPMENT_GUIDELINES.md: what firmware sets
before `init()` is the device's factory default; state restored from
persistence (an ETS download) wins over it; Factory Reset returns to it.
Group keys yield per table to a downloaded key table; Sequence Number
Sending keeps the higher value; setters that cannot hold a value before
init return NotInitialized / OperationNotReady instead of succeeding
silently. Found through the interop peer, whose Data Secure key init()
wiped. ~~Open: `SecureCommissioningOptions` cannot express "factory Tool Key
with Security Mode off"~~ — done, see "Follow-ups" below.

### Follow-ups (14 November 2026)

**TP1↔IP interface bridge** (`network::Tp1IpInterfaceBridge`). It is now the
cEMI server of the interface, and asynchronous: requests are queued and
`poll()` (on the task that polls the tunnelling server) drives the TP1
transmission, so the server's poll never blocks on the line's repetitions.

- *Tunnel addresses.* 03_08_04 §2.2.2: an L_Data.req with Source Address
  0000h goes onto TP1 with the connection's tunnel address; "Otherwise, the
  KNX frame shall be sent unchanged" (also 03_06_03 §4.1.5.3.3), so any other
  source is kept. Every assumed tunnel address is acknowledged on TP1 whether
  a connection is open on it or not ("Additional KNX Individual Addresses
  shall be permanent. The KNXnet/IP Server shall generate Layer-2 acknowledge
  frames for these"): `Tp1DataLinkLayer::setAdditionalIndividualAddresses()`
  reaches the bit-bang ISR's new individual acknowledge set (16 entries,
  linear scan) through Tp1MacPhysical and the medium backend, and the data
  link's receive filter.
- *L_Data.con.* 03_06_03 §4.1.5.3.4: after the transmission, the requested
  frame with code 2Eh and Confirm flag 0 (no error) or 1 (error); requests are
  confirmed one at a time in arrival order (§4.1.5.1, FIFO). Positive when the
  data link reports success; negative on NAK, BUSY or no L_ACK after the three
  repetitions, and when the request queue (8) is full. A group frame nobody
  acknowledged is confirmed positively, because the data link treats it as
  sent (its documented decision on 03_02_02 §2.4.2), so a cEMI client sees a
  negative confirmation for a group frame only on NAK or BUSY. The r-flag
  ("do not repeat") is not honoured: the data link has no per-frame
  repetition control.
- *L_Data.ind.* 03_08_04 §2.2.2: group frames to every connection,
  point-to-point frames only to the connection whose tunnel address is the
  destination (others are dropped; a client wanting all traffic uses
  Busmonitor, not implemented). A frame one connection sent successfully is
  indicated to the other connections by the same rule (footnote 2: each
  connection "logically appears as another device on the KNX bus").
- *Not done.* PID_ADDITIONAL_INDIVIDUAL_ADDRESSES and the T_Connect defence of
  idle additional addresses (§2.2.2), cEMI local device management, and
  Busmonitor tunnelling.
- *Tests.* Bridge unit tests (mock line): source fill, positive and negative
  confirmation for Timeout/NAK/BUSY, routing of indications by tunnel
  address, FIFO, non-request cEMI dropped. On the virtual TP1 bus with the
  real data link, MAC and ISR: positive confirmation after a bus ACK, negative
  after four unacknowledged copies, the ISR acknowledging a tunnel address and
  not a foreign one, the indication reaching only its connection. An ISR unit
  test for the additional acknowledge set. Interop (`knstax_interface_peer`, a
  tunnelling server and bridge on a scripted line): xknx 4.1.1 sends a group
  write, which returns only after its L_Data.con (C = 0, and C = 1 with the
  line NAKing; xknx itself ignores C), goes onto the line from tunnel address
  1.1.10, and receives a frame injected on the line. Interop: 34 passed.
- *Bench.* ETS6 and Home Assistant through a real interface on a TP1 line:
  confirmation timing, two tunnels at once, a download through a tunnel.

**Filter Table Realisation Type 3** (`network::FilterTableLoadControl`).
03_05_01 §5.3.1.5: the MaC allocates the table with an Additional Load
Control, reads PID_TABLE_REFERENCE and writes the bit field with memory
writes; §5.3.1.4: the Router Object's Load State Machine is Realisation Type 1
and the evaluation is negative in every state but Loaded.

- The Router Object declares PID_LOAD_STATE_CONTROL (5, PDT_CONTROL),
  PID_TABLE_REFERENCE (7, read-only) and PID_MCB_TABLE (27, PDT_GENERIC_08).
  `GenericInterfaceObject::bindProperty()` backs them with the load state
  machine instead of storage. The table appears at 8000h (8192 octets, below
  10000h so DMP_MemWrite_RCoV applies), registered as a memory region when the
  Router Object is published, i.e. only for a router role. The window is the
  FilterTable's own storage, so no RAM is added.
- Transitions per §4.23.2.3.3 Table 94 (recommended column): Start Loading
  from Unloaded/Loaded/Loading → Loading, Error stays; Load Completed only
  completes from Loading; Additional Load Controls in Loaded → Error; Unload
  from anywhere → Unloaded; unknown events ignored (§4.23.2.3.2). Allocation
  accepts Data Relative Allocation (0Bh, with fill), Relative Allocation (0Ah)
  and Absolute Data Allocation (00h) at 8000h only (03_05_02 §3.31.3.4); a
  size above 8192 or another address → Error. PID_TABLE_REFERENCE is zero
  while unallocated or Unloaded (§4.2.7). Unload clears the table (Table 93:
  "The data is undefined") and the MCB.
- MCB (§4.2.27): writable only in Loading. On Load Completed the device
  computes the CRC over the segment (§4.2.27.1.2); when the MaC wrote an MCB,
  a different CRC is an error in the data → Error (Table 93), otherwise the
  MCB is filled in with segment size 8192 and the CRC. The CRC is the variant
  whose check value the clause gives ("123456789" → E5CCh): the augmented
  formulation with FFFFh, i.e. the direct algorithm seeded 1D0Fh (seeded
  FFFFh it gives 29B1h).
- Every state change reaches the coupler: `setFilterTableLoaded()` on its
  policy and `refreshAckFilters()` for the ISR's group acknowledge.
- *Not done.* Neither the table nor its load state is persisted (already so
  for PID_ROUTETABLE_CONTROL downloads): after a restart the coupler is
  Loaded with an empty table and blocks every group frame until the next
  download. Master Reset does not reset the table (§5.3.1.2 asks for the
  implementation default on 02h/05h/07h). PID_ERROR_CODE is not provided.
- *Tests.* Unit tests of the state machine (transitions, allocation, fill,
  MCB, CRC check value); an ETS-like download through the real application
  layer (Unload, Start Loading, Data Relative Allocation, reference read,
  A_Memory_Write, MCB, Load Completed), a CRC mismatch ending in Error, Unload
  clearing the table; on a coupler stack port, the load state reaching the
  routing policy.
- *Bench capture needed:* which load path ETS uses for this mask (the Type 3
  load state machine with memory writes, or PID_ROUTETABLE_CONTROL), whether
  it writes the MCB and with what CRC, which allocation subtype it sends, and
  whether it uses A_Memory_Write or the extended memory services.

**SecureCommissioningOptions.** The Tool Key and the Security Mode are now
independent factory defaults: `std::optional<ToolKey> factoryToolKey` and
`bool securityModeAtFactoryReset` (replacing `enabled` and `toolKey`), both
applied before init() per the pre-init rule, so a stored ETS Tool Key or mode
wins and a Factory Reset returns to them. The TP1 `startCommissionedProduct`
overload taking a `Tp1MacPhysical` gained a `SecureCommissioningOptions`
argument (the four-argument form remains). Habinari resolves its serial
number and FDSK (eFuse, NVS fallback) before start and passes the FDSK with
the mode off; after start it only sets the serial on the Device Object and
logs. The log and certificate behaviour is unchanged; whether the certificate
is printed is decided by `habinari::identity::certificateDecision()`, which
has a host test. KNstaX product tests: a fresh device has the FDSK and mode
off; after an ETS download the ETS key and mode win over two restarts; after
a local Factory Reset (erased store, as `control_service_factory_reset`) the
FDSK is back with mode off. Master Reset 02h is covered by
`test_ets_connected_session`.

**Follow-ups, continued (15 November 2026).**
- **TP1 transmit race (production, fixed):** `finishTxOperation()` released
  the TX mutex before clearing its state, so a task waiting to send (the RX
  task answering ETS, or the application sending a group value) could lose
  its frame and keep the mutex, leaving TP1 unable to send until reboot.
  Found while making `test_knx_stack_e2e` deterministic.
- Coupler: the filter table, its load state and the routing configuration
  (PID 52–55, 67) are persisted and restored; Master Reset 02h/07h return
  them to factory values; filter-table memory writes are refused outside
  Loading; the Router Object has PID_ERROR_CODE.
- Still not persisted: PID_L2_COUPLER_TYPE (59) and PID_HOP_COUNT (61),
  which the forwarding path does not read.

**Bench (add):** a TP1 burst test — ETS reading properties while the
device publishes group values — to confirm the transmit race is gone on
hardware; NVS headroom on a coupler with a worst-case (random) filter
table; and a capture of an ETS6 coupler download (load path, MCB, whether
it writes the table window outside Loading).

### TP1 bit-bang ISR: specification, self-check and hot path (15 November 2026)

- The ISR core is specified in
  `components/KNstaX/docs/reference/tp1_bitbang_isr.md`: states, the legal
  composite transmitter modes, the anchored timing schedule, 14 invariants
  and the event contract with the layers above. Host builds check the
  invariants after every interrupt and `send()`, and the event grammar as
  each event is produced (`KNX_TP1_BITBANG_SELF_CHECK`). Comments in the ISR
  now describe the current behaviour only.
- Fixed: another device's DL-ACK starting before this one's cancelled the
  armed acknowledge, so a NAK or BUSY from this device could be lost. The
  armed slot now holds its own octet (`_ackSlotByte`).
- Fixed: after losing arbitration, the receiver restarted on a data bit of
  the winning frame and misdecoded it. The receiver now takes over the
  character at the diverging bit, and the winner's frame is received whole
  and acknowledged when addressed here.
- Fixed: a rejected start bit between two characters left the telegram open
  in `RX_IDLE`, and the next telegram was appended to it.
- Every reception this node abandons to transmit is now closed with `End`.
- Hot path: on single-timer TIMG chips (C6) the timer is read and rearmed
  through registers, and its alarm interrupt is the HAL's own (no gptimer
  call, spinlock or callback). The RX edge handler is registered directly
  with the GPIO ISR service. TX makes no timer reads, the ISR does no
  multiplications or library calls (parity no longer calls
  `__popcountsi2`), and hot fields sit at the start of the object. An audit
  of the `knx` ELF finds all 20 functions reachable from either interrupt
  in IRAM.
- Open: the MAC duplicates the ISR's "ACK window expected" rule, and
  `TxComplete` produces no event, so a frame to FFFFh abandoned after
  arbitration loss is reported as sent; the task-side event pump exists
  twice (target and host test driver). Both are in the spec, §9.

**Bench (add):** with `CONFIG_KNX_TP1_BITBANG_TIMING_STATS`, compare ISR
cycle counts and `timerJitterUs` with the previous firmware; confirm
`staleTimerAlarms` stays at zero and `startValidationRejects` does not grow
during an ETS download; check two devices acknowledging one group telegram
and two devices starting a frame at once (the loser must receive the
winner's frame).
