## 0. Preconditions
- [ ] 0.1 `replace-aztec-steering-sensor-with-bosch-lws`, `redesign-web-ui-service-console`, `add-aztec-steering-rate-mode` and `auto-detect-aztec-steering-sensor-direction` are implemented on `aztec` (firmware 1.5.1). This change is archived AFTER them. Its MODIFIED `CAN Communication Error Handling` quotes the LWS change's delta.
- [ ] 0.2 `move-aztec-vesc-to-can` is NOT implemented yet. It is archived immediately after this change, and its can-controller delta is already re-based on this one.
- [ ] 0.3 Confirm from the `Control_v0` schematic that GPIO38 is the MCP2515 `INT` pin at 3.3 V logic (TJA1051T/3 = 3.3 V I/O variant) with no other load. If no schematic is at hand, task 10.2 proves it on the bench.

## 1. Pure logic (host-testable)
- [ ] 1.1 Create `include/CanRxMonitor.h` (header-only, `<stdint.h>` only, no Arduino and no `Constants.h`), following design D6:
  - `class CanIntMonitor`, constructed with `windowMs`;
  - `update(nowMs, isrCount, frameCount)`, using window deltas with unsigned wrap-safe arithmetic;
  - states `ACTIVE` / `INACTIVE` (start: `ACTIVE`);
  - `intOk()`, `changed()` (true once per transition);
  - rule: frames > 0 and ISR = 0 → INACTIVE; ISR > 0 → ACTIVE; no frames → unchanged.
- [ ] 1.2 Create `include/Mcp2515Tx.h` (header-only, `<stdint.h>` only), following design D7:
  - `mcp2515EncodeTxHeader(id, ext, dlc, out[5])`:
    - standard: SIDH = id>>3, SIDL = (id&7)<<5;
    - extended: SIDH = id>>21, SIDL = ((id>>13)&0xE0) | 0x08 | ((id>>16)&0x03), EID8 = id>>8, EID0 = id;
    - DLC & 0x0F, clamped to 8.
  - `mcp2515PickFreeTx(readStatusByte)`: TXREQ bits 2/4/6 → buffer 0/1/2, or −1 when all are busy.
  - `mcp2515TxSidhAddr(n)` (0x31/0x41/0x51) and `mcp2515RtsCmd(n)` (0x81/0x82/0x84).
- [ ] 1.3 Add `test/test_can_rx_irq/test_main.cpp` (Unity). It covers:
  - Monitor:
    - starts ok;
    - frames without ISR for one full window → INACTIVE, with `changed()` once;
    - ISR again → ACTIVE, with `changed()` once;
    - a silent window keeps the state;
    - a partial window decides nothing;
    - `millis()` and counter wrap.
  - TX header:
    - std `0x7DF` → `FB E0 00 00 08` (DLC 8);
    - std `0x7C0` → `F8 00 00 00 02`;
    - std `0x305`;
    - ext `0x00000901` (SIDH 00, SIDL 08, EID8 09, EID0 01);
    - ext `0x18DAF110` round-trip against a reference decode (the `mcp2515_read_id` rule);
    - DLC > 8 clamped.
  - Free buffer: status `0x00` → 0, `0x04` → 1, `0x14` → 2, `0x54` → −1.
- [ ] 1.4 `platformio.ini` `[env:native]`: add `test_can_rx_irq` as its own indented line in the multi-line `test_filter` (the single-line form silently skips suites), and extend the comment. `build_src_filter` stays as it is, because both headers are header-only.

## 2. CAN controller: task, ISR, queue, mutex (`CANController.h/.cpp`)
- [ ] 2.1 Add the members:
  - `SemaphoreHandle_t rxSem_, spiMutex_`;
  - `QueueHandle_t rxQueue_`;
  - `TaskHandle_t rxTask_`;
  - `volatile bool reinitActive_`;
  - counters: `isrCount_` (volatile, ISR-only writer), `rxTaskFrames_`, `queueDrops_`, `capHits_`, `spiLockTimeouts_`, `rxOverflowCount_`, plus the window maxima `queueHwm_` and `maxQueueAgeMs_`;
  - `CanIntMonitor intMon_`;
  - `struct RxFrame { uint32_t id; uint8_t len; uint8_t data[8]; uint32_t tMs; }`.
  - Add the `lockSpi(ms)` / `unlockSpi()` helpers and an RAII `SpiGuard`.
- [ ] 2.2 `begin()`:
  - create `spiMutex_`, `rxSem_` and `rxQueue_` first;
  - run the whole existing init (`MCP_CAN` construction, `begin`, `applyRxFilters`, `setMode`, the CANSTAT readback) under the lock;
  - then assert CANINTE (2.6);
  - release, and only then log (move today's in-between prints after the release);
  - on success: `pinMode(PIN_CAN_INT, INPUT_PULLUP)`, then `attachInterrupt(PIN_CAN_INT, canIntIsr, FALLING)` unless `CAN_RX_TEST_NO_INT` is defined, then create `can_rx` with `xTaskCreatePinnedToCore(…, CAN_RX_TASK_STACK, this, CAN_RX_TASK_PRIORITY, &rxTask_, CAN_RX_TASK_CORE)`, then give `rxSem_` once;
  - on failure: delete whatever was created and return false (no task, no ISR).
- [ ] 2.3 Add `static void IRAM_ATTR canIntIsr()`. It only does `isrCount_++` and `xSemaphoreGiveFromISR` + `portYIELD_FROM_ISR`, through a static instance pointer. No SPI, no queue.
- [ ] 2.4 Add `static void canRxTask(void*)`, per design D2:
  - wait with `CAN_RX_WAIT_MS`;
  - skip while `reinitActive_`;
  - per frame: lock → `checkReceive()` + `readMsgBuf()` → unlock → `tMs = millis()` → `xQueueSend(…, 0)`, counting drops;
  - cap `CAN_RX_DRAIN_CAP`;
  - count lock timeouts and cap hits;
  - never log.
- [ ] 2.5 Add `pumpRxQueue()` and `routeRxFrame()`, per design D3:
  - call `pumpRxQueue()` first in `update()`, before the probe/normal branch, so it runs in every mode;
  - drain ≤ `CAN_RX_QUEUE_DEPTH` frames with `xQueueReceive(…, 0)`;
  - sample the high-water mark and the max age;
  - route: `dispatchSteeringFrame()` (unchanged, `millis()` timestamp) → the NORMAL WAITING_RESPONSE match into `responseData_` / `responseLen_` / `responseReady_` → the probe-in-flight match into `probeRxBuf_` / `probeRxLen_` / `probeRxReady_` → `logUnmatchedFrame()`.
- [ ] 2.6 CANINTE (design D5): after `begin()` and in `recoverFromBusOff()`, under the lock, `modifyRegister(MCP_CANINTE, 0xFF, CAN_CANINTE_VALUE)`, then `readRegister(MCP_CANINTE)`. After release, log `[CAN] CANINTE=0x%02X`, or an ERROR on mismatch.
- [ ] 2.7 Remove `drainRxBuffers()` (declaration and definition), plus the `checkReceive()`/`readMsgBuf()` loops in `tryReceiveResponse()` and `tryReceiveProbeResponse()`. Turn both into "consume the latched reply" and clear the latch whenever a new request is sent. Update the comments that mention "up to 5 messages".
- [ ] 2.8 `recoverFromBusOff()`:
  - set `reinitActive_`;
  - take the lock (the loop is the only caller);
  - `begin` + `applyRxFilters` + `setMode` + CANINTE + the CANSTAT/EFLG readback;
  - release, clear the flag, give `rxSem_`;
  - then log. Keep the queue contents.
- [ ] 2.9 Update the class Doxygen header and the `applyRxFilters()` comment ("caller holds the SPI lock"). Mark `readRegister()` / `modifyRegister()` / `applyRxFilters()` / `queueTx()` "requires lock", with a debug-build holder check (`xSemaphoreGetMutexHolder`) that counts violations.

## 3. Transmit paths under the mutex
- [ ] 3.1 Add `bool queueTx(uint32_t id, bool ext, uint8_t len, const uint8_t* data)` (caller holds the lock), per design D7:
  - READ STATUS (0xA0) → `mcp2515PickFreeTx()`;
  - one WRITE burst at `mcp2515TxSidhAddr(n)` (header + data);
  - RTS `mcp2515RtsCmd(n)`;
  - raw SPI with the library's `SPISettings(10000000, MSBFIRST, SPI_MODE0)` and `csPin_`, like `readRegister()`.
- [ ] 3.2 `sendOBDRequest()` / `sendMode03Request()`: lock (timeout → count, return false, do not touch `consecutiveSendFailures_`), `queueTx(0x7DF, false, 8, …)`, unlock. "All buffers busy" returns false and keeps today's failure log and counters.
- [ ] 3.3 The send-failure branch in `update()`: take one lock around `getError()`, `errorCountTX/RX()`, `abortTX()` and the ABAT clear. Call `recoverFromBusOff()` (which locks by itself) only after releasing it. Print after release.
- [ ] 3.4 The WAITING_RESPONSE timeout and probe-timeout `getError()` reads: lock around the read only, and call `recoverFromBusOff()` after release.
- [ ] 3.5 `sendLwsCfg()`: lock around `sendMsgBuf()` (still the library's confirmed transmit, held ≤ 5 ms), and print after release. The zero-set phases are otherwise unchanged.
- [ ] 3.6 Grep check: every `mcp_can_->`, `readRegister(`, `modifyRegister(` and `queueTx(` call sits in `begin()`, `recoverFromBusOff()`, `canRxTask()` or a locked block. No `Debug::` call sits inside a locked block.

## 4. Error flags, health and diagnostics
- [ ] 4.1 `checkRxOverflow()`:
  - call it from `update()` in every mode, throttled to `CAN_EFLG_CHECK_MS`;
  - lock around the EFLG read and clear;
  - increment `rxOverflowCount_` on each detected overflow;
  - keep the rate-limited log;
  - add the accessor `rxOverflowCount()` (used by `move-aztec-vesc-to-can`).
- [ ] 4.2 `logHealth()`: lock around the three reads, print after release. Behaviour is otherwise unchanged.
- [ ] 4.3 Add `logRxStats()`, the `[CAN] rx:` line every `CAN_HEALTH_LOG_INTERVAL` with the fields of design D6 (stack via `uxTaskGetStackHighWaterMark(rxTask_)`). Reset the window maxima after printing.
- [ ] 4.4 Call `intMon_.update(millis(), isrCount_, rxTaskFrames_)` from `update()`. On `changed()`, log `[CAN] INT inactive, polling fallback` or `[CAN] INT active`.
- [ ] 4.5 Add the accessors `isIntOk()`, `rxQueueDrops()`, `rxTaskFrames()`, `isrCount()`.

## 5. Constants (`include/Constants.h`)
- [ ] 5.1 Replace the `PIN_CAN_INT` comment: "MCP2515 INT (active low) → FALLING-edge ISR wakes the `can_rx` reader task; 2 ms polling fallback if dead".
- [ ] 5.2 Add to the CAN section, each with a one-line rationale comment:
  - `CAN_RX_TASK_PRIORITY 12`;
  - `CAN_RX_TASK_STACK 4096`;
  - `CAN_RX_TASK_CORE 1`;
  - `CAN_RX_WAIT_MS 2`;
  - `CAN_RX_DRAIN_CAP 16`;
  - `CAN_RX_QUEUE_DEPTH 32`;
  - `CAN_SPI_LOCK_TIMEOUT_MS 10`;
  - `CAN_INT_CHECK_MS 5000`;
  - `CAN_EFLG_CHECK_MS 100`;
  - `CAN_CANINTE_VALUE 0x03` (RX0IE | RX1IE only).
- [ ] 5.3 In the MCP2515 acceptance comment block, replace "faster than the loop can drain them" and "the only loss is one LWS frame on RX1OVR" with the task/queue wording.

## 6. Telemetry and web
- [ ] 6.1 `TelemetryManager.h/.cpp`: add `can_int_ok` and `can_rx_queue_drops`, set only when the CAN controller is initialized. `WebPortal.cpp`: emit them outside the `can_status == "connected"` block, and re-check the `StaticJsonDocument<4096>` capacity comment.
- [ ] 6.2 `data/index.html`, the "Стан CAN" tile:
  - one muted line `RX: переривання` / `RX: interrupt`, or the warning-coloured `RX: опитування (INT не працює)` / `RX: polling (INT dead)`;
  - `, втрачено N` / `, dropped N` when the drops are > 0;
  - nothing when the keys are missing;
  - new i18n keys in both dictionaries (parity).
  - "Увага" is unchanged.
- [ ] 6.3 Update the CAN card help (ⓘ) text in en and uk with one sentence on the receive mode.

## 7. Docs
- [ ] 7.1 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`: the GPIO38 rows (pin table, the SPI (CAN) table, the migration table) change from "optional/future — firmware polls" to "MCP2515 INT → reader task (polling fallback)". Add one paragraph under "SPI (CAN)": ISR → `can_rx` task → queue → `update()`; one SPI mutex; the `[CAN] rx:` line.
- [ ] 7.2 `openspec/project.md` Architecture Patterns: one bullet: "CAN RX: MCP2515 INT → FreeRTOS reader task → queue; all consumers stay in the main loop; one SPI mutex".
- [ ] 7.3 `WEB_PORTAL_SETUP.md`: the CAN tile's RX line and what "polling (INT dead)" means.
- [ ] 7.4 `DRONECAN_RESEARCH.md`: change the two "polled" remarks to "interrupt-driven since 1.6.0". The recommendation (no DroneCAN on the MCP2515) stays, and so does its bus-load reasoning.
- [ ] 7.5 `platformio.ini`: pin `mcp_can` to `https://github.com/coryjfowler/MCP_CAN_lib.git#34807287506afc6b4aef7c577392c1c4e32d1b45`, with a comment saying the design relies on its `mcp2515_init()` CANINTE and `readMsg()` flag clearing.

## 8. Version and changelog
- [ ] 8.1 `include/Constants.h`: `FIRMWARE_VERSION` "1.5.1" → **"1.6.0"**.
- [ ] 8.2 `include/Changelog.h`: new first entry `{"version":"1.6.0","date":"2026-10-29"}`:
  - uk:
    - "Приймання CAN переведено на переривання: кадри датчика кута більше не губляться під час роботи веб-панелі та OTA"
    - "Повторне калібрування не потрібне"
  - en:
    - "CAN receive moved to an interrupt-driven task: steering sensor frames are no longer lost while the web UI or OTA is busy"
    - "No recalibration required"

## 9. Build and test
- [ ] 9.1 `pio run` builds without new warnings.
- [ ] 9.2 `pio test -e native` passes: `test_lws305`, `test_steer_rate`, `test_steer_sensor_dir` and `test_can_rx_irq`.
- [ ] 9.3 `pio run -t buildfs` succeeds (`index.html` changed).
- [ ] 9.4 `openspec validate move-can-rx-to-interrupt --strict` passes, and so does `openspec validate move-aztec-vesc-to-can --strict`.

## 10. Bench verification [USER/MANUAL]
- [ ] 10.1 [USER/MANUAL] Boot log:
  - `[CAN] CANINTE=0x03`;
  - the filter line is unchanged;
  - with the LWS powered, the `[CAN] rx:` line shows `isr` close to `frames`, `drops=0`, `ovr=0`, `int=ok`, and `stack` ≥ 1500 bytes free.
- [ ] 10.2 [USER/MANUAL] `can_int_ok` is true in telemetry, and the CAN tile shows "RX: переривання". This proves that GPIO38 is the MCP2515 `INT`.
- [ ] 10.3 [USER/MANUAL] Web UI open (Діагностика + a second client), no OTA, 10 min, ECU and LWS on:
  - `steer_lws_miss` does not grow;
  - `can_rx_queue_drops` = 0;
  - `ovr` = 0;
  - `steer_lws_fps` ≈ 100.
- [ ] 10.4 [USER/MANUAL] The same, with an OTA upload running (design "Risks"):
  - `can_rx_queue_drops` = 0;
  - no LWS `timeout` fault;
  - `steer_lws_miss` growth only during the upload. Record the number per MB of firmware.
  - If a `timeout` appears, record it. It means a flash erase longer than 100 ms, not a receive bug.
- [ ] 10.5 [USER/MANUAL] OBD-II: RPM, coolant, module voltage, IAT and load update at their usual rates, and the RPM boost rate works during a gear change. The ECU probe ("Опитати ECU") completes with the same results as on 1.5.1.
- [ ] 10.6 [USER/MANUAL] "Встановити нуль датчика" (Set sensor zero) works: the zero-set frames are sent, the confirmation arrives, and no `tx_failed` appears.
- [ ] 10.7 [USER/MANUAL] Fallback: flash a test build with `-DCAN_RX_TEST_NO_INT` (or disconnect GPIO38 if the board allows). Within 5 s of LWS traffic, the log shows `[CAN] INT inactive, polling fallback` once, `can_int_ok` is false, the tile shows the warning line, the LWS stays fresh (no `timeout`) and `steer_lws_miss` stays ≈ 0. Flash the normal build back.
- [ ] 10.8 [USER/MANUAL] Re-init recovery:
  - Turn the ECU off with the LWS still powered and wait > 15 s. `[CAN] No data … re-initializing` appears, and after it `CANINTE=0x03` and `isr` keeps counting, with the LWS uninterrupted apart from ≤ 1 timeout during the reset.
  - Turn the ECU on: OBD-II data returns.
  - Optionally, force a bus-off (short CAN_H–CAN_L briefly during traffic): `Bus-off detected, recovering…`, then reception resumes on its own.
- [ ] 10.9 [USER/MANUAL] `lockto` stays 0 through all of the above. The "What's new" view shows 1.6.0 and the sidebar shows firmware 1.6.0.
