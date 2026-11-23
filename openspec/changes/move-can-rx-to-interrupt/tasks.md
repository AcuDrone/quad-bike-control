## 0. Preconditions
- [x] 0.1 Proposal approved by the owner, including the version choice (design D12: 1.5.38 by the rule, or 1.6.0) and the open questions in design.md.
- [x] 0.2 `add-can-bus-sniffer`, `add-event-journal-and-link-recovery` and `gate-obd-watchdog-and-pull-up-vesc-rx` are implemented on `master` (they are: 1.5.7, 1.5.26, 1.5.28). This change is **archived after** all three (its MODIFIED blocks quote their requirements).
- [x] 0.3 `move-aztec-vesc-to-can` is NOT implemented. Its proposal must be re-based on this version (no SPI mutex, `queueTx()` tags, version numbers) before it is.
- [x] 0.4 The chosen `FIRMWARE_VERSION` is still unclaimed by any change in `openspec/changes/` and on `aztec` / `old-board` at implementation time.
- [x] 0.5 Work starts on `master`; `aztec` and `old-board` get it only by plain merges of `master` (§9).

## 1. Pure law and native tests (`master`)
- [x] 1.1 `include/CanIoLaw.h` (header-only, `<stdint.h>` / `<string.h>` only, no Arduino, no `Constants.h`), design D3, D4, D6, D9:
  - `struct CanFrame { uint32_t id; uint8_t len; uint8_t data[8]; }`;
  - `template <uint8_t N> struct CanFrameRing`: `pushDropOldest()` (returns true when it dropped), `pushReject()` (false when full), `pop()`, `count()`, `hwm()`, `drops()`; no locking inside (the firmware guards it with a `portMUX`);
  - `canTxHeader(id, ext, dlc, out[5])`: SIDH, SIDL (EXIDE for extended), EID8, EID0, DLC clamped to 8 — the same bits as `mcp2515_write_id()` in `mcp_can.cpp:690-716`;
  - READ STATUS decoding: `rxPending(st)`, `txPending(st, buf)` (bits 0/1 = RX0IF/RX1IF, 2/4/6 = TXnREQ);
  - `txTimedOut(nowMs, loadedMs, timeoutMs)` (wrap-safe);
  - `mcp2515AliveAfterReset(canstat, canctrl)` (CANSTAT bits 7:5 = CONFIG and CANCTRL = `0x87`);
  - `canIntState(prev, hasIntPin, intWakes, frames)` → `NONE` / `OK` / `DEAD`, previous state kept when `frames == 0`.
- [x] 1.2 `test/test_can_io/test_main.cpp` (Unity), cases:
  - ring: FIFO order; `pushDropOldest` at full drops the oldest, keeps the newest 64, counts drops; `pushReject` at full refuses and keeps the content; `hwm` tracks the maximum; wrap-around over 1000 pushes;
  - TX header: standard `0x7DF` DLC 8 → `FB E0 00 00 08`; standard `0x7C0` DLC 2 → `F8 00 00 00 02`; standard `0x305`; extended `0x00000901` (VESC STATUS, ID 1) and `0x1FFFFFFF` checked against hand-computed `mcp2515_write_id()` output; DLC 15 clamps to 8;
  - READ STATUS: each RX/TX bit alone and combined;
  - TX timeout: just before / at / after 20 ms, across a `millis()` wrap;
  - liveness: `0x80`/`0x87` alive; `0x00`/`0x00`, `0xFF`/`0xFF`, `0x80`/`0xFF`, `0x00`/`0x87` not alive;
  - INT state: no pin → `NONE`; frames with zero INT wakes → `DEAD`; any INT wake → `OK`; no frames → unchanged.
- [x] 1.3 `platformio.ini` `[env:native]`: one indented `test_filter` line `test_can_io`; extend the suite list in the comment block. `pio test -e native` green.

## 2. Constants and build (`master`)
- [x] 2.1 `include/Constants.h`, CAN section: `CAN_TASK_PRIORITY` 12, `CAN_TASK_CONFIG_PRIORITY` 1, `CAN_TASK_CORE` 1, `CAN_TASK_STACK` 4096, `CAN_POLL_MS` 2, `CAN_RX_DRAIN_CAP` 16, `CAN_RX_RING` 64, `CAN_TX_RING` 8, `CAN_TX_TIMEOUT_MS` 20, `CAN_EFLG_POLL_MS` 10, `CAN_INIT_RETRY_MS` 5000, `CAN_BOOT_INIT_WAIT_MS` 1000, `CAN_INT_WINDOW_MS` 5000; each with a one-line reason (design D2–D9).
- [x] 2.2 Remove `CAN_MAX_CONSEC_SEND_FAILURES` and `CAN_SNIFF_DRAIN_PER_UPDATE`; `PIN_CAN_INT` comment becomes "MCP2515 INT → CAN task wake-up (FALLING edge); the task also polls every CAN_POLL_MS".
- [x] 2.3 `platformio.ini`: pin `MCP_CAN_lib.git#34807287506afc6b4aef7c577392c1c4e32d1b45`; add `-DDEBUG_MODE=0` to `[env]` `build_flags` with a comment (library prints to `Serial` otherwise, design D8). Clean build: `pio run` shows no "Setting Mask" strings in the binary (`strings .pio/build/*/firmware.elf | grep -c "Set Mask"` = 0).

## 3. CAN task skeleton (`master`, `src/CANController.cpp`, `include/CANController.h`)
- [x] 3.1 Members: RX / TX / event rings (`CanFrameRing`) with one `portMUX` each, command slot (`volatile` command + sequence), task handle, ISR counter, task-written counters (D9), chip-up flag, TXB0 tracking state.
- [x] 3.2 `begin()`: pins and `SPI.begin()` as today, create the task (D2 parameters), attach the ISR under `#if defined(PIN_CAN_INT) && !defined(CAN_TEST_NO_INT)` (`INPUT_PULLUP`, FALLING, IRAM ISR: count + `vTaskNotifyGiveFromISR` + `portYIELD_FROM_ISR`), post `INIT`, wait for its result at most `CAN_BOOT_INIT_WAIT_MS`, keep the existing boot log lines and return value (D7).
- [x] 3.3 Task loop per design D2: notification wait with `CAN_POLL_MS`, command execution, READ STATUS, drain ≤ `CAN_RX_DRAIN_CAP`, TXB0 service, EFLG every `CAN_EFLG_POLL_MS` (RXnOVR count + clear, TXBO event once per episode, TEC/REC/EFLG snapshot), wake-type / busy-time accounting. No logging anywhere in the task.
- [x] 3.4 Move every `mcp_can_->…`, `readRegister()`, `modifyRegister()`, `applyRxFilters()`, `applySniffFilters()` call into task-only functions. Grep check: these names appear only in task code paths and `begin()`'s pre-task pin setup.

## 4. Receive path (`master`)
- [x] 4.1 `update()`: drain the event ring (command results, TX results, bus-off) and then the RX ring (≤ `CAN_RX_RING` frames) at the very start, in every mode and also while `!initialized_` (events only), before any state machine step.
- [x] 4.2 Single router per design D3: `dispatchSteeringFrame()` first (unchanged), sniffer record while SNIFFING, OBD-II latch while WAITING_RESPONSE, probe latch while a probe request is in flight, else `logUnmatchedFrame()`.
- [x] 4.3 `tryReceiveResponse()` / `tryReceiveProbeResponse()` take the latched reply (RPM raw-frame debug line kept); sending a request clears the latch.
- [x] 4.4 Delete `drainRxBuffers()`, the 5-frame loops and `updateSniff()`'s 32-frame loop.
- [x] 4.5 `checkRxOverflow()` / `logHealth()` become loop-side readers of the task's counters and snapshot (same log lines and rate limits), called on every `update()` in every mode.

## 5. Transmit path (`master`)
- [x] 5.1 `queueTx(frame, tag, wantResult)` (loop only): push-reject into the TX ring, count `tx_qfull`, notify the task.
- [x] 5.2 Task: TXB0 load (WRITE burst from TXB0SIDH using `canTxHeader()`, TXP = 1, RTS `0x81`), completion by TXREQ clear, per-buffer abort after `CAN_TX_TIMEOUT_MS` (BIT MODIFY TXB0CTRL.TXREQ = 0), `TX_DONE` event when asked; frames asking for a result are failed at once while the chip is down.
- [x] 5.3 `sendOBDRequest()` / `sendMode03Request()` use `queueTx(…, OBD, false)`; delete `consecutiveSendFailures_`, the `abortTX()` call and the ABAT clear (`src/CANController.cpp:313-336` today); the send-failure branch keeps the reschedule and the retry count; bus-off now arrives as a task event and goes to `handleBusOff()`.
- [x] 5.4 `sendLwsCfg()` uses `queueTx(…, LWS_CFG, true)`; `serviceLwsZero()` gains the two "wait for TX result" phases (bounded by `CAN_TX_TIMEOUT_MS` + one cycle), `tx_failed` on a failed result or a full ring; the gap starts at the confirmed `[05 00]`. Payloads and the single-transmitter rule unchanged.

## 6. Init, re-init and sniffer through the task (`master`)
- [x] 6.1 Task-side attempt per design D6: priority drop to `CAN_TASK_CONFIG_PRIORITY`, liveness gate, `begin()`, filters stopping at the first failure, `setMode(MCP_NORMAL)`, CANSTAT + CANINTE readback, priority restore, self-notify, result event with `begin` / `filt` / `mode` flags, CANSTAT, EFLG.
- [x] 6.2 Loop-side `recoverFromBusOff()` / `attemptReinit()`: post `REINIT` and mark the attempt in flight; the existing journaling, counting, `CAN_REINIT_RETRY_MS` retry and `CAN_REINIT_MAX_ATTEMPTS` budget run when the result event arrives; no new trigger or command while one is in flight.
- [x] 6.3 Boot retry (D7): after a failed `INIT` the task retries every `CAN_INIT_RETRY_MS`; first failure journaled `CAN E init result=fail …`, later failures counted; success → loop does the `begin()` success work (`initialized_`, `lastReinitTime_`, PID stagger) and journals `CAN I init result=ok attempt= after_s=`.
- [x] 6.4 Sniffer: `startSniff()` keeps its refusals, posts `FILTERS_SNIFF`, returns `STARTED`; the window starts on success; failure → normal filters back (re-init if needed), sniff completes with `openFailed`; window end posts `FILTERS_NORMAL` (re-init on failure), then the existing completion. `SniffResults` gains `openFailed` and `queueDrops`; `overflows` comes from the task's counter delta over the window. Web sniffer view shows the two new fields (uk + en).
- [x] 6.5 Check that the LWS silence watchdog, the OBD-II watchdog (`CanWatchdogLaw.h`), first-frame timing and the journal economy are untouched apart from "attempt in flight" (design D10 table) — `test_can_watchdog` still green.

## 7. Diagnostics, telemetry, web (`master`)
- [x] 7.1 `CAN I stats …` journal line every `JOURNAL_SNAP_MS` from `update()` (window deltas, `miss` from `LwsSensor::missedFrames()` when a sink is attached, `-` otherwise); INT-state transitions journaled once (`CAN W int state=dead` / `CAN I int state=ok`); boot line includes the task stack high-water mark.
- [x] 7.2 `include/WebPortal.h` Telemetry + `src/TelemetryManager.cpp` + `createTelemetryJSON()`: `can_int` (static token), `can_rx_drop`, `can_rx_ovr`, `can_rx_hwm`, `can_tx_fail`, `can_busy_max_us`; update the capacity comment (≈ 6 slots, size unchanged).
- [x] 7.3 `data/index.html`: one muted line under "Стан CAN" / "CAN Status" per the web-telemetry delta, i18n keys in `en` and `uk`; nothing in Calibration; no attention item.

## 8. Docs, changelog, version, checks (`master`)
- [x] 8.1 `FIRMWARE_VERSION` → the approved number (1.5.38 by the rule).
- [x] 8.2 `include/Changelog.h` entry (uk + en, operator wording): CAN now served by its own task woken by the INT line (steering-angle frames no longer lost while the web page or the journal is busy); CAN comes back by itself if the controller was not ready at power-on; new receive line on the CAN status tile; upload the LittleFS image; no recalibration.
- [x] 8.3 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`: GPIO38 is now used (MCP2515 `INT`, FALLING edge, polling fallback); `WEB_PORTAL_SETUP.md`: the tile line and the sniffer's new fields; `DRONECAN_RESEARCH.md`: replace "the firmware polls the MCP2515" remarks.
- [x] 8.4 `pio run -e esp32-s3-devkitc-1` builds without new warnings; `pio test -e native` green; also build once with `-DCAN_TEST_NO_INT`.
- [x] 8.5 `openspec validate move-can-rx-to-interrupt --strict` passes.

## 9. Merges
- [ ] 9.1 Merge `master` → `aztec` (plain merge): `FIRMWARE_VERSION` resolves to the new number, `aztec`'s 1.5.37 changelog entry stays below the new one; the buggy wording ("багі") in the uk line; `pio run` on `aztec`, `pio test -e native`.
- [ ] 9.2 Merge `master` → `old-board` (plain merge): its `Constants.h` has no `PIN_CAN_INT`, so the build is polling-only — check that the merge did not add a `PIN_CAN_INT` for GPIO38 there; `pio run` on `old-board`.

## 10. Bench — both vehicles (code and bench are separate: nothing here is done in §1–9)
Common pass criteria: no reset or watchdog reboot; `CAN I stats` every 60 s; `bmax` ≤ 1 ms outside re-init / sniff windows; SNAP `loop_max` not worse than the 1.5.36 baseline over the same run.
- [ ] 10.1 Boot (each vehicle): journal shows `init result=ok`, telemetry `can_int` = `ok` on `Control_v0`; within a minute `int` ≈ `rx` and `poll` ≈ 0 in the stats line (proves GPIO38 follows MCP2515 `INT`; if not, record it — design Open Questions).
- [ ] 10.2 Steering-angle loss, journal flash writes OFF, ignition ON (engine running on the quad): 10 min with the web UI open on Diagnostics, page reloaded every ~30 s, the Event journal card refreshed, a WebSocket client on a second device, ≥ 10 gear changes, steering moved lock to lock → `miss` = 0, `drop` = 0, `ovr` = 0 in every stats line, no `STEER lws` fault.
- [ ] 10.3 Same with journal flash writes ON for 10 min: any `miss` comes in bursts ≤ 5 frames, `drop` = 0, no LWS `timeout`.
- [ ] 10.4 Long loop iterations: test build with an artificial 80 ms `delay()` in `loop()` every 2 s → `loop_overrun` journaled, `miss` = 0, `drop` = 0 (frames wait in the ring).
- [ ] 10.5 Polling fallback: build with `-DCAN_TEST_NO_INT` → `can_int` = `none`, tile "RX: polling (no INT wire)", `poll` ≈ `rx`; repeat 10.2 for 5 min → `miss` = 0.
- [ ] 10.6 Bus connector unplugged at boot (`Control_v0`): `init result=ok` (the chip needs no bus); `can_status` disconnected; with ignition OFF no `obd_silent` / `lws_silent`; OBD-II frames fail per frame (`txf` grows, at most one per poll), no hang, no abort of other frames; replug with ignition ON → OBD-II data and LWS frames return, `first_frame` journaled after any re-init.
- [ ] 10.7 Controller absent at boot (`old-board`, hand-wired module: unplug the MCP2515 module's VCC before power-on): journal `init result=fail` once, steering `can_down`, `loop_max` unaffected; plug it in → `init result=ok attempt=<n> after_s=<s>` within ≈ 5 s, LWS healthy without a reboot.
- [ ] 10.8 Watchdog re-init (buggy, ECU on): unplug the LWS connector → `lws_silent … -> reinit`, `reinit result=ok`, console `[CAN] task: cmd_max_us` ≈ 11–15 ms while `bmax` stays ≤ 1 ms, loop never stalls (`loop_max` unchanged); replug → `first_frame`.
- [ ] 10.9 LWS zero-set from the web (vehicle where it is used): succeeds as on 1.5.36; with the bus connector unplugged → aborted `tx_failed`, OBD-II polling resumes.
- [ ] 10.10 ECU probe from the web: same results as on 1.5.36.
- [ ] 10.11 Sniffer, standing still, 10 s: quad — the ECU broadcast flood appears in the table, `frames` ≥ the 1.5.36 figure, `queueDrops` and `overflows` shown, `bmax` recorded; afterwards normal filters (stats `ovr` back to 0, `rx` back to LWS + OBD-II rates); buggy — ECU + LWS IDs as before.
- [ ] 10.12 ArduinoOTA upload with the ignition ON: completes and reboots; the last pre-reboot stats line may show `drop` > 0 (consumers frozen, expected), nothing else abnormal. Web OTA: completes; `miss` bursts allowed (flash erases, design D11), no crash.
- [ ] 10.13 `old-board` after the merge: `can_int` = `none`, 10.2 for 5 min → `miss` = 0.
- [ ] 10.14 Chip down after a failed re-init (`old-board` or a `Control_v0` bench rig: unplug the MCP2515 supply with the ignition ON so three re-init attempts fail → `CAN E reinit result=fail`): no further journal lines while it stays down; plug it back → within ≈ 5 s `CAN I reinit reason=task_retry result=ok attempt=<n> after_s=<s>`, `first_frame`, LWS healthy without a reboot.

## 11. Archive
- [ ] 11.1 Archive after `add-can-bus-sniffer`, `add-event-journal-and-link-recovery` and `gate-obd-watchdog-and-pull-up-vesc-rx`, once the bench checks on both vehicles pass; `openspec validate --strict` after archiving.
