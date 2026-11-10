## 1. Journal core
- [x] 1.1 `include/Constants.h`: `JOURNAL_FILE_MAX_BYTES` (131072), `JOURNAL_LINE_MAX` (120), `JOURNAL_RAM_BUF_BYTES` (4096), `JOURNAL_FLUSH_MS` (1000), `JOURNAL_DEDUPE_MS` (2000), `JOURNAL_SNAP_MS` (60000), `JOURNAL_RTC_RING` (16), `JOURNAL_LOOP_OVERRUN_MS` (50), `JOURNAL_HEAP_WARN_BYTES` (40960, 40 KB), `JOURNAL_HEAP_HYST_BYTES` (8192), `JOURNAL_FLUSH_DEFER_MAX_MS` (10000)
- [x] 1.2 `include/EventJournalLaw.h` (header-only): source / level tokens, line formatter with truncation (`~`), dedupe table (24 keys, LRU, `sup=N`, 60 s pending flush), rotation decision
- [x] 1.3 Native test `test/test_event_journal`: format, truncation at 120 B, dedupe window and `sup=` count, exempt events, rotation threshold; add the suite to `platformio.ini` `[env:native]` (one indented `test_filter` line)
- [x] 1.4 `include/EventJournal.h` / `src/EventJournal.cpp`: static `log(src, lvl, event, fmt, ...)`, RAM buffer, mutex (loop try-take only), `service()` (flush ≤ 1/s when non-empty or buffer ≥ 75 %), `flushNow(why)`, `suspendFs()`, `requestClear()`
- [x] 1.5 Rotation: delete `/events.1`, rename `/events.log` → `/events.1`, new active; postponed while a download streams
- [x] 1.6 Boot counter: NVS namespace `journal`, key `boot`, incremented once in `begin()`
- [x] 1.7 `RTC_NOINIT_ATTR` ring (magic, version, CRC, boot, `prev_heap_min`, 16 lines; `static_assert` ≤ 2304 B); marked empty on a successful flush
- [x] 1.8 Boot sequence: `# boot=N fw=… reset=…` header, RTC replay (`RTC I replay n lost from_boot`, skipped on `POWERON` / bad CRC), then `SYS I boot reset fw heap prev_heap_min`
- [x] 1.9 Failure handling: retry, oldest-line drop with `journal_drop n=`, file output disabled after 3 consecutive failures or a failed mount
- [x] 1.10 SNAP every 60 s with the D3 field list; heap low / ok with 8 KB hysteresis
- [x] 1.11 Deferred flush (decision 2): `service(steeringDriving)` defers the flush while `SteeringController::isDriving()` (closed-loop drive, jog, nudge, rate drive, park pulse, nonzero effort), at most `JOURNAL_FLUSH_DEFER_MAX_MS` (10 s), then flushes anyway; `flushNow()` never deferred

## 2. Event sources wiring
- [x] 2.1 `src/main.cpp`: `EventJournal::begin()` right after LittleFS is mounted (lines logged earlier stay in RAM); `EventJournal::service()` as the last step of `loop()`
- [x] 2.2 `VehicleController::setInputSource()` → `SRC change`; `applyFailsafe()` → `failsafe_enter` / `failsafe_exit`
- [x] 2.3 `VehicleController::updateSteerMode()` → `STEER mode from to reason`
- [x] 2.4 `SteeringController::triggerStallStop()` → `STEER stall`; `setDiagLock()` → `STEER diag_lock on=`
- [x] 2.5 LWS health change (`SteeringController` on the sensor fault edge) → `STEER lws fault fps`
- [x] 2.6 `SteeringController::serviceDriver()`: `VESC link_lost` / `link_up` with the `LinkStats` snapshot, `link_down_snap` every 10 s while down, `fault code` on change
- [x] 2.7 `CANController`: `bus_off` (TEC/REC/EFLG), `obd_silent` / `lws_silent`, `reinit reason result attempt`, `first_frame lws_ms obd_ms`
- [x] 2.8 `PCA9557Expander` / `XL9535Expander`: keep the last `endTransmission()` / `requestFrom()` error (`lastError()`)
- [x] 2.9 `RelayController`: `I2C relay_fault addr err step` / `relay_ok`; `RELAY board_error` for detection / init failure and the starter escalation
- [x] 2.10 `BoardInputs`: `I2C input_fail addr err n` (on the stale edge) / `input_ok`
- [x] 2.11 `WebPortal` OTA success handler: `EventJournal::flushNow("ota")` before `ESP.restart()`; `EventJournal::suspendFs()` before `FILESYSTEM.end()` for a filesystem OTA

## 3. Web / API / UI
- [x] 3.1 `GET /api/events?tail=N` (text/plain; default 100, max 500; reads the active file backwards in ≤ 1 KB chunks, falls back to `.1`, appends unflushed RAM lines)
- [x] 3.2 `GET /api/events/download` (chunked text/plain, `.1` then active, `Content-Disposition` file name with the boot number)
- [x] 3.3 `POST /api/events/clear` → loop-executed clear, answered `{"ok":true}`
- [x] 3.4 `data/index.html`: «Журнал подій» / "Event journal" card in the System section after Network: last 100 lines (monospace, newest at the bottom), source filter (All + 9 sources), Refresh, Download, Clear (with confirm); refresh on open and every 5 s while the System section is visible; uk + en strings; ⓘ help text (what is logged, uploadfs erases the journal)
- [x] 3.5 Card shows a "journal unavailable (flash write failed)" note when `journal_bytes` is −1

## 4. VESC parser fix and tests
- [x] 4.1 `include/VescRxParser.h` (header-only): the existing state machine moved verbatim, plus `feed(byte, nowMs)` gap reset (`VESC_RX_GAP_MS` = 5), counters `bytes good drop_crc drop_stop resync bytes_since_good`
- [x] 4.2 `VescMotorDriver` uses the parser; on the `driverOk()` true → false transition flush UART RX and reset the parser; `LinkStats` getter on `IMotorDriver` (default zeros)
- [x] 4.3 `VESC W resync` journal line from the driver (deduped)
- [x] 4.4 Native test `test/test_vesc_rx`: dropped byte + 10 replies free of 0x02/0x03 at 300 ms → 0 decoded without the gap, 10 with it; clean stream; CRC error; bad stop byte; mid-burst stall loses ≤ 1 reply; add to `platformio.ini`

## 5. CAN watchdog
- [x] 5.1 `include/Constants.h`: `CAN_LWS_SILENT_MS` (1000), `CAN_LWS_WD_OBD_ALIVE_MS` (2000), `CAN_LWS_REINIT_MIN_MS` (5000), `CAN_REINIT_MAX_ATTEMPTS` (3)
- [x] 5.2 `CANController`: timestamp every dispatched `0x305`; arm after the first frame of the session
- [x] 5.3 LWS watchdog in `update()` with the D10 conditions (OBD alive, no zero-set, shared `lastReinitTime_`)
- [x] 5.4 `recoverFromBusOff()` returns a result from `begin()`, `applyRxFilters()`, `setMode()`; failed attempts retried on later `update()` calls, ≤ 3 per trigger; counter `can_reinit`
- [x] 5.5 Measure time to the first LWS / OBD frame after a re-init and journal it
- [x] 5.6 Ignition gating (decision 1): `CANController::setLwsPowered()` fed every loop from the relay ignition state (ACC or IGNITION); the watchdog runs only while powered, arms on the first `0x305` since power-on of the ignition, fixed 5 s spacing, no back-off; ignition OFF → no `lws_silent`, no re-init

## 6. I2C unlock and timeout
- [x] 6.1 `include/Constants.h`: `I2C_TIMEOUT_MS` (20), `I2C_UNLOCK_CLOCKS` (9), `I2C_RECOVER_MIN_MS` (1000), `BOARD_INPUT_UNLOCK_FAILS` (8)
- [x] 6.2 `include/I2cBusRecovery.h` / `src/I2cBusRecovery.cpp`: level check, 9-clock + STOP bit-bang, `Wire1.end()/begin()/setTimeOut()`, shared 1 s rate limit, counter `i2c_unlock`, `I2C W bus_stuck` line
- [x] 6.3 `main.cpp`: `Wire1.setTimeOut(I2C_TIMEOUT_MS)` after `Wire1.begin()`; update the I2C topology comment ("only I2cBusRecovery restarts Wire1")
- [x] 6.4 `RelayController` recovery path calls the recovery before `initPort` (bus evidence only, D11)
- [x] 6.5 `BoardInputs` calls it after 8 consecutive failed polls, then re-configures the expander
- [x] 6.6 Verify which I2C driver the pinned arduino-esp32 core uses and that `setTimeOut()` is honoured; note the result in `GPIO_PINOUT_CUSTOM_BOARD_S3.md`

## 7. Telemetry counters
- [x] 7.1 `include/WebPortal.h` Telemetry: `vesc_resync`, `can_reinit`, `i2c_unlock` (uint32, session totals), `journal_bytes` (int32, −1 = unavailable)
- [x] 7.2 `TelemetryManager::collectTelemetry()` fills them; `WebPortal` JSON adds the four keys
- [x] 7.3 `data/index.html`: counters shown in the Event journal card header

## 8. Docs, changelog, version
- [x] 8.1 `FIRMWARE_VERSION` → `"1.5.26"`
- [x] 8.2 `include/Changelog.h`: 1.5.26 entry (uk + en): event journal card, VESC link self-recovery, LWS CAN watchdog, I2C bus unlock
- [x] 8.3 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`: I2C2 unlock procedure, 20 ms timeout, the single Wire1 restart owner
- [x] 8.4 `WEB_PORTAL_SETUP.md`: Event journal card, `/api/events*`, line format, `uploadfs` / filesystem OTA erases the journal
- [x] 8.5 `pio run` builds without new warnings; `pio test -e native` passes; `openspec validate add-event-journal-and-link-recovery --strict`

## 9. Bench verification
- [ ] 9.1 Fresh boot after power-on: header + `boot reset=POWERON`, boot counter increments across reboots
- [ ] 9.2 Software reset / induced panic: unflushed lines replayed behind `RTC I replay`
- [ ] 9.3 App OTA keeps the journal and writes `reboot why=ota`; `uploadfs` erases it
- [ ] 9.4 1 h idle run: SNAP every 60 s, `jflush_max` < 100 ms, no flush while steering drives (except after 10 s), no LWS timeout / VESC link_lost / MAVLink loss correlated with flushes; note bytes per hour
- [ ] 9.5 Fill past 128 KB (test build with a small limit): rotation to `/events.1`, download returns `.1` + active in order
- [ ] 9.6 Web card: tail, source filter, download, clear (uk + en)
- [ ] 9.7 VESC: unplug / replug the UART RX wire mid-burst repeatedly; link recovers within one reply period, `resync` counted, `link_lost`/`link_up` snapshots present
- [ ] 9.8 CAN: unplug the LWS connector with the ECU on → `lws_silent … -> reinit` at ≥ 5 s spacing; replug → `first_frame`; ECU off as well → no LWS re-inits; ignition OFF (LWS unpowered) → no `lws_silent`, no re-init; ignition ON again → first frame re-arms
- [ ] 9.9 I2C2: hold SDA low briefly (pull-down via resistor on X15) during operation → `bus_stuck … released=1`, relays and opto inputs recover without reboot; unplugged relay board does not restart `Wire1`
- [ ] 9.10 Loop overrun event appears for an artificial 80 ms delay (test build) and names the phase
