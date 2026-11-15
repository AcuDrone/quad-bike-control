## 0. Preconditions
- [x] 0.1 Proposal approved by the owner (2026-11-15).
- [x] 0.2 Archive order: `add-gcs-vehicle-commands` is archived before this change (its "GCS Vehicle Command Interlocks" and "Ignition State Broadcast" are MODIFIED here). Done 2026-11-15: archived as `archive/2026-11-15-add-gcs-vehicle-commands` on `master`; both requirements are now in `specs/vehicle-systems` and `specs/mavlink-interface`, and the copies here match them apart from this change's own dwell / engine-RPM edits (checked 2026-11-15). If the main specs change again before this change is archived, re-copy both requirements and re-apply those edits.
- [x] 0.3 Work starts on `master` (tasks 1–5); `aztec` and `old-board` get it only by merging `master` (task 6).

## 1. Firmware (`master`)
- [x] 1.1 `include/Constants.h`: `ACC_PRECRANK_DWELL_MS` 2000 → 3000 with the comment "default; per vehicle from the web (NVS crank/dwell_ms)"; new `CRANK_DWELL_MS_MIN` (1000), `CRANK_DWELL_MS_MAX` (8000), `CRANK_NVS_NAMESPACE` ("crank"), `CRANK_DWELL_NVS_KEY` ("dwell_ms"), with the range reasoning (1 s minimum ECU wake-up, 8 s beyond any fuel-pump prime); bump `FIRMWARE_VERSION` to "1.5.33".
- [x] 1.2 `include/CrankLaw.h` (header-only, no Arduino): `crankDwellMsValid(ms)`, `crankDwellMsOrDefault(stored)`, `crankDwellElapsed(ignitionHighSinceMs, nowMs, dwellMs)` (0 sentinel → false, unsigned subtraction), `enum class CrankArmedAction { WAIT, SUPPRESS, ENGAGE }` + `crankArmedDecide(rpm, runningThreshold, ignitionHighSinceMs, nowMs, dwellMs)` (SUPPRESS checked first), `crankRpm(valid, rpm)`, `ignitionWentOff(wasOn, isOn)`.
- [x] 1.3 `RelayController`: member `precrankDwellMs_` (init `ACC_PRECRANK_DWELL_MS`), `setPrecrankDwellMs(uint16_t)` (RAM only, clamped through `crankDwellMsOrDefault`), `getPrecrankDwellMs()`; the armed branch of `update()` (`src/RelayController.cpp:476-490`) uses `crankArmedDecide(engineRpm, ENGINE_RUNNING_RPM_THRESHOLD, ignitionHighSince_, millis(), precrankDwellMs_)` with the same logs and state changes; the "Pre-crank dwell satisfied" log prints the dwell in ms. Header comments of `requestCrank()` / `update()` name the configured dwell instead of `ACC_PRECRANK_DWELL_MS`. Nothing else moves.
- [x] 1.4 `CANController::clearEngineRpm()`: `vehicleData_.engineRPM = 0` only (`dataValid`, `lastUpdateTime` and the other fields untouched); comment why (design D4).
- [x] 1.5 `VehicleController` (deviation: there is no `VehicleController::begin()`; the value is loaded in `initCAN()`, where the other VehicleController-owned NVS settings — `"brake"`, `"boost"`, `"speed"` — are loaded at boot):
  - `crankDwellMs_` loaded in `begin()` from NVS "crank"/`dwell_ms` (missing → default; out of range → logged, default, not rewritten) and pushed with `relayController_.setPrecrankDwellMs()`; `getCrankDwellMs()`; `setCrankDwellMs()` persists first, then applies (false on NVS failure);
  - top of `update()` (the block that reads the ignition state for `setLwsPowered()`): keep `prevIgnitionOn_`; on `ignitionWentOff()` call `canController_.clearEngineRpm()` and log one `[VEHICLE]` line;
  - `relayController_.update(crankRpm(canData.dataValid, canData.engineRPM))` at `src/VehicleController.cpp:298`;
  - `set_crank_dwell_ms` → `processSetCrankDwellCommand()`: round to a whole ms, validate 1000–8000 (NaN refused), `setCrankDwellMs()`; replies "Pre-crank delay set to N ms" / "Pre-crank delay out of range (1000-8000 ms)" / "Failed to save the pre-crank delay"; accepted from any input source.
- [x] 1.6 Update the RC ignition comment in `processMavlinkCommands()` (`:780-781`) and the GCS CRANK log to name the configured dwell.

## 2. Telemetry and web (`master`)
- [x] 2.1 `crank_dwell_ms` (int) in `WebPortal::Telemetry`, `TelemetryManager`, `createTelemetryJSON`; capacity comment updated (+1 slot, 9216 unchanged).
- [x] 2.2 `data/index.html`, "Vehicle Ignition & Lighting" card, right under the OFF / ACC / START buttons: label "Затримка стартера, с" / "Pre-crank delay, s", number input (`id="crank-dwell-input"`, min 1, max 8, step 0.5, inputmode decimal) + Save → `saveCrankDwell()` sending `set_crank_dwell_ms` with `Math.round(value × 1000)`; local validation 1–8 s with a translated message; hint "Запалювання ввімкнене N,N с перед стартером (RC і GCS; кнопка ПУСК — без затримки)" / "Ignition on for N.N s before the starter (RC and GCS; the START button has no delay)"; filled from `crank_dwell_ms` when not being edited; reply line `crank-dwell-status` fed by replies starting with "Pre-crank delay" / "Failed to save the pre-crank delay"; help key `crank_dwell_help` added to the card's ⓘ list; uk + en keys (`lbl_crank_dwell`, `crank_dwell_hint`, `crank_dwell_help`, `invalid_crank_dwell`).

## 3. Tests (`master`)
- [x] 3.1 `test/test_crank_law/test_main.cpp` + an indented `test_filter` line in `platformio.ini` (and its header comment), named cases:
  - `test_dwell_valid_bounds`: 999 / 8001 / 0 rejected, 1000 / 3000 / 8000 accepted;
  - `test_dwell_or_default`: 0, 999, 8001, 65535 → 3000; 1500 → 1500;
  - `test_dwell_elapsed_sentinel`: `ignitionHighSince` 0 → never elapsed;
  - `test_dwell_elapsed_boundary`: at 3000 ms dwell, 2999 ms → false, 3000 ms → true;
  - `test_dwell_elapsed_wraparound`: since = 0xFFFFF000, now = 0x00000400 (5120 ms later) → elapsed at 3000;
  - `test_armed_wait_engage`: rpm 0, before the dwell → WAIT, after → ENGAGE; with dwell 1000 vs 8000 at 2 s → ENGAGE vs WAIT;
  - `test_armed_suppress_first`: rpm 1500 at threshold 1500 → SUPPRESS before and after the dwell; 1499 → not SUPPRESS; threshold 800 (aztec) with rpm 800 → SUPPRESS;
  - `test_crank_rpm_validated`: (false, 1600) → 0; (true, 1600) → 1600; (true, 0) → 0;
  - `test_stale_rpm_after_off`: the bug sequence on values — stored 1600 valid, `ignitionWentOff(true,false)` → clear → `crankRpm(true, 0)` = 0 → `crankArmedDecide` → WAIT then ENGAGE;
  - `test_ignition_went_off_edge`: (true,false) → true; (false,false), (false,true), (true,true) → false.
- [x] 3.2 `pio test -e native` green; `pio run -e esp32-s3-devkitc-1` builds.

## 4. Docs and release (`master`)
- [x] 4.1 `include/Changelog.h` 1.5.33 entry (uk + en, operator wording): the delay before the starter is set in the ignition card, default 3 s (was 2 s), applies to the RC switch and the GCS; the auto-crank is no longer skipped right after switching the engine off at high RPM.
- [x] 4.2 `MAVLINK_SETUP.md`: ignition row of the `MAV_CMD_USER_2` table ("2 s pre-crank" → "pre-crank delay set in the web, 1–8 s, default 3 s since 1.5.33") and the `IGN_STATE` text if it names 2 s; `EFI_STATUS.rpm` note that it reads 0 right after ignition OFF.
- [x] 4.3 `../../API/README.md` (UGV/API): version-history line for 1.5.33 (configurable pre-crank delay → `IGN_STATE` 3 lasts the set delay; `EFI_STATUS.rpm` 0 after OFF), as for every `MAVLINK_SETUP.md` change.
- [x] 4.4 `WEB_PORTAL_SETUP.md` "Ignition & lighting" bullet: mention the pre-crank delay row.
- [x] 4.5 `openspec validate add-precrank-dwell-setting --strict` passes.

## 5. Bench — quad (`master` 1.5.33)
- [ ] 5.1 Upload the LittleFS image; the row shows 3.0 on first boot; 0.5 and 9 are refused locally, `set_crank_dwell_ms` 999 / 8001 refused by the firmware; set 4.5, reboot, it reads back 4.5.
- [ ] 5.2 Stale RPM: run the engine above 1500 RPM, switch OFF with the RC switch, flip straight back to ON (< 5 s): the log shows the RPM clear at OFF, no "Crank suppressed", the starter engages after the set delay. Repeat after waiting > 5 s with the switch OFF. Repeat with the GCS ignition OFF → ON and with the web OFF → START.
- [ ] 5.3 Find the delay that starts reliably: OFF → ON at 2 / 3 / 4 s (and more if needed), at least five attempts each, cold and warm; record the value and leave it stored.
- [ ] 5.4 During cranking the `[CAN] RPM=` log keeps updating (ECU answers while the starter turns); note whether the engine flares above 1500 on catch or the starter runs to the 5 s timeout.
- [ ] 5.5 Engine running, flip the RC switch OFF → ON quickly enough that the ECU stays up: still no re-crank of a running engine (RPM ≥ 1500 → "Crank suppressed").

## 6. Merges
- [x] 6.1 Merge `master` → `aztec`: in `VehicleController::update()` keep `relayController_.update(getEngineRpm())`; take the clear at OFF, the setting, `CrankLaw.h` and the web row; `Constants.h` keeps `ENGINE_RUNNING_RPM_THRESHOLD` 800 and gets 1.5.33; `Changelog.h` 1.5.33 above the aztec 1.5.32 entry ("багі"); `platformio.ini` keeps the aztec-only suites. `pio test -e native`, `pio run` on `aztec`.
- [ ] 6.2 Merge `master` → `old-board`: GPIO `RelayController` — apply the setter and the `crankArmedDecide()` call to its armed branch (`src/RelayController.cpp:183-197`) by hand; `VehicleController.cpp:229` → `crankRpm(...)`; keep the direct-GPIO relays and inputs; resolve `index.html` / `TelemetryManager.cpp` by hand; 1.5.33. `pio test -e native`, `pio run` on `old-board`.

## 7. Bench — buggy and old board, archive
- [ ] 7.1 Buggy (`aztec` 1.5.33): LittleFS upload; 5.1, 5.2 (threshold 800, tacho fitted: confirm the effective RPM source after OFF is TACHO 0, not a stale CAN value), 5.3; record the buggy's delay.
- [ ] 7.2 Old board (`old-board` 1.5.33): LittleFS upload; 5.1 and 5.2 on the bench harness.
- [ ] 7.3 Archive this change (the dependency `add-gcs-vehicle-commands` was archived 2026-11-15, task 0.2).
