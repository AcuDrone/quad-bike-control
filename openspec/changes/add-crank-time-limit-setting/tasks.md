## 0. Preconditions
- [x] 0.1 Proposal approved by the owner (2026-11-17, "роби"): no caution line, no pause / attempt limit, step 0.5 s confirmed (see proposal "Owner decisions").
- [ ] 0.2 Archive order: `add-precrank-dwell-setting` is archived before this change (its versions of "Ignition State Control Integration" and "Ignition State Broadcast" are the base of the MODIFIED copies here). After that archive, compare both requirements in `specs/vehicle-systems` and `specs/mavlink-interface` with the copies here; if anything else changed, re-copy and re-apply only this change's timeout wording.
- [x] 0.3 Work starts on `master` (tasks 1–5); `aztec` and `old-board` get it only by merging `master` (task 6).

## 1. Firmware (`master`)
- [x] 1.1 `include/Constants.h`: `CRANKING_TIMEOUT` stays 5000 with the comment "default; per vehicle from the web (NVS crank/max_ms)"; new `CRANK_MAX_MS_MIN` (2000), `CRANK_MAX_MS_MAX` (10000), `CRANK_MAX_NVS_KEY` ("max_ms"), with the reasoning (2 s was the old cap and too short for a cold start; 10 s is the owner's ceiling; the literals MUST match `CrankLaw.h`); bump `FIRMWARE_VERSION` to "1.5.36".
- [x] 1.2 `include/CrankLaw.h`: `CRANK_LAW_MAX_MS_MIN` / `_MAX` / `_DEFAULT` (2000 / 10000 / 5000), `crankMaxMsValid(ms)`, `crankMaxMsOrDefault(stored)`, `crankTimedOut(startMs, nowMs, limitMs)` (unsigned subtraction, `>=`); header comment names this change.
- [x] 1.3 `RelayController`: members `crankMaxMs_` (init `CRANKING_TIMEOUT`) and `crankLimitMs_` (the limit of the running attempt); `setCrankMaxMs(uint16_t)` (RAM only, through `crankMaxMsOrDefault`), `getCrankMaxMs()`; `setIgnitionState(CRANKING)` copies `crankMaxMs_` into `crankLimitMs_` beside `crankingStartTime_`; `update()` uses `crankTimedOut(crankingStartTime_, millis(), crankLimitMs_)` with the same log, which now also prints the limit. Header comments (`IgnitionState::CRANKING`, `update()`) name the configured limit instead of `CRANKING_TIMEOUT`. The RPM check stays first; nothing else moves.
- [x] 1.4 `VehicleController`:
  - `crankMaxMs_` loaded in `initCAN()` from NVS "crank"/`max_ms` in the same `prefs.begin(CRANK_NVS_NAMESPACE, true)` block as `dwell_ms` (missing → default; out of range → logged, default, not rewritten) and pushed with `relayController_.setCrankMaxMs()`; boot log line names both values; `getCrankMaxMs()`;
  - `setCrankMaxMs()` persists first, then applies (false on NVS failure); log "[VEHICLE] Starter time limit set to N ms (from the next attempt)";
  - `set_crank_max_ms` → `processSetCrankMaxCommand()`: round to a whole ms, validate 2000–10000 (NaN refused), `setCrankMaxMs()`; replies "Starter time limit set to N ms" / "Starter time limit out of range (2000-10000 ms)" / "Failed to save the starter time limit"; accepted from any input source.

## 2. Telemetry and web (`master`)
- [x] 2.1 `crank_max_ms` (int) in `WebPortal::Telemetry`, `TelemetryManager`, `createTelemetryJSON`; capacity comment: +1 scalar slot (≈ 16 B of memory, ≈ 22 B of JSON), worst case ≈ 8.60 KB, ≈ 0.62 KB under 9216 (size unchanged).
- [x] 2.2 `data/index.html`, card "🚀 Запуск двигуна" / "🚀 Engine start" (Calibration), under the pre-crank delay row: label "Макс. час роботи стартера" / "Max starter time", slider `id="crank-max-input"` `type="range"` min 2, max 10, step 0.5, value on the right (`crank-max-val`, "5,0 с" / "5.0 s"); the same handlers as the delay slider (label live on `input`, one `set_crank_max_ms` with `Math.round(value × 1000)` on `change`, drag guard with `pointerdown` / `pointerup` / `pointercancel`); follows `crank_max_ms` when not held (older firmware without the key: slider keeps 5); hint `crank-max-hint` "Стартер крутить щонайбільше N,N с за спробу; раніше вимикається, щойно двигун завівся (обороти). Зміна діє з наступної спроби" / "The starter runs at most N.N s per attempt and stops earlier once the engine runs (RPM). A change applies from the next attempt"; no caution line (owner decision 2026-11-17); reply line `crank-max-status` fed by replies starting with "Starter time limit" / "Failed to save the starter time limit"; card ⓘ becomes `openHelp('crank_dwell_help,crank_max_help', 'card_engine_start')`; uk + en keys `lbl_crank_max`, `crank_max_val`, `crank_max_hint`, `crank_max_help`. Help text: what the limit is, that the engine-running RPM stops the starter earlier, that it applies to the RC switch, the GCS and the START button, that a change applies from the next attempt, why not to go long (heat, battery, and a running engine cranked to the limit if the RPM reads low), 2–10 s, default 5 s.
- [x] 2.3 The HTML comment above the card and the JS block comment name both settings.

## 3. Tests (`master`)
- [x] 3.1 `test/test_crank_law/test_main.cpp` (existing suite, no `platformio.ini` change), named cases:
  - `test_max_valid_bounds`: 1999 / 10001 / 0 rejected, 2000 / 5000 / 10000 accepted;
  - `test_max_or_default`: 0, 1999, 10001, 65535 → 5000; 7500 → 7500;
  - `test_timed_out_boundary`: limit 5000: 4999 ms → false, 5000 ms → true; limit 10000: 9999 → false, 10000 → true;
  - `test_timed_out_wraparound`: start = 0xFFFFF000, now = 0x00000400 (5120 ms later) → timed out at 5000, not at 10000.
- [x] 3.2 `pio test -e native` green; `pio run -e esp32-s3-devkitc-1` builds.

## 4. Docs and release (`master`)
- [x] 4.1 `include/Changelog.h` 1.5.36 entry (uk + en, operator wording): the maximum starter time per attempt is set in Calibration → Engine start, 2–10 s, default 5 s as before; applies to the RC switch, the ground station and the START button; a change applies from the next attempt; why not to go long, without a warning line (in Ukrainian the buggy is "багі").
- [x] 4.2 `MAVLINK_SETUP.md`: ignition row of the `MAV_CMD_USER_2` table ("starter ≤ 5 s" → "starter ≤ the max starter time set in the web, 2–10 s, default 5 s since 1.5.36"); the `IGN_STATE` row: 3 lasts the delay plus at most the set starter time.
- [x] 4.3 `../../API/README.md` (UGV/API): `IGN_STATE` and `param1` 2 rows — version note "макс. час стартера з веб — 1.5.36" (2–10 с, типово 5 с).
- [x] 4.4 `WEB_PORTAL_SETUP.md`: the max starter time row under Calibration → "Engine start" (`set_crank_max_ms`, saved to NVS, applies from the next attempt).
- [x] 4.5 `openspec validate add-crank-time-limit-setting --strict` passes.

## 5. Bench — quad (`master` 1.5.36)
- [ ] 5.1 Upload the LittleFS image; Calibration → Engine start shows both sliders; the new one shows 5,0 с on first boot and stops at 2 and 10; no caution line at any value; one reply per release, none while dragging; `set_crank_max_ms` 1999 / 10001 refused by the firmware; set 7, reboot, it reads back 7; the ⓘ shows both help entries.
- [ ] 5.2 Fuel or ignition kill so the engine cannot start: web START with the limit at 2 s → the log shows "Cranking timeout" at ≈ 2 s; at 7 s → ≈ 7 s; RC ON and GCS ignition ON (after the dwell) the same.
- [ ] 5.3 Change the limit while the starter turns (2 s → 10 s during a 2 s attempt): the attempt stops at ≈ 2 s; the next one runs to ≈ 10 s.
- [ ] 5.4 A normal start: the starter still drops on "Engine started!" well before the limit; leave 5 s stored unless the owner picks another value.

## 6. Merges
- [ ] 6.1 Merge `master` → `aztec`: `Constants.h` keeps `ENGINE_RUNNING_RPM_THRESHOLD` 800 and gets 1.5.36; `Changelog.h` 1.5.36 ("багі"); the telemetry document grows to `StaticJsonDocument<10240>` (worst case ≈ 9.07 KB with the new key; the aztec capacity comment already asks for growth on the next addition) and the comment is updated (≈ 1.17 KB headroom). `pio test -e native`, `pio run` on `aztec`.
- [x] 6.2 Merge `master` → `old-board`: GPIO `RelayController` — the latch in `setIgnitionState(CRANKING)` (`:103`) and `crankTimedOut()` at the timeout check (`:218`) by hand where the merge conflicts; 1.5.36. `pio test -e native`, `pio run` on `old-board`.

## 7. Bench — buggy and old board, archive
- [ ] 7.1 Buggy (`aztec` 1.5.36): LittleFS upload; before storing any value above 5 s, start the engine and confirm the idle RPM reads ≥ 800 from TACHO (ppr settled); 5.1–5.4; record the buggy's value.
- [ ] 7.2 Old board (`old-board` 1.5.36): LittleFS upload; 5.1 and 5.2 on the bench harness.
- [ ] 7.3 Archive this change (after `add-precrank-dwell-setting`, task 0.2).
