Markup / i18n / JS source: the approved mockup
`/private/tmp/claude-501/-Users-s-zalozniy-Documents-SourceCode-UGV-firmware-quad-bike-control/dbc165e1-da1a-49e4-bd00-bef04142603f/scratchpad/ui-mockups/ppr-mockup.html`,
built from `data/index.html` by `.../scratchpad/apply_ppr.py` then `.../scratchpad/split_cards.py`
(same directory). Carry their DOM, i18n strings and JS into the real page **minus the mockup-only
parts** listed in 3.6. The scratchpad is temporary. If the scripts are gone, design D3–D5 and the
lists below are authoritative.

## 0. Preconditions
- [x] 0.1 The proposal is approved (user decisions 2026-11-02: variant A page-side measurement, a number input for the limiter floor, the card split as in the mockup).
- [x] 0.2 Version slot: `master` is at 1.5.3. At apply time take the next free version on `master`: **1.5.4** (patch per `openspec/project.md`: service-console convenience, no recalibration or wiring change), unless `move-can-rx-to-interrupt` (1.6.0) lands first, in which case 1.6.1. Do not take 1.6.0 or 1.7.0, which `move-can-rx-to-interrupt` and `move-aztec-vesc-to-can` claim.
- [x] 0.3 Branch: implement on `master` (quad), then merge `master` into `aztec` (task 8). Done 2026-11-02 (merge 020e9dd, both branches pushed).
- [x] 0.4 Archive order (done 2026-11-02: `add-speed-limiter-integral-floor` archived first): `add-speed-limiter-integral-floor` is archived before this change, because this change MODIFIES "Speed-Limiter Floor Configuration via Web Interface", which only that change adds. Whichever applies second re-checks that its MODIFIED block still quotes the other's text in full.

## 1. Firmware telemetry
- [x] 1.1 `include/VehicleController.h`: add `uint32_t getSpeedPulseTotal() const { return speedSensor_.getPulseTotal(); }` next to `getSpeedPulsesPerRev()`, with a one-line Doxygen comment: "running pulse total since boot, for the web ppr measurement; counts up in either rotation direction".
- [x] 1.2 `include/TelemetryManager.h` / `src/TelemetryManager.cpp`: add `uint32_t speed_pulse_total` to the telemetry struct and fill it from `getSpeedPulseTotal()` in the hall speed sensor block (outside any CAN gate).
- [x] 1.3 `src/WebPortal.cpp` `createTelemetryJSON()`: `doc["speed_pulse_total"] = telemetry.speed_pulse_total;` next to `speed_ppr` (a plain unsigned integer, not `serialized`). Update the capacity comment: 15 → 16 hall-speed / limiter / distance fields (one more slot, key stored by pointer, ~30 B of JSON). Re-check that 4096 still leaves room for the transient `probe` object; no capacity change is expected.
- [x] 1.4 No command, no NVS key, no change to `SpeedSensor` (PCNT config, overflow accumulation and `pulseTotal_` stay as they are; design D2).

## 2. Web: card split (`split_cards.py`)
- [x] 2.1 Card 1: give the existing card `id="speed-sensor-card"`; it keeps the ppr input + Save, the circumference input + Save and `#speed-status-msg`. Its ⓘ calls `openHelp('speed_sensor_help', 'card_speed_sensor')`.
- [x] 2.2 Card 2 `id="speed-limit-card"`, directly after card 1 in the Calibration grid, title `card_speed_limit_steer` ("🚦 Speed limit and steering" / "🚦 Обмеження швидкості та кермо"), ⓘ `openHelp('speed_limit_help,speed_limiter_hint,speed_limit_floor_hint,steer_scale_hint,steer_test_speed_hint', 'card_speed_limit_steer')`. Move into it unchanged: the limiter floor number input + Save (first block, `margin-top: 14px`), the speed-limit row and the `speed-limit-ceil-row` (saturation badge), steering authority, scaling base rows, reason, test-speed row, scaling base input, bench test speed input. Add `#speed-limit-status-msg` at the bottom. Drop the mockup comment line.
- [x] 2.3 `speedStatusMsg()` writes to `let speedStatusTarget = 'speed-status-msg'`. `saveSpeedPpr`, `saveSpeedCirc`, `pprMeasSave` set it to `speed-status-msg`; `saveSpeedLimitFloor`, `saveSteerScaBase`, `sendSteerTestSpeed` set it to `speed-limit-status-msg` before any message.
- [x] 2.4 `handleWebSocketMessage()` reply branch: also match `Steering scale base` and `Test speed`. Today those firmware replies are never shown (design D5).
- [x] 2.5 All `data-bind` names stay. The Diagnostics Speed card and its shared rendering path are untouched. The update of `speed-limit-floor-input` from telemetry keeps working (same id).

## 3. Web: measurement block (`apply_ppr.py`)
- [x] 3.1 Insert the `#ppr-meas` block after the ppr input row in card 1: title, turns input `#ppr-meas-turns` (1–20, default 5), `#ppr-meas-start` (Start/Stop), `#ppr-meas-cancel`, live `#ppr-meas-live` with `#ppr-meas-count` and `#ppr-meas-hint`, result box `#ppr-meas-result-box` with `#ppr-meas-result`, `#ppr-meas-warn`, `#ppr-meas-save`, `#ppr-meas-again`. Use the existing inline style conventions as in the script.
- [x] 3.2 Pure helper `pprMeasCompute(delta, turns)` → `{kind: 'none'|'range'|'ok', ppr, rounded, warn}` per design D3. Constants `PPR_MIN = 1`, `PPR_MAX = 1000` (comment: mirror `SPEED_PPR_MIN` / `SPEED_PPR_MAX`), warning rule `5 * Math.abs(delta - rounded * turns) >= turns`. No `Math.max(1, …)`.
- [x] 3.3 State machine per design D4: `idle` / `measuring` / `done`. `updateTelemetry()` stores the last numeric `speed_pulse_total` (else `null`) and, while measuring, updates the delta `(total - start) >>> 0`. Stop freezes the delta from the last received total at once and moves to `done`, with no settle delay (user decision 2026-11-02). `pprMeasRender()` drives the button texts, colours and visibility as in the mockup. Start is disabled unless `wsConnected` and the last total is numeric. The turns input is locked unless `idle`.
- [x] 3.4 Result rendering: `ok` → `ppr_meas_result` (one decimal via `pprFmt`, comma in uk), warning per `warn`, Save shown; `none` → `ppr_meas_no_pulses` in danger colour, Save hidden; `range` → the division line followed by `invalid_speed_ppr` in danger colour, Save hidden.
- [x] 3.5 `pprMeasSave()` writes `rounded` into `#speed-ppr-input`, sets `speedStatusTarget = 'speed-status-msg'` and calls `saveSpeedPpr()`, so validation, `sendCommand('speed_cal_ppr', …)` and the "Saving…" message are shared. The block stays in `done`.
- [x] 3.6 Remove the mockup-only parts: the simulated counter (`setInterval` in `pprMeasToggle`, `pprMeas.timer`, `pprMeas.total` as a fake), the "simulate 350" link `#ppr-meas-sim350` and `pprMeasSim350()`, the force-enable loop appended to `updateControlsAvailability()`, the on-load telemetry seed / `showSection('calib')` / `scrollIntoView` override, the `#ppr-meas-status` element and `savedMsg`, every `MOCKUP` comment. The page still starts on the remembered section.
- [x] 3.7 Hooks: `ws.onclose` resets the block to `idle` and re-renders it; `updateConnectionStatus()` re-renders it (Start follows the connection); `setLanguage()` re-renders it after `renderChangelog()`. The block is not added to the take-control gating in `updateControlsAvailability()` (matches `speed_cal_ppr`).
- [x] 3.8 No `localStorage` for the measurement.

## 4. Help and i18n
- [x] 4.1 `speed_sensor_help` (en/uk): append the three procedure sentences from `apply_ppr.py`, adjusted to match the hint (user decision 2026-11-02):
  - the second sentence starts with lifting the wheel and selecting neutral: en "Lift the wheel and select neutral, then turn it by hand exactly that many turns in one direction; wait until the count stops changing, then press Stop." / uk "Підніміть колесо й увімкніть нейтраль, потім прокрутіть його рукою рівно стільки обертів в один бік; дочекайтеся, поки лічильник перестане змінюватися, і натисніть «Зупинити».";
  - add "Turning back also adds pulses." / "Оберт назад теж додає імпульси." (design D2).
- [x] 4.2 New keys in both `en` and `uk`, text from the scripts: `card_speed_limit_steer`, `speed_limit_help`, `ppr_meas_title`, `ppr_meas_turns`, `ppr_meas_start`, `ppr_meas_stop`, `ppr_meas_cancel`, `ppr_meas_live`, `ppr_meas_hint`, `ppr_meas_result`, `ppr_meas_frac_warn`, `ppr_meas_save`, `ppr_meas_again`, `ppr_meas_no_pulses`. Not carried over: `ppr_meas_saved`, `ppr_meas_sim350`.
- [x] 4.2a `ppr_meas_hint` replaces the mockup text (user decision 2026-11-02; `%s` = turns):
  - en: "Lift the wheel, select neutral; turn it by hand exactly %s turns in one direction (mark on the tyre). Wait until the count stops changing, then press Stop."
  - uk: "Підніміть колесо, нейтраль; крутіть рукою рівно %s обертів в один бік (мітка на шині). Дочекайтеся, поки лічильник перестане змінюватися, і натисніть «Зупинити»."
- [x] 4.3 EN/UK key parity checked programmatically (extract the `I18N` object with node and compare key sets; every `data-i18n` / `data-i18n-title` attribute resolves in `en`). Not by eye.

## 5. Docs
- [x] 5.1 `WEB_PORTAL_SETUP.md` Calibration list: replace the "Speed sensor" bullet with two bullets: **Speed sensor** (pulses per revolution with the measurement, wheel circumference) and **Speed limit and steering** (limiter floor, speed-limit readout, steering authority, scaling base, bench test speed).
- [x] 5.2 `WEB_PORTAL_SETUP.md`: rename "Limiter floor (Speed sensor card)" to "(Speed limit and steering card)". Add a short "Measuring pulses per revolution" paragraph covering:
  - the procedure (wheel off the ground, transmission in neutral, chalk mark, N turns in one direction, Stop);
  - `speed_pulse_total`;
  - the 0.2 warning;
  - that reverse turning adds pulses;
  - that the turns add to the odometer/trip;
  - that nothing is saved until "Save as pulses/rev".

## 6. Version and changelog
- [x] 6.1 `include/Constants.h`: `FIRMWARE_VERSION` "1.5.3" → **"1.5.4"** (or per task 0.2).
- [x] 6.2 `include/Changelog.h`: new first entry `{"version":"1.5.4","date":"<implementation date>"}`:
  - uk: "Імпульси на оберт колеса тепер можна виміряти з веб-панелі (прокрутити колесо N обертів); картка датчика швидкості розділена на «Датчик швидкості» та «Обмеження швидкості та кермо»"
  - en: "Wheel pulses per revolution can now be measured from the web portal (turn the wheel N times); the speed sensor card is split into \"Speed sensor\" and \"Speed limit and steering\"" (escape the quotes inside the raw-string JSON as needed, or use typographic quotes)

## 7. Build and test
- [x] 7.1 `pio run` builds without new warnings on `master`.
- [x] 7.2 `pio test -e native`: all 44 tests pass, unchanged (no firmware logic is host-testable here; design D6).
- [x] 7.3 `pio run -t buildfs` (LittleFS image with the updated `index.html`).
- [x] 7.4 Extract the page script and run `node --check` on it. In the browser console (or node, on the extracted `pprMeasCompute`), check this table (no JS test framework is added):

  | delta | turns | kind | ppr | rounded | warn |
  |------:|------:|------|----:|--------:|------|
  | 350 | 5 | ok | 70.0 | 70 | no |
  | 351 | 5 | ok | 70.2 | 70 | yes (boundary) |
  | 349 | 5 | ok | 69.8 | 70 | yes (boundary) |
  | 352 | 5 | ok | 70.4 | 70 | yes |
  | 353 | 5 | ok | 70.6 | 71 | yes |
  | 1400 | 20 | ok | 70.0 | 70 | no |
  | 1401 | 20 | ok | 70.05 | 70 | no |
  | 141 | 2 | ok | 70.5 | 71 | yes |
  | 700 | 1 | ok | 700 | 700 | no |
  | 0 | 5 | none | — | — | — (no Save) |
  | 2 | 5 | range | 0.4 | 0 | — (no Save) |
  | 1001 | 1 | range | 1001 | 1001 | — (no Save) |
  | `(150 - 4294967196) >>> 0` = 250 (start 4294967196, now 150) | 5 | ok | 50.0 | 50 | no |
- [x] 7.5 i18n parity check (task 4.3) passes.
- [x] 7.6 `openspec validate add-speed-ppr-measurement --strict`.

## 8. Merge
- [x] 8.1 (done 2026-11-02, merge 020e9dd on `aztec`) Merge `master` into `aztec` (buggy) and check that `pio run` and `pio run -t buildfs` build there without new warnings. The change is vehicle-neutral; resolve only branch-specific context (Changelog / version line, aztec-only cards such as Gear sensor codes around the split).

## 9. Bench verification [USER/MANUAL]
Archived 2026-11-02 at the user's request before bench verification; bugs found later come back as new changes.
- [ ] 9.1 [USER/MANUAL] Quad, wheel off the ground, neutral: Start with 5 turns, turn the wheel by hand exactly 5 turns in one direction, Stop. The live count rises during the turns. Wait until it stops changing, then press Stop: the result freezes at once. The result is 350 / 5 = 70.0 → 70, with no warning. Repeat at a very slow hand speed: same result (confirms the sensor switches at hand speed).
- [ ] 9.2 [USER/MANUAL] Deliberately turn 4.5 or 5.5 turns with 5 entered: the fractional-part warning is shown.
- [ ] 9.3 [USER/MANUAL] Reverse rotation: turn 5 turns backwards → still ≈ 350 (counts up). Turn 5 forward then 1 back: the count is ≈ 420, not 280. Confirm the behaviour matches design D2; if it does not, stop and report before saving anything.
- [ ] 9.4 [USER/MANUAL] Save as pulses/rev → the reply "Pulses/rev set to 70" appears in the Speed sensor card's status line; reboot → the ppr input still shows 70. Cancel and Measure again return to idle without sending anything (no reply appears).
- [ ] 9.5 [USER/MANUAL] Disconnect (WiFi off / reboot) during a measurement → the block returns to idle. Reload the page mid-measurement → idle.
- [ ] 9.6 [USER/MANUAL] Both cards' ⓘ open the right help in en and uk. Saving the limiter floor, scaling base and test speed shows the reply in card 2's status line. Saving ppr / circumference shows it in card 1's. At 375 px wide the two cards stack in one column without horizontal scrolling, and the measurement block's buttons do not overflow.
- [ ] 9.7 [USER/MANUAL] The web "What's new" and the sidebar show 1.5.4.
