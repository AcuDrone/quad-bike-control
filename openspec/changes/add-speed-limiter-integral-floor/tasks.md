## 0. Preconditions
- [ ] 0.1 The proposal is approved. Braking stays out of scope.
- [ ] 0.2 Version slot: the brief targets **1.6.0**, but the pending `move-can-rx-to-interrupt` (1.6.0) and `move-aztec-vesc-to-can` (1.7.0) already claim 1.6.0 and 1.7.0. At implementation time, take the next free minor version after what has actually shipped (1.6.0 if this change ships first). Renumber the other two proposals' version tasks if they slip behind it.
- [ ] 0.3 No other pending change modifies "Autopilot-Sourced Maximum-Speed Limit" or "Proportional Speed-Limiter Throttle Taper". The web deltas are ADDED only, so there is no archive-order constraint (the `redesign-web-ui-service-console` "Speed-Limit and Limiter-Ceiling Telemetry" delta is untouched).

## 1. Pure law (host-testable)
- [ ] 1.1 Create `include/SpeedLimiterLaw.h` (header-only, no Arduino / `Constants.h`):
  - `struct SpeedLimiterParams { taperFraction, taperMinMs, taperMaxMs, floorPct, kiPctSPerMs, risePctS, slewPctS, bindMarginPct }`;
  - `struct SpeedLimiterState { float ceilPct; float trimPct; bool primed; }` with `reset()` (100 / 0 / false);
  - `float speedLimiterBand(float limitMs, const SpeedLimiterParams&)`;
  - `float speedLimiterTaper(float speedMs, float limitMs, float bandMs, float floorPct)`;
  - `float speedLimiterStep(SpeedLimiterState&, const SpeedLimiterParams&, float demandPct, float speedMs, float limitMs, float dtS)`. It implements design D3 exactly: first call (`!primed`) adopts the clamped taper with trim 0; dt is capped at 1 s by the caller and clamped again here; it returns `min(demand, ceil)`.
- [ ] 1.2 Add `test/test_speed_limiter/test_main.cpp` (Unity). Cases:
  - band: 2.78 → 0.556, 5.56 → 1.112, 11.1 → 1.4, 1.39 → 0.5 m/s;
  - taper: 100 % at `L − B`, floor at `L`, the midpoint value, the floor above `L`, with floor 0 and floor 10;
  - first call: ceil = taper, trim = 0, no slew applied;
  - over the limit: 1.4 m/s excess, trim from 100 falls to 0 in 2.3–2.5 s of 20 ms steps; a large excess is capped at the slew;
  - under the limit and binding: the trim rises at ≤ 10 %/s; it holds when demand < ceil − 1 %; it resets to 0 at `v ≤ L − B`;
  - clamps: trim ∈ [0, 100 − floor], ceil ∈ [floor, 100]; changing the floor mid-run re-clamps;
  - slew: ceil never moves more than 80 %/s × dt in either direction;
  - steady state on a toy plant (first-order speed response to throttle, `u*` = 30 %): it converges to `L` ± 0.05 m/s; with `u*` = 5 % and floor 10 %, it saturates at the floor (speed > L);
  - multiple calls per loop: two calls of dt/2 give the same result as one call of dt (within 1e-3).
- [ ] 1.3 Add `test_speed_limiter` to `[env:native]` `test_filter` in `platformio.ini`, one suite per indented line (the single-line form silently skips suites).

## 2. Constants (`include/Constants.h`)
- [ ] 2.1 Remove `SPEED_LIMIT_TAPER_BAND_MS` and `SPEED_LIMIT_FLOOR_PCT`. Add the following, each with a one-line rationale comment taken from design D1–D3/D6:
  - `SPEED_LIMIT_TAPER_FRACTION 0.20f`, `SPEED_LIMIT_TAPER_MIN_MS 0.5f`, `SPEED_LIMIT_TAPER_MAX_MS 1.4f`;
  - `SPEED_LIMIT_FLOOR_DEFAULT_PCT 0.0f`, `SPEED_LIMIT_FLOOR_MIN_PCT 0.0f`, `SPEED_LIMIT_FLOOR_MAX_PCT 20.0f`, `SPEED_LIMIT_FLOOR_NVS_KEY "thr_floor"`;
  - `SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS 30.0f`, `SPEED_LIMIT_TRIM_RISE_PCT_S 10.0f`, `SPEED_LIMIT_BIND_MARGIN_PCT 1.0f`;
  - `SPEED_LIMIT_SAT_MARGIN_MS 0.3f`, `SPEED_LIMIT_SAT_HOLD_MS 2000`, `SPEED_LIMIT_SAT_UI_HOLD_MS 10000` (design D6: entry debounce and display hold; there is no saturation log interval).
- [ ] 2.2 Rewrite the "Proportional taper" comment block: proportional band, taper + integral trim, floor default 0 % = calibrated idle and why that is not a hard cut. Keep `SPEED_LIMIT_CEILING_SLEW_PCT_S 80.0f` unchanged.
- [ ] 2.3 Update the NVS comment near the speed calibration defaults: namespace `"speed"` also holds `thr_floor` (owned by `VehicleController`).

## 3. Limiter (`VehicleController.h/.cpp`)
- [ ] 3.1 Replace `limiterCeilingPct_` / `limiterLastMs_` with `SpeedLimiterState limiter_`, `uint32_t limiterLastMs_` and `float limiterFloorPct_` (default `SPEED_LIMIT_FLOOR_DEFAULT_PCT`). Add saturation state: `satSinceMs_`, `satActive_`, `satClearedMs_`. `getSpeedLimitCeilingPct()` returns `limiter_.ceilPct`. Add `getSpeedLimitFloorPct()`, `isSpeedLimitSaturated()` (returns `satActive_`) and `isSpeedLimitSatHeld()` (`satActive_`, or `satClearedMs_ != 0` and `millis() − satClearedMs_ < SPEED_LIMIT_SAT_UI_HOLD_MS`, evaluated at call time so the hold expires even when `applySpeedLimit()` is not running).
- [ ] 3.2 `applySpeedLimit()`:
  - keep `getSpeedLimitMs()`, `logSpeedLimitChange()` (the existing rate-limited limit-change log, unchanged), the no-limit branch and the fail-open branch. Each now calls `limiter_.reset()` and clears the live saturation (`satSinceMs_` = 0; if `satActive_` was true, set it false and set `satClearedMs_` = now so the hold starts and is not erased);
  - otherwise compute dt (capped at 1 s; 0 on the first call) and call `speedLimiterStep()`.
- [ ] 3.3 Saturation detection per design D6, after the step: raw condition `ceil ≤ floor + 0.5` and `v > L + SPEED_LIMIT_SAT_MARGIN_MS`; `satActive_` becomes true once the raw condition has held for `SPEED_LIMIT_SAT_HOLD_MS`; on the first evaluation where it fails, `satActive_` = false and `satClearedMs_` = now. No `Debug::printf` line is emitted for entering or leaving saturation, and no MAVLink message is sent.
- [ ] 3.4 Load `thr_floor` from `"speed"` in `initCAN()` next to `steer_sca_base` (read-only `Preferences`). NaN or out-of-range → log `[SPEED] Stored limiter floor X out of range (0-20 %) — ignored` and use the default. Always log `[SPEED] Limiter floor: X %` at boot.
- [ ] 3.5 `processSetSpeedLimitFloorCommand(float, WebPortal&)`: validate (NaN, 0–20), write `thr_floor` (check the `putFloat` return value; on failure respond with an error and keep the RAM value unchanged), update `limiterFloorPct_`, respond "Speed-limiter floor set to X %". Route `set_speed_limit_floor` in `processWebCommand()` next to `speed_cal_circ`.
- [ ] 3.6 Doxygen:
  - `applySpeedLimit()`: the law, anti-windup, fail-open;
  - the speed-calibration commands block: drop "There is deliberately no limiter command"; say instead that there is no *ceiling* command and that the floor is the one limiter setting.

## 4. Telemetry and web portal
- [ ] 4.1 `TelemetryManager.h/.cpp`: add `speed_limit_floor` (float), `speed_limit_sat` (bool) and `speed_limit_sat_held` (bool), filled from `getSpeedLimitFloorPct()`, `isSpeedLimitSaturated()` and `isSpeedLimitSatHeld()`.
- [ ] 4.2 `WebPortal.cpp`: emit `doc["speed_limit_floor"] = serialized(String(floor, 1))`, `doc["speed_limit_sat"]` and `doc["speed_limit_sat_held"]` (JSON booleans, present in every frame). Re-check the JSON document capacity comment.
- [ ] 4.3 `data/index.html` Speed sensor settings card:
  - a "Limiter floor (%)" input with a Save button (`saveSpeedLimitFloor()`), validated 0–20 locally, sending `set_speed_limit_floor`;
  - synced from `speed_limit_floor` with the existing `syncSpeedInput()` helper.
- [ ] 4.4 `data/index.html` limiting row (both the Diagnostics Speed card and the settings card, through the shared rendering path): append an "at floor" marker while a limit is in force and `speed_limit_ceil` ≤ `speed_limit_floor` + 0.5.
- [ ] 4.4a `data/index.html` saturation badge on the same row, through the same shared rendering path (design D6):
  - show the row while `speed_limit_ceil` < 100 **or** `speed_limit_sat_held` is true;
  - while `speed_limit_sat_held` is true, render the row as a warning badge in the console's standard warning styling (reuse the existing `--warning` styling, as the attention list and `.warning` box do; no new colour or class family);
  - text: `speed_limit_sat` ("Limiter at floor, speed above limit") while `speed_limit_sat` is true; `speed_limit_sat_held` ("Limiter was at floor, speed above limit") while only the hold is active;
  - a missing key is treated as false (older firmware), so the row behaves exactly as today.
  - Do not keep any page-side timer for the hold; the page renders what the firmware reports.
- [ ] 4.5 The ⓘ help (en/uk) of the Speed sensor card: the floor text from the web-control spec, plus one sentence that the limiter now holds the limit itself (integral), and one that the "Limiter at floor, speed above limit" badge (kept for 10 s after it clears) means the floor is too high or the vehicle is going downhill without braking.
- [ ] 4.6 i18n parity: `lbl_speed_limit_floor`, `invalid_speed_limit_floor`, `speed_limit_at_floor`, `speed_limit_sat`, `speed_limit_sat_held` and the help texts exist in both `en` and `uk`.

## 5. Docs
- [ ] 5.1 `WEB_PORTAL_SETUP.md`: the Speed sensor paragraph gets the floor setting and its meaning.
- [ ] 5.2 Check that `MAVLINK_SETUP.md` describes `SPEED_MAX` behaviour without the 7.5 % or 1.4 m/s numbers; update it if it does not.

## 6. Version and changelog
- [ ] 6.1 `include/Constants.h`: `FIRMWARE_VERSION` "1.5.1" → **"1.6.0"** (minor: limiter behaviour change, new setting), or the next free minor per task 0.2.
- [ ] 6.2 `include/Changelog.h`: a new first entry `{"version":"1.6.0","date":"<implementation date>"}`:
  - uk: "Обмежувач швидкості тепер сам утримує ліміт: якщо швидкість вища — газ зменшується далі, аж до холостого ходу", "Мінімальний газ обмежувача налаштовується в картці «Датчик швидкості» (0–20 %, типово 0 %)", "На низьких лімітах газ знімається ближче до ліміту, а не з половини", "Повторне калібрування не потрібне"
  - en: "The speed limiter now holds the limit itself: above it, throttle keeps closing down to idle", "The limiter's minimum throttle is set in the Speed sensor card (0–20 %, default 0 %)", "At low limits, throttle is withdrawn close to the limit instead of from half of it", "No recalibration required"

## 7. Build and test
- [ ] 7.1 `pio run` builds without new warnings on `aztec`.
- [ ] 7.2 `pio test -e native` passes every listed suite, including `test_speed_limiter`, and all of them are reported as run.
- [ ] 7.3 `pio run -t buildfs` (LittleFS image with the updated `index.html`).
- [ ] 7.4 `openspec validate add-speed-limiter-integral-floor --strict`.
- [ ] 7.5 Port to `master` (quad): cherry-pick or re-apply. The law, constants and setting are vehicle-neutral. Resolve only branch-specific context (Changelog / version line).

## 8. Bench verification [USER/MANUAL]
- [ ] 8.1 [USER/MANUAL] First boot on the new firmware: the log shows `[SPEED] Limiter floor: 0.0 %`, and telemetry shows `speed_limit_floor` = 0. Set 5 % from the portal, reboot: 5 % is restored. Set 25 %: refused locally and by the firmware.
- [ ] 8.2 [USER/MANUAL] Wheels off the ground, `SPEED_MAX` = 2.78 (10 km/h), full demand: the ceiling (`speed_limit_ceil`) starts falling only above ≈ 8 km/h, and the wheel speed settles at 10 ± 0.5 km/h with the ceiling at a steady value above the floor. No servo snap is audible or visible.
- [ ] 8.3 [USER/MANUAL] Drive above the limit with the floor forced to 20 % (wheels off the ground or on flat ground, `SPEED_MAX` = 2.78, full demand, 20 % alone holding the speed above 10 km/h): confirm the "Limiter at floor, speed above limit" badge appears on the limiter row in the console's warning styling within ≈ 2 s, and that `speed_limit_sat` is true. Then slow down below the limit: the badge switches to "was at floor" and stays ≥ 10 s after slowing down, then disappears (`speed_limit_sat_held` false). Reload the page during the hold: the badge is still shown. No saturation line appears in the serial log; the `[SPEED] Speed limit: ...` change line still does when `SPEED_MAX` is changed.
- [ ] 8.4 [USER/MANUAL] Hold at the limit for 30 s: the servo does not hunt visibly, and `speed_limit_ceil` varies by ≤ ± 5 %.
- [ ] 8.5 [USER/MANUAL] Lift off to 0 % at the limit, wait 3 s, re-apply full: the ceiling is the taper value for the current speed (no stale low ceiling), and the speed returns to the limit without overshoot > 1 km/h.
- [ ] 8.6 [USER/MANUAL] Unplug the speed sensor while limiting: the fail-open warning appears, `speed_limit_ceil` = 100 and the demand passes through. Set `SPEED_MAX` = 0: `speed_limit_ceil` = 100, no limiting.
- [ ] 8.7 [USER/MANUAL] Field repeat of the original case on the buggy (flat ground, 10 km/h limit, full stick): the speed holds 10 km/h ± 1 km/h. If not, note whether the console showed the saturation badge (and `speed_limit_ceil` / `speed_limit_floor` at the time) before any further tuning.
- [ ] 8.8 [USER/MANUAL] The web "What's new" and the sidebar show the new version.
