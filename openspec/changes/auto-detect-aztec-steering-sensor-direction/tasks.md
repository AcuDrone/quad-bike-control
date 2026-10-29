## 0. Preconditions
- [x] 0.1 `replace-aztec-steering-sensor-with-bosch-lws`, `redesign-web-ui-service-console` and `add-aztec-steering-rate-mode` are implemented on `aztec` (firmware 1.5.0). This change is archived AFTER all three, because its MODIFIED blocks quote their deltas.
- [x] 0.2 `move-aztec-vesc-to-can` (1.6.0) is independent of this change: it modifies none of the same requirements and keeps its own version.

## 1. Pure logic (host-testable)
- [x] 1.1 Create `include/SteerSensorDir.h` (no Arduino / `Constants.h` dependency):
  - `steerSensorPosition(angleRaw, inv)` → `inv ? +raw : −raw` (int32);
  - `steerSensorRate(speedDegS, cw, inv)` → `(cw ? +s : −s)`, negated when `inv`;
  - `enum class SteerDirAction { STORE, TOGGLE_AND_STORE, REFUSE_TOO_CLOSE, REFUSE_CONTRADICTS }`, `struct SteerDirResult { action, inv, pos, clearOpposite }`;
  - `steerDirCapture(left, pos, inv, oppositeSet, minSpan)` (design D2: span check first, then side, then toggle only while the opposite is unset).
- [x] 1.2 Add `test/test_steer_sensor_dir/test_main.cpp` (Unity). Cases:
  - normal mount: left capture (−400, inv=false) → STORE; right capture (+400) → STORE;
  - mirrored mount detected left-first: left at +400, inv=false, right unset → TOGGLE_AND_STORE, inv=true, pos=−400, clearOpposite;
  - mirrored mount detected right-first: right at −400, left unset → TOGGLE_AND_STORE, pos=+400;
  - second capture confirms: after a left-first toggle (inv=true, left set), right at +400 → STORE, inv stays true;
  - second capture contradicts: inv=true, left set, right at −400 → REFUSE_CONTRADICTS, inv unchanged;
  - below span: ±(minSpan−1) on either button and either sign → REFUSE_TOO_CLOSE, no toggle; exactly ±minSpan on the expected side → STORE;
  - no toggle when the opposite limit is already set (normal mount, both set, left at +400 → REFUSE_CONTRADICTS);
  - `steerSensorPosition` / `steerSensorRate` sign table for inv false/true.
- [x] 1.3 Add the suite to `[env:native]` `test_filter` in `platformio.ini`, one suite per indented line (the single-line form silently skips suites):
  ```ini
  test_filter =
      test_lws305
      test_steer_rate
      test_steer_sensor_dir
  ```

## 2. Sensor sign (`LwsSensor.h/.cpp`)
- [x] 2.1 Add `setInverted(bool)` / `isInverted()` (default false). `position()` and `rateDegS()` use the `SteerSensorDir.h` helpers. `angleRaw()` and the direction bit stay raw.
- [x] 2.2 Update the Doxygen comments of `position()` / `rateDegS()`. Confirm that `test_lws305` passes unchanged with the default flag.

## 3. Steering controller (`SteeringController.h/.cpp`)
- [x] 3.1 Add `inv_` (RAM copy), `isSensorInverted()`, and `lastCaptureToggled_` with an accessor. Push `inv_` into `lws_` after `loadCalibration()` and on every toggle.
- [x] 3.2 Rewrite `captureLimit()` per design D3:
  - use `steerDirCapture()`;
  - refusal texts "too close to zero" and the "contradicting the stored … limit — … Set sensor zero, capture both limits" text;
  - on a toggle: apply the flag, clear the opposite limit (RAM + NVS), reset the stall/progress baselines, log `[STEER] sensor direction: mirrored|normal (inferred from the <side> limit capture) — capture the <other> limit again`;
  - then store as today.
- [x] 3.3 First-frame log: keep the raw sensor angle and sign, and add `inv=<0|1>`. The `[STEER] Initialized` line: say "pos = ∓angle per sensor direction" instead of "pos = -angle".
- [x] 3.4 Re-check the sign audit (design D5) against the final code. Add a comment at the closed-loop `setSpeed()` noting that it relies on the sensor sign (flag) matching the motor sign.
- [x] 3.5 Update the class Doxygen header (direction flag, inference, no ping-pong).

## 4. NVS (`SteeringController::loadCalibration()/saveLimit()/eraseLimits()`)
- [x] 4.1 `STEER_CAL_VERSION` = 4, key `inv` (bool).
- [x] 4.2 v3 → v4: keep `lim_l` / `lim_r`, write `inv` = false and `cal_ver` = 4. Log `NVS calibration v3 -> v4: limits kept, sensor direction normal`.
- [x] 4.3 Any other version: the existing key-by-key removal, plus `inv`; write `cal_ver` = 4 and `inv` = false. Never `clear()` (keeps `steer_sca_base`).
- [x] 4.4 Toggle write order in one `Preferences` session: the limit, then `inv`, then remove the opposite key. `eraseLimits()` keeps `inv`.
- [x] 4.5 The boot line `[STEER] Calibration loaded: …` includes `sensor direction: normal|mirrored`.

## 5. Capture replies, telemetry and web UI
- [x] 5.1 `VehicleController::processSteerCalCommand()`:
  - on success with `lastCaptureToggled_`, append " — sensor direction detected: mirrored|normal — capture the <other> limit again" instead of "capture the other limit too";
  - pass refusals through unchanged.
- [x] 5.2 `TelemetryManager`: add `steer_sensor_inv`. `WebPortal.cpp`: emit `doc["steer_sensor_inv"]` (always a bool). Re-check the JSON capacity comment.
- [x] 5.3 `data/index.html` "Кермо: стан": the Sensor tile shows " · дзеркально" / " · mirrored" (key `steer_sensor_mirrored`, muted) while `steer_sensor_inv` is true, and nothing otherwise.
- [x] 5.4 `data/index.html` Калібрування steering card: the status line includes the marker.
- [x] 5.5 Help (ⓘ) en/uk:
  - Calibration: direction inferred at the first limit capture; after a re-mount, Set sensor zero and capture both limits; check right after the first capture that Jog ▶ makes the angle grow;
  - "Кермо: стан": the meaning of "mirrored".
- [x] 5.6 i18n parity: every new key exists in both `en` and `uk`.

## 6. Docs
- [x] 6.1 `include/Constants.h`:
  - SIGN bullet: pos = −angle (normal) / +angle (mirrored, NVS `inv`), inferred at limit capture;
  - NVS bullet: layout v4 = `cal_ver`, `lim_l`, `lim_r`, `inv`, v3 kept, older wiped;
  - `STEER_CAL_MIN_SPAN` comment: also the minimum angle that may toggle the direction.
- [x] 6.2 `WEB_PORTAL_SETUP.md` Calibration paragraph: one sentence on direction inference, the re-mount procedure and the Jog ▶ check.
- [x] 6.3 `MAVLINK_SETUP.md`: confirm that nothing needs changing (it only describes the firmware sign).

## 7. Version and changelog
- [x] 7.1 `include/Constants.h`: `FIRMWARE_VERSION` "1.5.0" → "1.5.1".
- [x] 7.2 `include/Changelog.h`: new first entry `{"version":"1.5.1","date":"2026-10-29"}`:
  - uk: "Напрямок датчика кута визначається автоматично при калібруванні лімітів; дзеркальний монтаж більше не потребує перемонтажу", "Після першого ліміту перевірте: Jog ▶ збільшує кут", "Повторне калібрування не потрібне"
  - en: "Steering sensor direction is detected at limit capture; a mirrored mount no longer needs re-fitting", "After the first limit, check that Jog ▶ increases the angle", "No recalibration required"

## 8. Build and test
- [x] 8.1 `pio run` builds without new warnings.
- [x] 8.2 `pio test -e native` passes `test_lws305`, `test_steer_rate` and `test_steer_sensor_dir`, and all three suites are listed as run.
- [x] 8.3 `pio run -t buildfs` (LittleFS image with the updated `index.html`).
- [x] 8.4 `openspec validate auto-detect-aztec-steering-sensor-direction --strict`.

## 9. Bench verification [USER/MANUAL]
- [ ] 9.1 [USER/MANUAL] v3 → v4: flash 1.5.1 on a vehicle calibrated under 1.5.0 (normal mount). The serial log shows `v3 -> v4: limits kept`, the steering is still calibrated, `steer_sensor_inv` = false, and percent steering works as before.
- [ ] 9.2 [USER/MANUAL] Mirrored sensor:
  - jog straight ahead, Set sensor zero, jog to the left lock, Set Left;
  - the log shows `[STEER] sensor direction: mirrored`, and the reply says "sensor direction detected: mirrored — capture the right limit again";
  - the left limit is negative.
- [ ] 9.3 [USER/MANUAL] After 9.2, Jog ▶: `steer_angle` grows (and `steer_rate` > 0) while the wheels turn right.
- [ ] 9.4 [USER/MANUAL] Jog to the right lock, Set Right → calibration complete. Percent steering (web slider / test buttons, then the MAVLink stick) drives the correct way and stops on target. `steering_pct` / `STEER_POS` are + to the right.
- [ ] 9.5 [USER/MANUAL] Діагностика → "Кермо: стан": the Sensor tile shows "дзеркально" (uk) / "mirrored" (en), and the calibration status line shows it too. Reboot: the flag and both limits persist.
- [ ] 9.6 [USER/MANUAL] Contradiction: with both limits set, jog toward the right lock past the span and press Set Left → refused with the "contradicting the stored right limit … Set sensor zero" text. Limits and flag are unchanged.
- [ ] 9.7 [USER/MANUAL] Re-mount the sensor normally, then Set sensor zero → limits erased, flag still mirrored. Set Left → the log shows `sensor direction: normal`. Set Right completes. `steer_sensor_inv` = false and the marker is gone.
- [ ] 9.8 [USER/MANUAL] Jog / Nudge and Set sensor zero work before any capture on a mirrored mount, and rate mode (sensor unplugged, MANUAL) is unaffected.
- [ ] 9.9 [USER/MANUAL] The web "What's new" shows 1.5.1, and the sidebar shows firmware 1.5.1.
