# Change: Rate-limit throttle opening (anti-stall), per-vehicle setting from the web

## Why
The quad bike's engine stalls when the throttle is snapped open from idle: the butterfly lets in
air faster than the injection enriches, and the CVT starts pulling load before the RPM rises.
Nothing in the firmware limits how fast the throttle opens — the MAVLink, web and speed-limiter
paths all hand their percentage straight to the servo, which travels idle → full in ≈0.15 s.

## What Changes
- New pure law `include/ThrottleSlewLaw.h` (header-only, no Arduino):
  `throttleSlewStep(targetPct, lastPct, dtS, ratePctS)` → new percent. Rising is limited to
  `lastPct + rate·dt`; falling (and holding) passes through immediately; `rate <= 0` is a
  passthrough. Target is clamped to 0–100 %. A tiny `ThrottleSlewState { lastPct }` with
  `reset(pct)` keeps the state testable.
- Applied once, at the servo input: `ThrottleController::setThrottlePercent()` runs the law
  before `percentToUs()`. Every percent path therefore gets it — MAVLink apply, the web
  `set_throttle` command, the per-loop web re-limit in `update()`, and the speed-limiter output
  (which is the argument of those calls). No caller changes.
- Paths that bypass it, by design:
  - `idle()` (fail-safe, web takeover, gear-change idle hold and boost release, brake override):
    closing is always instantaneous; `idle()` also resets the law's last percent to 0, so the next
    opening ramps from idle.
  - `setThrottleUs()` (gear-boost PID; it already has its own `TRANS_GEAR_BOOST_SLEW_RATE_US`
    = 16 µs/step slew): bypasses the law and re-seeds the last percent from the written pulse
    (`usToPercent`), so a percent command after the PID ramps from where the servo really is.
  - Calibration jog (`jogIdle`/`jogFull`) drives the servo directly; `saveCalibration` /
    `cancelCalibration` return to idle and reset the last percent to 0.
- Timing: `ThrottleController` keeps `lastPct_` and `lastMs_` (0 = "never" sentinel). `dt` is the
  time since the previous write, capped at `THROTTLE_OPEN_SLEW_MAX_DT_S` (0.1 s), so a stalled
  loop or a path that was silent for seconds cannot open the throttle in one step.
- New persisted per-vehicle setting, the opening rate in %/s, in the EXISTING throttle calibration
  namespace `"throttle"` (owned by `ThrottleController`, beside `idle_us`/`full_us`), key
  `open_slew` (float). Constants: `THROTTLE_OPEN_SLEW_DEFAULT_PCT_S` (60),
  `THROTTLE_OPEN_SLEW_MIN_PCT_S` (30), `THROTTLE_OPEN_SLEW_MAX_PCT_S` (100),
  `THROTTLE_OPEN_SLEW_NVS_KEY` ("open_slew"), `THROTTLE_OPEN_SLEW_MAX_DT_S` (0.1).
  Accepted values: **0 = off** (exactly today's behaviour) or 30–100 %/s. A stored NaN or
  out-of-range value is logged and ignored (default used, not rewritten).
- Range reasoning:
  - The engine needs ≈0.5–1 s to come up from idle before it can take load through the CVT, so a
    1–2 s idle → full ramp is the useful window.
  - Upper bound 100 %/s (idle → full in 1 s): in the operator's judgement anything faster brought
    the stall back, so offering it would only invite the fault; 0 still restores today's snap
    exactly when wanted.
  - Lower bound 30 %/s (idle → full in 3.3 s): the slowest rate that is still drivable.
  - Default 60 %/s: idle → full in 1.67 s, and 0 → 30 % (the range where the stall happens) in
    0.5 s.
  - 0 is kept as an explicit off switch, separate from the range, so the operator can restore
    today's behaviour exactly.
- Interaction with the speed limiter: the limiter's ceiling slew (`thr_slew`, default 80 %/s)
  moves the ceiling; this new limit moves the demand that reaches the servo. Both apply in series
  (limiter output → opening slew → servo), so on opening the stricter of the two wins; on closing
  only the limiter's ceiling slew can slow the fall, as today. The "Proportional Speed-Limiter
  Throttle Taper" text that "the driver's own throttle movements are never slowed" is narrowed to
  "never slowed by the limiter" and points at the new opening rate.
- Web command `set_throttle_open_slew` (value %/s, rounded to a whole number): accepted
  regardless of the input source, like `set_speed_limit_slew`; validated (0 or 30–100), persisted
  first, then applied. Replies: "Throttle open rate set to N %/s", "Throttle open rate off",
  "Throttle open rate out of range (0 or 30-100 %/s)", "Failed to save the throttle open rate".
- Telemetry key `throttle_open_slew` (integer %/s, 0 = off).
- Web UI: a row "Плавність газу (%/с)" / "Throttle opening rate (%/s)" under the "Плавність
  обмежувача (%/с)" row in the "Обмеження швидкості та кермо" card — number input 0–100, step 5,
  Save button `saveThrottleOpenSlew()`, local validation, a live hint "0 → 100 % за X.X с"
  (100 / value) or "0 = вимкнено" when 0, filled from telemetry; one help sentence; uk + en; the
  card's reply filter also routes `Throttle open rate …` replies to its status line.
- Native test suite `test/test_throttle_slew` (registered in `platformio.ini` `test_filter`).
- Firmware 1.5.16 with a changelog bullet (uk + en).

## Impact
- Affected specs: vehicle-systems (MODIFIED "Throttle Control System", MODIFIED "Proportional
  Speed-Limiter Throttle Taper", ADDED "Configurable Throttle Opening Rate"), web-control (ADDED
  "Throttle Opening Rate Configuration via Web Interface"), web-telemetry (ADDED "Throttle Opening
  Rate Telemetry").
- Affected code: `include/ThrottleSlewLaw.h` (new), `include/ThrottleController.h`,
  `src/ThrottleController.cpp`, `include/Constants.h`, `include/VehicleController.h`,
  `src/VehicleController.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`,
  `src/TelemetryManager.cpp`, `data/index.html`, `include/Changelog.h`, `platformio.ini`,
  `test/test_throttle_slew/test_main.cpp` (new).
- Behaviour change on existing units: with no stored key the default 60 %/s applies, so throttle
  opening becomes rate-limited after the update (intended — this is the fix). Setting 0 restores
  the previous behaviour exactly. Closing is never slowed.
- No NVS migration: a missing `open_slew` key reads as the default.
- Ships after 1.5.15. The pending `add-speed-limiter-slew-setting` change only ADDs requirements
  and does not conflict with the MODIFIED taper requirement here.
