# Change: Per-vehicle speed-limiter throttle rate (ceiling slew) from the web

## Why
The operator finds the speed limiter closes and returns the throttle too abruptly. The ceiling
rate limit is a compile-time constant (`SPEED_LIMIT_CEILING_SLEW_PCT_S` = 80 %/s), so tuning it
needs a reflash, and the right value differs per vehicle.

## What Changes
- The ceiling slew becomes a persisted per-vehicle setting: NVS namespace `"speed"`, key
  `thr_slew` (float, %/s), default `SPEED_LIMIT_SLEW_DEFAULT_PCT_S` = 80 %/s (today's behaviour),
  accepted range `SPEED_LIMIT_SLEW_MIN_PCT_S`–`SPEED_LIMIT_SLEW_MAX_PCT_S` (10–200 %/s).
  `SPEED_LIMIT_CEILING_SLEW_PCT_S` is removed; the live value feeds `SpeedLimiterParams.slewPctS`,
  which also keeps capping the trim's fall (intended).
- Web command `set_speed_limit_slew` (value in %/s), same pattern as `set_speed_limit_floor`:
  validated, persisted first, then applied; English ack/error replies.
- Telemetry key `speed_limit_slew` (one decimal) beside `speed_limit_floor`.
- Web UI: a "Плавність обмежувача (%/с)" / "Limiter throttle rate (%/s)" row (10–200, step 5)
  with a Save button and a one-line hint in the "Обмеження швидкості та кермо" card; the card
  help gains one sentence. The status line now also shows the `Speed-limiter …` replies (floor
  and rate), which the old message filter missed.
- Native test: a lower slew makes the ceiling fall slower over the same interval.
- Shipped in firmware 1.5.12 (changelog bullet).

## Impact
- Affected specs: vehicle-systems, web-control, web-telemetry
- Affected code: `include/Constants.h`, `include/VehicleController.h`, `src/VehicleController.cpp`,
  `include/WebPortal.h`, `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `data/index.html`,
  `include/Changelog.h`, `test/test_speed_limiter/test_main.cpp`
- No NVS migration: an absent key reads as the default, so existing units behave as before.
