# Change: Brake tuning card — full travel time and hold drive from the web, real `brake_pct`

## Why
The brake is a DC linear actuator on a BTS7960 with no position feedback except the "released"
endstop (In5). `VehicleController::updateBrakeControl()` estimates the position open-loop by time:
it always drives at ±255 and advances the estimate at `100 / BRAKE_FULL_TRAVEL_TIME` %/ms
(1500 ms for 0 → 100 %), then, while a non-zero target is held, pushes with `BRAKE_HOLD_SPEED`
(30 PWM) against the hydraulic return. Both numbers are compile-time constants, measured once on
one vehicle. The two vehicles (quad, Aztec buggy) have different actuators and brake hydraulics,
so a wrong travel time makes every brake percentage wrong (50 % lands at 35 % or 70 % of the real
stroke), and a wrong hold drive either lets the brake creep back or keeps the motor stalled
against the cylinder. Changing them today needs a reflash per vehicle.

Tuning them blind is also impossible: the telemetry key `brake_pct` is hard-coded to 0.0
(`src/TelemetryManager.cpp`), so the operator cannot see the estimate the firmware is acting on,
nor the endstop state.

## What Changes
- New pure law `include/BrakeLaw.h` (header-only, no Arduino):
  - `brakeRatePctPerMs(travelMs)` → `100 / travelMs` (%/ms) at full drive; the existing
    `speedFraction` factor stays in `updateBrakeControl()` (always 1.0 today);
  - `brakeTravelMsValid(ms)` (1000–3000) and `brakeHoldPwmValid(pwm)` (0–100), plus
    `brakeTravelMsOrDefault()` / `brakeHoldPwmOrDefault()` used when loading from NVS.
- Two persisted per-vehicle settings in a NEW NVS namespace `"brake"`, owned by
  `VehicleController` (the brake estimate lives there):
  | Setting | NVS key | Type | Range | Default |
  |---|---|---|---|---|
  | full travel time | `travel_ms` | uint16, ms | 1000–3000 | 1500 (`BRAKE_FULL_TRAVEL_TIME`) |
  | hold drive | `hold_pwm` | uint8, PWM 0–255 scale | 0–100 | 30 (`BRAKE_HOLD_SPEED`) |
  Loaded at boot; a missing key reads as the default; a stored out-of-range value is logged and
  ignored (default used, not rewritten). Constants: `BRAKE_TRAVEL_MS_MIN` (1000),
  `BRAKE_TRAVEL_MS_MAX` (3000), `BRAKE_HOLD_PWM_MIN` (0), `BRAKE_HOLD_PWM_MAX` (100),
  `BRAKE_NVS_NAMESPACE` ("brake"), `BRAKE_TRAVEL_NVS_KEY` ("travel_ms"),
  `BRAKE_HOLD_NVS_KEY` ("hold_pwm"); `BRAKE_FULL_TRAVEL_TIME` and `BRAKE_HOLD_SPEED` become the
  defaults.
- `updateBrakeControl()` uses the runtime values: the movement rate is
  `brakeRatePctPerMs(brakeTravelMs_) × speedFraction`, the hold push is
  `setSpeed(brakeHoldPwm_)`, and a hold of 0 calls `stop()` (coast, no push).
- Applied immediately on change: a new travel time changes the rate of the running estimate from
  the next control-loop iteration (the estimate is not reset or rescaled); a new hold drive is
  used at the next hold cycle. Persist first, then apply (like the other cards).
- Web commands `set_brake_travel_ms` (value ms, rounded to a whole ms) and `set_brake_hold_pwm`
  (value, rounded to a whole number), accepted regardless of the input source (settings, not
  driving). Replies: "Brake travel time set to N ms", "Brake travel time out of range
  (1000-3000 ms)", "Brake hold set to N", "Brake hold off" (0), "Brake hold out of range (0-100)",
  "Failed to save the brake setting".
- Telemetry:
  - `brake_pct` now reports the firmware's current position estimate (0–100) instead of 0.0, and
    the existing panel tile "Гальмо" / "Brake" shows it (no UI change needed beyond the data);
  - new `brake_travel_ms` (int), `brake_hold_pwm` (int), `brake_released` (bool: the In5 endstop,
    false when the expander snapshot is invalid — exactly `isBrakeReleased()`).
- Web UI: new card "🛑 Гальмо" / "🛑 Brake" right after the "Throttle Calibration" card (the
  tuning cards are a single scrolling page, no tabs): the travel time is a slider (range input
  1000–3000 ms, step 50, default 1500) with a live value label "N мс" / "N ms" next to it, sending
  `set_brake_travel_ms` once on release (`change`), not while dragging; the hold drive is a row
  (number input 0–100, step 5, + Save) with local validation and a hint
  "0 = без утримання" / "0 = no hold push"; a status line "Кінцевик: відпущено / ні" /
  "Endstop: released / no" from `brake_released`, the card's own reply line and a help (ⓘ) text;
  uk + en. No reset-to-default button (the park card has none).
- Native tests `test/test_brake_law` (registered in `platformio.ini` `test_filter` + the header
  comment).
- Firmware 1.5.23 with a changelog bullet (uk + en).

Out of scope (follow-ups in design.md): closed-loop or current-sensed position, stall / overrun
guard on extension, brake–throttle interlocks, a parking brake on start-up, recalibrating the
estimate from the endstop beyond what exists today.

## Impact
- Affected specs: vehicle-systems (MODIFIED "Brake Control System", ADDED "Configurable Brake
  Travel Time and Hold Drive"), web-control (ADDED "Brake Tuning Card"), web-telemetry (ADDED
  "Brake Position and Tuning Telemetry").
- Affected code: `include/BrakeLaw.h` (new), `include/Constants.h`, `include/VehicleController.h`,
  `src/VehicleController.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`,
  `src/TelemetryManager.cpp`, `data/index.html`, `include/Changelog.h`, `platformio.ini`,
  `test/test_brake_law/test_main.cpp` (new).
- Behaviour on existing units: no stored keys → 1500 ms / 30, exactly today's behaviour. The only
  visible change is that `brake_pct` is no longer always 0.
- No NVS migration: a new namespace; missing keys read as the defaults.
- LittleFS image must be uploaded (new card, new i18n keys).
- Both branches: the aztec branch has the same brake code; the merge into aztec is expected to be
  conflict-free apart from the usual version / changelog lines.
