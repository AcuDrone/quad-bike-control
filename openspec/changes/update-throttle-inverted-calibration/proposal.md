# Change: Accept inverted throttle servo calibration

## Why
On the aztec buggy the throttle servo is mounted mirrored: 0 % throttle is 2200 µs and 100 % is
800 µs. `ThrottleController` already maps percent linearly between `idleUs_` and `fullUs_`, which works
for `full < idle`, but the calibration save rejected such a pair ("Full must exceed idle by >= 100us")
and the NVS load check silently fell back to the quad-bike defaults.

## What Changes
- Calibration validation becomes direction-agnostic: idle and full SHALL each lie within
  [`THROTTLE_SERVO_MIN_US`, `THROTTLE_SERVO_MAX_US`] and `|full - idle| >= THROTTLE_MIN_SPAN_US`.
  Rejection text: "Idle and full must differ by >= 100us".
- The same rule is applied on NVS load; an inverted but valid stored pair is kept.
- `usToPercent()` normalises the span sign so an inverted window reports 0..100 % between the
  endpoints and clamps outside them (no negative-denominator integer-division surprises).
- The boot and save debug logs mark an inverted pair, e.g.
  `[THROTTLE] Calibration saved idle=2200us full=800us (inverted)`.
- Web calibration help text (uk + en) states that inverted mounting is supported.
- Defaults stay 901 / 1344 µs (quad bike); the aztec is calibrated through the web UI.
- Changelog: one line in the 1.2.0 entry (uk + en), no version bump.

Audited and already direction-agnostic (no change): `percentToUs()` (MAVLink/RC and web throttle),
the gear-boost PID (works in percent, slews in µs symmetrically), `idle()`, web sliders (800–2200 µs,
no client-side ordering check), telemetry `throttle_idle_us` / `throttle_full_us` / `throttle_us`.

## Impact
- Affected specs: `vehicle-systems`
- Affected code: `src/ThrottleController.cpp`, `include/ThrottleController.h`, `include/Constants.h`
  (comments), `data/index.html` (help text), `include/Changelog.h`
