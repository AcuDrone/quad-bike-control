# Change: Speed limiter — configurable floor, proportional taper band and integral trim

## Why
On the buggy (ODES 800 EFI, CVT) the pilot set a 10 km/h limit (`SPEED_MAX` = 2.78 m/s) and the
vehicle drove at 30 km/h with a valid speed reading. The limiter is proportional only. Its ceiling
falls to `SPEED_LIMIT_FLOOR_PCT` (7.5 %) at the limit and stays there. On this engine and CVT, 7.5 %
throttle is enough to hold about 30 km/h. Once the ceiling sits on the floor, the limiter has
nowhere further to go.

There is a second defect. The taper band is a fixed 1.4 m/s. At a 10 km/h limit the limiter
starts withdrawing authority at 5 km/h, half the limit. At a high limit the same band is fine.

Neither defect is vehicle-specific. The quad (`master`) runs the same code.

## What Changes
- **Floor default 0 % (calibrated idle) and configurable.** `SPEED_LIMIT_FLOOR_PCT` becomes the
  compile-time default (0 %) of a persisted setting, range 0–20 %
  (`SPEED_LIMIT_FLOOR_MIN_PCT` / `SPEED_LIMIT_FLOOR_MAX_PCT`):
  - stored in NVS namespace `"speed"`, key `thr_floor` (float, %). It is owned by
    `VehicleController`, like `steer_sca_base` in `"steering"`. The retired `lim_on` / `lim_kmh`
    names are not reused;
  - an out-of-range or NaN stored value is ignored, and the default is used;
  - new web command `set_speed_limit_floor` (accepted regardless of the input source, like
    `speed_cal_ppr` / `speed_cal_circ`);
  - an input next to ppr / circumference in the Speed sensor settings card;
  - new telemetry key `speed_limit_floor`.
  0 % is the calibrated idle position (`ThrottleController::percentToUs(0)` = `idleUs_`). On a CVT
  vehicle that disengages the drive. The approach to it stays gradual (taper and slew), so it is
  not a hard cut.
- **Proportional taper band.** The fixed `SPEED_LIMIT_TAPER_BAND_MS` is replaced by
  `band = clamp(0.20 × limit, 0.5 m/s, 1.4 m/s)` (`SPEED_LIMIT_TAPER_FRACTION`,
  `SPEED_LIMIT_TAPER_MIN_MS`, `SPEED_LIMIT_TAPER_MAX_MS`). Resulting bands:
  - 10 km/h: 0.56 m/s, the taper starts at 8 km/h;
  - 20 km/h: 1.11 m/s, the taper starts at 16 km/h;
  - 40 km/h: 1.4 m/s, the taper starts at 35 km/h (as today).
- **Integral trim on the ceiling.** A trim term `T` (0 … 100 − floor %) is added on top of the
  proportional taper: `ceiling = clamp(P(v) + T, floor, 100)`, then slew-limited exactly as today
  (80 %/s). `T` behaves as follows:
  - **above the limit**, `T` falls at `Ki × excess` (Ki = 30 %/s per m/s, step capped at the slew).
    The ceiling keeps descending to the floor;
  - **below the limit, inside the band, and only while the limiter is actually binding**, `T` rises
    at `Ki × deficit`, capped at `SPEED_LIMIT_TRIM_RISE_PCT_S` = 10 %/s. It returns exactly the
    authority needed to hold the limit, so the steady state converges to the limit rather than
    drooping below it, whatever the engine torque, CVT ratio or grade;
  - `T` holds while the driver's demand is below the ceiling;
  - `T` is reset to 0 below the band, on "no limit", on an invalid sensor, and on the first
    evaluation.
  Fail-open is unchanged. Braking is out of scope.
- **Saturation indicator in the web console.** The operator cannot read the serial log in the
  field, so the "limiter has run out of authority" case is shown in the web console and not
  logged. The firmware detects it when the ceiling is within 0.5 % of the floor while the speed
  stays more than 0.3 m/s above the limit, continuously for 2 s. It clears at once when either
  part stops holding. Two new telemetry booleans:
  - `speed_limit_sat`: the live condition;
  - `speed_limit_sat_held`: true while `speed_limit_sat` is true and for 10 s after it clears.
    The hold lives in the firmware, so every connected or late-joining client shows the same
    state.
  The limiter row ("Limiting X %") becomes a warning badge in the console's standard warning
  styling: "Limiter at floor, speed above limit" while the condition holds, and a "was at floor"
  wording during the hold. In the field case the badge would have appeared within 2 s and stayed
  up after the driver slowed down. No serial log line and no MAVLink message are added for
  saturation. The existing rate-limited log of limit *changes* is unchanged.
- **Spec wording.** The scenario "No persisted limiter configuration" keeps its name. Its content
  is narrowed to "no persisted *ceiling*": the floor is the one persisted limiter parameter, and
  the ceiling still has exactly one source (`SPEED_MAX`). "The throttle SHALL NOT be cut to zero" is replaced by "the ceiling SHALL never
  step; it reaches the floor only gradually".
- **Version.** `FIRMWARE_VERSION` gets a minor bump. The brief says **1.6.0**, but 1.6.0 and 1.7.0
  are already claimed by the pending `move-can-rx-to-interrupt` and `move-aztec-vesc-to-can`. The
  release number is the next free minor at implementation time (see tasks 0.2 / 8.1). A
  `Changelog.h` entry (uk/en) is added.

## Impact
- Affected specs:
  - `vehicle-systems`:
    - MODIFIED "Autopilot-Sourced Maximum-Speed Limit" (persisted floor allowed, ceiling still
      single-source);
    - MODIFIED "Proportional Speed-Limiter Throttle Taper" (proportional band, configurable floor,
      the ceiling is taper + trim);
    - ADDED "Speed-Limiter Integral Trim" and "Configurable Speed-Limiter Throttle Floor".
  - `web-control`: ADDED "Speed-Limiter Floor Configuration via Web Interface".
  - `web-telemetry`: ADDED "Speed-Limiter Floor Telemetry" (`speed_limit_floor`) and
    "Speed-Limiter Saturation Indicator" (`speed_limit_sat`, `speed_limit_sat_held`, and the
    warning badge on the limiter row). `speed_limit_ceil` keeps its meaning (the final applied
    ceiling). The integral trim is not exported.
  - `speed-sensor`: no delta. `SpeedSensor` keeps its keys. Its retired-key cleanup removes only
    `lim_on` / `lim_kmh` and never touches `thr_floor`.
- Affected code:
  - `include/Constants.h` (limiter constants, version), `include/Changelog.h`;
  - `include/VehicleController.h`, `src/VehicleController.cpp` (`applySpeedLimit`, floor
    load/setter, command routing, saturation detection and hold);
  - `include/TelemetryManager.h`, `src/TelemetryManager.cpp`, `src/WebPortal.cpp`;
  - `data/index.html` (input, saturation badge on the limiter row, i18n, ⓘ help);
  - new pure header `include/SpeedLimiterLaw.h` and a native test suite
    `test/test_speed_limiter/`, `platformio.ini`;
  - `WEB_PORTAL_SETUP.md`.
- Archive order: all deltas are against the current `openspec/specs/`. The two MODIFIED
  requirements are not touched by any pending change. `redesign-web-ui-service-console` modifies
  "Speed-Limit and Limiter-Ceiling Telemetry", which this change deliberately leaves alone (ADDED
  only), so there is no ordering constraint.
- Safety: fail-open on an invalid sensor and "no limit" behave exactly as today. The ceiling never
  rises faster than the slew. The trim can only add authority up to the point where the speed
  sits at the limit. Above the limit it can only remove authority. The lower floor means the
  limiter can now bring the throttle back to idle at and above the limit. That is the point of
  the change, and the operator can raise it to 20 % per vehicle.
- Behaviour change for operators: limits now hold more tightly (no 7.5 % residual, less early
  withdrawal at low limits). No wiring change and no recalibration.
