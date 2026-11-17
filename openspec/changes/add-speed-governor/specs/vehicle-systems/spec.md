## ADDED Requirements
### Requirement: Speed Governor
The system SHALL provide an opt-in speed governor. While the governor switch is ON, an effective
speed limit `L` > 0 exists (the same limit the maximum-speed limiter enforces: the usable
`SPEED_MAX`, and on the buggy the lower of it and the fixed gear ceiling), the speed reading is
valid and no bypass condition holds, the governor SHALL be ACTIVE: the throttle input — the RC
throttle (channel 2 above centre) or the standing web throttle demand — SHALL be a speed setpoint,
and the throttle SHALL be computed by a closed loop on the measured wheel speed. The law SHALL live
in a pure header (`include/SpeedGovernorLaw.h`) without Arduino dependencies. The governor SHALL
apply regardless of the autopilot's flight mode. It SHALL NOT command the brake: its lowest output
SHALL be the calibrated idle (0 %).

#### Scenario: Stick maps to a speed up to the limit
- **WHEN** the governor is ACTIVE with limit `L` and the throttle input is `s` %
- **THEN** for `s` ≤ `GOV_STICK_DEADBAND_PCT` (5 %) the target speed SHALL be 0
- **AND** above it the target SHALL be `vMin + (s − 5)/(100 − 5) × (L − vMin)`, with
  `vMin = min(GOV_MIN_SETPOINT_MS (1.4 m/s), 0.5 × L)`
- **AND** at `s` = 100 % the target SHALL equal `L`, so no speed above the limit can be commanded

#### Scenario: Setpoint is ramped
- **WHEN** the target speed changes
- **THEN** the setpoint used by the loop SHALL move toward it by at most the configured
  acceleration (m/s²) per second upward and the configured deceleration per second downward
- **AND** a lowered limit SHALL lower the target and the setpoint SHALL follow at the deceleration

#### Scenario: PI on the speed error with anti-windup
- **WHEN** the governor is ACTIVE with setpoint `vr` and measured speed `v`
- **THEN** the output SHALL be `clamp(Kp × (vr − v) + I, 0, uMax)`, where `I` integrates
  `Ki × (vr − v)` over time and is clamped to 0–`uMax`
- **AND** `v` SHALL be the speed reading as reported by the speed sensor, without additional
  filtering and without an error dead-band
- **AND** `I` SHALL NOT integrate further in the direction of a saturated output (output at `uMax`
  with a positive error, or at 0 with a negative error)
- **AND** the output SHALL move toward that value by at most the throttle rate `thr_slew` per second
- **AND** the elapsed time used SHALL be capped at one second

#### Scenario: Default gains at a 10 km/h limit
- **WHEN** the governor runs with the build's default settings (see "Speed Governor Settings"),
  limit 10 km/h, stick at 100 %, integrator 0, and the speed is 0 with the setpoint already ramped
  to 10 km/h (2.78 m/s)
- **THEN** the proportional part SHALL ask for `Kp` × 2.78: about 2.8 % on the buggy (`aztec`,
  Kp 1.0) and about 8.3 % on the quad bike (`master`, `old-board`, Kp 3.0)
- **AND** the output SHALL be bounded by the build's default start throttle limit (4 % on the
  buggy, 10 % on the quad bike) until the wheels turn

#### Scenario: Start throttle limit while the wheels do not turn
- **WHEN** the governor is ACTIVE and the measured speed is below `GOV_STANDSTILL_MS`
- **THEN** `uMax` SHALL be the configured start throttle limit instead of 100 %
- **AND** the integrator SHALL therefore not wind up beyond the start throttle limit while the
  vehicle stands (engine off, clutch not engaged, stuck)

#### Scenario: Stick released means stop
- **WHEN** the throttle input is at or below the 5 % dead-band
- **THEN** the throttle SHALL go to idle at once, without the output rate limit
- **AND** the setpoint and the integrator SHALL be reset to 0

#### Scenario: Downhill, the governor cannot hold the speed
- **WHEN** the output is at idle and the speed stays above the setpoint
- **THEN** the governor SHALL leave the throttle at idle and SHALL NOT act on the brake
- **AND** the web-only saturation indicator (see "Speed Governor Telemetry") SHALL report it

#### Scenario: Autopilot mode is not consulted
- **WHEN** the autopilot reports any flight mode
- **THEN** the governor's behaviour SHALL be the same as in MANUAL

### Requirement: Speed Governor Settings
The governor SHALL be configured by persisted per-vehicle settings in NVS namespace `"speed"`
(owned by `VehicleController`, shared with the speed sensor and the limiter, never cleared):

| Setting | Key | Type | Range | Default, buggy (`aztec`) | Default, quad bike (`master`, `old-board`) |
|---|---|---|---|---|---|
| switch | `gov_en` | bool | — | off | off |
| proportional gain | `gov_kp` | float, % per m/s | 0.2–10 | 1.0 | 3.0 |
| integral gain | `gov_ki` | float, %/s per m/s | 0–5 | 0.35 | 1.0 |
| setpoint acceleration | `gov_acc` | float, m/s² | 0.3–3.0 | 1.0 | 1.0 |
| setpoint deceleration | `gov_dec` | float, m/s² | 0.5–5.0 | 2.5 | 2.5 |
| start throttle limit | `gov_launch` | float, % | 1–40 | 4 | 10 |

The ranges SHALL be the same on every build; the defaults SHALL be those of the vehicle the build
is for (the column matching the build's vehicle type: 2 = buggy on `aztec`, 1 = quad bike on
`master` and `old-board`), and a build whose compiled governor defaults are tagged for another
vehicle type SHALL fail to compile. The same values SHALL apply at every effective limit (no
scheduling by the limit). They SHALL be loaded at start-up; an absent key SHALL read as the
build's default; a stored NaN or out-of-range value SHALL be logged and ignored (default used, not
rewritten). A default SHALL apply only while the key is absent or invalid: a valid stored value
SHALL never be replaced by a default, including after a firmware update that changes the
defaults. A new value SHALL be
validated, persisted first and applied only after the write succeeded, and SHALL take effect on the
next governor evaluation.

#### Scenario: Default on a unit without stored values
- **WHEN** the system starts and no `gov_*` key has been written
- **THEN** the governor switch SHALL be off and the throttle path SHALL be exactly as before this
  change
- **AND** when the switch is turned on the governor SHALL use acceleration 1.0 m/s² and
  deceleration 2.5 m/s², and on the buggy Kp 1.0, Ki 0.35 and a start throttle limit of 4 %, on
  the quad bike Kp 3.0, Ki 1.0 and a start throttle limit of 10 %

#### Scenario: Defaults belong to the build's vehicle
- **WHEN** the firmware is built for the buggy (vehicle type 2) while the compiled governor
  defaults are tagged as the quad bike's (vehicle type 1), or the other way round
- **THEN** the build SHALL fail with a message naming the governor defaults

#### Scenario: Stored values survive a change of defaults
- **WHEN** valid `gov_*` values are stored and a firmware with different compiled defaults is
  flashed
- **THEN** the stored values SHALL stay in use and SHALL NOT be rewritten

#### Scenario: Start throttle limit down to low single digits
- **WHEN** the operator sets the start throttle limit to 1 %
- **THEN** the value SHALL be accepted
- **AND** 0.5 % or 41 % SHALL be rejected

#### Scenario: Stored settings are restored
- **WHEN** the system starts with valid stored values
- **THEN** those values SHALL be in use, per vehicle

#### Scenario: Invalid stored value
- **WHEN** a stored `gov_*` value is NaN or outside its range
- **THEN** it SHALL be ignored and logged, the default SHALL be used, and the key SHALL NOT be
  rewritten

#### Scenario: Setting rejected
- **WHEN** a new value is outside its range or the NVS write fails
- **THEN** the value in use SHALL stay unchanged

### Requirement: Speed Governor Hand-Over and Speed-Sensor Fail-Open
Whenever the governor is not ACTIVE the throttle SHALL follow the path that existed before this
change (direct throttle, with the maximum-speed limiter where a limit exists). A change between
the two SHALL NOT produce a throttle step, except a closing to idle commanded by the operator or a
safety clip. An invalid speed reading SHALL make the governor stop governing (fail-open): the
throttle input SHALL become a direct throttle again without any governor restriction.

#### Scenario: Switch off is today's behaviour
- **WHEN** the governor switch is off
- **THEN** the commanded throttle SHALL be identical to the firmware without the governor, including
  the maximum-speed limiter

#### Scenario: No effective limit means direct throttle
- **WHEN** the switch is on and the effective limit is zero
- **THEN** the throttle input SHALL pass through as today

#### Scenario: Speed signal lost while governing
- **WHEN** the governor is ACTIVE and the speed reading becomes invalid
- **THEN** the governor SHALL stop governing and the throttle SHALL move from the governor's last
  output toward the direct (today's) value by at most `thr_slew` per second, in either direction
- **AND** once it has reached that value the throttle input SHALL be a direct throttle, so a stick
  at 100 % SHALL mean full throttle
- **AND** a stick moved into the dead-band or a brake command during the blend SHALL close the
  throttle to idle at once

#### Scenario: No pulse since boot counts as invalid
- **WHEN** the system has not seen a speed pulse since boot
- **THEN** the speed reading SHALL count as invalid for the governor and the throttle SHALL be direct
  until the first pulse, with the same no-speed warnings as any invalid speed
- **AND** with the first pulse the governor SHALL engage bumplessly from the throttle then
  commanded

#### Scenario: Engagement is bumpless
- **WHEN** the governor becomes ACTIVE — switch turned on, a limit appears, the speed becomes valid,
  a bypass ends, or the first evaluation after a gap longer than `GOV_MAX_GAP_MS`
- **THEN** the setpoint SHALL start at the measured speed, the output at the throttle percent `u0`
  last commanded to the throttle controller, and the integrator at `u0` but not above the start
  throttle limit
- **AND** the first governor output SHALL equal `u0`

#### Scenario: Take-over from a high direct throttle
- **WHEN** the governor becomes ACTIVE while the last commanded throttle is above the start
  throttle limit (for example 100 % on the first launch after boot, or 40 % when a 10 km/h limit
  appears at 25 km/h)
- **THEN** the first output SHALL equal that throttle, the integrator SHALL start at the start
  throttle limit
- **AND** the output SHALL then move toward the governor's value by at most `thr_slew` per second

#### Scenario: Turning the switch off while driving
- **WHEN** the switch is turned off while the governor is ACTIVE
- **THEN** the throttle SHALL blend to today's value at `thr_slew` as in the speed-loss case

### Requirement: Speed Governor Yields to Brake, Gear Change and Fail-Safe
The governor SHALL never fight the operator's brake, the gear-change throttle handling or the
fail-safe. The brake command path SHALL be unchanged by this change.

#### Scenario: RC brake on the combined channel
- **WHEN** RC channel 2 is below centre (brake)
- **THEN** the throttle input SHALL be 0 and the governor SHALL stop: idle at once, setpoint and
  integrator reset
- **AND** the brake SHALL be applied exactly as before this change

#### Scenario: Web brake while the governor is on
- **WHEN** the governor is ACTIVE and a brake target of 5 % or more is commanded from the web
- **THEN** the governor SHALL hold the throttle at idle and keep the setpoint and integrator reset
  while the brake target stays at 5 % or more
- **AND** with the governor switch off a web brake SHALL NOT change the web throttle, as before

#### Scenario: Neutral, unknown gear and gear change bypass the governor
- **WHEN** the expected gear is N or UNKNOWN, a gear change is active, the gear-change boost PID or
  idle hold owns the throttle, or the throttle is being calibrated
- **THEN** the governor SHALL NOT be evaluated and SHALL be reset
- **AND** the throttle SHALL be produced exactly as before this change, including the clip to
  `TRANS_UNKNOWN_GEAR_THROTTLE_MAX`, applied without a blend
- **AND** when the bypass ends the governor SHALL engage bumplessly

#### Scenario: Fail-safe
- **WHEN** fail-safe is entered
- **THEN** the throttle SHALL go to idle as before and the governor SHALL re-engage bumplessly from
  idle after the fail-safe ends

#### Scenario: Opening rate limit stays at the servo input
- **WHEN** the governor output rises and the throttle opening rate limit is configured
- **THEN** the output SHALL still pass through the opening rate limit before the servo

### Requirement: Ceiling Limiter Stands Down While the Speed Governor Is Active
While the governor is ACTIVE the maximum-speed ceiling limiter SHALL NOT be evaluated on the
throttle. This takes precedence over the statement in "Proportional Speed-Limiter Throttle Taper"
that the limiter is evaluated on every control-loop iteration for the autopilot and web throttle
paths (and, on the buggy, over "Fixed Per-Gear Maximum-Speed Ceiling" enforcing the gear ceiling
through the limiter law: the governor enforces it instead). In every other governor state the
limiter SHALL behave as specified in the limiter requirements ("Proportional Speed-Limiter Throttle
Taper", "Speed-Limiter Integral Trim", "Configurable Speed-Limiter Throttle Floor"), unchanged.

#### Scenario: No double limiting
- **WHEN** the governor is ACTIVE
- **THEN** the limiter state SHALL be reset on every evaluation (ceiling 100 %, trim 0, first-call
  sentinel) and its saturation indicator SHALL be cleared
- **AND** `speed_limit_ceil` SHALL report 100

#### Scenario: Limiter resumes when the governor stops
- **WHEN** the governor leaves the ACTIVE state while a limit is in force and the speed is valid
- **THEN** the limiter SHALL resume with its first-evaluation rule (taper adopted directly, zero
  trim), and the hand-over blend SHALL smooth the difference to the governor's last output
