## Context

ArduRover already has this feature. `AP_MotorsUGV::output_regular()` (Rover-4.7.0,
`libraries/AR_Motors/AP_MotorsUGV.cpp`) does:

```
if (is_positive(base) && fabsf(v) > base) {
    steering *= base / fabsf(v);
}
```

with `base = MOT_SPD_SCA_BASE` (default 1.0, range 0..10) and `v` from
`AR_AttitudeControl::get_forward_speed()`. In MANUAL the whole block is gated on `MANUAL_OPTIONS`
bit 0.

The problem is not the formula, it is `v`. `get_forward_speed()` asks the AHRS for a velocity
estimate and, if EKF3 has none, **returns the raw GPS ground speed instead**. On this vehicle the
EKF's horizontal velocity source *is* the ESP32 (`VISO_TYPE = 1`, `EK3_SRC1_VELXY = 6`,
`VISION_POSITION_DELTA`), and the ESP32 deliberately goes **silent** rather than send zeros when
its hall sensor is unhealthy. So the exact moment the speed source fails is the moment ArduPilot
starts steering by GPS. On 2026-09-24 a parked vehicle with a 56 m/s GPS reading scaled steering
to 0.018. There is no ArduPilot parameter that disables that fallback while a GPS is present.

The ESP32, by contrast, has the speed *and* knows whether to believe it: `SpeedSensor::getSpeedMs()`
with `SpeedSensor::isValid()` (`include/SpeedSensor.h:64` — `initialized_ && everPulsed_ &&
!suspicious_`). Moving the scaling here is not a new feature so much as moving an existing one to
the side of the link that owns the measurement.

Constraints that shaped the design:

- **The firmware never writes an autopilot parameter.** Disabling `MANUAL_OPTIONS` bit 0 is an
  operator step, documented, verified on the bench — not a `PARAM_SET`.
- **`include/MavlinkInterface.h` must stay free of mavlink headers**, so any new handler takes
  decoded scalars (the `handleServoOutputRaw` / `handleParamValue` pattern).
- **There is already a parameter subscription** for `SPEED_MAX`. A second one must reuse it, not
  duplicate it, and must not perturb it.
- **Fail-open, and nothing subtler.** The operator asked for the simplest possible fault policy,
  and for this feature the simplest policy is also the correct one (see Decision 4).

## Goals / Non-Goals

- Goals:
  - Scale MANUAL steering by a locally measured, locally validated road speed.
  - Use ArduPilot's formula and ArduPilot's base value, so the vehicle feels the same as it did
    when the feature lived on the autopilot.
  - Make the applied scale visible on the ground station, in the portal and in the log.
  - Make it testable on a bench that is not moving and is not armed.
- Non-Goals:
  - Repairing ArduPilot's GPS fallback, or arbitrating between the two implementations. The
    autopilot's copy is switched off; there is no "both".
  - Any change to `SpeedSensor`, the odometry path, the `SPEED_MAX` limiter or the transmission
    interlock.
  - A curve, a table, a deadband-aware blend, or any refinement of the formula. It is
    `min(1, base/v)` and nothing else.

## Decisions

### 1. The formula is ArduPilot's, applied to the deviation from centre

`scale = min(1, base / v)` for `v > base`, `scale = 1` otherwise; `base <= 0` disables scaling
entirely, exactly as `is_positive(base)` does upstream. It is applied as
`steeringPct *= scale`, which — because `steeringPct` is already signed around a centre of 0 —
scales the *deviation from centre* and leaves centre at centre. A driver who is going straight
feels nothing; a driver at full lock at 5× the base speed gets a fifth of the travel.

- Alternatives considered: a gentler curve (e.g. `sqrt(base/v)`) or a floor under the scale. Both
  rejected: this vehicle's handling was tuned against ArduPilot's curve, and matching it exactly
  means `MOT_SPD_SCA_BASE` carries over as-is and the behaviour before and after this change is
  comparable. A different curve would make the change a re-tune as well as a migration.

### 2. Base value: local NVS first, autopilot second, otherwise nothing

`steer_sca_base` in NVS namespace `"steering"` (m/s) takes priority; a value of 0 or an absent
key means "not set". Failing that, `MOT_SPD_SCA_BASE` read from the autopilot over the existing
read-only parameter subscription. Failing both, `scale = 1` with a rate-limited warning on the log
and in `STATUSTEXT`, at `STEER_SCALE_WARN_MS`, modelled on the limiter's `SPEED_LIMIT_WARN_MS`.

Local-first, because the operator asked for one: the vehicle can be commissioned and driven with
the scaling working before anyone has touched the autopilot, and a bench without a Pixhawk still
has a base value. Autopilot-second, because `MOT_SPD_SCA_BASE` is the number this feature was
tuned with and it is visible from the ground station.

- Alternatives considered: autopilot-only (matching `SPEED_MAX`'s "one source, no fallback"
  decision). Rejected here because the two features have opposite failure directions. A missing
  `SPEED_MAX` means *no speed limit* — a dangerous invisible fallback, hence one source. A missing
  `steer_sca_base` means *full steering authority* — the known-good state. The argument that
  forced single-sourcing on the limiter does not apply.
- Alternatives considered: a compile-time default. Rejected — a default the operator never chose
  would silently restrict steering on a vehicle nobody configured, and "any fault → no scaling" is
  the whole point.

### 3. The subscription is generalised, not duplicated

`MavlinkInterface`'s `SPEED_MAX` subscription becomes a small fixed table of two subscribed
parameters, each with its own id string, upper bound, stored value, receipt time and poll
timestamp. The poll cadence (`MAVLINK_PARAM_FIRST_DELAY_MS`, `MAVLINK_PARAM_POLL_MS`), the
staleness rule (`MAVLINK_PARAM_STALE_MS`), the sender filter, the 17-byte `param_id` copy and the
NaN/inf/negative/out-of-range rejection are shared verbatim. `hasSpeedMaxParam()` /
`getSpeedMaxMs()` / `getSpeedMaxAgeMs()` keep their exact current semantics and are reimplemented
on top of the table, so no existing caller changes. The requests are staggered one loop iteration
apart so the two `PARAM_REQUEST_READ`s do not land in the same 115200-baud frame window.

- Alternative considered: a second, parallel copy of the whole subscription. Rejected — two copies
  of the value-hygiene rules is exactly how one of them ends up missing the `param_id`
  termination fix.

### 4. Any fault disables scaling, with no intermediate states

The scale is computed only when **all** of the following hold; otherwise it is 1:

| Condition | Why 1 is the right answer when it fails |
|---|---|
| A base value is available and > 0 | Nothing to scale by |
| `SpeedSensor::isValid()` | The speed is unknown, and a guess is how ArduPilot got into trouble |
| The autopilot is in MANUAL (or the mode is unknown / stale) | Other modes already scale on the FC |
| The command source is the autopilot | The web path is a bench control |

And `v <= base` gives 1 by the formula itself, so a parked vehicle and a decayed-to-zero reading
need no special case.

This is the operator's explicit choice and it is recorded here so it is not "improved" later.
Speed scaling is a comfort/assist feature: a vehicle that loses it is a vehicle that steers the
way it did before the feature existed. A vehicle that *keeps* a stale restriction after a sensor
fault is a vehicle whose driver cannot turn. **There is deliberately no hold, no last-known-good
speed, no conservative substitute speed and no minimum scale floor.** Every one of those would
create a state in which a fault leaves the steering restricted, which is the failure this change
was written to eliminate.

- Risk accepted: a hall sensor that fails at 20 km/h restores full steering authority instantly
  (subject to the slew of Decision 5), which the driver will feel. That is the intended direction:
  more authority, not less, and the slew keeps it from being a snap.

### 5. The scale is slew-limited, symmetrically

`STEER_SCALE_SLEW_PER_S` (units of scale per second, both directions), applied to the *scale* and
not to the steering command, with `dt` capped at one second and the first evaluation after the
scaling becomes active adopting the target directly. This is the same shape as
`SPEED_LIMIT_CEILING_SLEW_PCT_S` (`include/Constants.h:495`) and its use in
`VehicleController::applySpeedLimit()` (`src/VehicleController.cpp:588`), for the same reason: the
driver's own steering movements must never be slowed by the assist, only the authority envelope
moves at a limited rate.

One symmetric constant rather than separate rise and fall rates, because the operator asked for
the simplest version and because the asymmetric case (restore authority fast, withdraw it slowly)
would need a justification this feature does not have — at road speed the scale changes as fast as
the vehicle accelerates, which is slow.

### 6. MANUAL is detected from the decoded `HEARTBEAT`

`MavlinkInterface::handleHeartbeat()` currently takes only `(sysid, compid)`
(`src/MavlinkInterface.cpp:230`) and the `HEARTBEAT` case in the parse loop
(`src/MavlinkInterface.cpp:113`) does not decode the message at all. It gains `base_mode` and
`custom_mode`, stored for the learned autopilot only, with accessors `hasAutopilotMode()` /
`getAutopilotCustomMode()`. Rover MANUAL is `custom_mode == 0`.

Mode unknown, never received, or the heartbeat stale → **treat as MANUAL**, i.e. scaling applies.
This is the one place where "unknown" does not mean "off", and the reasoning is that MANUAL is the
mode this vehicle is driven in: guessing MANUAL when the answer is unavailable means the assist
works in the common case, and the cost of guessing wrong is that a mode that would have scaled on
the autopilot gets scaled here instead — the same number, computed from a better speed.

### 7. Observability: one named float, one JSON field, one log line

- `STEER_SCA` as a `NAMED_VALUE_FLOAT` at 1 Hz from component 25, on the same slow timer as
  `VESC_TEMP` / `VESC_OK`. It carries the **applied** scale (post-slew), 0..1, and is `1.0` — never
  `NaN` — whenever scaling is off, because "not scaling" is a definite answer and the ground
  station should plot it as a flat line at 1, not as a gap.
- `steer_scale` in the telemetry JSON, with the base in use and which source it came from, so the
  portal can say *why* the scale is what it is.
- A `[STEER]` line on change, epsilon-gated on `STEER_SCALE_LOG_EPSILON` and held off by
  `STEER_SCALE_LOG_MIN_MS`, following `logSpeedLimitChange()` (`src/VehicleController.cpp:564`)
  including its rule that the remembered value is **not** updated while a log is suppressed, so the
  settled value is logged next time rather than lost.

### 8. Bench override: `set_test_speed`, RAM only, 60-second fuse

`set_test_speed <m/s>` substitutes a speed for the scaling calculation only. It is never
persisted, it clears itself after `STEER_SCALE_TEST_SPEED_MS` (60 s), and it is surfaced as `TEST`
in the log and in the portal for as long as it is live. It does **not** affect
`VISION_POSITION_DELTA`, `VFR_HUD`, the `SPEED_MAX` limiter, the transmission interlock or the
odometer, so the bench configuration is structurally incapable of becoming a driving
configuration.

It exists because the alternative is testing this on a moving vehicle. ArduPilot keeps emitting
steering in `SERVO_OUTPUT_RAW` while disarmed (only the throttle channel is zeroed), so a stationary,
disarmed bench can drive the full steering path — everything except the speed.

## Risks / Trade-offs

- **Double scaling if `MANUAL_OPTIONS` is not cleared.** The autopilot scales from its EKF/GPS
  speed and the ESP32 scales again from the hall sensor. → `MANUAL_OPTIONS = 0` is a required step
  in the `MAVLINK_SETUP.md` parameter table and a bench-verification item in `tasks.md`.
- **Clearing `MANUAL_OPTIONS` also removes ArduPilot's ground-speed-based steering reversal in
  reverse.** With the bit clear the reversal keys off throttle sign instead, which on this vehicle
  (mechanical gearbox, throttle never commanded negative) never triggers. → Recorded as a
  behaviour note in the docs task; no code depends on it.
- **The base now lives in two places** (ESP32 NVS and `MOT_SPD_SCA_BASE`), which is the arbitration
  shape that was deliberately removed from the speed limiter. → Mitigated by the priority being
  fixed and one-directional, by the portal and the telemetry stating which source is in use, and
  by the failure mode of "wrong base" being a mildly different steering feel rather than an
  invisible safety ceiling.
- **Authority returns abruptly on a sensor fault at speed.** → Accepted, deliberately; bounded by
  `STEER_SCALE_SLEW_PER_S`.
- **`set_test_speed` left on.** → 60-second auto-clear, visible in the portal, marked `TEST` in the
  log, and confined to the scaling calculation.

## Migration Plan

1. Flash firmware and filesystem (`data/` changed).
2. Set `steer_sca_base` in the portal, or leave it at 0 and let `MOT_SPD_SCA_BASE` supply it.
3. **Set `MANUAL_OPTIONS = 0` on the autopilot** and confirm the steering no longer weakens with
   GPS speed while parked.
4. Bench-verify with `set_test_speed` (stationary, disarmed).
5. Field-verify on a straight line to 20 km/h.

Rollback: set `steer_sca_base = 0` and clear `MOT_SPD_SCA_BASE`, which disables ESP32 scaling
without a reflash; restore `MANUAL_OPTIONS` bit 0 to return the feature to the autopilot.

## Open Questions

- None. The fault policy, the base priority, the MANUAL-only gate and the override's lifetime were
  all settled with the operator on 2026-09-24.
