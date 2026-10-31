# Design: Detect the steering sensor direction at limit capture

## Context
- The Bosch LWS reports the angle in its own sign: + = left (counter-clockwise when the sensor is
  seen from its label side, in the intended mount). Byte 3 bit 7 is the rotation direction
  (1 = clockwise).
- `LwsSensor::position()` returns `−angleRaw()`. `rateDegS()` returns +speed for clockwise. Both
  give the firmware sign + = right. `SteeringController::getPosition()` / `getRateDegS()` pass
  them through, and everything else uses those two.
- With a mirrored mount both signs flip. `captureLimit()` then refuses the left lock ("left limit
  must be left of zero"), so the mount cannot be calibrated.
- NVS `"steering"` layout v3: `cal_ver` = 3, `lim_l`, `lim_r` (0.1°, firmware sign). The same
  namespace holds `steer_sca_base`, so it is never `clear()`ed.
- The operator presses "Set Left" at the physical left lock and "Set Right" at the right lock. The
  button is the only ground truth for "left" the firmware has.

## Goals / Non-Goals
- Goal: a mirrored mount calibrates with the normal Calibration flow, with no extra button or
  parameter.
- Goal: closed-loop steering never runs with limits captured under a different sign.
- Goal: an already calibrated v3 vehicle stays calibrated after the update.
- Non-goal: detecting a reversed **motor** polarity (VESC wiring). The motor sign is fixed
  (`setSpeed(+)` = wheels right). The operator checks it with Jog before calibrating, as today.
- Non-goal: inferring the direction from motion (jog direction against the sensor rate). That was
  option 2, and the user rejected it.
- Non-goal: a run-time "wrong-way" guard for the closed loop (see Open Questions).

## Decisions

### D1 — Where the flag is applied: `LwsSensor`
`LwsSensor` gets `setInverted(bool)` / `isInverted()`. `position()` and `rateDegS()` apply it
through two pure helpers in `include/SteerSensorDir.h`:

```
int32_t steerSensorPosition(int16_t angleRaw, bool inv);  // inv ? +raw : −raw
int32_t steerSensorRate(int32_t speedDegS, bool cw, bool inv);  // (cw ? +s : −s) × (inv ? −1 : +1)
```

`angleRaw()` and the direction bit stay in the sensor's own sign, for the logs. Putting the flag in
`LwsSensor` keeps one sign point for every consumer: `getPosition()`, `getRateDegS()`, telemetry and
MAVLink need no change. `SteeringController` owns the persistent value (NVS) and pushes it into
`lws_` at load and on every toggle. The default is `false`, so `test_lws305` stays valid unchanged.

### D2 — Inference rule (pure, `include/SteerSensorDir.h`)
```
enum class SteerDirAction : uint8_t { STORE, TOGGLE_AND_STORE, REFUSE_TOO_CLOSE, REFUSE_CONTRADICTS };
struct SteerDirResult { SteerDirAction action; bool inv; int32_t pos; bool clearOpposite; };

SteerDirResult steerDirCapture(bool left, int32_t pos /* under current inv */, bool inv,
                               bool oppositeSet, int32_t minSpan);
  |pos| < minSpan                          -> REFUSE_TOO_CLOSE   (inv, pos unchanged)
  on the expected side (left: pos < 0)     -> STORE              (inv, pos)
  wrong side && !oppositeSet               -> TOGGLE_AND_STORE   (!inv, −pos, clearOpposite = true)
  wrong side &&  oppositeSet               -> REFUSE_CONTRADICTS (inv, pos unchanged)
```
- The span check comes first. A small angle carries no reliable sign, so it never toggles.
  Today's order (wrong side first) is reversed. The only visible difference: a small wrong-side
  capture now says "too close to zero".
- `−pos` is exact: toggling the flag negates the derived position of the same raw angle.
- **No ping-pong**: a toggle is allowed only while the opposite limit is unset. After a toggle on
  "Set Left", the left limit is stored under the new flag. A wrong-side "Set Right" then finds the
  left limit stored and is refused. It does not flip back. Re-capturing the same side again while
  the opposite is still unset may toggle again. That is harmless, because nothing else is stored.
- **"Drop the opposite limit"**: an invariant every stored limit satisfies is "captured under the
  current flag". Every toggle clears the opposite. A v3 → v4 migration sets `inv = false`, the
  flag v3 limits were captured with. With the strict "opposite unset" condition, there is
  therefore never an opposite limit to drop when a toggle happens. `clearOpposite` is still
  returned and applied, as a defensive no-op that keeps the invariant explicit if the rule is ever
  relaxed.
- **Re-mount after a full calibration**: both limits are stored, so a contradicting capture is
  refused. That is intended. A re-mount moves the sensor zero anyway. The remedy in the message and
  in the help is: jog straight ahead → "Set sensor zero" (erases both limits) → capture Left and
  Right. The first capture then re-decides the flag.

### D3 — `SteeringController::captureLimit()`
1. `!sensorOk_` → "sensor fault" (unchanged).
2. `r = steerDirCapture(left, getPosition(), inv_, oppositeSet, STEER_CAL_MIN_SPAN)`.
3. Act on `r.action`:
   - REFUSE_TOO_CLOSE: `captureError_` = "too close to zero".
   - REFUSE_CONTRADICTS: `captureError_` = "reads <side> of zero, contradicting the stored <other>
     limit — sensor zero off-centre or sensor re-mounted: jog straight ahead, Set sensor zero,
     capture both limits".
4. On TOGGLE_AND_STORE:
   - `inv_ = r.inv`, `lws_.setInverted(inv_)`, clear the opposite limit (RAM + NVS key);
   - reset the stall and progress baselines (`lastStallPosition_`, `lastProgressPos_`) to the new
     `getPosition()`;
   - set `lastCaptureToggled_ = true`;
   - log `[STEER] sensor direction: mirrored (inferred from the left limit capture) — capture the
     right limit again` (or `normal`).
5. STORE / TOGGLE_AND_STORE then continue as today: `stop()`, drop boot centring, store `r.pos`,
   revalidate, beyond-safe warning.
6. NVS write order in one `Preferences` session: the limit, then `inv`, then remove the opposite.
   A power loss between these writes leaves at most one limit valid under the stored flag. The
   steering is then not calibrated, and the closed loop cannot run.

`VehicleController::processSteerCalCommand()` adds the reply text. On a toggle, the success
message becomes "Steering left limit saved: −412.3° — sensor direction detected: mirrored — capture
the right limit again". A refusal passes `captureError_` through unchanged.

### D4 — NVS layout v4 and migration
| Stored `cal_ver` | Action |
|---|---|
| 4 | load `lim_l`, `lim_r`, `inv` (missing `inv` → false) |
| 3 | keep `lim_l` / `lim_r`, write `inv` = false, write `cal_ver` = 4; log `NVS calibration v3 -> v4: limits kept, sensor direction normal` |
| anything else | as today: remove the obsolete keys one by one, plus `lim_l`, `lim_r`, `inv`; write `cal_ver` = 4, `inv` = false; recalibration required |

- Why bump the layout at all, when a missing `inv` would read as false anyway? It is the downgrade
  guard. Firmware 1.5.0 sees `cal_ver` ≠ 3 and erases the limits. It would otherwise run a
  mirrored calibration with the fixed negation, which is a positive-feedback closed loop.
- 1.5.0 leaves an orphan `inv` key behind. A later upgrade writes `inv` = false again in the
  v3 → v4 step, which matches the limits that 1.5.0 captured.
- `saveLimit()` / `eraseLimits()` write `cal_ver` = 4. `eraseLimits()` (zero-set) keeps `inv`.
- `STEER_CAL_VERSION` becomes 4. The constant name is unchanged.

### D5 — Sign-sensitivity audit of `SteeringController.cpp` (by code reading)
| Spot | Sign use | Verdict |
|---|---|---|
| Stall check in `update()` (`abs(currentPos − lastStallPosition_)`) | absolute delta | safe. The stall direction is `jogDir_` (motor sign), or the error sign of a closed-loop move, which needs calibration. |
| `triggerStallStop()` / `latchBlocks()` | motor direction (`lastDriveDir_`, jog dir) | safe: motor sign, independent of the sensor |
| Anti-stall boost (`abs(rel − lastProgressPos_)`) | absolute delta, closed loop only | safe |
| Range guard (`abs(rel)`, `(rel>0)==(error>0)`) | relative sign of position and error | safe under any consistent sign; closed loop only |
| Closed loop `motor_.setSpeed(error > 0 ? + : −)` | **assumes the sensor sign matches the motor sign** | THE sign-critical spot. It is correct only once the flag is right and the motor polarity is right. It is gated by `calibrated_`, which needs both limits under one flag (D2/D3). |
| `getSteeringPercent()`, `setSteeringPercent()` mapping | firmware sign vs limits | consistent, since the limits share the flag |
| Zero-set confirm (`abs(angleRaw) ≤ STEER_LWS_ZERO_TOLERANCE`) | absolute raw | safe |
| `refreshSensor()` recovery baseline reset, `logState()` | raw angle printed, pos printed | safe. The log keeps the raw sign. |
| Jog / nudge / rate drive | no position | safe |
| **Care**: a toggle while baselines hold old-sign values | `lastStallPosition_`, `lastProgressPos_` | reset at the toggle (D3 step 4). `stop()` already ends any drive. |

Residual hazard, which exists today and is not introduced by this change: suppose the sensor is
re-mounted mirrored at the same zero and the operator does not recalibrate. Both stored limits stay
"valid", and a percent command then drives away from its target. That runaway is bounded by the
range guard (550°), the 7.5 s move timeout, and the position stall stop at the lock. This change
does not detect it. See Open Questions.

### D6 — Telemetry and UI
- `steer_sensor_inv` (bool) is always emitted from the RAM copy of the flag. It adds about 25 bytes
  to the JSON, so re-check the document capacity comment.
- Діагностика → "Кермо: стан" → Sensor tile: after the status / fault label, a muted
  " · дзеркально" / " · mirrored" (i18n key `steer_sensor_mirrored`) while true.
- Калібрування → steering status line: "<sensor> · mirrored | <calibration>".
- The raw LWS line is unchanged. It shows status bits and counters, no angle. The raw-sign angle
  is visible in the serial log (`logState()` `angle=` and the first-frame line).
- ⓘ help, en/uk:
  - Calibration card: "The sensor direction (normal or mirrored mount) is detected from the first
    limit capture. After re-mounting the sensor: jog straight ahead, Set sensor zero, then capture
    both limits. After the first capture, Jog ▶ must make the angle grow; if it shrinks,
    the Left/Right buttons were swapped — Set sensor zero again and recapture."
  - "Кермо: стан": what "mirrored" means.
  - The en/uk dictionaries keep key parity.

### D7 — MAVLink and docs
- MAVLink: no new message. `STEER_POS`, `STEER_MODE`, the VESC floats and the `STATUSTEXT`s are
  unchanged. `STEER_POS` was already defined in firmware sign. `MAVLINK_SETUP.md` is unchanged,
  because it describes the firmware sign only.
- `include/Constants.h`: rewrite the SIGN bullet and the NVS v4 layout bullet.
- `WEB_PORTAL_SETUP.md`: one sentence in the Calibration paragraph on direction inference and the
  re-mount procedure.

## Risks / Trade-offs
- **Swapped buttons are no longer caught (new operator-error path).** On a normally mounted,
  uncalibrated vehicle, "Set Left" pressed at the RIGHT lock now toggles the flag to mirrored
  instead of being refused. "Set Right" pressed at the left lock then agrees, and the calibration
  "completes". The flag is then wrong:
  - the position is + toward the physical LEFT;
  - the motor `setSpeed(+)` still turns the wheels physically right;
  - so the closed loop has positive feedback and drives away from every target. It is bounded by
    the position stall stop at the lock (500 ms), the 7.5 s move timeout and the 550° range guard.

  This is the inherent ambiguity of option 1 (the button is the only ground truth). 1.5.0 refused
  that first capture. Mitigations in this change:
  - the toggle is always logged and named in the web reply;
  - the "mirrored" marker is shown in both cards;
  - the help and the bench tasks say to check, right after the first capture, that Jog ▶ makes the
    angle grow (+ = right). Before the first capture this check is meaningless on a mirrored
    mount. If ▶ makes the angle shrink, the buttons were swapped: press "Set sensor zero" at
    straight-ahead and capture again.

  A run-time wrong-way guard would close the gap; see Open Questions. Option 2 (infer from the jog
  direction) would avoid it but was not chosen.
- A toggle clears nothing that exists in practice (D2). The calibration-incomplete state after a
  toggle comes from the opposite limit not being set yet.

## Migration
v3 → v4 is automatic and keeps the limits (D4). A downgrade to 1.5.0 erases the limits. Firmware
1.5.1 also needs the matching LittleFS image for the new marker and help texts. An old image just
ignores `steer_sensor_inv`.

## Open Questions
- Should a closed-loop "wrong-way" guard be added? It would stop and log when |error| grows by more
  than X over the stall window while driving. It would catch:
  - a swapped Left/Right capture (Risks);
  - a re-mount without recalibration (D5).

  It is out of scope here, and it is the user's call.
