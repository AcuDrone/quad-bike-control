# Change: Detect the steering sensor direction at limit capture (aztec)

## Why
The Bosch LWS steering-angle sensor (CAN `0x305`) can be fitted mirrored on the steering shaft. It
has just been fitted that way on the buggy. The sensor's sign is + = left. The firmware negates it
once, in `LwsSensor::position()` and `rateDegS()`, to get its own sign + = right. With a mirrored
mount every angle and every rate is inverted. The limit capture then refuses ("left limit must be
left of zero"), see the SIGN comment in `include/Constants.h` and `SteeringController::captureLimit()`.
Today the only fix is to re-mount the sensor.

## What Changes
- **New persistent flag `inv`** (sensor mirrored, bool) in the NVS namespace `"steering"`. The
  layout goes from v3 to **v4** (`cal_ver` = 4, `lim_l`, `lim_r`, `inv`). The namespace is never
  cleared as a whole: `steer_sca_base` lives there too.
- **The flag sets the sign**: position = `inv ? +angle : −angle`, and the rate likewise. Everything
  downstream already uses the firmware sign (+ = right) and is unchanged. That covers closed-loop
  control, limits, percent mapping, telemetry `steer_angle` / `steer_rate` and MAVLink `STEER_POS`.
- **The direction is inferred at limit capture** (user decision, option 1). The pure rule lives in
  the new `include/SteerSensorDir.h`:
  - The capture is on the expected side (Left ≤ −`STEER_CAL_MIN_SPAN`, Right ≥ +span): the limit is
    stored and the flag is unchanged.
  - The capture is on the wrong side by at least the span, and the opposite limit is **unset**:
    the flag is toggled and saved, the position is recomputed and the limit is stored. The opposite
    limit is cleared, so the calibration is incomplete until the other side is captured. The log
    shows `[STEER] sensor direction: mirrored` / `normal`, and the web reply says so.
  - The capture is on the wrong side and the opposite limit is **stored**: the capture is refused
    and the flag is not toggled (no ping-pong). The message points at an off-centre sensor zero or
    a re-mount, with the fix: jog straight ahead, "Set sensor zero", capture both limits.
  - |position| < `STEER_CAL_MIN_SPAN`: refused as too close to zero, as today, with no toggle.
- **Jog / Nudge, rate mode and "Set sensor zero" do not use the sign.** Nothing breaks before the
  first capture. Percent commands stay refused until both limits are stored.
- **Migration v3 → v4 keeps the limits**, with `inv` = false. v3 limits were captured with the
  fixed negation, so no recalibration is needed. Older layouts are wiped as today.
- **Web**:
  - telemetry key `steer_sensor_inv` (bool);
  - the Sensor tile in Діагностика → "Кермо: стан" shows a "дзеркально" / "mirrored" marker;
  - the status line of the Калібрування steering card also shows it;
  - the ⓘ help texts (en/uk) describe the inference;
  - the capture replies carry the toggle and refusal messages (en).
- **Logs**: the direction is logged on a toggle, in the boot calibration line and in the
  first-frame line. The first-frame line keeps the raw sensor sign.
- **Version**: `FIRMWARE_VERSION` 1.5.0 → **1.5.1** (patch: a fix for a mounting limitation, no
  wiring change and no recalibration). A `Changelog.h` entry (uk/en) is added.

## Impact
- Affected specs:
  - `vehicle-systems`: MODIFIED "Steering Control System" and "Steering Limit Calibration Around
    the Sensor Zero"; ADDED "Steering Sensor Direction Inference".
  - `vehicle-actuators`: MODIFIED "Bosch LWS Steering Angle Sensor" (the position/rate sign).
  - `web-telemetry`: MODIFIED "Steering Angle Sensor Telemetry".
  - `web-control`: MODIFIED "Steering Calibration and Sensor Zero via Web Interface".
  - `mavlink-interface`, `debug-logging`: no delta. MAVLink messages (`STEER_POS`, `STEER_MODE`,
    …) are unchanged. The new log lines are specified with the steering requirements, as earlier
    steering changes did.
- Affected code:
  - `include/SteerSensorDir.h` (new, pure), `include/LwsSensor.h`;
  - `include/SteeringController.h`, `src/SteeringController.cpp`, `src/VehicleController.cpp`
    (capture replies);
  - `include/TelemetryManager.h`, `src/TelemetryManager.cpp`, `src/WebPortal.cpp`;
  - `data/index.html`, `include/Constants.h`, `include/Changelog.h`;
  - `test/test_steer_sensor_dir/` (new), `platformio.ini`, `WEB_PORTAL_SETUP.md`.
- **Archive order**: the deltas are written against the specs as they will be after these
  changes are archived, in this order: `replace-aztec-steering-sensor-with-bosch-lws`, then
  `redesign-web-ui-service-console`, then `add-aztec-steering-rate-mode`. The MODIFIED blocks
  quote their pending deltas, so this change SHALL be archived after all three.
  `move-aztec-vesc-to-can` (1.6.0) touches none of the requirements modified here.
- Safety: the flag changes only through a limit capture. A capture that toggles it always leaves
  the calibration incomplete. Percent / closed-loop steering therefore never runs on limits
  captured under a different sign. A firmware downgrade to 1.5.0 sees `cal_ver` = 4 and erases
  the limits, so it can never run a mirrored calibration while ignoring the flag.
- New operator-error path: on a normal mount, pressing "Set Left" at the right lock now toggles
  the flag instead of being refused. Pressing the swapped "Set Right" then completes a calibration
  with an inverted closed loop. It is bounded by the stall stop, the move timeout and the range
  guard. It is surfaced by the log, the reply and the "mirrored" marker. The help prescribes a
  Jog ▶ check after the first capture. See `design.md` "Risks" and Open Questions.
- Not breaking: no wiring change. A calibrated v3 vehicle stays calibrated.
