# Change: Steering speed scaling moves from the autopilot to the ESP32

## Why
ArduRover scales steering authority with road speed in MANUAL mode (`MANUAL_OPTIONS` bit 0):
`scale = MOT_SPD_SCA_BASE / groundspeed`. The `groundspeed` it uses comes from
`AR_AttitudeControl::get_forward_speed()`, which reads EKF3 velocity and — when the EKF has no
velocity estimate — **silently falls back to the raw GPS ground speed**. That fallback cannot be
switched off in ArduPilot while a GPS is attached.

On this vehicle (ArduRover 4.7.0, `VISO_TYPE = 1`, `EK3_SRC1_VELXY = 6`) the EKF's velocity *is*
the ESP32's wheel odometry (`VISION_POSITION_DELTA`). So whenever the ESP32 goes quiet — the
silence-over-zeros policy on a faulty hall sensor, a pulled TELEM lead, a reboot — the autopilot
quietly swaps a good wheel speed for a GPS number nobody is watching. **Observed 2026-09-24: a
parked vehicle, GPS reporting 56 m/s, steering scaled to 0.018 — effectively no steering at all.**
The failure is silent, it happens exactly when the speed source is already unhealthy, and the
result is the opposite of fail-safe.

**Decision (operator's, 2026-09-24): do the scaling on the ESP32, where the speed is measured
directly by the hall sensor and its validity is known, and turn it off on the autopilot
(`MANUAL_OPTIONS = 0`).** The ESP32 never writes autopilot parameters, so `MANUAL_OPTIONS = 0` is
a documented operator step in `MAVLINK_SETUP.md`, not something the firmware enforces.

The second half of the decision is the fault policy, and it is deliberately the simplest one
available: **any fault disables the scaling.** Speed scaling is a comfort and assist feature, not
a safety function — a vehicle with full steering authority is the known-good state. So an invalid
speed reading, a missing base value, an unknown autopilot mode or a dead link all resolve to
`scale = 1`. No hold, no conservative fallback speed, no minimum-scale floor, nothing that can
leave the driver with restricted steering because a sensor failed.

## What Changes
- **Steering commands from the autopilot are scaled by road speed on the ESP32.** Between
  `MavlinkInterface::getSteering()` and `SteeringController::setSteeringPercent()`,
  `VehicleController` multiplies the command's deviation from centre by
  `scale = min(1, base / v)` — ArduPilot's own formula, unchanged. `v <= base` and `base <= 0`
  both give `scale = 1`, exactly as ArduPilot reads them.
- **The base speed has two sources, in order.** First the ESP32's own NVS value `steer_sca_base`
  (m/s, NVS namespace `"steering"`), settable from the web portal like the existing calibration
  setters. If it is absent or zero, the autopilot's `MOT_SPD_SCA_BASE`, subscribed over MAVLink
  through the **same** read-only mechanism that already subscribes `SPEED_MAX` (poll every
  `MAVLINK_PARAM_POLL_MS`, stale after `MAVLINK_PARAM_STALE_MS`, accepted only from the learned
  autopilot). If neither is available, there is no scaling, and a rate-limited warning says so.
  The existing `SPEED_MAX` subscription is generalised to carry two parameters; its behaviour does
  not change.
- **MANUAL only.** `MavlinkInterface` starts decoding `HEARTBEAT` instead of only timestamping it,
  and exposes the autopilot's `base_mode` / `custom_mode`. Scaling applies only when the learned
  autopilot reports Rover `custom_mode == 0` (MANUAL). In every other mode the autopilot computes
  steering from speed itself and a second, invisible reduction on top of it is not acceptable. An
  unknown or stale mode is treated as MANUAL, because that is the mode the vehicle is driven in
  and the consequence of being wrong is only that the assist keeps working.
- **Any fault disables scaling.** The scale is applied only while `SpeedSensor::isValid()` is
  true. False for any reason — never pulsed since boot, suspicious-loss latch, not initialised —
  gives `scale = 1`. A valid sensor reading of 0 m/s (parked, or decayed after
  `SPEED_STALE_TIMEOUT_MS`) also gives `scale = 1`, because `v <= base` is the no-scaling case.
- **The scale itself is rate-limited**, at `STEER_SCALE_SLEW_PER_S` per second in both directions,
  modelled on `SPEED_LIMIT_CEILING_SLEW_PCT_S`, so the authority change is felt as a gradual
  weighting rather than a step at the wheel.
- **Observability**: a sixth `NAMED_VALUE_FLOAT`, `STEER_SCA` (the applied scale, 0..1, 1 Hz from
  component 25, alongside `VESC_TEMP` and `VESC_OK`); a `steer_scale` field in the web telemetry
  JSON with the base and its source; and a `[STEER]` log line on change, held off like
  `logSpeedLimitChange`.
- **Bench test override**: a web command `set_test_speed <m/s>` substitutes a speed for the
  scaling calculation only, in RAM, auto-clearing after `STEER_SCALE_TEST_SPEED_MS` (60 s). It is
  shown in the portal and marked `TEST` in the log. It does **not** touch the odometry sent to the
  autopilot, the `SPEED_MAX` limiter, or the transmission interlock — so the bench cannot
  accidentally become a driving configuration.
- **Docs**: `MAVLINK_SETUP.md` gains `MANUAL_OPTIONS = 0` as a required deployment step in the
  autopilot parameter table (`VISO_TYPE = 1` stays as it is), a note that clearing the bit also
  removes ArduPilot's ground-speed-based steering reversal in reverse — harmless here, as the
  reversal then keys off throttle sign and this vehicle's mechanical gearbox never commands
  negative throttle — and `STEER_SCA` in the `NAMED_VALUE_FLOAT` section.

## Impact
- Affected specs:
  - `vehicle-systems` — **ADDED** `Speed-Scaled Steering Authority` and
    `Steering Speed-Scaling Base Value`.
  - `mavlink-interface` — **ADDED** `Autopilot Flight-Mode Awareness`,
    `Autopilot Steering Speed-Scaling Base Parameter Subscription` and
    `Steering Scale Telemetry via NAMED_VALUE_FLOAT`. ADDED rather than MODIFIED throughout: the
    existing reporting requirement is already carried as a MODIFIED block by an unarchived change,
    and each of these is an orthogonal concern that stands on its own.
  - `web-control` — **ADDED** `Steering Speed-Scaling Configuration via Web Interface`.
  - `web-telemetry` — **ADDED** `Steering Speed-Scaling Telemetry`.
- **Archive order**: `add-vesc-esc-telemetry` should be archived **before** this change. Its
  requirement `Steering VESC Telemetry via NAMED_VALUE_FLOAT` says "exactly five"
  `NAMED_VALUE_FLOAT` names; `STEER_SCA` is a sixth name admitted under the same scoped exception,
  and that wording has to be updated to six when this change lands.
- Affected code: `include/Constants.h`, `include/MavlinkInterface.h` + `src/MavlinkInterface.cpp`,
  `include/VehicleController.h` + `src/VehicleController.cpp`, `include/WebPortal.h` +
  `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `data/index.html`, `MAVLINK_SETUP.md`.
- `data/` changes, so deployment needs **both** `pio run -t upload` and `pio run -t uploadfs`.
- **Field migration**: `MANUAL_OPTIONS = 0` on the autopilot is a required commissioning step. A
  vehicle that gets this firmware while `MANUAL_OPTIONS` still has bit 0 set will be scaled
  **twice** — once by the autopilot (from the untrustworthy EKF/GPS speed this change exists to
  escape) and once by the ESP32.

## Out of Scope
- `VISION_POSITION_DELTA` wheel odometry, the `SPEED_MAX` throttle limiter and the transmission
  interlock are untouched. `VISO_TYPE = 1` stays.
- No `PARAM_SET`, and no autopilot parameter is ever written — a repo principle, not a preference.
- The GPS/EKF fallback inside ArduPilot is not fixed; it is avoided.
- `SpeedSensor` is not changed. Its fault detection, its validity rules and its decay policy stay
  exactly as they are — this change only consumes `isValid()` and `getSpeedMs()`.
- The web `set_steering` path is not scaled: it is a bench and maintenance control with no road
  speed behind it.
