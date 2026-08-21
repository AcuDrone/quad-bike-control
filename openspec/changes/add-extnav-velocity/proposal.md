# Change: Feed hall wheel speed into the EKF3 as external-navigation velocity (VISION_SPEED_ESTIMATE)

## Why
The hall-effect wheel speed sensor added by `add-hall-speed-sensor` (GPIO8 / X2, PCNT, 5 Hz
sampling, ~28.4 mm per pulse) is the only speed source this vehicle has, and today it is
**display-only**: it reaches `VFR_HUD.groundspeed` and the web UI, and nothing else. Nothing
reaches the autopilot's state estimator.

The quad has **no reliable GPS**. Without a velocity source the Rover 4.7.0 EKF3 dead-reckons on
IMU alone, so velocity and position drift within seconds and the operator has no trustworthy
speed on the HUD, no usable velocity for any assisted mode, and no constraint that says "the
vehicle is stopped" while it is standing still. A wheel-speed measurement is exactly the aiding
source the estimator is missing — it is accurate at low speed, unaffected by sky view, and
already measured 5 times a second.

This change sends that speed to the autopilot as a **fusable measurement**
(`VISION_SPEED_ESTIMATE`, msg 103 → `AP_VisualOdom_MAV` → EKF3 ExternalNav velocity) instead of
a display value, and adds the one piece of inbound data required to do that correctly: the
autopilot's `ATTITUDE` yaw, needed to rotate a scalar wheel speed into the earth-frame NED
velocity vector the EKF consumes.

## What Changes
- **New inbound subscription: `ATTITUDE` (msg 30)**, requested at `MAVLINK_ATTITUDE_RATE_HZ`
  (10 Hz) with the same belt-and-braces pair the servo stream already uses
  (`REQUEST_DATA_STREAM(MAV_DATA_STREAM_EXTRA1)` + `MAV_CMD_SET_MESSAGE_INTERVAL`). Only the
  learned autopilot (`sysid == targetSystem_ && compid == MAV_COMP_ID_AUTOPILOT1`) may supply
  yaw — a second component on the link must not be able to rotate our velocity vector.
- **Split the stream re-request gate.** Today one gate re-requests `SERVO_OUTPUT_RAW` whenever
  the command rate is low. It becomes two independent gates so a healthy servo stream cannot
  mask a dead attitude stream (and vice versa): servo re-requested below
  `MAVLINK_STREAM_MIN_RATE_HZ`, attitude below `MAVLINK_ATTITUDE_MIN_RATE_HZ`.
- **New outbound message: `VISION_SPEED_ESTIMATE` (103)** from component 25, in the existing
  `MAVLINK_REPORT_TX_MS` (200 ms / 5 Hz) block, right after `VFR_HUD`. Earth-frame NED, m/s,
  signed: `x = v·cos(yaw)`, `y = v·sin(yaw)`, `z = 0`, with
  `v = (speedKmh / 3.6) · travelDirection`. `usec` comes from `esp_timer_get_time()` (64-bit
  monotonic boot µs — never 32-bit `micros()`, which wraps every ~71 min), `covariance[0] = NaN`
  (let ArduPilot use `VISO_VEL_M_NSE`), `reset_counter = 0`.
- **Silence over zeros.** The message is skipped entirely — not sent as zeros — when any of four
  gates fails: link down, `!speedValid` (this covers the sensor's `suspicious_` wire-fault latch,
  so a decayed 0 from a cut sensor wire can never be fused as "stopped"), yaw stale
  (`MAVLINK_ATTITUDE_TIMEOUT_MS`), or rolling in NEUTRAL above
  `MAVLINK_VISO_NEUTRAL_ZERO_KMH` (direction sign unrecoverable). **Healthy zeros ARE sent**: a
  stationary zero-velocity update is the single most valuable drift constraint available with no
  GPS.
- **Direction comes from the PHYSICAL gear**, not the assumed/commanded one:
  `VehicleController::getTravelDirection()` maps `transmission_.getPhysicalGear()` (opto switches
  on the input expander) to `−1` REVERSE / `+1` LOW·HIGH / `0` NEUTRAL·UNKNOWN. An EKF
  measurement must never be signed by an assumption.
- **`MavlinkInterface::StateReport` gains `int8_t travelDirection`**, populated in `src/main.cpp`.
  The sign policy lives in the vehicle layer; the MAVLink interface stays a transport.
- **Five new constants** in the MAVLink block of `include/Constants.h`, and the 1 Hz `[MAV]`
  debug line is extended with `viso:<count> yaw:<age> att:<rate>` so all four gates are
  observable on serial without a GCS.
- **`MAVLINK_SETUP.md`**: new `VISION_SPEED_ESTIMATE` row, a new "External navigation velocity
  (EKF3 wheel-speed aiding)" section with the ArduPilot parameter table, plus in-passing repair
  of the stale wiring table (it still lists the pre-`Control_v0` GPIO8/GPIO15 pair — GPIO8 is now
  the speed sensor itself) and of the stale claim that vehicle speed is not reported.
- **No** new task, ISR, timer or allocation: one extra `case` in the existing parse loop and one
  extra pack/write in the existing 5 Hz block, on the single cooperative loop.

## Impact
- Affected specs:
  - `mavlink-interface` — **ADDED** `Autopilot Attitude Subscription` and **ADDED**
    `External Navigation Velocity Reporting`. Deliberately **not** MODIFIED: the existing
    `Vehicle State Reporting via Standard MAVLink Messages` requirement is already carried as a
    MODIFIED block by two unarchived changes (`add-hall-speed-sensor`, `add-ecu-telemetry-pids`),
    and a third rewrite of the same requirement would make archive order load-bearing. Both new
    requirements stand alone, so ADDED keeps this change order-independent.
- Affected code: `include/Constants.h` (5 constants), `include/MavlinkInterface.h` +
  `src/MavlinkInterface.cpp` (ATTITUDE handling, `requestAttitudeStream()`, split re-request
  gate, `sendVisionSpeedEstimate()`, `StateReport.travelDirection`, debug line),
  `include/VehicleController.h` + `src/VehicleController.cpp` (`getTravelDirection()`),
  `src/main.cpp` (one `report.*` assignment), `MAVLINK_SETUP.md`.
- No web, i18n, NVS or LittleFS surface — **no `pio run -t uploadfs` required**, a firmware flash
  is sufficient.
- **Autopilot-side configuration is required before anything is fused**: `VISO_TYPE=1` and
  `EK3_SRC1_VELXY=6` (full table in `MAVLINK_SETUP.md`). Until then the ESP32 sends the message
  and ArduPilot ignores it — no regression, and every existing message is untouched.
- **Firmware-build caveat**: `VISO_*` parameters exist only in builds that include
  `AP_VisualOdom` — the fmuv3 (2 MB) Pixhawk 2.4.8 target, **not** fmuv2 (1 MB). Verify the
  parameter exists on the actual airframe before relying on this path.
