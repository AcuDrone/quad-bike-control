# Change: Feed hall wheel speed into the EKF3 as body-frame odometry (VISION_POSITION_DELTA)

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

**This change was originally written around `VISION_SPEED_ESTIMATE` (earth-frame NED velocity,
requiring an inbound `ATTITUDE` subscription and a yaw rotation on the ESP32). On 2026-08-21 that
design was falsified — in the ArduPilot-4.7 source and then on the bench against a real Pixhawk
2.4.8.** EKF3 never *starts* aiding from external-navigation velocity: `readyToUseExtNav()` is
position-only, so with `EK3_SRC1_VELXY=6` and no position source the filter stays in
constant-position mode, where the velocity observation is given `EK3_NOAID_M_NSE` noise and is
mathematically discarded. The bench confirmed it: `VISION_SPEED_ESTIMATE` at 5 Hz with the origin
set left the EKF flags stuck at `0x00A7` (const-pos) and the reported velocity at zero.

The message that *does* work is `VISION_POSITION_DELTA` — the only MAVLink message routed to
`writeBodyFrameOdom()`, which `readyToUseBodyOdm()` accepts under `EK3_SRC1_VELXY=6`. Switching to
it on the same bench moved EKF3 into `AID_RELATIVE` (flags `0x012F`) and
`GLOBAL_POSITION_INT` velocity tracked an injected 10 km/h. Full citations and bench evidence are
in `design.md`.

Body frame is also strictly simpler: the delta is measured along the vehicle's own longitudinal
axis, so the yaw dependency — and with it the entire inbound `ATTITUDE` path, its stream request,
its staleness gate and the circular "aid the estimator with its own yaw" argument — **disappears**.

## What Changes
- **New outbound message: `VISION_POSITION_DELTA` (11011, ArduPilotMega dialect)** from component
  25, in the existing `MAVLINK_REPORT_TX_MS` (200 ms / 5 Hz) block, right after `VFR_HUD`.
  Body-frame: `delta_position = {v·dt, 0, 0}` (x forward, metres), `delta_angle = {0, 0, 0}` (the
  autopilot's own gyros own the rotation), `confidence = 100` when healthy,
  `time_usec = esp_timer_get_time()` (64-bit monotonic boot µs — never 32-bit `micros()`, which
  wraps every ~71 min), and `time_delta_usec` = the **actually measured** interval since the last
  send, not an assumed 200 ms.
- **Delta re-baselining.** A delta is an integral, so it is only valid over an interval that was
  actually observed. Every gate invalidates the baseline on the way out, and a gap longer than
  `MAVLINK_VISO_MAX_DT_US` trips the same rule, so the first send after any break **skips one
  interval** rather than multiplying the current speed by a long-dead `dt`.
- **Silence over zeros**, now three gates rather than four: link down, `!speedValid` (this covers
  the sensor's `suspicious_` wire-fault latch, so a decayed 0 from a cut sensor wire can never be
  fused as "stopped"), or rolling in NEUTRAL above `MAVLINK_VISO_NEUTRAL_ZERO_KMH` (direction sign
  unrecoverable). **Healthy zeros ARE sent**: a stationary zero-motion update is the single most
  valuable drift constraint available with no GPS.
- **Direction still comes from the PHYSICAL gear**, not the assumed/commanded one:
  `VehicleController::getTravelDirection()` maps `transmission_.getPhysicalGear()` to
  `−1` REVERSE / `+1` LOW·HIGH / `0` NEUTRAL·UNKNOWN. An EKF measurement must never be signed by
  an assumption. `MavlinkInterface::StateReport.travelDirection` and its `src/main.cpp` population
  are **unchanged** from the previous revision of this change.
- **REMOVED — the entire inbound attitude path**, which the body-frame message makes dead weight:
  the `ATTITUDE` case, `handleAttitude()`, `requestAttitudeStream()`, `yawRad_`,
  `lastAttitudeTime_`, `attitudeSeen_`, `attRateWindowCount_`, `lastAttRate_`, `isYawFresh()`,
  `getYawAgeMs()`, the attitude half of the split stream re-request gate (the gate reverts to the
  single servo-stream condition it was before), and the `MAVLINK_ATTITUDE_*` constants. Nothing
  else in `src/` or `include/` referenced any of them.
- **Constants**: the `MAVLINK_VISO_*` block in `include/Constants.h` keeps `MAVLINK_VISO_ENABLED`
  and `MAVLINK_VISO_NEUTRAL_ZERO_KMH`, drops the three attitude constants, and gains
  `MAVLINK_VISO_CONFIDENCE` (100) and `MAVLINK_VISO_MAX_DT_US` (1 s).
- The 1 Hz `[MAV]` debug line loses its `yaw:`/`att:` fields and carries
  `viso:<count> dt:<ms>` instead — enough to see both that the message is flowing and that the
  baseline is not being repeatedly re-established by a flapping gate.
- **`MAVLINK_SETUP.md`**: the extnav section is rewritten for the delta message, with the verified
  parameter table (which `VISO_*` parameters actually apply on this path, and which do not), the
  GPS-less vs GPS-fitted `GPS1_TYPE` / `EK3_SRC1_POSXY` split, and the EKF-origin prerequisite.
- **No** new task, ISR, timer or allocation: the parse loop loses one `case` and the existing 5 Hz
  block gains one pack/write, on the single cooperative loop.

## Impact
- Affected specs:
  - `mavlink-interface` — **ADDED** `External Navigation Odometry Reporting`. The previously
    proposed `Autopilot Attitude Subscription` requirement is dropped from this change entirely
    (never archived, so nothing to REMOVE). Deliberately **not** MODIFIED: the existing
    `Vehicle State Reporting via Standard MAVLink Messages` requirement is already carried as a
    MODIFIED block by two unarchived changes (`add-hall-speed-sensor`, `add-ecu-telemetry-pids`),
    and a third rewrite of the same requirement would make archive order load-bearing. The new
    requirement stands alone, so ADDED keeps this change order-independent.
- Affected code: `include/Constants.h` (`MAVLINK_VISO_*` block), `include/MavlinkInterface.h` +
  `src/MavlinkInterface.cpp` (`sendVisionPositionDelta()`, odometry baseline state, debug line,
  and the removal of the whole attitude path), `MAVLINK_SETUP.md`.
  `include/VehicleController.h` + `src/VehicleController.cpp` (`getTravelDirection()`) and
  `src/main.cpp` (one `report.*` assignment) are **unchanged** by this revision.
- No web, i18n, NVS or LittleFS surface — **no `pio run -t uploadfs` required**, a firmware flash
  is sufficient.
- **Autopilot-side configuration is required before anything is fused**: `VISO_TYPE=1`,
  `EK3_SRC1_VELXY=6`, and — on this GPS-less vehicle — `GPS1_TYPE=0` with `EK3_SRC1_POSXY=0`
  (full table in `MAVLINK_SETUP.md`). Until then the ESP32 sends the message and ArduPilot ignores
  it — no regression, and every existing message is untouched.
- **A GPS-less vehicle also needs an EKF origin set once per boot**, from the ground station. The
  ESP32 deliberately does not send `SET_GPS_GLOBAL_ORIGIN` — see the open question in `design.md`.
- **Firmware-build caveat**: `VISO_*` parameters exist only in builds that include
  `AP_VisualOdom` — the fmuv3 (2 MB) Pixhawk 2.4.8 target, **not** fmuv2 (1 MB). Confirmed present
  on the actual airframe on 2026-08-21.
