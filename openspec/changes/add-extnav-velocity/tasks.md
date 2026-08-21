## 1. Constants
- [x] 1.1 Add `MAVLINK_VISO_ENABLED` (1), `MAVLINK_ATTITUDE_RATE_HZ` (10), `MAVLINK_ATTITUDE_TIMEOUT_MS` (500), `MAVLINK_ATTITUDE_MIN_RATE_HZ` (4.0f) and `MAVLINK_VISO_NEUTRAL_ZERO_KMH` (0.5f) to the MAVLink transport block of `include/Constants.h`, with one-line rationale comments in the existing style.

## 2. MavlinkInterface: inbound ATTITUDE
- [x] 2.1 Add yaw state to `include/MavlinkInterface.h`: `yawRad_`, `lastAttitudeTime_`, `attitudeSeen_`, `attRateWindowCount_`, `lastAttRate_`, `visionSpeedTxCount_`; initialize all in the constructor.
- [x] 2.2 Add public accessors `isYawFresh()`, `getYawAgeMs()`, `getVisionSpeedTxCount()`.
- [x] 2.3 Add `case MAVLINK_MSG_ID_ATTITUDE` to the `update()` dispatch, guarded by `msg.sysid == targetSystem_ && msg.compid == MAV_COMP_ID_AUTOPILOT1`, storing yaw + timestamp and bumping the attitude rate counter via a `handleAttitude()` handler.
- [x] 2.4 Add `requestAttitudeStream()` mirroring `requestServoOutputStream()`: `REQUEST_DATA_STREAM(MAV_DATA_STREAM_EXTRA1, MAVLINK_ATTITUDE_RATE_HZ)` + `MAV_CMD_SET_MESSAGE_INTERVAL(MAVLINK_MSG_ID_ATTITUDE, 1e6/rate µs)`.
- [x] 2.5 Split the re-request gate in `update()` into two independent conditions (servo below `MAVLINK_STREAM_MIN_RATE_HZ`, attitude below `MAVLINK_ATTITUDE_MIN_RATE_HZ`) sharing the `MAVLINK_STREAM_REREQUEST_MS` spacing timer.
- [x] 2.6 Fold the attitude rate into the existing 1 s rate window (no second timer).
- [x] 2.7 Extend the 1 Hz `[MAV]` debug line with `viso:<count> yaw:<age>ms att:<rate>Hz`.

## 3. MavlinkInterface: outbound VISION_SPEED_ESTIMATE
- [x] 3.1 Add `int8_t travelDirection;` to `MavlinkInterface::StateReport` with a unit comment (+1 forward gear, -1 reverse, 0 neutral/unknown), keeping the struct dependency-free.
- [x] 3.2 Add `#include <esp_timer.h>` to `src/MavlinkInterface.cpp` and declare `void sendVisionSpeedEstimate(const StateReport&);`.
- [x] 3.3 Implement the four gates: `!isLinkUp()`, `!state.speedValid`, `!isYawFresh()`, and `travelDirection == 0 && speedKmh > MAVLINK_VISO_NEUTRAL_ZERO_KMH` — each returning without sending. Healthy zeros must still be sent.
- [x] 3.4 Pack and send: `usec = esp_timer_get_time()`, `x = v*cosf(yawRad_)`, `y = v*sinf(yawRad_)`, `z = 0`, `cov[0] = NAN`, `reset_counter = 0`, with `v = (speedKmh/3.6f) * travelDirection`; increment `visionSpeedTxCount_`.
- [x] 3.5 Call it from the existing `MAVLINK_REPORT_TX_MS` block in `report()`, after the `VFR_HUD` send, guarded by `#if MAVLINK_VISO_ENABLED`.

## 4. Vehicle plumbing
- [x] 4.1 Add `int8_t getTravelDirection() const;` to `include/VehicleController.h` beside `getCurrentGearString()`, with a doc comment stating it uses the PHYSICAL gear and why.
- [x] 4.2 Implement it in `src/VehicleController.cpp` mapping `transmission_.getPhysicalGear()`: `GEAR_REVERSE → -1`, `GEAR_LOW`/`GEAR_HIGH → +1`, `GEAR_NEUTRAL`/`GEAR_UNKNOWN` (default) → `0`.
- [x] 4.3 Populate `report.travelDirection = vehicleController.getTravelDirection();` in `src/main.cpp` alongside the existing `report.*` assignments.

## 5. Documentation (`MAVLINK_SETUP.md`)
- [x] 5.1 Fix the stale wiring table: RX is GPIO18 (`PIN_MAVLINK_RX`), TX is GPIO17 (`PIN_MAVLINK_TX`); drop the stale "do not use GPIO 9" note; add the X9 header / BSS138 level-shifter / 115200-only note. GPIO8 is now the hall speed sensor.
- [x] 5.2 Correct the servo stream rate figure from 50 Hz to 25 Hz (`MAVLINK_SERVO_OUTPUT_RATE_HZ`).
- [x] 5.3 Add a `VISION_SPEED_ESTIMATE` (103) row to the reported-state table: component 25, 5 Hz, gated.
- [x] 5.4 Fix the stale claim that vehicle speed is not reported — `VFR_HUD.groundspeed` carries the hall-sensor speed.
- [x] 5.5 Add the "External navigation velocity (EKF3 wheel-speed aiding)" section with the ArduPilot parameter table (`VISO_TYPE`, `VISO_VEL_M_NSE`, `VISO_DELAY_MS`, `VISO_POS_X/Y/Z`, `VISO_QUAL_MIN`, `EK3_SRC1_VELXY`) and both caveats: the fmuv3-vs-fmuv2 build / `VISO_TYPE` existence check, and keeping `EK3_SRC1_POSXY=3` if any GPS is fitted (ArduPilot issue #23485).

## 6. Build gate
- [x] 6.1 `pio run` completes with no errors and no new warnings.

## 7. Bench verification
- [x] 7.1 ESP32 alone with `DebugFeature::MAVLINK` on and no autopilot: the 1 Hz line shows `viso:0` and a stale yaw — confirm the interface stays silent rather than emitting zero-velocity spam. (Done 2026-08-21 bench: `viso:0 yaw:stale att:0.0Hz` at 1 Hz, link down, zero VISION_SPEED_ESTIMATE TX over a 2-minute window.)
- [ ] 7.2 Bridge ArduPilot Rover SITL to UART1 through a 3.3 V USB-TTL adapter (`sim_vehicle.py -v Rover --serial1=uart:/dev/ttyUSB0:115200`) and drive the hall input with a signal generator — this exercises the real `handle_vision_speed_estimate` path and proves `VISO_TYPE` exists in the build.
- [ ] 7.3 Mission Planner → MAVLink Inspector → sysid 1 / compid 25: confirm `VISION_SPEED_ESTIMATE` arrives at ~5 Hz with plausible `x`/`y`.
- [ ] 7.4 Gate matrix — force each gate in turn (pull the hall signal mid-motion to trip the suspicious latch; stop the `ATTITUDE` stream; neutral with the wheel spinning; unplug the gear input expander) and confirm the TX counter **stops** in each case rather than the message continuing with zeros.
- [ ] 7.5 Sign and frame checks: reverse gear flips the sign of `x`/`y`; at fixed headings `x ≈ v·cos(yaw)` and `y ≈ v·sin(yaw)` against the reported `ATTITUDE.yaw`.

## 8. On-vehicle verification
- [ ] 8.1 Set the autopilot parameters from `MAVLINK_SETUP.md`; first confirm `VISO_TYPE` actually exists in the full parameter list (fmuv3 build check).
- [ ] 8.2 Steady straight-line drive: EKF velocity tracks `VFR_HUD.groundspeed`; dataflash `XKF3 IVN/IVE` innovations are small and zero-mean and `XKF4 SV` stays below ~0.3.
- [ ] 8.3 Reverse leg: confirm no innovation step of roughly twice the speed (would indicate a sign error).
- [ ] 8.4 Stop in neutral: EKF velocity is held near zero by the zero-velocity updates; a gear-change sequence causes only a brief silence.
- [ ] 8.5 Tune `VISO_DELAY_MS` from the 150 ms starting point against the innovations (spikes only during acceleration/deceleration indicate a delay mismatch) and record the final value back into `MAVLINK_SETUP.md`. A persistent speed-proportional bias instead indicates a pulses-per-revolution / circumference calibration error — recalibrate rather than retune.

## 9. Validate
- [x] 9.1 `openspec validate add-extnav-velocity --strict` passes with no errors.
