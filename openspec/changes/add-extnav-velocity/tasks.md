> **Revised 2026-08-21.** The original `VISION_SPEED_ESTIMATE` (earth-frame velocity) design was
> falsified in the ArduPilot-4.7 source and on the bench — see `design.md`. This list is the
> body-frame `VISION_POSITION_DELTA` design. Bench items already satisfied under the old design
> are re-stated against the new code rather than carried over as ticks.

## 1. Constants
- [x] 1.1 Rework the `MAVLINK_VISO_*` block of `include/Constants.h`: keep `MAVLINK_VISO_ENABLED` (1) and `MAVLINK_VISO_NEUTRAL_ZERO_KMH` (0.5f), add `MAVLINK_VISO_CONFIDENCE` (100.0f) and `MAVLINK_VISO_MAX_DT_US` (1000000), each with a one-line rationale comment in the existing style.
- [x] 1.2 Remove `MAVLINK_ATTITUDE_RATE_HZ`, `MAVLINK_ATTITUDE_TIMEOUT_MS` and `MAVLINK_ATTITUDE_MIN_RATE_HZ`, and rewrite the block comment for the body-frame design.

## 2. MavlinkInterface: remove the inbound attitude path
- [x] 2.1 Delete the `case MAVLINK_MSG_ID_ATTITUDE` from the `update()` dispatch and the `handleAttitude()` handler.
- [x] 2.2 Delete `requestAttitudeStream()`.
- [x] 2.3 Revert the split stream re-request gate to the single `lastCmdRate_ < MAVLINK_STREAM_MIN_RATE_HZ` condition, and drop the attitude fold-in from the 1 s rate window.
- [x] 2.4 Delete the yaw state (`yawRad_`, `lastAttitudeTime_`, `attitudeSeen_`, `attRateWindowCount_`, `lastAttRate_`) and the `isYawFresh()` / `getYawAgeMs()` accessors, including their `begin()` initialisation.
- [x] 2.5 Verify by grep that nothing else in `src/` or `include/` referenced any of the above before removing it.

## 3. MavlinkInterface: outbound VISION_POSITION_DELTA
- [x] 3.1 Rename the TX counter to `odomTxCount_` and its accessor to `getOdomTxCount()`; add `lastOdomTimeUs_` (`int64_t`, 0 = baseline invalid) and `lastOdomDtMs_` (debug), initialised in the constructor and reset in `begin()`.
- [x] 3.2 Replace `sendVisionSpeedEstimate()` with `sendVisionPositionDelta(const StateReport&)`, keeping the `#include <esp_timer.h>` already present.
- [x] 3.3 Implement the three gates — `!isLinkUp()`, `!state.speedValid`, and `travelDirection == 0 && speedKmh > MAVLINK_VISO_NEUTRAL_ZERO_KMH` — each invalidating the baseline (`lastOdomTimeUs_ = 0`) and returning. Healthy zeros must still be sent.
- [x] 3.4 Implement the dt re-baseline rule: no baseline, a non-positive dt, or a dt above `MAVLINK_VISO_MAX_DT_US` re-establishes the baseline from the current clock and skips exactly one interval.
- [x] 3.5 Pack and send: `time_usec = esp_timer_get_time()`, `time_delta_usec = dt`, `angle_delta = {0,0,0}`, `position_delta = {v*dt, 0, 0}`, `confidence = MAVLINK_VISO_CONFIDENCE`, with `v = (speedKmh/3.6f) * travelDirection`; increment `odomTxCount_`.
- [x] 3.6 Call it from the existing `MAVLINK_REPORT_TX_MS` block in `report()`, after the `VFR_HUD` send, guarded by `#if MAVLINK_VISO_ENABLED`.
- [x] 3.7 Replace the `viso:/yaw:/att:` debug line with `viso:<count> dt:<ms>`.
- [x] 3.8 Update the class doc comment on `MavlinkInterface` and the `StateReport.travelDirection` comment to name `VISION_POSITION_DELTA`.

## 4. Vehicle plumbing (unchanged — verify only)
- [x] 4.1 `VehicleController::getTravelDirection()` (physical gear → −1/+1/0) is kept exactly as-is; confirm no edit was needed.
- [x] 4.2 `report.travelDirection = vehicleController.getTravelDirection();` in `src/main.cpp` is kept exactly as-is; confirm no edit was needed.

## 5. Documentation (`MAVLINK_SETUP.md`)
- [x] 5.1 Replace the `VISION_SPEED_ESTIMATE` row in the reported-state table with `VISION_POSITION_DELTA` (11011), and update footnote ² for the three gates.
- [x] 5.2 Remove the `ATTITUDE` stream from the inbound-streams paragraph (`SERVO_OUTPUT_RAW` is now the only subscription) and drop the `SRx_EXTRA1` note.
- [x] 5.3 Rewrite the "External navigation velocity" section as "External navigation odometry": the falsification of the velocity path, the body-frame message, and why no yaw is needed.
- [x] 5.4 Rewrite the parameter table against the verified source: which `VISO_*` apply to the delta path (`VISO_TYPE`, `VISO_DELAY_MS`, `VISO_POS_*`, `VISO_ORIENT`) and which do **not** (`VISO_QUAL_MIN`, `VISO_VEL_M_NSE`, `VISO_SCALE`), plus the confidence → `EK3_VIS_VERR_MIN/MAX` mapping and the `velErr < 1.0` aiding gate.
- [x] 5.5 Document the GPS-less vs GPS-fitted split (`GPS1_TYPE` 0 vs 2, `EK3_SRC1_POSXY` 0 vs 3), including the ArduPilot-4.7 `GPS_TYPE` → `GPS1_TYPE` rename, and the once-per-boot EKF origin requirement.
- [x] 5.6 Sweep the whole document for stale references to the earth-frame message, the yaw rotation and the attitude stream.

## 6. Build gate
- [x] 6.1 `pio run` completes with no errors and no new warnings (clean rebuild 2026-08-21: SUCCESS, zero warnings originating in `src/` or `include/`).

## 7. Bench verification
- [x] 7.1 Autopilot parameters set and the delta path proven to fuse. (Done 2026-08-21, real Pixhawk 2.4.8 / Rover 4.7.0 fmuv3: `VISO_TYPE` confirmed present; `VISO_TYPE=1`, `EK3_SRC1_VELXY=6`, `EK3_SRC1_POSXY=0`, `GPS1_TYPE=0`, origin set from the GCS. `VISION_SPEED_ESTIMATE` @ 5 Hz left EKF flags at `0x00A7` (const-pos) with velocity 0; switching to `VISION_POSITION_DELTA` @ 5 Hz moved EKF3 to `AID_RELATIVE` (flags `0x012F`) and `GLOBAL_POSITION_INT` velocity tracked an injected 10 km/h at |v| ≈ 2.6–2.7 m/s. Speed and direction were bench-injected — the gear sensor was unplugged and the hall path was not driven.)
- [x] 7.1b SITL end-to-end protocol validation (2026-08-21, Rover 4.7 SITL on the bench Jetson, injected via MAVLink2Rest at 5 Hz): with `VISO_TYPE=1`, `EK3_SRC1_VELXY=6`, `EK3_SRC1_POSXY=0`, `GPS1_TYPE=0` and an EKF origin set, EKF3 engaged AID_RELATIVE (flags 0x00A7→0x012F) within seconds; fused velocity tracked a scripted profile within ~1 % at 10 km/h cruise, decayed to zero on stop, and inverted exactly on a reverse leg (−1.39 m/s → |v|=1.38 with both NED components mirrored). `PreArm: VisOdom` went not-healthy→healthy with the stream. Validates the message semantics, param set, dt integration and sign convention — the ESP32 firmware path itself is covered by 7.2-7.6.
- [x] 7.1c SITL steering-scaling proof (2026-08-21, same rig): with `MANUAL_OPTIONS=1`, `MOT_SPD_SCA_BASE=2.5`, MANUAL mode armed, full-lock steering via RC override, the injected odometry attenuated `servo1_raw` exactly per `base/speed` — 1900 at standstill, 1860 (scale 0.90) at 10 km/h, 1620 (scale 0.30) at 30 km/h. This closes the loop on the feature's purpose: wheel speed reaches the EKF and the EKF speed drives Manual-mode steering desensitization. Note: instantaneous speed STEPS (8.3→1.0 m/s) are innovation-rejected and can wander the estimate until a timeout reset — physically real ramps track cleanly; reinforces the silence-over-zeros gate design (a dying sensor must go silent, not step to zero).
- [ ] 7.2 Re-run the silent-when-alone check against the NEW code: ESP32 with `DebugFeature::MAVLINK` on and no autopilot — the 1 Hz line must show `viso:0` and no `VISION_POSITION_DELTA` on the wire. (The old design passed this on 2026-08-21, but the gate code has been rewritten.)
- [ ] 7.3 Mission Planner → MAVLink Inspector → sysid 1 / compid 25: confirm `VISION_POSITION_DELTA` arrives at ~5 Hz with `time_delta_usec` ≈ 200 000 and `position_delta[0]` ≈ `v·0.2`.
- [ ] 7.4 Gate matrix — force each gate in turn (pull the hall signal mid-motion to trip the suspicious latch; drop the link; neutral with the wheel spinning; unplug the gear input expander) and confirm the TX counter **stops** in each case rather than the message continuing with zeros.
- [ ] 7.5 Re-baseline check: close a gate for several seconds, reopen it, and confirm the first message after the gap is skipped and the next carries a `time_delta_usec` near 200 000 — never the length of the outage. The serial `dt:` field should never report a value near `MAVLINK_VISO_MAX_DT_US`.
- [ ] 7.6 Sign check on the bench: reverse gear flips the sign of `position_delta[0]`.

## 8. On-vehicle verification
- [ ] 8.1 **Prerequisite:** calibrate the compass. The body-frame delta is rotated by the EKF's own attitude, so a bad heading now silently steers the fused travel direction with nothing on the ESP32 side to gate it.
- [ ] 8.2 Fill in `VISO_POS_X/Y/Z` with the measured lever arm from the IMU to the measuring wheel hub.
- [ ] 8.3 Real-wheel fusion check (the bench run used an injected speed): drive the hall sensor from the actual wheel and confirm the EKF velocity tracks `VFR_HUD.groundspeed` on a steady straight-line run.
- [ ] 8.4 Dataflash `XKF3 IVN/IVE` innovations are small and zero-mean and `XKF4 SV` stays below ~0.3.
- [ ] 8.5 Reverse leg: confirm no innovation step of roughly twice the speed (would indicate a sign error).
- [ ] 8.6 Stop in neutral: EKF velocity is held near zero by the zero-motion updates; a gear-change sequence causes only a brief silence.
- [ ] 8.7 Tune `VISO_DELAY_MS` from its 10 ms default toward the ~150 ms the latency budget suggests, against the innovations (spikes only during acceleration/deceleration indicate a delay mismatch), and record the final value back into `MAVLINK_SETUP.md`. A persistent speed-proportional bias instead indicates a pulses-per-revolution / circumference calibration error — recalibrate rather than retune.
- [ ] 8.8 Settle the EKF-origin workflow (GCS-set each boot vs a stored origin in firmware) and record the decision — see the open question in `design.md`.

## 9. Validate
- [x] 9.1 `openspec validate add-extnav-velocity --strict` passes with no errors.
