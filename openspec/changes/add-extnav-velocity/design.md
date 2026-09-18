## Context
The ESP32-S3 (`Control_v0`) is MAVLink 2 sysid 1 / compid 25 on the Pixhawk's TELEM2 ↔ UART1
(GPIO18 RX / GPIO17 TX, 115200, header X9, BSS138 level shifter). It already:

- decodes `SERVO_OUTPUT_RAW` as its command input (`src/MavlinkInterface.cpp:81-92`),
- learns the autopilot's addressing from `HEARTBEAT`,
- re-requests the servo stream while its measured rate is low,
- sends `HEARTBEAT` (1 Hz) plus `EFI_STATUS` + `VFR_HUD` every `MAVLINK_REPORT_TX_MS` = 200 ms,
  where `VFR_HUD.groundspeed` already carries the hall wheel speed.

The vehicle runs **ArduPilot Rover 4.7.0** on a Pixhawk 2.4.8 (fmuv3 build) and has **no reliable
GPS**. The hall sensor (`include/SpeedSensor.h`, PCNT on GPIO8/X2, sampled at 5 Hz, ~28.4 mm/pulse)
is the only speed measurement on the vehicle, and it currently informs nothing but a display field.

Everything here runs on the existing single cooperative loop: no new task, ISR or timer, and all
rate limiting is `millis()`-based, matching the rest of the firmware.

## Goals / Non-Goals
- **Goals**: deliver the hall wheel speed to the EKF3 as a measurement it will *actually fuse*;
  never fuse a measurement whose validity or sign is in doubt; keep every suppression reason
  visible on the serial debug line; document the autopilot-side parameters that are verified to
  apply on this path.
- **Non-Goals**: no position or attitude output; no quadrature wheel-encoder emulation; no new
  sensor hardware; no change to command decoding, fail-safe behaviour or any existing outbound
  message; no ground-station plugin work (`VISION_POSITION_DELTA` needs none — ArduPilot consumes
  it directly); no `SET_GPS_GLOBAL_ORIGIN` from the ESP32 (see Open Questions).

## Decisions

### Transport choice: `VISION_POSITION_DELTA`, after `VISION_SPEED_ESTIMATE` was falsified

This section supersedes the original decision. The first revision of this change chose
`VISION_SPEED_ESTIMATE` (msg 103) and built an inbound `ATTITUDE` subscription to rotate the
scalar wheel speed into earth-frame NED. **On 2026-08-21 that path was falsified twice — in the
ArduPilot-4.7 source, and then on the bench against a real Pixhawk 2.4.8 running Rover 4.7.0.**

#### Source evidence (ArduPilot-4.7, `libraries/AP_NavEKF3/`, verified by reading)

EKF3 will **never begin fusing** external-navigation velocity from `EK3_SRC1_VELXY=6` unless
aiding is *already* active from some other source:

- `readyToUseExtNav()` (`AP_NavEKF3_Control.cpp:612`) is **position-only** — it gates on
  `getPosXYSource() == EXTNAV` and `extNavDataToFuse`. Velocity has no equivalent entry point, so
  `EK3_SRC1_VELXY=6` alone never satisfies it.
- `readyToUseBodyOdm()` (`AP_NavEKF3_Control.cpp:562`) **does** accept
  `useVelXYSource(EXTNAV)` (`:565`) — but it requires *body-odometry* data
  (`:572`, `bodyOdmMeasTime_ms` fresh within 200 ms **and** `bodyOdmDataNew.velErr < 1.0f`), and
  the only thing that ever populates that buffer is `writeBodyFrameOdom()`
  (`AP_NavEKF3_Measurements.cpp:113`). `VISION_SPEED_ESTIMATE` does not reach it: it lands in
  `writeExtNavVelData()` via `AP_VisualOdom_MAV::handle_vision_speed_estimate`
  (`AP_VisualOdom_MAV.cpp:71-88`). **`writeBodyFrameOdom()` is fed by exactly one MAVLink
  message — `VISION_POSITION_DELTA`**, through
  `AP_VisualOdom_Backend::handle_vision_position_delta_msg` (`AP_VisualOdom_Backend.cpp:41` →
  `AP::ahrs().writeBodyFrameOdom(...)` at `:62`).
- While the filter sits in constant-position mode, the horizontal velocity observation is given
  `EK3_NOAID_M_NSE` noise (`AP_NavEKF3_PosVelFusion.cpp:718`,
  `R_OBS[0] = sq(constrain_ftype(_noaidHorizNoise, 0.5, 50.0))`). At the default 10 m/s that is
  `R = 100`, so the measurement is mathematically discarded. `ResetVelocity()` additionally zeroes
  the velocity states while the filter is not `AID_ABSOLUTE`.

In short: with velocity-only ExtNav on a GPS-less vehicle, `VISION_SPEED_ESTIMATE` is a message
ArduPilot receives, logs, and never acts on. The chicken-and-egg is structural, not a tuning
problem.

#### Bench evidence (2026-08-21, real Pixhawk 2.4.8, Rover 4.7.0)

| Configuration | Result |
|---------------|--------|
| `VISION_SPEED_ESTIMATE` @ 5 Hz, EKF origin set, `EK3_SRC1_VELXY=6` | EKF flags stuck at **`0x00A7`** (constant-position), reported velocity **0**. No fusion, indefinitely. |
| `VISION_POSITION_DELTA` @ 5 Hz (body-frame `delta_position = v·dt` forward, `delta_angle` zeros, `confidence = 100`, real `dt` in `time_delta_usec`) | EKF3 entered **`AID_RELATIVE`** (flags **`0x012F`**), and `GLOBAL_POSITION_INT` velocity tracked an injected 10 km/h at **\|v\| ≈ 2.6–2.7 m/s** (10 km/h = 2.778 m/s). |

Both runs reached AP through `AP_VisualOdom` with `VISO_TYPE=1` and `EK3_SRC1_VELXY=6`. The
working run had `EK3_SRC1_POSXY=0` and `GPS1_TYPE=0`; the vehicle's pre-bench values were
`EK3_SRC1_POSXY=3` / `GPS1_TYPE=2`, which must be restored if a GPS is ever fitted.

#### Rejected alternatives

| Mechanism | Verdict |
|-----------|---------|
| `VISION_SPEED_ESTIMATE` (msg 103) | **Rejected — falsified.** Reaches `writeExtNavVelData()`, which no `readyToUse*` predicate can bootstrap from on a GPS-less vehicle. Verified dead in source *and* on the bench (above). This was the previous design. |
| `WHEEL_DISTANCE` (msg 9000) | **Dead end.** ArduPilot only *transmits* this message; there is no receive handler, so nothing on the vehicle would ever read it. |
| `AP_WheelEncoder` (`WENC_TYPE=1`) | Works in principle — `readyToUseBodyOdm()` also accepts `useVelXYSource(WHEEL_ENCODER)` — but it is a **quadrature-pulse** driver: it wants A/B wires into two Pixhawk AUX pins. That means re-wiring the hall signal away from the ESP32 (or splitting it) and fabricating a phase-B signal we do not have. Declined — extra wiring for no extra information. |
| **`VISION_POSITION_DELTA` (msg 11011)** | **Chosen.** The one software path that demonstrably starts and sustains EKF3 aiding on this vehicle. Body-frame, so no heading dependency at all. |

#### What the choice buys, beyond working at all

The earth-frame design needed the autopilot's yaw to rotate a scalar into a vector. That forced an
inbound `ATTITUDE` subscription, a stream-request path, a staleness gate, a source-guard against
other components on the link, and a documented circular dependency (aiding the estimator with a
value rotated by the estimator's own yaw). **A body-frame delta needs none of it** — the wheel
measures along the vehicle's longitudinal axis, which is exactly the frame the message speaks, and
EKF3 rotates it with its own attitude. The entire attitude path is deleted, and the yaw-freshness
gate with it.

### Message packing

| Field | Value | Rationale |
|-------|-------|-----------|
| `time_usec` | `esp_timer_get_time()` | 64-bit monotonic boot µs. Explicitly **not** Arduino `micros()`, whose 32-bit return wraps every ~71 minutes; a wrapped timestamp would present a fresh sample as a very old one. ArduPilot corrects the boot-time offset itself. |
| `time_delta_usec` | **measured** interval since the previous send | ArduPilot divides by exactly this value to recover velocity (`bodyOdmDataNew.vel = delPos / delTime`, `AP_NavEKF3_Measurements.cpp:131`). The cooperative loop's tick jitters around 200 ms, so an assumed constant would inject a proportional velocity error. |
| `delta_angle` | `{0, 0, 0}` | The autopilot's own gyros own the rotation. `writeBodyFrameOdom()` turns this into `angRate = delAng / delTime`; a fabricated zero rotation rate is the honest statement "this sensor measures no rotation", and the wheel genuinely does not. |
| `delta_position` | `{v·dt, 0, 0}` metres, body frame | x forward. y (right) and z (down) are exactly 0 — the wheel measures only the longitudinal axis. |
| `confidence` | `100` when healthy (`MAVLINK_VISO_CONFIDENCE`) | See the velErr mapping below. The message is *suppressed* rather than sent with a low confidence, so no other value is ever transmitted. |
| `v` | `(speedKmh / 3.6) · travelDirection` | Signed by the **physical** gear. |

Rate: 5 Hz, riding the existing `MAVLINK_REPORT_TX_MS` block — no new scheduler. The underlying
sensor only produces a new sample at 5 Hz, so a faster send rate would transmit duplicates and
nothing more. (ArduPilot rate-limits the buffer itself at `sensorIntervalMin_ms` and rejects any
`delTime < dtEkfAvg`, `AP_NavEKF3_Measurements.cpp:123` — 5 Hz is comfortably inside both.)

### `dt` re-baselining after a gate closes

A velocity message is stateless: skip one and the next is still correct. **A delta is an
integral**, and is only meaningful over an interval that was actually observed. If a gate closes
for 10 s and then reopens, computing `dt` from the last *send* would multiply the current speed by
a 10-second interval and hand the EKF a single enormous jump in distance — the worst thing this
feature could inject, and precisely the kind of step that trips (or, worse, drags) the innovation
gate.

The rule is explicit and belt-and-braces:

1. **Every gate invalidates the baseline on its way out** (`lastOdomTimeUs_ = 0`), so a closure of
   any duration is recorded as a break rather than silently spanned.
2. **A gap longer than `MAVLINK_VISO_MAX_DT_US` (1 s) is also treated as a break**, which catches
   anything that stalls the report loop without passing through a gate.
3. In either case the send is **skipped and the baseline re-established** from the current clock,
   so exactly one interval (200 ms of travel) is discarded per break and the *following* send is
   integrated over a real, observed interval.

Discarding 200 ms of distance is harmless — the EKF simply coasts on IMU for one tick, exactly as
it does for any other suppressed sample. Clamping `dt` instead was rejected: a clamp would still
report a distance the vehicle may not have travelled, and it would do so silently.

The transmitted `dt` is surfaced on the 1 Hz debug line, so a baseline that keeps being
re-established (a flapping gate) is visible as a `dt` that never settles near 200 ms.

### Confidence → `velErr` mapping (verified, not assumed)

`AP_VisualOdom_Backend::handle_vision_position_delta_msg` passes `packet.confidence` through
unmodified as the `quality` argument of `writeBodyFrameOdom()`. There, `velErr` is computed at
`AP_NavEKF3_Measurements.cpp:138`:

```cpp
bodyOdmDataNew.velErr = frontend->_visOdmVelErrMin +
                        (frontend->_visOdmVelErrMax - frontend->_visOdmVelErrMin) *
                        (1.0f - 0.01f * quality);
```

with `EK3_VIS_VERR_MIN` defaulting to **0.1 m/s** and `EK3_VIS_VERR_MAX` to **0.9 m/s**. So:

| `confidence` | `velErr` (defaults) | Note |
|--------------|---------------------|------|
| 100 | 0.10 m/s | what this firmware sends |
| 50 | 0.50 m/s | — |
| 0 | 0.90 m/s | — |

**`readyToUseBodyOdm()` requires `velErr < 1.0f`** (`AP_NavEKF3_Control.cpp:572`). With the
default `EK3_VIS_VERR_MAX = 0.9` every confidence value clears that bar, but the margin is only
0.1 m/s — **raising `EK3_VIS_VERR_MAX` above 1.0 would silently stop aiding from ever starting.**
Sending `confidence = 100` keeps the observation at the tight end and keeps the aiding gate
satisfied regardless of how `EK3_VIS_VERR_MAX` is tuned. This is also why confidence is a constant
rather than a health signal: the firmware's health policy is *suppression*, and a low-confidence
message would be a second, weaker way of saying something the gates already say by staying quiet.

Note the consequence: **`VISO_VEL_M_NSE` is NOT used on this path.** It is read only by
`AP_VisualOdom_MAV::handle_vision_speed_estimate` (`AP_VisualOdom_MAV.cpp:79`). The delta path's
noise comes entirely from `EK3_VIS_VERR_MIN`/`MAX` and the confidence field.

### Which `VISO_*` parameters actually apply to the delta path

Read from `AP_VisualOdom_Backend.cpp:41-78` and `AP_VisualOdom.cpp`:

| Parameter | Applies? | Evidence |
|-----------|----------|----------|
| `VISO_TYPE` | **Yes** — must be non-zero | `AP_VisualOdom::handle_vision_position_delta_msg` returns early if `!enabled()` (`AP_VisualOdom.cpp:186-189`). `1` (MAVLink) is what the bench used. The handler itself lives on the *base* backend, so it is not MAV-specific. |
| `VISO_DELAY_MS` | **Yes** | Passed as `_frontend.get_delay_ms()` (`Backend.cpp:67`) and subtracted from the timestamp inside `writeBodyFrameOdom()` (`Measurements.cpp:128`). Default **10 ms**. |
| `VISO_POS_X/Y/Z` | **Yes** | Passed as `_frontend.get_pos_offset()` (`Backend.cpp:68`) and stored as `body_offset` — the lever arm from the IMU to the measuring wheel. |
| `VISO_ORIENT` | **Yes** | Both `angle_delta` and `position_delta` are rotated by `get_orientation()` before dispatch (`Backend.cpp:48-52`). Leave at `0` (`ROTATION_NONE`): this firmware already emits body-frame-forward. **Note this differs from the velocity path, where `VISO_ORIENT` is not applied** — the old design's central complaint does not apply here. |
| `VISO_QUAL_MIN` | **No** | `handle_vision_position_delta_msg` performs **no quality gating**. The `_quality >= get_quality_min()` check exists only in the `handle_pose_estimate` / `handle_vision_speed_estimate` paths (`AP_VisualOdom_MAV.cpp:49, 77`). The `confidence` field is used for `velErr` only. |
| `VISO_VEL_M_NSE` | **No** | Used only by `handle_vision_speed_estimate` (`AP_VisualOdom_MAV.cpp:79`). See above. |
| `VISO_SCALE` | **No** | `get_pos_scale()` is applied only in `handle_pose_estimate` (`AP_VisualOdom_MAV.cpp:30`). |

### Autopilot parameter requirements

| Parameter | GPS-less (bench-proven) | With a GPS fitted |
|-----------|-------------------------|-------------------|
| `VISO_TYPE` | `1` (MAVLink) | `1` |
| `EK3_SRC1_VELXY` | `6` (ExternalNav) | `6` |
| `EK3_SRC1_POSXY` | **`0`** (None) | **`3`** (GPS) |
| `GPS1_TYPE` | **`0`** (None) | `2` (or whatever the receiver needs) |
| `VISO_DELAY_MS` | `10` default → tune | same |
| `VISO_POS_X/Y/Z` | rear-hub lever arm | same |

`GPS1_TYPE` is the ArduPilot-4.7 name; on earlier firmware the same parameter is `GPS_TYPE`. On the
bench it was set to `0` so that a non-existent (or unhealthy) GPS could not hold the filter
waiting for a position source that would never arrive. **If a GPS is later fitted, restore
`GPS1_TYPE=2` and `EK3_SRC1_POSXY=3`** — velocity-only ExternalNav with no position source can
time out the EKF position estimate (ArduPilot issue #23485), and a real GPS is a better position
source than none.

`EK3_SRC1_VELZ` is left alone: the vertical channel stays on baro, and the message reports no
vertical travel.

### EKF origin on a GPS-less vehicle

With `GPS1_TYPE=0` the EKF has no way to learn where it is, and several code paths (and the whole
`GLOBAL_POSITION_INT` output used to verify fusion on the bench) require a valid origin. **The
origin must be set once per boot from the ground station** — Mission Planner's "Set EKF Origin"
(a `SET_GPS_GLOBAL_ORIGIN` message, or `MAV_CMD_DO_SET_GLOBAL_ORIGIN`).

The ESP32 deliberately does **not** send it. It has no position source of its own, so any origin
it produced would be invented; and an origin is a one-shot, operator-meaningful act (it defines
where the vehicle's whole relative frame is anchored), not something a peripheral should assert on
every boot. Recorded as an open question below, since it is a real operational wart.

### Gate policy: silence over zeros, but healthy zeros are real data

Three gates, each suppressing the message entirely **and invalidating the delta baseline**:

| # | Gate | Why not just send 0 |
|---|------|---------------------|
| 1 | `!isLinkUp()` | No autopilot to receive it. |
| 2 | `!state.speedValid` | Covers the sensor's `suspicious_` wire-fault latch (`include/SpeedSensor.h:55`). A cut signal wire looks exactly like "stopped" — decaying to 0 km/h. Fusing that as a zero-motion update while the vehicle is actually rolling would drag the EKF velocity to zero and corrupt the position estimate. This is the single most dangerous zero in the system. |
| 3 | `travelDirection == 0 && speedKmh > MAVLINK_VISO_NEUTRAL_ZERO_KMH` | Rolling in neutral (or mid-shift): the wheel says the vehicle is moving but nothing says which way. A wrong sign is a `2v` error; a skipped sample costs nothing. |

The old gate 3 — **yaw staleness — is deleted**. A body-frame delta has no rotation to get wrong.

**The deliberate exception**: gate 3 is written so that a *healthy* zero still gets through —
stationary in neutral, idling, sitting at a stop — because with no GPS a zero-motion update is the
most valuable observation available. It is the only thing that tells the EKF "your drifting
velocity estimate is wrong, you are not moving", and it arrives 5 times a second while parked.
`MAVLINK_VISO_NEUTRAL_ZERO_KMH` = 0.5 km/h is the line between "genuinely stopped" (send it) and
"coasting in neutral" (stay quiet), sized to the sensor's own quantisation floor rather than to
zero, so sensor noise around standstill does not silence the most useful case.

### Physical gear, never assumed gear

Unchanged from the previous revision, and unchanged in code. The wheel sensor is unsigned — it
counts pulses and cannot tell forward from reverse. The sign comes from the gearbox, and there are
two candidate sources:

- `getCurrentGear()` / `getTargetGear()` — the controller's **assumed** gear. The transmission is
  sensorless-and-time-based in its command path, so these express *intent*.
- `getPhysicalGear()` (`include/TransmissionController.h:106`) — read from the four opto switches
  on the PCA9557 input expander. This is *measurement*.

`getTravelDirection()` uses `getPhysicalGear()`. Signing an EKF observation with an assumption is
exactly the failure mode that produces a `2v` velocity innovation (a reverse leg reported as
forward), which is the largest error this feature could possibly inject. If the expander is
faulted or the switches are ambiguous, `getPhysicalGear()` returns `GEAR_UNKNOWN` → direction 0 →
the sample is skipped, which is the correct degraded behaviour.

During a shift the physical gear is briefly `GEAR_UNKNOWN`, so a few samples are dropped per shift
(plus one more to re-baseline). Harmless: shifts are interlocked below 5 km/h, so the dropped
samples carry almost no information.

### Latency budget and `VISO_DELAY_MS`

The wheel speed is a **window average**, not an instantaneous sample: the PCNT count is divided by
the elapsed window, so the reported value is centred roughly half a window in the past. At speed
the window is short and the lag is on the order of 100 ms; at a crawl the window stretches while
it waits for pulses, and the effective lag grows toward ~1 s. Adding the ≤200 ms transmit
quantisation and ~1 ms of link time, **`VISO_DELAY_MS` in the region of 150 ms is a starting
point, not a constant** (note the parameter's own default is 10 ms) — it should be tuned against
dataflash innovations (`XKF3 IVN/IVE`). The lag is inherently speed-dependent and a single scalar
cannot cancel it; the practical consequence is that innovations will be largest during hard
acceleration and braking, which is expected.

## Risks / Trade-offs
- **A delta spanning an unobserved interval would inject a huge position step.** → The
  re-baseline rule above, applied on every gate exit and on any gap over 1 s.
- **`EK3_VIS_VERR_MAX > 1.0` silently disables aiding.** Not a hypothetical: `readyToUseBodyOdm()`
  needs `velErr < 1.0`, and nothing reports why aiding never started. → Send `confidence = 100`
  so `velErr` sits at the `EK3_VIS_VERR_MIN` end regardless, and document the interaction.
- **`AID_RELATIVE`, not `AID_ABSOLUTE`.** Body-frame odometry with no position source gives the
  filter a *relative* solution: velocity and dead-reckoned position relative to the origin, with
  unbounded position drift over time. That is a genuine improvement over const-pos (which was
  zero information), but it is not GPS and must not be treated as such. → Documented; the
  operational answer is that the origin is operator-set and position is not trusted for
  navigation.
- **Yaw is now entirely the autopilot's problem.** The body-frame delta is rotated by EKF3's own
  attitude, so a badly calibrated compass turns forward travel into travel in the wrong direction.
  The previous design shared this weakness (it consumed the same yaw) but made it visible. → A
  compass calibration is now a hard prerequisite of the on-vehicle verification.
- **Wrong sign injects a `2v` error** — the worst failure this change can cause. → Mitigated by
  using the physical gear, by gating `UNKNOWN` to silence, and by an explicit reverse-leg check in
  the on-vehicle verification.
- **Wheel slip and scale error are unmodelled.** A calibration error in pulses-per-revolution or
  tyre circumference appears as a *speed-proportional bias*, which the EKF cannot distinguish from
  real motion and will happily fuse. → A persistent proportional bias in the innovations is the
  signature to look for, and the fix is recalibration, not retuning.
- **`VISO_*` parameters may not exist on the airframe.** `AP_VisualOdom` is compiled out of
  1 MB-flash targets. → Confirmed present on this vehicle's fmuv3 build on 2026-08-21.
- **RX parsing load is now *lower* than before this change**, since the `ATTITUDE` subscription is
  gone entirely.

## Migration Plan
1. Flash the firmware. Nothing changes on the vehicle: with `VISO_TYPE = 0` ArduPilot ignores
   `VISION_POSITION_DELTA` entirely, and every existing message is byte-identical.
2. Confirm on the serial `[MAV]` line that `viso:` counts up at ~5 Hz and `dt:` sits near 200 ms.
3. Set the parameters from `MAVLINK_SETUP.md` for the GPS-less configuration, and set the EKF
   origin from the GCS.
4. Confirm `EK3_SRC1_VELXY = 6` produces EKF flags with `AID_RELATIVE` set (not `0x00A7`), and
   validate against dataflash innovations on a straight-line drive.
5. **Rollback** is a single parameter: `VISO_TYPE = 0` (or `EK3_SRC1_VELXY` back to its previous
   value) stops all fusion instantly, with no firmware change and no reboot of the ESP32.

## Open Questions
- **EKF origin workflow.** The origin must be set from the GCS once per boot, and the vehicle is
  otherwise unusable in any position-aware mode until it is. Options if this proves too fragile in
  practice: (a) have the ESP32 send `SET_GPS_GLOBAL_ORIGIN` with a fixed, operator-configured
  home coordinate stored in NVS; (b) accept it as a documented pre-flight step. Deferred until
  there is on-vehicle experience — inventing an origin in firmware is a decision that deserves
  evidence, not a guess.
- **Final `VISO_DELAY_MS`** — the real value comes from innovation tuning on the vehicle and
  should be written back into `MAVLINK_SETUP.md` once measured.
- **Whether `delta_angle` should carry a real yaw rate.** It is zeroed today. If the EKF's own
  gyro-derived rotation ever proves to disagree badly with the wheel path on tight turns, feeding
  a measured rotation would be the next lever — but this vehicle has no independent rotation
  sensor, so there is nothing honest to put there today.
- **Whether the ~1 s effective lag at crawl speed warrants gating the message below some minimum
  speed.** Deferred: the zero-motion update at standstill is too valuable to risk suppressing, and
  no evidence exists yet that crawl-speed samples hurt.
