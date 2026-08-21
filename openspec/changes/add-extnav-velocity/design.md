## Context
The ESP32-S3 (`Control_v0`) is MAVLink 2 sysid 1 / compid 25 on the Pixhawk's TELEM2 ↔ UART1
(GPIO18 RX / GPIO17 TX, 115200, header X9, BSS138 level shifter). It already:

- decodes `SERVO_OUTPUT_RAW` as its command input (`src/MavlinkInterface.cpp:81-92`),
- learns the autopilot's addressing from `HEARTBEAT` (`:154-167`),
- re-requests the servo stream while its measured rate is low (`:119-124`),
- sends `HEARTBEAT` (1 Hz) plus `EFI_STATUS` + `VFR_HUD` every `MAVLINK_REPORT_TX_MS` = 200 ms
  (`:256-317`), where `VFR_HUD.groundspeed` already carries the hall wheel speed.

The vehicle runs **ArduPilot Rover 4.7.0** on a Pixhawk 2.4.8 and has **no reliable GPS**. The
hall sensor (`include/SpeedSensor.h`, PCNT on GPIO8/X2, sampled at 5 Hz, ~28.4 mm/pulse) is the
only speed measurement on the vehicle, and it currently informs nothing but a display field.

Everything here runs on the existing single cooperative loop: no new task, ISR or timer, and all
rate limiting is `millis()`-based, matching the rest of the firmware.

## Goals / Non-Goals
- **Goals**: deliver the hall wheel speed to the EKF3 as a fusable earth-frame velocity; get the
  yaw needed to rotate it from the autopilot itself; never fuse a measurement whose validity or
  sign is in doubt; make every suppression reason visible on the serial debug line; document the
  autopilot-side parameters.
- **Non-Goals**: no position or attitude output (velocity only); no quadrature wheel-encoder
  emulation; no new sensor hardware; no change to command decoding, fail-safe behaviour or any
  existing outbound message; no ground-station plugin work (`VISION_SPEED_ESTIMATE` needs none —
  ArduPilot consumes it directly).

## Decisions

### Transport choice: why `VISION_SPEED_ESTIMATE` and not `WHEEL_DISTANCE`
Three mechanisms can put wheel speed into a Rover EKF. Checked against the Rover-4.7.0 source:

| Mechanism | Verdict |
|-----------|---------|
| `WHEEL_DISTANCE` (msg 9000) | **Dead end.** ArduPilot only *transmits* this message; there is no receive handler, so nothing on the vehicle would ever read it. |
| `AP_WheelEncoder` (`WENC_TYPE=1`) | Works, but it is a **quadrature-pulse** driver: it wants A/B wires into two Pixhawk AUX pins. That means re-wiring the hall signal away from the ESP32 (or splitting it) and fabricating a phase-B signal we do not have. Declined — extra wiring for no extra information. |
| **`VISION_SPEED_ESTIMATE` (msg 103)** | **Chosen.** `GCS_MAVLINK::handle_vision_speed_estimate` → `AP_VisualOdom_MAV::handle_vision_speed_estimate` → `EKF3 writeExtNavVelData`. Pure software on our side, no wiring change, and it lands in the estimator as a first-class velocity observation with a tunable noise and delay. |

Cost of the choice: it is an *external navigation* interface, so it speaks earth-frame vectors,
not scalars — which is what forces the yaw dependency below.

### Yaw must come from the autopilot's `ATTITUDE`
`VISION_SPEED_ESTIMATE` carries a velocity **vector in the earth NED frame**. The wheel gives a
**scalar** along the vehicle's longitudinal axis. Converting one to the other needs heading:

```
x =  v · cos(yaw)      y =  v · sin(yaw)      z = 0
```

Two facts make this non-negotiable:

1. **`VISO_ORIENT` does not rotate velocity.** It is applied to the position/attitude path only,
   so we cannot hand ArduPilot a body-frame vector and ask it to rotate. The rotation must happen
   here, before transmit.
2. **We have no heading source of our own.** There is no magnetometer or IMU on the ESP32. The
   autopilot's fused yaw is the only heading available, so we subscribe to `ATTITUDE` (msg 30)
   and use `ATTITUDE.yaw` (radians, NED, +ve clockwise from north).

This creates a hard dependency, and therefore a hard gate: **stale yaw ⇒ no message**
(`MAVLINK_ATTITUDE_TIMEOUT_MS` = 500 ms ≈ 5 missed frames at the requested 10 Hz). Sending a
velocity rotated by a stale heading is worse than sending nothing — the EKF would fuse a vector
pointing in a direction the vehicle is no longer travelling, and its own innovation gate would
have no way to tell that apart from a genuine turn.

Note the circularity this introduces and its bound: we aid the estimator with a value rotated by
the estimator's own yaw. This is acceptable because yaw error and velocity magnitude are
independently observable and the wheel speed constrains only the *magnitude* — but it does mean
this path can never fix a yaw error, only a velocity one. The 10 Hz request rate (2× the 5 Hz
transmit rate) keeps yaw fresher than the measurement it rotates.

**Source guarding**: the `ATTITUDE` case checks `msg.sysid == targetSystem_ &&
msg.compid == MAV_COMP_ID_AUTOPILOT1`. Companion computers, gimbals and GCS-side tools all emit
`ATTITUDE` on a shared link; accepting any of them would let an unrelated component silently
rotate a fused measurement.

### Physical gear, never assumed gear
The wheel sensor is unsigned — it counts pulses and cannot tell forward from reverse. The sign
comes from the gearbox, and there are two candidate sources:

- `getCurrentGear()` / `getTargetGear()` — the controller's **assumed** gear. The transmission is
  sensorless-and-time-based in its command path, so these express *intent*.
- `getPhysicalGear()` (`include/TransmissionController.h:106`) — read from the four opto switches
  on the PCA9557 input expander. This is *measurement*.

`getTravelDirection()` uses `getPhysicalGear()`. Signing an EKF observation with an assumption is
exactly the failure mode that produces a `2v` velocity innovation (a reverse leg reported as
forward), which is the largest error this feature could possibly inject. If the expander is
faulted or the switches are ambiguous, `getPhysicalGear()` returns `GEAR_UNKNOWN` → direction 0 →
the sample is skipped, which is the correct degraded behaviour.

During a shift the physical gear is briefly `GEAR_UNKNOWN` (no switch asserted mid-travel), so a
few samples are dropped per shift. Harmless: shifts are interlocked below 5 km/h, so the dropped
samples carry almost no information, and the EKF simply coasts a few hundred ms on IMU.

### Gate policy: silence over zeros, but healthy zeros are real data
Four gates, each suppressing the message entirely:

| # | Gate | Why not just send 0 |
|---|------|---------------------|
| 1 | `!isLinkUp()` | No autopilot to receive it. |
| 2 | `!state.speedValid` | Covers the sensor's `suspicious_` wire-fault latch (`include/SpeedSensor.h:55`). A cut signal wire looks exactly like "stopped" — decaying to 0 km/h. Fusing that as a zero-velocity update while the vehicle is actually rolling would drag the EKF velocity to zero and corrupt the position estimate. This is the single most dangerous zero in the system. |
| 3 | `!isYawFresh()` | Rotation invalid (above). |
| 4 | `travelDirection == 0 && speedKmh > MAVLINK_VISO_NEUTRAL_ZERO_KMH` | Rolling in neutral (or mid-shift): the wheel says the vehicle is moving but nothing says which way. A wrong sign is a `2v` error; a skipped sample costs nothing. |

**The deliberate exception**: gate 4 is written so that a *healthy* zero still gets through —
stationary in neutral, idling, sitting at a stop — because with no GPS a zero-velocity update is
the most valuable observation available. It is the only thing that tells the EKF "your drifting
velocity estimate is wrong, you are not moving", and it arrives 5 times a second while parked.
`MAVLINK_VISO_NEUTRAL_ZERO_KMH` = 0.5 km/h is the line between "genuinely stopped" (send it) and
"coasting in neutral" (stay quiet), sized to the sensor's own quantisation floor rather than to
zero, so sensor noise around standstill does not silence the most useful case.

### Message packing details
- **`usec` = `esp_timer_get_time()`** — 64-bit monotonic microseconds since boot. Explicitly
  **not** Arduino `micros()`, whose 32-bit return wraps every ~71 minutes; a wrapped timestamp
  would present as a ~71-minute-old measurement and be rejected (or worse, mis-ordered) by
  ArduPilot's buffer. ArduPilot jitter-corrects the boot-time offset itself, so a monotonic
  local clock is the correct thing to send — no attempt to sync to autopilot time is needed.
- **`covariance[0] = NaN`** — the documented "no covariance supplied" encoding. ArduPilot then
  uses the operator-tuned `VISO_VEL_M_NSE` instead of a number we would have to invent. We have
  no honest per-sample covariance to report.
- **`reset_counter = 0`** — never incremented. That field signals a discontinuity in an external
  estimator's own frame; a wheel sensor has no frame to reset.
- **`z = 0`** exactly, not "unknown" — the vehicle is a ground rover and its vertical velocity in
  the body-longitudinal sense genuinely is zero. On a slope this under-reports true NED vertical
  motion; that is accepted, and is why `EK3_SRC1_VELZ` is left alone (baro keeps the vertical
  channel).
- **Rate**: 5 Hz, riding the existing `MAVLINK_REPORT_TX_MS` block — no new scheduler. ArduPilot
  accepts external-nav velocity across roughly 4-50 Hz, and the underlying sensor only produces a
  new sample at 5 Hz, so a faster send rate would transmit duplicates and nothing more.

### Latency budget and `VISO_DELAY_MS`
The wheel speed is a **window average**, not an instantaneous sample: the PCNT count is divided by
the elapsed window, so the reported value is centred roughly half a window in the past. At speed
the window is short and the lag is on the order of 100 ms; at a crawl the window stretches while
it waits for pulses, and the effective lag grows toward ~1 s. Adding the ≤200 ms transmit
quantisation and ~1 ms of link time, **`VISO_DELAY_MS ≈ 150` is a starting point, not a constant**
— it should be tuned against dataflash innovations (`XKF3 IVN/IVE`). The lag is inherently
speed-dependent and a single scalar cannot cancel it; the practical consequence is that
innovations will be largest during hard acceleration and braking, which is expected and is why
`VISO_VEL_M_NSE = 0.2` (a deliberately loose 0.2 m/s) rather than a tight value.

### Split re-request gate
Today a single condition re-requests `SERVO_OUTPUT_RAW` whenever `lastCmdRate_` is low. Reusing
it for both streams would be wrong in both directions: a healthy 25 Hz servo stream would mask a
dead attitude stream forever (no yaw, permanent silence, no re-request), and a dead servo stream
would spam attitude requests it does not need. Each stream therefore gets its own rate window and
its own threshold, sharing only the `MAVLINK_STREAM_REREQUEST_MS` spacing timer — which is
sufficient because both re-requests are cheap and idempotent.

The attitude rate window folds into the **existing** 1 s window computation rather than adding a
second timer: one `if (now - rateWindowStart_ >= 1000)` block updates both rates.

## Risks / Trade-offs
- **`VISO_*` parameters may not exist on the airframe.** `AP_VisualOdom` is compiled out of
  1 MB-flash targets. A Pixhawk 2.4.8 running the **fmuv2** build has no `VISO_TYPE` and this
  feature is inert. → Verify the parameter exists in Mission Planner's full parameter list before
  wiring anything; flash the **fmuv3** (2 MB) build if it is missing. The ESP32 side degrades
  silently and safely: it keeps transmitting, ArduPilot keeps ignoring.
- **Velocity-only ExtNav with no position source can time out the EKF position estimate**
  (ArduPilot issue #23485). → If any GPS is fitted, keep `EK3_SRC1_POSXY = 3` (GPS) alongside
  `EK3_SRC1_VELXY = 6` (ExternalNav) rather than switching position to ExtNav as well. This is
  the documented interaction, not a theoretical one.
- **Wrong sign injects a `2v` error** — the worst failure this change can cause. → Mitigated by
  using the physical gear, by gating `UNKNOWN` to silence, and by an explicit reverse-leg check
  in the on-vehicle verification (innovations must not show a `2v` step on the reverse leg).
- **Wheel slip and scale error are unmodelled.** A calibration error in pulses-per-revolution or
  tyre circumference appears as a *speed-proportional bias*, which the EKF cannot distinguish
  from real motion and will happily fuse. → `VISO_VEL_M_NSE = 0.2` keeps the measurement loose
  enough not to dominate; a persistent proportional bias in the innovations is the signature to
  look for, and the fix is recalibration, not retuning.
- **Circular dependency on autopilot yaw** (see above) — bounded, and made fail-safe by the
  staleness gate.
- **Extra RX parsing load**: `ATTITUDE` at 10 Hz is 36-byte payloads, ~500 B/s on a 115200 link
  (~11 kB/s capacity). Negligible, and the existing drain loop is already non-blocking.

## Migration Plan
1. Flash the firmware. Nothing changes on the vehicle: with `VISO_TYPE = 0` (default) ArduPilot
   ignores `VISION_SPEED_ESTIMATE` entirely, and every existing message is byte-identical.
2. Confirm on the serial `[MAV]` line that `att:` shows ~10 Hz and `yaw:` an age in the tens of
   ms — i.e. the subscription works — before touching a single autopilot parameter.
3. Set the parameters from `MAVLINK_SETUP.md` and confirm the message arrives in Mission
   Planner's MAVLink Inspector under sysid 1 / compid 25 at ~5 Hz.
4. Enable fusion (`EK3_SRC1_VELXY = 6`) last, and validate against dataflash innovations on a
   straight-line drive.
5. **Rollback** is a single parameter: `VISO_TYPE = 0` (or `EK3_SRC1_VELXY` back to its previous
   value) stops all fusion instantly, with no firmware change and no reboot of the ESP32.

## Open Questions
- Final `VISO_DELAY_MS` — 150 ms is a starting point; the real value comes from innovation tuning
  on the vehicle and should be written back into `MAVLINK_SETUP.md` once measured.
- Whether the ~1 s effective lag at crawl speed warrants gating the message below some minimum
  speed. Deferred: the zero-velocity update at standstill is too valuable to risk suppressing,
  and no evidence exists yet that crawl-speed samples hurt.
