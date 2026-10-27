## Context
The `aztec` branch (ODES Junglecross 800 buggy, `Control_v0` board) currently runs firmware 1.2.0
with the dual-AS5600 steering feedback from `add-aztec-dual-as5600-steering`:
- a fine sensor on I2C1 `Wire` (X14);
- a coarse sensor on I2C2 `Wire1` (X15);
- unwrapped fine counts, turn recovery, a mismatch latch, and three-point session calibration in
  NVS `"steering"` (`cal_ver`=2, `fc_raw`/`cc_raw`/`fl_pos`/`cl_raw`/`fr_pos`/`cr_raw`);
- a coordinated `Wire1` recovery in `main.cpp`.

`jog()` and `nudge()` refuse to run while `!sensorOk_`.

The Bosch LWS 0 265 005 486 was characterised on the bench on 2026-10-24. The source is
`~/Documents/SourceCode/Arduino/CAN-TEST/README.md`, sections "Датчик кута повороту керма Bosch
0 265 005 486 (ID 0x305)", "Конфігураційний кадр 0x7C0" and "Вихід за діапазон (±600°)", and the
decoder is `CAN-TEST/src/lws305.h`. Key facts:
- Absolute multi-turn angle, 0.1°/bit. Standard ID `0x305`, DLC 6, every 10 ms, sent by itself
  after 12 V power-up. 500 kbit/s.
- Working range about ±600°. Beyond it the sensor latches a fault (OK=0, cal=0, fault=9, angle
  `0x7FFF`), and only a 12 V power cycle clears it. The stored zero survives.
- Zero is set with `0x7C0 [05 00]`, about 100 ms, then `0x7C0 [53 30]`. Zero and ID persist. A
  bare `03 00` moves the sensor's output ID to `0x000`.

Ground truth in today's CAN code (`src/CANController.cpp`):
- The coryjfowler `mcp_can` library drives the MCP2515, 8 MHz crystal, polled (no INT), with a
  non-blocking OBD-II state machine (IDLE / WAITING_RESPONSE) and a separate ECU-probe mode.
- `applyRxFilters()` programs mask 0x7F8 / filter 0x7E8 into **both** masks and all six filters.
  RXB0 = mask 0 + filters 0–1; RXB1 = mask 1 + filters 2–5.
- The library enables BUKT (RXB0 → RXB1 rollover) in `begin()`.
- Three read paths each drain at most 5 frames per call and log unmatched frames, rate-limited:
  `drainRxBuffers()` in IDLE, `tryReceiveResponse()` in WAITING_RESPONSE, and
  `tryReceiveProbeResponse()` while probing.
- When no OBD reply arrives for `CAN_REINIT_NO_DATA_MS` (15 s), the chip is re-initialised.

Decisions already agreed with the user, and not re-opened here:
1. Direct mount on the steering shaft.
2. One shared CAN bus.
3. 12 V from the ignition key.
4. All AS5600 hardware and logic removed.
5. Centre = sensor zero, and NVS holds the limits only.
6. A web "set sensor zero" button with no preconditions.
7. Manual jog and nudge always available, sensorless if need be.
8. Sign conversion.
9. Units in 0.1°.
10. Slow pulsed jog deferred.

## Goals / Non-Goals
- Goals:
  - Absolute steering angle from one CAN sensor, at boot, without homing.
  - The same fail-safe policy for closed-loop control: any sensor fault stops a position move and
    rejects percent commands.
  - Manual jog and nudge that work without a sensor, so the vehicle can be centred and zeroed from
    scratch.
  - OBD polling keeps working as it does today on the shared bus.
  - Delete all dual-sensor code paths.
- Non-Goals:
  - An AS5600 fallback or any degraded mode on a second sensor.
  - Automatic clearing of the latched range fault (no relay on the sensor supply).
  - Changing the LWS output ID, or any `0x7C0` payload other than the two fixed frames.
  - A pulsed slow-jog mode (deferred).
  - Changing the MAVLink message set.

## Decisions

### D1. Frame decoding (reuses `lws305.h`)
A new header, `include/Lws305.h`, has no Arduino dependency so it can be unit-tested natively. It
ports the bench-verified functions from `CAN-TEST/src/lws305.h` unchanged:

| Bytes | Field | Decoding |
|-------|-------|----------|
| 0–1 | angle | `int16` big-endian, 0.1°/bit, `0x7FFF` = invalid, **positive = CCW = LEFT** |
| 2 | speed | `uint8` × 4 °/s, `0xFF` = invalid |
| 3 | status | bit7 direction (1 = CW), bits 6..3 fault code (0 = none), bit2 adjusted, bit1 calibrated, bit0 sensor OK |
| 4 | check | high nibble = XOR of the 8 nibbles of bytes 0..3 XOR counter; low nibble = 4-bit rolling counter |
| 5 | speed (optimised) | `uint8` × 4 °/s, `0xFF` = invalid (decoded, shown only in the debug log) |

- A frame is **accepted** only if all of these hold:
  - standard ID `0x305`;
  - DLC ≥ 6 (`LWS_DLC_MIN`; PSA-RE documents DLC 7, and both are accepted);
  - `lwsCrcOk()` passes;
  - the counter differs from the last accepted frame's counter.
- A repeated counter means a duplicate or a frozen sender. It is counted as a counter error and
  the frame is dropped.
- A counter jump greater than 1 means lost frames. It is counted (`miss`), but the frame is
  accepted, because every frame is absolute.
- A checksum failure is counted (`crc_err`) and the frame is dropped.
- Bench vectors (README "Формат кадру 0x305"), with the check byte computed for a chosen counter:
  - `0B 32 00 07 49 00` → angle +286.6° (sensor), speed 0, status adj + cal + OK, counter 9
  - `0C 77 01 87 13 00` → +319.1°, 4 °/s, CW, status OK, counter 3
  - Latched pattern: `7F FF FF 4C 55 00` → angle invalid, speed invalid, OK=0, cal=0, adj=1,
    fault 9, counter 5

### D2. Sign and units
- **Fixed negation**: the firmware position is `pos = −angle`, in 0.1° (int32), so positive =
  RIGHT. That is the convention of `IMotorDriver::setSpeed` (positive = wheel right) and of
  `steering_pct`.
- The derived `invert_` flag of the AS5600 era is removed.
- Rationale: the AS5600 direction depended on magnet orientation, so it had to be derived. The LWS
  direction is a documented property of the part.
- A sensor mounted mirrored is caught at calibration: a Left capture at the left lock would read
  positive and be refused ("left limit must be left of zero"). The closed loop can therefore never
  run with the sign inverted.
- Telemetry also uses the firmware convention: `steer_angle` positive = right. The sensor's raw
  angle appears only in the debug log.

### D3. MCP2515 filter plan (one bus, two acceptance paths)
| Buffer | Mask | Filters | Accepts |
|--------|------|---------|---------|
| RXB0 | MASK0 = `0x7F8` | RXF0, RXF1 = `0x7E8` | OBD-II replies `0x7E8–0x7EF` (unchanged) |
| RXB1 | MASK1 = `0x7FF` | RXF2..RXF5 = `0x305` | LWS angle frame only |

- Same `<< 16` register encoding as today, with `ext = 0` on every mask and filter (extended
  frames rejected).
- New constants: `CAN_RX_OBD_MASK` / `CAN_RX_OBD_FILTER` (renamed from `CAN_RX_ID_*`),
  `CAN_RX_LWS_MASK` = `0x7FF`, and `STEER_LWS_CAN_ID` = `0x305`.
- BUKT stays enabled, as the library sets it. When RXB0 is full, an OBD reply rolls over into RXB1
  instead of being lost.
  - The only loss case is RXB1 already holding an unread LWS frame while another arrives. That is
    `RX1OVR`, and it loses one LWS frame, which is harmless.
  - The overflow log is already rate-limited. Overflow counts are expected to rise slightly and are
    not a fault.
- Each path has its own buffer, so a 100 Hz LWS stream can no longer occupy the buffer an OBD reply
  needs, and an OBD reply never displaces the LWS stream.
- The chip re-init path (`CAN_REINIT_NO_DATA_MS`) reuses `applyRxFilters()`, so the new plan is
  re-applied after every reset.

### D4. Dispatch from every receive path
- One private helper, `bool CANController::dispatchSteeringFrame(id, ext, len, buf)`, is called
  first for every frame read in all three read paths: the IDLE drain, the WAITING_RESPONSE receive
  and the probe receive. For a standard `0x305` frame it calls the attached sink,
  `LwsSensor::onFrame(buf, len, millis())`, and returns true. The frame is then neither treated as
  an OBD candidate nor logged as unmatched.
- The 5-frame bound per call is kept. At 100 Hz, one LWS frame arrives every 10 ms, and the loop
  runs far faster than that, so the bound is never the limit in steady state.
- `CANController::update()` runs before `SteeringController::update()` in each loop iteration
  (both in `VehicleController::update()`). The steering therefore sees frames from the same
  iteration.
- Everything stays on the Arduino loop task. There is no ISR, no queue and no lock: the sink is a
  plain object owned by the steering controller, attached once in `setup()`.
- When the CAN controller failed to initialise, or has no sink attached, the steering sensor state
  is `can_down`.

### D5. Sensor health and fault state machine
`LwsSensor` keeps the last accepted frame, its arrival time and the counters. `SteeringController`
derives the health from it on every `update()`:

```
token = zeroing        if a zero-set sequence is running (D7)
      | can_down       if the CAN controller is not initialised / no sink
      | timeout        if no ACCEPTED frame for STEER_LWS_TIMEOUT_MS (100 ms) — includes "never
                       received", checksum-only traffic and a frozen counter
      | range_latched  if the last frame has fault code 9 (out-of-range latch; restart the vehicle)
      | sensor_fault   if OK bit = 0 or fault code ≠ 0 (any other code)
      | not_calibrated if calibrated bit = 0 (no stored zero, e.g. after 05 00 alone)
      | angle_invalid  if angle = 0x7FFF
      | none
sensorOk = (token == none) && healthyStreak ≥ STEER_LWS_RECOVER_FRAMES
```

- Faults take effect **immediately**, with no debounce. The checksum already rules out corrupted
  status bits, and staleness is the debounce for lost frames.
- Recovery needs `STEER_LWS_RECOVER_FRAMES` (5, which is 50 ms) consecutive good frames, so a
  marginal sensor does not chatter.
- On the transition to a fault:
  - a closed-loop position move is stopped (motor coast), and `isMoving_` is cleared;
  - **an active jog or nudge is NOT stopped**. It continues open loop, without a stall backstop,
    until its refresh timeout or pulse end (D8);
  - the fault is logged once with its token.
- The rest of the fault model:
  - Percent commands are rejected while `!sensorOk`.
  - Health is regained automatically, with no latch in the firmware. The only "latched" state is
    the sensor's own `range_latched`, and only a power cycle of the vehicle clears it.
  - `getSensorFault()`, `faultToken()`, `isSensorOk()` and `isCalibrated()` keep their names, so
    the MAVLink and telemetry call sites stay small.

### D6. Calibration = sensor zero + two limits
- **Centre** is the sensor's own stored zero: firmware position 0 = angle 0.0. There is no centre
  capture and no centre offset in NVS.
- **Limits**: `captureLeftLimit()` and `captureRightLimit()` need `sensorOk`. They store the current
  `pos` (0.1°, firmware sign).
  - Left must be ≤ `−STEER_CAL_MIN_SPAN`; right must be ≥ `+STEER_CAL_MIN_SPAN`.
  - The safe range (D9) is **not** checked: a limit beyond ±`STEER_LWS_SAFE_RANGE` is accepted
    and saved like any other. The capture response then carries a warning ("beyond safe range,
    near the sensor latch at ~600°"), and telemetry `steer_range_warning` is true while either
    stored limit is beyond it (D14).
  - A refused capture names the reason: "sensor fault", "left limit must be left of zero" or "too
    close to zero".
- Each capture is persisted **immediately** and writes only its own key. The steering is
  *calibrated* when both limits are stored and valid. There are no session slots and no
  "all in one power-on" rule: limits are absolute against a zero that the sensor keeps.
- The motor is stopped on each capture, as today.

### D7. Zero-set sequence and its confirmation
Trigger: the web command `steer_lws_zero`, which is accepted regardless of the input source, like
the other calibration commands, and has **no ignition, RPM or standstill precondition**.

1. `SteeringController::requestSensorZero()`:
   - It returns "busy" while a sequence is already running, and "CAN unavailable" when the CAN
     controller is down.
   - Otherwise it stops any move or jog, clears a pending boot centring, and asks
     `CANController::startLwsZero()`.
   - That call returns busy while an ECU probe is running. The probe is short and bounded, and the
     operator simply presses again.
2. `CANController` runs a small sub-state machine next to the OBD one (OBD polling is paused):
   - `WAIT_IDLE`: let an in-flight OBD request finish or time out (≤ `CAN_RESPONSE_TIMEOUT`, 200 ms).
     No new OBD request is sent.
   - `SEND_RESET`: send `0x7C0` DLC 2 `05 00`. A send failure aborts with "tx failed" and resumes
     OBD.
   - `GAP`: wait `STEER_LWS_ZERO_GAP_MS` (100 ms), without blocking.
   - `SEND_ZERO`: send `0x7C0` DLC 2 `53 30`. A send failure aborts with "tx failed".
   - `DONE`: resume OBD polling where the scheduler left off.
   - The OBD stale timeout cannot trip: the pause is ≤ ~400 ms against 5000 ms.
   - Both payloads are `static const uint8_t[2]` in `CANController.cpp`. The sub-state machine is
     the **only** code that transmits on `0x7C0`, and no API takes a `0x7C0` payload from outside.
     A unit test asserts that `53 30` encodes output ID `0x305`: `((0x30 << 8) | 0x53) >> 4`.
3. **Confirmation**, in `SteeringController`:
   - After `SEND_ZERO`, wait up to `STEER_LWS_ZERO_CONFIRM_MS` (1000 ms) for an accepted frame with
     OK = 1, fault = 0, calibrated = 1 and |angle| ≤ `STEER_LWS_ZERO_TOLERANCE` (1.0°).
   - The result is `ok`, or `failed` with a reason: `range_latched` (fault 9 seen, so restart the
     vehicle and retry), `no_frames` (timeout), `not_confirmed` (frames arrive, but cal ≠ 1 or the
     angle is not about 0), or `tx_failed`.
   - The result is reported in the command response and in telemetry (`steer_zero_state`,
     `steer_zero_error`).
4. **Limits are erased** (both NVS keys) as soon as `05 00` has been sent, whatever the outcome.
   From that point the old zero is no longer guaranteed, so limits measured against it are
   meaningless. The UI then asks for Left and Right again.
5. During the sequence the token is `zeroing` and `sensorOk` is false. This is expected: after
   `05 00` the sensor reports cal = 0 and angle `0x7FFF` until `53 30`.

The sensorless centring flow this enables:
1. Jog or nudge, open loop, to the mechanical straight-ahead position.
2. Restart the vehicle. This power-cycles the sensor and clears any range latch.
3. Press "Set sensor zero".
4. Jog to each lock and capture Left and Right.

### D8. Sensorless manual steering (rationale and safety note)
- `jog()` and `nudge()` no longer check `sensorOk_`. They check only `steerDriverOk_` (the VESC
  link) and the stall latch.
- With a healthy sensor, nothing changes: the stall detector and the stall latch run as today. The
  range guard (D9) does **not** apply to jog or nudge; the operator may drive past ±550°.
- With the sensor faulted, absent or invalid, they drive **open loop**:
  - **There is no stall backstop**, because there is no position to watch.
  - What stops the motor:
    - the jog refresh timeout `STEER_JOG_TIMEOUT_MS` (500 ms without a refresh from the held web
      button);
    - the nudge pulse length `STEER_NUDGE_MS`;
    - the VESC's own *Motor Current Max* clamp and its fault-code stop, which is the stall-stop
      path, unchanged.
  - A sensor fault that appears **during** a jog does not stop it. It continues sensorless under
    the same bounds.
- Rationale:
  - Without this, a vehicle with a missing or unzeroed sensor cannot be driven to the centre in the
    first place, and the zero-set flow (D7) is impossible.
  - The operator is watching the wheels and holding a dead-man button. The VESC current clamp
    bounds the force at the mechanical lock.
- **Safety note**:
  - Holding jog against a lock while sensorless loads the steering rack at the VESC current limit
    until the button is released.
  - The UI shows a "sensorless — no stall protection" note on the jog controls whenever
    `steer_sensor_ok` is false.
  - The VESC *Motor Current Max* must be set to a value the rack can hold indefinitely. That is
    already a bench prerequisite of the VESC change.

### D9. Range guard (closed-loop moves only, healthy sensor only)
- `STEER_LWS_SAFE_RANGE` = 5500 (550.0°). It keeps a margin under the measured ~600° latch point.
- **Scope**: only closed-loop position moves driven by percent commands: the MAVLink
  `SERVO_OUTPUT_RAW` path, the web slider and test buttons, boot centring and the failsafe centre.
  These need a healthy sensor anyway.
- **Clamping, not refusal**: the target, after the clamp to the stored limits, is clamped again to
  ±`STEER_LWS_SAFE_RANGE`. A limit stored beyond 550° therefore maps ±100 % to ±550° instead of
  the limit; the command is still accepted.
- **Outward stop**: a closed-loop move whose drive direction is outward while |pos| ≥ the safe
  range is stopped (for example after an overshoot). Driving back inward is always allowed.
- **Not guarded**: jog, nudge and the Left/Right limit captures. Calibration must be able to reach
  and record the real locks wherever the zero is; the operator watches the wheels and holds the
  dead-man button. A capture beyond 550° is accepted with a warning (D6, `steer_range_warning`).
- Why: a zero set at the wrong place (for example near a lock) can put the other lock beyond 600°.
  Automatic steering must never latch the sensor; manual steering may, at the operator's choice,
  and the latch (fault code 9, token `range_latched`) is cleared only by restarting the vehicle.
- While sensorless there is no guard, because there is no angle (and no closed loop).

### D10. Constant retuning (AS5600 counts → 0.1°)
1 AS5600 count = 360/4096 ≈ 0.088°, which is 0.88 units of 0.1°. The old values are converted to
about the same physical angle, rounded to a sensible step, and marked **bench-tune**:

| Constant | Old (counts) | ≈ angle | New (0.1°) | Note |
|----------|-------------|---------|------------|------|
| `STEER_POSITION_TOLERANCE` | 10 | 0.88° | **10** (1.0°) | ten sensor LSBs; bench-tune |
| `STEER_STALL_THRESHOLD` | 5 | 0.44° | **5** (0.5°) | progress in 500 ms below this = stall; bench-tune |
| `STEER_BOOST_MIN_PROGRESS` | 3 | 0.26° | **3** (0.3°) | per 100 ms check; bench-tune |
| `STEER_KP` | 1.0 duty/count | duty saturates at 255 counts ≈ 22.4° | **1.15f** duty per 0.1° | saturates at ≈ 222 units ≈ 22.2°; bench-tune |
| `STEER_CAL_MIN_SPAN` | 50 | 4.4° | **300** (30.0°) | deliberately larger: limits are hundreds of degrees from zero, so 30° only catches a capture made at the centre by mistake; bench-tune |
| `STEER_RETARGET_TOLERANCE` | 70 | 6.2° (≈ 1 % of the old half-travel) | **50** (5.0°) | ≈ 1 % of a ~500° half-travel; bench-tune |
| `STEER_MOVE_TIMEOUT` | 7500 ms | — | 7500 ms | unchanged; bench-check a full lock-to-lock move |

New constants:
- `STEER_LWS_CAN_ID` (0x305)
- `STEER_LWS_CFG_ID` (0x7C0)
- `STEER_LWS_TIMEOUT_MS` (100)
- `STEER_LWS_RECOVER_FRAMES` (5)
- `STEER_LWS_SAFE_RANGE` (5500)
- `STEER_LWS_ZERO_GAP_MS` (100)
- `STEER_LWS_ZERO_CONFIRM_MS` (1000)
- `STEER_LWS_ZERO_TOLERANCE` (10)
- `CAN_RX_LWS_MASK` (0x7FF)

Removed constants:
- `STEER_SENSOR_FAULT_DEBOUNCE`, `STEER_COARSE_POLL_MS`, `STEER_SENSOR_MISMATCH_COUNTS`
- `STEER_COARSE_MIN_PER_TURN`, `STEER_COARSE_MIN_GAP`, `STEER_CAL_LINEARITY_COUNTS`
- `STEER_SENSOR_REPROBE_MS`, `STEER_TURN_RECOVER_SAMPLES`
- `AS5600_RECOVER_AFTER_FAILS`, `I2C2_BUS_RECOVER_MS`

`STEER_BOOT_CENTER_TIMEOUT_MS` (5000) is kept.

### D11. NVS migration
- Namespace `"steering"`. It is shared with `steer_sca_base`, so it must **never** be wiped with
  `clear()`.
- New layout: `cal_ver` = **3**, `lim_l` and `lim_r` (int32, 0.1°, firmware sign; `INT32_MIN` if
  unset).
- On load, if `cal_ver` ≠ 3:
  - Remove, one by one, every obsolete key that exists: `fc_raw`, `cc_raw`, `fl_pos`, `cl_raw`,
    `fr_pos`, `cr_raw` (v2), and `c_ang`, `l_ang`, `r_ang` (v1).
  - Write `cal_ver` = 3 with no limits.
  - Log "recalibration required".
  - The steering is then *not calibrated*: percent commands are rejected (fail-safe), and jog and
    nudge still work.
- Erasing at load rather than at the first save means no firmware build ever sees a mix of layouts.
- **BREAKING**: every vehicle must re-zero (if the sensor's zero is not already at straight-ahead)
  and re-capture Left and Right after flashing.
- Rollback to 1.2.0 finds `cal_ver` = 3 and treats it as uncalibrated, so it needs the old
  three-point calibration with the AS5600 hardware refitted.

### D12. Boot behaviour
- `begin()` no longer probes any I2C device. It loads the limits, and the sensor starts as
  `timeout` until frames arrive.
- `setup()` calls `requestBootCenter()` whenever the steering is calibrated. It no longer requires
  a healthy sensor at that instant: the LWS and the ECU come up on the ignition key and may start
  sending after the ESP32 boots.
- `update()` issues the centre move once, when the steering is calibrated, the sensor is healthy and
  the VESC link is up. The request is dropped silently after `STEER_BOOT_CENTER_TIMEOUT_MS`.
- Any accepted steering, jog or nudge command, a capture or a zero-set cancels a pending centring.
- No homing, as before.

### D13. What is deleted
- Files: `include/AS5600Sensor.h`, `src/AS5600Sensor.cpp`.
- `SteeringController`:
  - `fine_`/`coarse_` and every fine/coarse/turn/session member;
  - `sampleFine`, `sampleCoarse`, `crossCheck`, `resetRecovery`, `coarseLin`, `expectedFine`,
    `revalidateCalibration` (ratio, seam, linearity);
  - the session slots and pre-fill, `finalizeCalibration`, `discardCalSession`;
  - `captureCenter`;
  - the accessors `getFineRaw`, `getCoarseRaw`, `getTurn`, `getRatio`, `hasResidual`,
    `getResidual`, `isTurnRecovered`, `isMismatchLatched`, `getCoarseConsecutiveFailures`,
    `getDisplayPosition`/`getDisplayPoint`, `isCalPending`, `hasCalPoint`, `isCalSessionLost`,
    `getCalError`;
  - the `begin()` overload that takes I2C pins.
- `main.cpp`: `recoverI2C2IfWedged()`, the `Wire.begin()` on GPIO1/2, and the I2C1 boot scan. The
  I2C2 boot scan expects 0x1A, 0x1C and 0x1F.
- Telemetry: `steer_position` (counts), `steer_center`, `steer_fine_raw`, `steer_coarse_raw`,
  `steer_turn`, `steer_ratio`, `steer_residual`, `steer_cal_points`, `steer_cal_error`,
  `steer_cal_lost`.
- Web command `steer_cal_center`: it is answered with a failure pointing to "Set sensor zero".
- `PIN_STEER_SDA`/`PIN_STEER_SCL` are renamed `PIN_I2C1_SDA`/`PIN_I2C1_SCL` (GPIO1/2), defined but
  unused. This keeps the documented X14 contingency for the relay board one bus argument away.

### D14. Telemetry semantics (web JSON)
- `steer_angle`: degrees, 1 decimal, firmware sign (positive = right). `null` unless the last
  accepted frame is fresh and its angle is valid.
- `steer_rate`: deg/s, signed (positive = turning right, from the CW bit). `null` when the speed
  byte is `0xFF` or there is no fresh frame.
- `steer_left`, `steer_right`: stored limits in degrees, `null` when unset.
- `steer_lws_ok`, `steer_lws_cal`, `steer_lws_adj` (booleans) and `steer_lws_fault_code` (0–15):
  from the last accepted frame. `null` when no frame has ever been accepted.
- `steer_sensor_fault`: the D5 token. It is never `none` while `steer_sensor_ok` is false.
- `steer_sensor_ok`, `steer_calibrated`: same meaning as before (healthy sensor; both limits valid).
- `steer_range_warning`: `true` while either stored limit is beyond ±`STEER_LWS_SAFE_RANGE`
  (closed-loop moves are then clamped short of that limit and the limit is near the ~600° latch),
  otherwise `false`.
- `steer_lws_fps`: accepted frames in the last second (about 100 when healthy).
- `steer_lws_crc_err`, `steer_lws_cnt_err`, `steer_lws_miss`: cumulative since boot.
- `steer_zero_state`: `idle` / `running` / `ok` / `failed`. `steer_zero_error`: a reason token or
  `null`.
- `steering_pct` and MAVLink `STEER_POS` keep their percent semantics.

## Alternatives rejected
- **Gear-down mount**: unnecessary. The real lock-to-lock is inside the ±600° range, and gearing
  adds backlash and a ratio to calibrate.
- **Relay- or MOSFET-powered sensor for automatic latch clearing**: one more output and more logic
  for a rare event. The sensor is on the ignition key, so restarting the vehicle clears it.
- **Keeping the AS5600 pair as a fallback**: it keeps all the complexity and the I2C2 cable risk
  this change exists to remove, and a two-source arbitration has its own failure modes.
- **A firmware-side zero (centre offset in NVS) instead of the sensor zero**: it duplicates state
  the sensor already persists, can drift from it after a sensor swap, and would let the ±600° latch
  window sit off-centre without anyone noticing. The sensor zero centres that window on
  straight-ahead.

## Risks / Trade-offs
- **Sensorless jog has no stall backstop** (D8). The mitigations are the dead-man refresh timeout,
  the VESC current clamp and the UI warning.
- **Bus load and the OBD cadence**:
  - LWS adds 100 frames/s (≈ 105 bits each, ≈ 2 % of 500 kbit/s). OBD traffic is a few
    frames per second.
  - The separate RXB1 path and BUKT keep OBD replies safe. Bench-verify that RPM polling keeps its
    cadence during a gear change (`CAN_POLL_INTERVAL_RPM_BOOST`, 50 ms).
- **Chip re-init gap**:
  - The no-OBD-data re-init (every ≥ 15 s while the ECU is silent, for example ignition on with
    the engine ECU unpowered) resets the MCP2515. That causes a short LWS gap: `begin()` has a
    5 ms reset delay plus SPI setup.
  - If the gap exceeds 100 ms, the steering faults briefly and recovers by itself.
  - Bench-measure the gap. If it is too long, skip the re-init while LWS frames are being accepted.
    That is a follow-up, not part of this change.
- **Two 120 Ω terminators only**: adding a node can leave the bus with three terminators, or
  with the stub too long. Verify on the vehicle (task 9.1).
- **A wrong zero**: if the zero is set off-centre, the ±550° guard still keeps closed-loop moves
  out of the latch, but the percent mapping is asymmetric and may be clamped short of one lock.
  The fix is the documented flow: jog to centre, restart, zero.
- **Manual drive past 550°**: jog and nudge are not guarded, so an operator can latch the sensor
  (fault 9) while calibrating. Mitigations: `steer_range_warning` and the UI note next to a limit
  beyond 550°; recovery is a vehicle restart (the stored zero survives).
- **Behaviour change for operators**: the centre button is gone, recalibration is mandatory after
  the update, and jog works with a dead sensor.

## Migration Plan
1. Remove both AS5600s and the geared-down shaft. Mount the LWS directly on the steering shaft and
   wire CAN-H/CAN-L to the MCP2515 bus. Check that there are exactly two 120 Ω terminators. Wire
   12 V from the ignition key, with ground in common.
2. Flash firmware 1.3.0 and the LittleFS image.
3. On first boot the old keys are erased and the steering reports *not calibrated*.
4. If the sensor's zero is not straight-ahead:
   1. Jog to straight-ahead.
   2. Restart the vehicle.
   3. Press "Set sensor zero" and wait for "zero confirmed".
5. Jog to the left lock and capture Left; jog to the right lock and capture Right.
6. Restart the vehicle: boot centring runs, and `steer_angle` reads about 0.
7. Rollback: re-flash 1.2.0 and refit the AS5600 hardware. The limits are lost either way.

## Open Questions
- None blocking. The D10 values and the re-init gap are settled on the bench.
