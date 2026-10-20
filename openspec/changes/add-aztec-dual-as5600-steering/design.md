## Context
The `aztec` branch is a new vehicle, a buggy, built on the same `Control_v0` board, with the same
Flipsky VESC steering drive on UART2 (X6) and the same web calibration flow. Only the steering
position feedback changes. The steering shaft turns ±1.7 turns from centre, so a single AS5600
(12-bit, 0–4095 counts per turn, about 0.088° per count) is no longer absolute. A second AS5600
sits on a geared-down shaft that sweeps less than 360° over the whole lock-to-lock travel, so that
second sensor *is* absolute but coarse. The exact gear ratio is not known; the firmware has to work
it out during calibration.

Ground truth in today's code:
- `AS5600Sensor` is hard-wired to `Wire`. After 5 failed reads in a row, its `recoverBus()`
  tears down and restarts `Wire`.
- `SteeringController` keeps raw angles (`c_ang`/`l_ang`/`r_ang` in NVS namespace `"steering"`) and
  computes wrap-safe positions with `wrapDelta`. It declares a sensor fault after
  `STEER_SENSOR_FAULT_DEBOUNCE` (5) bad samples in a row and recovers automatically.
- `BoardInputs` (I2C2 `0x1A`) recovers by *re-configuring the device* every
  `BOARD_INPUT_RECOVER_MS`. `RelayController` (I2C2 `0x1F`) verifies each write and latches a fault.
  **Nothing restarts `Wire1` today.**
- All I2C traffic, including web commands, which are queued and run in `loop()`, happens on the
  Arduino loop task, so no transaction is ever in flight while another loop-task step runs.
- The calibration capture response prints `getRawAngle()`. Web telemetry publishes
  `steer_position` (the raw angle) and `steer_center`/`steer_left`/`steer_right` (raw angles, `-1`
  when unset). The UI checks `>= 0` to decide what to show. MAVLink `STEER_POS` is percent.

## Goals / Non-Goals
- Goals:
  - Absolute multi-turn steering position at boot without homing.
  - The same fail-safe policy as today: any sensor fault stops the motor and rejects commands.
  - The same calibration buttons.
  - The control loop stays in today's units (fine-sensor counts).
- Non-Goals:
  - Running on one sensor when the other has failed.
  - Calibrating in degrees.
  - Supporting the single-sensor quad-bike mechanics on this branch.
  - A runtime switch between vehicles.
  - Changing the MAVLink message set.

## Decisions

### D1. Working position = unwrapped fine counts relative to centre
- `fineRaw` (0..4095) is read every loop.
- `turn` is a signed integer turn counter, updated incrementally by
  `turn += wrapCarry(fineRaw, prevFineRaw)`, where a jump of more than 2048 counts counts as a wrap.
  This is valid because the shaft cannot move half a turn (about 0.3 s at full steering speed)
  between two loop samples.
- `pos = fineRaw + 4096·turn − fineCenterRaw`. It is an int32, in sensor direction.
- The control loop uses `rel = invert ? −pos : pos`, so positive means right, as today.
- Percent mapping, P + boost, stall detection, the stall latch and the tolerances are unchanged in
  structure. `wrapDelta` is used only for the per-sample wrap carry.

### D2. Coarse sensor linearised with a "seam" in its unused arc
The coarse sweep is less than 360° but may be lopsided around centre, so the shortest-delta trick
does not apply. Instead:
- From the coarse left and right captures, and the coarse centre capture that decides which way the
  travel goes, the firmware finds the travel arc and its complement, the **unused gap**. The
  **seam** is the middle of that gap.
- `coarseLin(c) = (c − seam) mod 4096` is then continuous and increases or decreases steadily over
  the whole travel, with margin up to half the gap beyond each limit.
- The gap must be at least `STEER_COARSE_MIN_GAP` (256 counts, about 22.5°). Otherwise the
  calibration is rejected, because the sweep is too close to 360° to tell where the travel ends.

### D3. Ratio and line fit derived from the two limits
- `R = (coarseLin(cR) − coarseLin(cL)) / (fineR − fineL)`, in coarse counts per fine count, with a
  sign. `fineL` and `fineR` are the limit positions in fine counts from centre, sensor direction.
- The expected fine position from the coarse sensor is
  `fineExp(c) = fineL + (coarseLin(c) − coarseLin(cL)) / R`.
- Checks applied when calibration is captured or reloaded:
  1. `fineL` and `fineR` are on opposite sides of 0, each at least `STEER_CAL_MIN_SPAN` away.
  2. `coarseLin(cL) < coarseLin(cC) < coarseLin(cR)`, or the reverse order. This means the coarse
     sensor moves steadily in one direction as the fine sensor turns.
  3. `|R|·4096 ≥ STEER_COARSE_MIN_PER_TURN` (150 coarse counts per steering-shaft turn). This
     keeps a quarter-turn mismatch at 37 or more coarse counts, well above the AS5600's noise and
     typical gear backlash.
  4. Gap ≥ `STEER_COARSE_MIN_GAP`, which also gives a total coarse span below 4096.
  5. Linearity: `|fineExp(cC)| ≤ STEER_CAL_LINEARITY_COUNTS` (512). The centre capture must lie on
     the line through the two limits. This catches a slipping gear, a wrong magnet or a turn
     counted wrongly during calibration.
- `R`, the seam and `invert` are derived values. They are recomputed by `revalidateCalibration()`
  and never stored.

### D4. Turn recovery at boot and after any sensor fault
- When the steering is calibrated and both sensors give a good sample:
  - `k = round((fineExp(c) − (fineRaw − fineCenterRaw)) / 4096)`
  - `pos = fineRaw − fineCenterRaw + 4096·k`
  - `residual = pos − fineExp(c)`
- A single sample is never trusted. Recovery succeeds only after `STEER_TURN_RECOVER_SAMPLES` (5)
  **consecutive** coarse polls each give the same turn and `|residual| ≤
  STEER_SENSOR_MISMATCH_COUNTS` (1024, a quarter turn). "The same turn" is compared as the
  correction `k − turn` to the incremental counter, so a fine wrap between polls (wheel pushed by
  hand while the motor is stopped) is not a disagreement. A poll with a residual beyond the
  threshold, a different turn, a bad coarse read or no fresh fine sample restarts the count. The
  1024 threshold is below 2048 counts, so the turn choice within one sample can never be ambiguous.
- If recovery samples keep disagreeing for `STEER_SENSOR_FAULT_DEBOUNCE` polls in a row, the fault
  token becomes `mismatch`. This recovery-time mismatch is not latched: no position was ever
  accepted, so recovery simply continues and succeeds once 5 polls agree.
- Recovery runs at boot (once the calibration is loaded) and every time either sensor's fault
  clears (I2C or magnet). Until it succeeds, `isSensorOk()` is false, the motor stays stopped,
  commands are rejected and the fault token is `recovering` (or `mismatch`, above).
- `begin()` primes up to `STEER_SENSOR_FAULT_DEBOUNCE + STEER_TURN_RECOVER_SAMPLES` back-to-back
  samples so that a healthy system is normally recovered before `setup()` decides on boot
  centering.

### D5. Runtime consistency check
- While calibrated, `residual = pos − fineExp(coarse)` is computed on every coarse sample.
- `|residual| > STEER_SENSOR_MISMATCH_COUNTS` on `STEER_SENSOR_FAULT_DEBOUNCE` coarse samples in a
  row raises a mismatch fault that **latches**: `sensorOk` stays false and the token stays
  `mismatch` even if later coarse samples agree again. A persistent disagreement means a slipped
  gear or a lost turn; re-deriving the turn from one side cannot tell which side is wrong, so the
  operator must see it.
- The latch clears only when a calibration is completed (D7: all three points captured in the
  current power-on, which requires both sensors to be individually healthy and redefines the
  frame) or on a reboot (followed by D4 recovery). Points are never pre-filled from the stored
  calibration while a mismatch is latched, so a single capture cannot clear it. The residual keeps
  being reported while latched, for diagnosis.
- The coarse sensor is polled every `STEER_COARSE_POLL_MS` (10 ms) rather than on every loop. It is
  used only for turn recovery and the consistency check, and the slower poll keeps I2C2 traffic
  down for the expanders. The worst-case time to detect a coarse fault is about 50 ms.

### D6. Combined fault policy
- `sensorOk = fineOk && coarseOk && (!calibrated || (turnRecovered && !mismatch && !mismatchLatched))`.
- Each sensor has its own debounce counter (`STEER_SENSOR_FAULT_DEBOUNCE`, the same glitch filter
  as today). A sample is good only when the I2C read succeeds and the magnet-detect bit is set.
- On a fault the motor stops, moves and jogs are cancelled, and commands are rejected, exactly as
  in today's path.
- The fault reason is kept as a token for telemetry and logs: `none`, `fine_i2c`, `fine_magnet`,
  `coarse_i2c`, `coarse_magnet`, `mismatch` or `recovering` (both sensors healthy and calibrated,
  turn not yet recovered). Priority is in that order after `none`; the token is never `none` while
  `sensorOk` is false.
- While uncalibrated, there is no ratio, so there is no mismatch check. Jogging is allowed when both
  sensors are healthy, because the stall detector still runs on the fine position.

### D7. Calibration points in any order (same three buttons)
- **Session frame**: the firmware tracks `sessionPos = fineRaw + 4096·sessionTurn`, where
  `sessionTurn` counts fine-sensor wraps since boot. It is never re-derived by turn recovery or
  changed by a calibration, so it stays continuous for the whole power-on as long as the fine
  sensor is.
- **Three session slots** (RAM only): centre, left and right, each `{valid, sessionPos, coarseRaw}`.
  - Each capture button requires both sensors individually healthy and a fresh fine sample. A
    mismatch or a not-yet-recovered turn does not block a capture: completing a calibration is the
    way out of a stale one.
  - A button writes **only its own slot**; pressing it again re-records that point. No capture
    clears another point. The motor is stopped on every capture.
  - Early feedback: a limit closer than `STEER_CAL_MIN_SPAN` to a recorded centre (or a centre that
    close to a recorded limit) is refused immediately ("too close to center", "too close to left
    limit", "too close to right limit").
- **Pre-fill**: when the stored calibration is complete and valid and its absolute turn is trusted
  (calibrated, turn recovered, no mismatch, no latched mismatch), the empty slots are filled from it
  at capture time (`centre = sessionPos − pos`, `left = centre + fineLeft`, `right = centre +
  fineRight`, coarse values from storage). A calibrated vehicle can therefore re-capture a single
  point. When the turn is not trusted the slots start empty and all three points are required.
- **Finalization**: as soon as all three slots are valid, the candidate `fineCenterRaw = centre
  mod 4096`, `fineLeft/Right = slot − centre` and the coarse values are checked with D3.
  - Accepted: it becomes the active calibration, `turn` is set so that `pos = sessionPos − centre`,
    the turn counts as recovered, any mismatch (also a latched one) is cleared, the calibration is
    persisted, and the slots are emptied (they can be re-derived by pre-fill while trusted).
  - Rejected: nothing is saved, the previously active calibration stays in force, the slots are
    kept so the wrong point can be re-captured, and the response and telemetry name the failed
    check, for example "ratio too small", "no coarse gap" or "center not on line".
- **Unfinished points are discarded** on a reboot (slots start empty; only complete calibrations are
  persisted and loaded) and on any sensor fault that may break the session frame: a fine or coarse
  sensor fault, or a runtime mismatch latching. Telemetry reports the discard until the next
  capture. All three points therefore have to be captured within one power-on without a sensor
  fault in between.
- **Why a session frame**: limits are stored as fine counts from the centre. Without a known
  absolute turn (uncalibrated, or after a reboot) only the incremental count since boot relates the
  three captures to each other, and it is valid only while the fine sensor is continuously tracked.

### D8. Bus-agnostic `AS5600Sensor` and I2C2 recovery
- `AS5600Sensor(TwoWire& bus, const char* label, bool mayRecoverBus)`. `begin(sda, scl)` keeps the
  pins only for recovery.
- **Fine sensor (`Wire`, `mayRecoverBus = true`)**: keeps today's `recoverBus()` (end + begin with
  `I2C_BUS_FREQ_HZ`). I2C1 still has only one device.
- **Coarse sensor (`Wire1`, `mayRecoverBus = false`)**: never calls `end()` or `begin()` on `Wire1`.
  It reports failures and exposes `consecutiveFailures()`.
- **Coordinated recovery owned by `main.cpp`**:
  - It runs only after `boardInputs.update()` and `vehicleController.update()` have finished for
    the iteration, so no `Wire1` transaction is in flight.
  - It restarts `Wire1` with the exact parameters it was opened with, and only when **both** of
    these hold:
    - the coarse AS5600 has at least `AS5600_RECOVER_AFTER_FAILS` failures in a row;
    - `BoardInputs::isFaulted()` is true.

    Two independent devices failing together point to the bus, not a device.
  - It is rate-limited to once per `I2C2_BUS_RECOVER_MS` (2000 ms) and logged once per attempt.
  - Restarting the ESP32 I2C controller does not touch PCA9557 registers, so relay outputs and
    expander configuration stay as they were. `BoardInputs` re-configures itself anyway on its
    1000 ms recovery cadence, and `RelayController` keeps its verify-on-write policy.
  - If only the coarse sensor fails while the expanders read fine, the bus is healthy. The firmware
    raises a steering fault and does not reset the bus.
- **Alternatives considered**:
  - (a) Report only, with no recovery on I2C2. This is simpler, but the driver's known "stuck in
    ESP_ERR_INVALID_STATE" mode would then hold steering, gear inputs and relays faulted until a
    reboot.
  - (b) Let the coarse driver restart `Wire1` itself. That could cut a relay write-verify sequence
    in half and break the "drivers never re-open a bus they do not own" rule.
  - (c) Move the coarse sensor off I2C2: an AS5600L at a different address on X14, or the AS5600
    analogue or PWM `OUT` pin on a free GPIO such as GPIO3 or GPIO38. Both mean hardware changes
    the user has not chosen. They are noted as a fallback if X15 proves noisy.

### D8a. Boot centering deferred to `update()`
- The VESC link is only known to be up after its first reply, which `update()` sees. A
  `setSteeringPercent(0)` from `setup()` was always rejected, so boot centering never ran.
- `setup()` calls `requestBootCenter()` when calibrated, sensors OK and turn recovered. `update()`
  issues the centre move once, when `calibrated && sensorOk && steerDriverOk`, and logs
  "Moving to center" at that moment.
- The request is dropped silently after `STEER_BOOT_CENTER_TIMEOUT_MS` (5000 ms), so a VESC that
  comes up late does not make the vehicle steer by itself much later. An accepted steering, jog or
  nudge command, or a calibration capture, cancels it; a rejected command does not.

### D9. Boot without a sensor
Today a sensor that does not answer at boot is never retried (`initialized_` stays false). Under
the new design:
- Calibration is **always loaded**.
- A sensor that did not answer is re-probed every `STEER_SENSOR_REPROBE_MS` (1000 ms).
- When it starts answering, turn recovery (D4) runs and steering becomes available without a
  reboot. This satisfies the existing "resume automatically when valid readings return" scenario
  for a cable plugged in late.

### D10. NVS layout
- Namespace `"steering"` (shared with `steer_sca_base`, so it must **never** be wiped with
  `clear()`).
- Keys:
  - `cal_ver` = 2
  - `fc_raw`, `cc_raw`: centre captures, 0..4095, `-1` if unset
  - `fl_pos`, `fr_pos`: limits as int32 fine counts from centre, sensor direction, `INT32_MIN` if
    unset
  - `cl_raw`, `cr_raw`: coarse values at the limits
- The firmware removes `c_ang`, `l_ang` and `r_ang` individually the first time it saves. Values
  are loaded only when `cal_ver == 2`. Otherwise the steering is uncalibrated and needs
  recalibrating.

### D11. Telemetry semantics (web JSON)
- One frame throughout: **fine counts from the captured centre, in sensor direction**. This is the
  frame the operator sees during calibration, before `invert` is known.
- When no centre has been captured, `steer_position` is counted from the boot sample of the fine
  sensor, so jogging still shows movement.
- While calibration points are pending (D7), `steer_position`, `steer_left`, `steer_center` and
  `steer_right` show the session points instead of the stored calibration: relative to the centre
  point when it is recorded, otherwise relative to the first point captured (which then reads 0).
  This keeps the readouts meaningful whatever the capture order.
- Fields:
  - `steer_position`: int
  - `steer_center`: `0` when set, `null` when unset (the display frame is relative to the centre
    whenever one is recorded)
  - `steer_left`, `steer_right`: int, or `null` when unset
  - `steer_fine_raw`: 0..4095, or `null` if the fine sensor is not readable
  - `steer_coarse_raw`: 0..4095, or `null` if the coarse sensor is not readable
  - `steer_turn`: int, the turn index relative to the centre turn
  - `steer_ratio`: float, coarse counts per fine turn, `null` when uncalibrated
  - `steer_residual`: int fine counts, `null` when uncalibrated or a sensor is faulted
  - `steer_sensor_fault`: the D6 token
  - `steer_sensor_ok`, `steer_calibrated`: unchanged
  - `steer_cal_points`: string of the points recorded this power-on, a subset of `"LCR"`
    (`""` when none are pending)
  - `steer_cal_error`: the failed check of the last rejected finalization, `null` otherwise
  - `steer_cal_lost`: `true` after unfinished points were discarded by a sensor fault, until the
    next capture
- The sentinel for "unset" changes from `-1` to `null`, because positions can now be negative.
- `steering_pct` and MAVLink `STEER_POS` keep percent semantics.

### D12. Constants re-tuned for 3.4-turn travel
- Travel is now about ±6963 counts instead of about ±700, so 1 % of travel is about 70 counts.
- `STEER_RETARGET_TOLERANCE` (10 counts) would become 0.14 % of travel. MAVLink jitter alone would
  then restart the move and stall timers and bring back the old re-command bug. The new initial
  value is **70** (about 1 % of half-travel).
- `STEER_MOVE_TIMEOUT` must cover a full lock-to-lock move on the new gearing.
- `STEER_POSITION_TOLERANCE`, `STEER_KP`, `STEER_STALL_THRESHOLD`, `STEER_BOOST_MIN_PROGRESS` and
  `STEER_CAL_MIN_SPAN` keep their units and are bench-tuned.

## Risks / Trade-offs
- **A long coarse-sensor cable on the shared I2C2 bus** can disturb the relay board and the gear
  inputs. For example, a shorted SDA line would hold the whole bus.
  - Mitigation: keep the X15 cable short and screened, and keep the expanders' own fail-safes.
  - The coordinated recovery handles controller wedges. It cannot handle a line held low
    physically.
  - Fallback: alternative (c) in D8.
- **Gear backlash between the steering shaft and the coarse shaft** shows up as residual noise.
  - Mitigation: the 1024-count mismatch threshold is generous (90° of shaft), and the linearity
    check rejects bad gearing at calibration.
- **A latched mismatch needs operator action** (capturing all three points again, or a reboot).
  This is deliberate: an automatic re-derivation could silently pick the wrong side.
- **Turn tracking needs a short loop period.** The incremental counter assumes less than half a turn
  between fine samples (about 0.3 s at full speed). An unplugged X15 makes every I2C2 user hit the
  50 ms `Wire` timeout per loop; the loop period in that case must be measured on the bench.
- **Turn counter skips during a long loop stall**, such as a slow NVS or LittleFS write. This needs
  a stall of more than about 0.3 s at full steering speed.
  - While calibrated, the consistency check turns it into a mismatch fault (motor stop), never a
    silent offset.
  - While uncalibrated, only unfinished calibration points are affected, and the line-fit check
    rejects the result.
- **Behaviour change for operators**: the three points can be captured in any order but must all be
  captured in one power-on; a reboot or a sensor fault discards unfinished points. Every vehicle
  must be recalibrated after flashing.

## Migration Plan
1. Flash the 1.2.0 firmware and the LittleFS image.
2. On first boot the old keys are ignored, and the steering reports "not calibrated" with commands
   rejected (fail-safe).
3. Jog to straight-ahead and capture the centre, and jog to each lock and capture the left and
   right limits, in any order, without rebooting in between.
4. Check that `steer_ratio` and a `steer_residual` close to 0 appear in the calibration card, then
   power-cycle and confirm the position is recovered.
5. Rollback: re-flash 1.1.0. The old keys are gone, so the quad-bike firmware also needs a
   recalibration. This branch is not meant for that vehicle anyway.

## Open Questions
- None blocking. Bench tuning will set the constant values in D12.
