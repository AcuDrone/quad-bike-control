## 1. Sensor driver
- [x] 1.1 Make `AS5600Sensor` bus-agnostic: constructor `(TwoWire& bus, const char* label, bool mayRecoverBus)`; replace every `Wire.` with the bound bus; log with the label
- [x] 1.2 Keep `recoverBus()` only when `mayRecoverBus` (fine sensor on `Wire`); expose `consecutiveFailures()` for the coarse sensor
- [x] 1.3 Add a non-blocking re-probe so a sensor absent at boot is retried every `STEER_SENSOR_REPROBE_MS`

## 2. Steering controller
- [x] 2.1 Hold two sensors (fine on `Wire`, coarse on `Wire1`); change `begin()` to take both buses/pins; always load calibration
- [x] 2.2 Incremental turn tracking on every good fine sample; unwrapped position `fineRaw + 4096·turn − fineCenterRaw`
- [x] 2.3 Per-sensor debounce and fault reason (`fine_i2c`, `fine_magnet`, `coarse_i2c`, `coarse_magnet`, `mismatch`, `recovering`); combined `sensorOk_`; token never `none` while `sensorOk_` is false; stop/reject on fault as today
- [x] 2.4 Coarse poll every `STEER_COARSE_POLL_MS`; seam-linearised coarse value; runtime residual check with debounce; runtime mismatch latches until a completed calibration or reboot
- [x] 2.5 Turn recovery from the coarse sensor at boot and on every faulted→good transition, accepted only after `STEER_TURN_RECOVER_SAMPLES` consecutive agreeing coarse polls (same turn, residual ≤ `STEER_SENSOR_MISMATCH_COUNTS`)
- [x] 2.6 Calibration points in any order: session frame (fine wraps since boot); center/left/right slots, each button writes only its own slot; "too close" early check; pre-fill empty slots from a trusted stored calibration; finalize when all three are set (validate, set turn, clear mismatch, persist; keep slots on rejection); discard unfinished slots on reboot, fine/coarse fault or latched mismatch; load only complete calibrations
- [x] 2.7 `revalidateCalibration()`: arc/gap/seam, ratio, invert, ordering, min per-turn, min gap, center-on-line check; report failed check reason
- [x] 2.8 NVS v2 keys (`cal_ver`, `fc_raw`, `cc_raw`, `fl_pos`, `cl_raw`, `fr_pos`, `cr_raw`); remove `c_ang`/`l_ang`/`r_ang` individually; never `clear()` the namespace (`steer_sca_base` lives there)
- [x] 2.9 New accessors for telemetry: position, limits (optional), fine raw, coarse raw, turn, ratio, residual, fault reason; remove `getRawAngle()` callers
- [x] 2.10 Update debug logs (`[STEER] pos=… turn=… coarse=… resid=…`)

## 3. Constants
- [x] 3.1 Add `STEER_COARSE_POLL_MS` (10), `STEER_SENSOR_MISMATCH_COUNTS` (1024), `STEER_COARSE_MIN_PER_TURN` (150), `STEER_COARSE_MIN_GAP` (256), `STEER_CAL_LINEARITY_COUNTS` (512), `STEER_SENSOR_REPROBE_MS` (1000), `STEER_TURN_RECOVER_SAMPLES` (5), `STEER_BOOT_CENTER_TIMEOUT_MS` (5000), `I2C2_BUS_RECOVER_MS` (2000)
- [x] 3.2 Re-tune for ±6963-count travel: `STEER_RETARGET_TOLERANCE` → 70; review `STEER_MOVE_TIMEOUT` for a full lock-to-lock move; bench-check `STEER_POSITION_TOLERANCE`, `STEER_KP`, `STEER_STALL_THRESHOLD`, `STEER_BOOST_MIN_PROGRESS`, `STEER_CAL_MIN_SPAN`
- [x] 3.3 Update the steering and I2C comment blocks (I2C1 = fine AS5600 on X14; I2C2 = U10/U2 + relay board + coarse AS5600 on X15; NVS keys)

## 4. Boot and bus coordination (`src/main.cpp`)
- [x] 4.1 Pass `Wire`/`Wire1` to the steering controller; update the topology comment and expected scan list (I2C2: 0x1A, 0x1C, 0x1F, 0x36)
- [x] 4.2 Coordinated `Wire1` recovery after the loop's I2C users: coarse sensor failures ≥ `AS5600_RECOVER_AFTER_FAILS` AND `boardInputs.isFaulted()`, rate-limited by `I2C2_BUS_RECOVER_MS`, logged
- [x] 4.3 Boot centering only when calibrated, sensors healthy and turn recovered; deferred via `requestBootCenter()` until the VESC link is up, dropped after `STEER_BOOT_CENTER_TIMEOUT_MS`

## 5. Telemetry and web UI
- [x] 5.1 `WebPortal.h` telemetry struct + `TelemetryManager.cpp`: new frame for `steer_position`/`steer_left`/`steer_right`/`steer_center`; add `steer_fine_raw`, `steer_coarse_raw`, `steer_turn`, `steer_ratio`, `steer_residual`, `steer_sensor_fault`
- [x] 5.2 `WebPortal.cpp`: serialize unset/unreadable values as JSON `null` (not `-1`, not NaN)
- [x] 5.3 `VehicleController::processSteerCalCommand`: response with fine position + coarse raw; failure reason text
- [x] 5.4 `data/index.html`: calibration card readout (position, L/C/R, fine/coarse raw, turn, ratio, residual, fault label), `null` → `--`, updated en/uk help text and fault-token labels
- [x] 5.6 Calibration progress telemetry (`steer_cal_points`, `steer_cal_error`, `steer_cal_lost`) and display frame for pending points; UI ✓ marks, missing points, rejection reason, discard notice; any-order en/uk help text; capture response says complete/still needed
- [x] 5.5 Confirm MAVLink `STEER_POS` is unchanged in code (percent, gated by `isSensorOk()` + `isCalibrated()`)

## 6. Docs, version, changelog
- [x] 6.1 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`: X14 = fine AS5600 (now connected); X15 = relay board + coarse AS5600 0x36; I2C allocation table, GPIO1/2 and 47/48 rows, scan list
- [x] 6.2 Bump `FIRMWARE_VERSION` to `1.2.0` in `include/Constants.h`
- [x] 6.3 Add a `1.2.0` entry (uk + en) to `include/Changelog.h`: two-sensor multi-turn steering for the buggy; recalibration required after update

## 7. Verification (hardware-in-loop)
- [x] 7.1 `pio run` builds clean; `openspec validate add-aztec-dual-as5600-steering --strict` passes
- [ ] 7.2 Boot scan shows both AS5600s (I2C1: 0x36; I2C2: 0x1A, 0x1C, 0x1F, 0x36)
- [ ] 7.3 Calibrate on the buggy; verify `steer_ratio` plausible and `steer_residual` near 0 across full travel
- [ ] 7.4 Power-cycle at several wheel positions (incl. > 1 turn from center each side): position recovered without homing
- [ ] 7.5 Fault injection: unplug fine sensor, unplug coarse sensor, remove each magnet, slip coarse gear (mismatch) — motor stops, commands rejected, correct fault token (`recovering` while the turn is re-acquired), auto-resume after restore for I2C/magnet faults; mismatch stays latched until all three points are re-captured or reboot
- [ ] 7.6 Unplug the coarse sensor while relays and gear inputs keep working (no `Wire1` reset); verify relays/gear inputs unaffected during a coordinated recovery
- [ ] 7.7 MAVLink stream at 25 Hz: re-command guard holds (no timer reset from jitter), stall detection and move timeout still fire
- [ ] 7.8 Measure loop period with X15 unplugged mid-move (Wire timeout 50 ms × failing I2C2 users); fine-sensor turn tracking needs loop < ~300 ms
- [ ] 7.9 Any-order calibration from uncalibrated: Left → Center → Right, Center → Left → Right and Right → Left → Center each finish CALIBRATED with the same limits (±a few counts); ✓ marks and "still needed" follow each press; no point clears another
- [ ] 7.10 Calibrated vehicle: re-capture only Left at a new lock — calibration re-saved at once, center and right unchanged; a Left too close to center is refused
- [ ] 7.11 Latched mismatch: pressing Center alone keeps the mismatch latched and shows "still needed: Left, Right"; capturing all three clears it
- [ ] 7.12 Reboot after two points: both discarded, all three needed; unplug the fine (then the coarse) sensor between points: points discarded and the card shows the discard notice
- [ ] 7.13 Rejected set (e.g. left and right on the same side): failure response names the check, the card shows it, previous calibration still active; re-capturing the wrong point completes the calibration
