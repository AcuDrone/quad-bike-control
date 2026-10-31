## 1. Decoder (Arduino-free) and native tests
- [x] 1.1 Add `include/Lws305.h`: port `CAN-TEST/src/lws305.h` (`lwsCrcOk`, `lwsAngleRaw`, `lwsAngleValid`, `lwsOk`, `lwsCal`, `lwsAdj`, `lwsFault`, `lwsDirCW`, `lwsCounter`, `LWS_DLC_MIN`, `LWS_ANGLE_INV`), plus the speed decode (×4 °/s, `0xFF` invalid) and the two fixed config payloads `LWS_CFG_RESET = {0x05,0x00}` / `LWS_CFG_ZERO_ID305 = {0x53,0x30}`; no Arduino includes
- [x] 1.2 Add `include/LwsSensor.h` / `src/LwsSensor.cpp`: `onFrame(buf, len, nowMs)` with accept rules (DLC ≥ 6, checksum, counter changed), counters (`crcErr`, `cntErr`, `miss`), frames-per-second window, last accepted frame + timestamp, fault token evaluation (D5 order) and the healthy-streak of `STEER_LWS_RECOVER_FRAMES`; no Arduino dependency in the decision logic (time passed in)
- [x] 1.3 Add `[env:native]` (Unity) to `platformio.ini` and `test/test_lws305/test_main.cpp` with vectors from the CAN-TEST README:
  - `0B 32 00 07 49 00` → angle +286.6° (raw 2866), firmware pos −2866, speed 0, OK=1 cal=1 adj=1 fault=0, counter 9, checksum OK
  - `0C 77 01 87 13 00` → +319.1° (raw 3191), speed 4 °/s, direction CW, status OK, counter 3
  - the same frames with one bit flipped in byte 0 → checksum rejected, `crcErr` incremented
  - a repeated counter → rejected, `cntErr` incremented; counter 3 → 6 → accepted, `miss` += 2
  - `7F FF FF 4C 55 00` → angle invalid, speed invalid, OK=0, cal=0, adj=1, fault 9 → token `range_latched`
  - OK=1 cal=0 with angle `0x7FFF` → `not_calibrated`; OK=1 cal=1 fault=2 → `sensor_fault`; DLC 5 → ignored
  - no frame for > `STEER_LWS_TIMEOUT_MS` → `timeout`; healthy only after 5 consecutive good frames
  - `((0x30 << 8) | 0x53) >> 4 == 0x305` and `LWS_CFG_ZERO_ID305` ≠ `{0x03,0x00}` (guards the ID-0x000 trap)
- [x] 1.4 `pio test -e native` passes

## 2. CAN controller (shared bus)
- [x] 2.1 Rename `CAN_RX_ID_MASK`/`CAN_RX_ID_FILTER` to `CAN_RX_OBD_*`; add `CAN_RX_LWS_MASK` (0x7FF), `STEER_LWS_CAN_ID` (0x305), `STEER_LWS_CFG_ID` (0x7C0)
- [x] 2.2 `applyRxFilters()`: RXB0 = mask0 0x7F8 + filters 0–1 = 0x7E8; RXB1 = mask1 0x7FF + filters 2–5 = 0x305; update its log line; re-init path keeps calling it
- [x] 2.3 `attachLwsSink(LwsSensor*)`; private `dispatchSteeringFrame()` called first in `drainRxBuffers()`, `tryReceiveResponse()` and `tryReceiveProbeResponse()`; LWS frames are not logged as unmatched
- [x] 2.4 Zero-set sub-state machine (`WAIT_IDLE` → `SEND_RESET` → `GAP` → `SEND_ZERO` → `DONE`): pauses new OBD requests, lets an in-flight one finish, `STEER_LWS_ZERO_GAP_MS` gap without blocking, resumes polling; `startLwsZero()` refuses when not initialized / probe running / already running; aborts with `tx_failed` on a send error; only code that transmits on 0x7C0, payloads only the two constants
- [x] 2.5 Expose zero-set phase/result to the steering controller (sent-reset flag, sent-zero timestamp, abort reason)

## 3. Steering controller
- [x] 3.1 Remove the AS5600 members, the dual-sensor logic, the session slots, `captureCenter()` and the accessors listed in design D13; `begin()` takes no I2C arguments; constructor no longer takes buses
- [x] 3.2 Own an `LwsSensor`; position `pos = −angle` (0.1°, int32); health/token per D5; on fault stop a closed-loop move only (jog/nudge continue)
- [x] 3.3 `jog()`/`nudge()`: drop the `sensorOk_` check (keep the `steerDriverOk_` check and the stall latch); stall detection only while the sensor is healthy; **no** range guard on jog/nudge
- [x] 3.4 `setSteeringPercent()`: rejected unless both limits valid and sensor healthy; map −100/0/+100 to left/0/right; clamp to limits, then to ±`STEER_LWS_SAFE_RANGE` (D9); stop a closed-loop move driving outward at |pos| ≥ the safe range (boot centring and failsafe centre go through the same path)
- [x] 3.5 `captureLeftLimit()`/`captureRightLimit()`: D6 validation and reasons (no safe-range check), immediate per-key save, motor stop; a capture beyond ±`STEER_LWS_SAFE_RANGE` succeeds with a warning
- [x] 3.6 `requestSensorZero()`: stop motion, cancel boot centering, start the CAN sequence, erase both limits (NVS + RAM) once the reset frame is sent, confirm within `STEER_LWS_ZERO_CONFIRM_MS` (OK=1, fault=0, cal=1, |angle| ≤ `STEER_LWS_ZERO_TOLERANCE`), result `ok` / `failed` + reason
- [x] 3.7 NVS v3 (`cal_ver`=3, `lim_l`, `lim_r`); migration on load removes every v1/v2 key individually; never `clear()` the namespace
- [x] 3.8 Boot centering: requested whenever calibrated; issued once when sensor healthy + VESC up; dropped after `STEER_BOOT_CENTER_TIMEOUT_MS`; cancelled by any accepted manual command, capture or zero-set
- [x] 3.9 Debug logs: `[STEER] angle=… pos=… tok=… fps=…` on state change and rate-limited while moving; raw sensor sign shown once at boot
- [x] 3.10 Delete `include/AS5600Sensor.h` and `src/AS5600Sensor.cpp`; no remaining references (`rg AS5600 src include` only in comments that describe history, preferably none)

## 4. Constants
- [x] 4.1 Add `STEER_LWS_TIMEOUT_MS` 100, `STEER_LWS_RECOVER_FRAMES` 5, `STEER_LWS_SAFE_RANGE` 5500, `STEER_LWS_ZERO_GAP_MS` 100, `STEER_LWS_ZERO_CONFIRM_MS` 1000, `STEER_LWS_ZERO_TOLERANCE` 10
- [x] 4.2 Re-express in 0.1° (bench-tune): `STEER_POSITION_TOLERANCE` 10, `STEER_STALL_THRESHOLD` 5, `STEER_BOOST_MIN_PROGRESS` 3, `STEER_KP` 1.15f, `STEER_CAL_MIN_SPAN` 300, `STEER_RETARGET_TOLERANCE` 50; keep `STEER_MOVE_TIMEOUT` 7500
- [x] 4.3 Remove `STEER_SENSOR_FAULT_DEBOUNCE`, `STEER_COARSE_POLL_MS`, `STEER_SENSOR_MISMATCH_COUNTS`, `STEER_COARSE_MIN_PER_TURN`, `STEER_COARSE_MIN_GAP`, `STEER_CAL_LINEARITY_COUNTS`, `STEER_SENSOR_REPROBE_MS`, `STEER_TURN_RECOVER_SAMPLES`, `AS5600_RECOVER_AFTER_FAILS`, `I2C2_BUS_RECOVER_MS`
- [x] 4.4 Rename `PIN_STEER_SDA`/`PIN_STEER_SCL` → `PIN_I2C1_SDA`/`PIN_I2C1_SCL` (defined, unused); rewrite the steering, I2C and CAN comment blocks (LWS on CAN, sign, units, NVS v3, filter plan)
- [x] 4.5 Bump `FIRMWARE_VERSION` to `1.3.0`

## 5. Boot and wiring (`src/main.cpp`, `VehicleController`)
- [x] 5.1 Remove `recoverI2C2IfWedged()` and its call; stop starting `Wire`; I2C2 boot scan expects 0x1A, 0x1C, 0x1F
- [x] 5.2 Construct the steering controller without buses; attach its `LwsSensor` to the CAN controller in `setup()`; keep `canController_.update()` before `steering_.update()`
- [x] 5.3 `VehicleController`: route `steer_lws_zero`; answer `steer_cal_center` with a failure pointing to "Set sensor zero"; capture responses in degrees with reasons, plus the beyond-safe-range warning; zero-set response reports the final result

## 6. Telemetry and web UI
- [x] 6.1 `WebPortal.h` telemetry struct + `TelemetryManager.cpp`: add the D14 fields (including `steer_range_warning`); remove the dual-sensor fields
- [x] 6.2 `WebPortal.cpp`: serialize `null` for unset/invalid values (never NaN, never sentinel numbers)
- [x] 6.3 `data/index.html` calibration card: angle/rate/limits readout, status bits, fault-code, fps, counters, zero-set state; remove the Center button and fine/coarse/turn/ratio/residual/session UI; add "Set sensor zero" with a confirm dialog (limits will be erased); jog/nudge never disabled by sensor state; sensorless warning; a note next to a limit beyond ±550° when `steer_range_warning` is true (latch threshold near); en/uk help text and fault-token labels (`range_latched` → "restart the vehicle")
- [x] 6.4 Confirm MAVLink `STEER_POS` gate is `steerSensorOk && steerCalibrated` and update the `MavlinkInterface.h` field comment (CAN sensor healthy / both limits stored)

## 7. Docs and changelog
- [x] 7.1 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`: GPIO1/2 rows (`PIN_I2C1_*`, X14 free), GPIO47/48 rows and X15 row (no coarse AS5600), X14 row, I2C allocation table, boot-scan note, remove the I2C2-recovery note; CAN section: LWS on the MCP2515 bus, termination note
- [x] 7.2 `MAVLINK_SETUP.md`: `STEER_POS` validity text and the steering fault table (new tokens, `range_latched` = restart the vehicle; sensorless jog note)
- [x] 7.3 `include/Changelog.h`: `1.3.0` entry, uk + en — steering angle from the Bosch CAN sensor (AS5600 removed); "Set sensor zero" button; centre = sensor zero, only Left/Right captured; jog works without a sensor; recalibration required after the update

## 8. Build and validation
- [x] 8.1 `pio run` builds clean with no warnings from the changed files
- [x] 8.2 `pio test -e native` passes (task 1.4)
- [x] 8.3 `openspec validate replace-aztec-steering-sensor-with-bosch-lws --strict` passes

## 9. Verification (hardware-in-loop)
- [ ] 9.1 [USER/MANUAL] Fit the LWS directly on the steering shaft; wire CAN-H/CAN-L to the MCP2515 bus beside the ECU, 12 V from the ignition key; measure ≈ 60 Ω between CAN-H and CAN-L with power off (exactly two terminators)
- [ ] 9.2 [USER/MANUAL] Boot: `steer_lws_fps` ≈ 100, `steer_lws_crc_err` stays 0, engine RPM/coolant/etc. still update at their normal cadence with the engine running; `[CAN]` log shows the two-path filter line
- [ ] 9.3 [USER/MANUAL] Gear change with the engine running: RPM polling at `CAN_POLL_INTERVAL_RPM_BOOST`, no `dataValid` loss, steering stays healthy
- [ ] 9.4 [USER/MANUAL] Sign: turn the wheel right by hand → `steer_angle` increases (positive); left → negative
- [ ] 9.5 [USER/MANUAL] Fresh flash: steering reports not calibrated, old keys gone (log), percent commands rejected, jog and nudge work
- [ ] 9.6 [USER/MANUAL] Sensorless centring: unplug the sensor CAN or 12 V → token `timeout`, jog/nudge still drive and stop on button release (≤ 500 ms) and at nudge end; reconnect, jog to straight-ahead, restart the vehicle, press "Set sensor zero" → state `ok`, `steer_angle` ≈ 0.0, limits shown as `--`
- [ ] 9.7 [USER/MANUAL] Capture Left and Right at the locks: values saved immediately, `steer_calibrated` true; a Left capture right of zero and one closer than 20° are each refused with the named reason; a capture beyond 550° is saved, the response warns and `steer_range_warning` is true with the UI note shown
- [ ] 9.8 [USER/MANUAL] Restart the vehicle at several wheel angles: boot centering moves to 0.0 ± tolerance without homing
- [ ] 9.9 [USER/MANUAL] MAVLink at 25 Hz: smooth tracking, re-command guard holds, stall detection and move timeout still fire; `STEER_POS` matches `steering_pct` and goes `NaN` on a sensor fault
- [ ] 9.10 [USER/MANUAL] Fault injection during a closed-loop move: pull the sensor CAN → move stops within ~100 ms, token `timeout`, percent rejected; reconnect → healthy again without reboot
- [ ] 9.11 [USER/MANUAL] Range guard: with a zero deliberately set near one lock and the far limit captured beyond 550°, a +/−100 % percent command (web slider, MAVLink) stops at 550° (no `range_latched`), while jog toward the far lock is not stopped by the guard; then re-zero at centre and re-capture. Only if the latch is ever reached: token `range_latched`, restart clears it, zero preserved
- [ ] 9.12 [USER/MANUAL] Zero-set while the engine runs and while an OBD request is in flight: sequence completes, OBD resumes, no stale-data loss; zero-set during an ECU probe is refused as `busy`
- [ ] 9.13 [USER/MANUAL] Measure the LWS gap during a forced MCP2515 re-init (ECU unpowered, ignition on, ≥ 15 s): note whether `timeout` appears; record the gap for the follow-up decision in design Risks
- [ ] 9.14 [USER/MANUAL] Bench-tune and record final values of the D10 constants (`STEER_KP`, tolerances, `STEER_CAL_MIN_SPAN`, `STEER_RETARGET_TOLERANCE`, `STEER_MOVE_TIMEOUT`)
