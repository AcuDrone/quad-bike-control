# Tasks: update-steering-as5600

## 1. AS5600 sensor driver
- [x] 1.1 Add `include/AS5600Sensor.h` / `src/AS5600Sensor.cpp`: `begin(sda, scl)` (Wire @ 400 kHz, addr 0x36), `read(rawAngle, magnetOk)` — single 3-byte burst from STATUS (0x0B) through RAW ANGLE (0x0C/0x0D), so magnet status comes free with every angle read
- [ ] 1.2 Bench-verify on serial log: raw angle tracks shaft rotation, magnet flag drops when magnet removed

## 2. Constants
- [x] 2.1 `include/Constants.h`: remove `PIN_STEER_ENCODER_A/B`, `PCNT_UNIT_STEER`, `STEER_DEFAULT_CENTER`, `STEER_HOMING_TIMEOUT`
- [x] 2.2 Add `PIN_STEER_SDA` (GPIO 41), `PIN_STEER_SCL` (GPIO 42), `LEDC_CH_STEER_RPWM` (6), `LEDC_CH_STEER_LPWM` (7), `STEER_KP`, `STEER_PWM_MIN_DUTY`, `STEER_PWM_MAX_DUTY`, `STEER_CAL_MIN_SPAN`; re-scope `STEER_POSITION_TOLERANCE` / `STEER_STALL_THRESHOLD` to AS5600 counts

## 3. SteeringController rework
- [x] 3.1 Replace `EncoderCounter*` with `AS5600Sensor` + owned `BTS7960Controller`; `begin()` initializes both, checks magnet
- [x] 3.2 Wrap-safe relative position (`((raw - center + 2048) & 4095) - 2048`); remove `autoHome()`, `attachEncoder()`, `moveRight()`/`moveLeft()` digital drive
- [x] 3.3 P-control `update()`: duty = clamp(Kp·|error|, min, max) toward target, stop within tolerance; keep move-timeout and stall backstops; jog API for calibration
- [x] 3.4 Calibration API: `captureCenter()` / `captureLeftLimit()` / `captureRightLimit()` (persist NVS `steering/c_ang,l_ang,r_ang` with span sanity check), `loadCalibration()`, calibrated/sensor-ok getters
- [x] 3.5 Asymmetric `setSteeringPercent()` / `getSteeringPercent()` mapping (−100→left limit, 0→center, +100→right limit); reject commands when uncalibrated or sensor invalid
- [x] 3.6 Sensor-fault handling in `update()`: I2C failure or magnet lost → immediate stop + SERVO debug log

## 4. Integration
- [x] 4.1 `src/main.cpp`: drop `steeringEncoder`/`attachEncoder`/`autoHome` block; init steering (I2C pins, LEDC ch 6/7), `loadCalibration()`, move to center when calibrated + magnet OK
- [x] 4.2 `src/VehicleController.cpp` (+ header): replace `set_steer_center`/`move_steer_center` with `steer_cal_center`/`steer_cal_left`/`steer_cal_right` (capture current angle) and `steer_jog` (direction/stop); expose limits + sensor-ok for telemetry
- [x] 4.3 Delete `include/EncoderCounter.h`, `src/EncoderCounter.cpp` (steering was the last user)

## 5. Telemetry + web UI
- [x] 5.1 `WebPortal.h`/`WebPortal.cpp`/`TelemetryManager.cpp`: `steer_position`/`steer_center` in AS5600 counts; add `steer_left`, `steer_right`, `steer_sensor_ok`, `steer_calibrated`
- [x] 5.2 `data/index.html`: rework steering calibration section — live angle readout, jog left/right (momentary), Set center / Set left / Set right buttons, sensor/magnet status indicator
- [ ] 5.3 Flash web UI with `pio run -t uploadfs`

## 6. Verification
- [x] 6.1 `~/.platformio/penv/bin/pio run` builds clean (no EncoderCounter/PCNT references remain)
- [ ] 6.2 Bench test: calibrate center + both limits, verify −100/0/+100% sweep, wrap-crossing travel, stall/timeout stop, magnet-removal stop, MAVLink + failsafe centering
- [x] 6.3 `openspec validate update-steering-as5600 --strict` passes
