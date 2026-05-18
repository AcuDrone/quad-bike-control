## 1. Constants
- [x] 1.1 Add to `Constants.h`: `TRANS_GEAR_BOOST_TARGET_RPM`, `TRANS_GEAR_BOOST_TIMEOUT`, `TRANS_GEAR_BOOST_PID_KP`, `TRANS_GEAR_BOOST_PID_KI`, `TRANS_GEAR_BOOST_PID_KD`, `CAN_POLL_INTERVAL_RPM_BOOST`
- [x] 1.2 Remove (or zero) unused constants: `TRANS_THROTTLE_BOOST_PERCENT`, `TRANS_THROTTLE_BOOST_DURATION`

## 2. CANController — runtime RPM poll rate
- [x] 2.1 Add `setRPMPollInterval(uint32_t ms)` declaration to `include/CANController.h`
- [x] 2.2 Implement `setRPMPollInterval()` in `src/CANController.cpp` — update `pidTable_[0].interval`

## 3. VehicleController — PID state
- [x] 3.1 In `include/VehicleController.h`: replace `throttleBoostStartTime_` / `throttleBoostActive_` with PID members (`pidIntegral_`, `pidPrevError_`, `gearBoostStartTime_`, `gearBoostActive_`, `lastPIDThrottleAngle_`)
- [x] 3.2 Rename `updateThrottleBoost()` declaration to `updateGearBoostPID()`

## 4. VehicleController — PID implementation
- [x] 4.1 Implement `updateGearBoostPID()` in `src/VehicleController.cpp`:
  - On first call (activation): reset integral, derivative, start timer, call `canController_.setRPMPollInterval(CAN_POLL_INTERVAL_RPM_BOOST)`
  - Guard: skip if engine RPM < `ENGINE_RUNNING_RPM_THRESHOLD` (engine not running)
  - Guard: skip (hold last angle) if CAN data is invalid
  - Compute PID: `error = target - rpm`, `integral += error * dt`, `derivative = (error - prev) / dt`; clamp integral for anti-windup
  - Map output angle, clamp to `[THROTTLE_MIN_ANGLE, THROTTLE_MAX_ANGLE]`, call `throttle_.setAngle()`
  - Timeout: if elapsed > `TRANS_GEAR_BOOST_TIMEOUT`, deactivate and restore CAN poll rate
- [x] 4.2 On deactivation path (timeout or `needsThrottleBoost()` false): call `canController_.setRPMPollInterval(CAN_POLL_INTERVAL_RPM)`, clear active flag

## 5. VehicleController — integration
- [x] 5.1 In `VehicleController::update()`: replace `if (transmission_.needsThrottleBoost() || throttleBoostActive_)` guard with `if (transmission_.needsThrottleBoost() || gearBoostActive_)` calling `updateGearBoostPID()`
- [x] 5.2 Ensure `processSBusCommands()` and `processThrottleCommand()` skip throttle update when `gearBoostActive_` is true

## 6. Validation
- [x] 6.1 Build and confirm no compile errors
- [ ] 6.2 Bench test: trigger gear change with engine running, observe RPM held at target via serial log and web telemetry
- [ ] 6.3 Verify throttle returns to SBUS command after gear change completes
- [ ] 6.4 Verify timeout fallback fires if gear change hangs
