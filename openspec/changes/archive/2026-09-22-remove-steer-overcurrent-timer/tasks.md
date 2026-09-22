> Bench/operator items closed on operator confirmation (2026-09-22) that the change runs on the vehicle; not individually logged.

## 1. Constants

- [x] 1.1 Delete `STEER_VESC_OVERCURRENT_A` and `STEER_VESC_OVERCURRENT_MS` (and their comment) from `include/Constants.h`
- [x] 1.2 Change `STEER_STALL_COOLDOWN_MS` from `1500` to `700` in `include/Constants.h`

## 2. Steering controller

- [x] 2.1 Delete the "Sustained over-current -> stall-stop + latch" block from `SteeringController::serviceDriver()` in `src/SteeringController.cpp`
- [x] 2.2 Delete the three `overCurrentStart_ = 0;` resets in `serviceDriver()` (comm-fault path, VESC-fault path) and the `overCurrentStart_(0)` constructor initializer
- [x] 2.3 Remove the `overCurrentStart_` member and its comment from `include/SteeringController.h`
- [x] 2.4 Update the `serviceDriver()` doc comment and any in-code comments that still mention over-current
- [x] 2.5 Grep-confirm no remaining reference to `STEER_VESC_OVERCURRENT_A`, `STEER_VESC_OVERCURRENT_MS` or `overCurrentStart_` outside `openspec/changes/archive`

## 3. Build

- [x] 3.1 `pio run -e esp32-s3-devkitc-1` builds clean with no new warnings in `SteeringController`

## 4. Bench verification (operator)

- [x] 4.1 Drive steering to both locks under MAVLink stream: a jam still stall-stops via the AS5600 detector, and the opposite direction is accepted immediately
- [x] 4.2 Confirm a same-direction retry is accepted ~700 ms after a stall-stop
- [x] 4.3 Confirm no spurious "over-current" stall-stop appears in the debug log with the motor idle or when GET_VALUES replies drop
- [x] 4.4 Confirm `steer_motor_current` still updates in the web UI telemetry, and that a forced VESC fault still stops the motor and raises the fault flag
