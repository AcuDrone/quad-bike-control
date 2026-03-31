## 1. Setup
- [x] 1.1 Add steering BTS7960 GPIO pins and encoder pins to `Constants.h` (GPIO 9, 10, 11, 12)
- [x] 1.2 Add steering calibration NVS keys and position tolerance to `Constants.h`
- [x] 1.3 Remove servo-related steering constants (`PIN_STEERING_PWM`, `LEDC_CH_STEERING`, `STEERING_SERVO_*`, `STEERING_*_ANGLE`)

## 2. SteeringController
- [x] 2.1 Create `SteeringController.h` with class interface (begin, moveRight/Left/stop, setPosition, update, calibration, NVS)
- [x] 2.2 Implement `SteeringController.cpp` with GPIO digital write for direction, encoder for position, stall detection
- [x] 2.3 Implement `setSteeringPercent()` mapping from -100..+100 to encoder range within limits
- [x] 2.4 Implement NVS save/load/clear for center and limits

## 3. Integrate into main.cpp
- [x] 3.1 Replace `ServoController steeringServo` with `SteeringController steeringActuator`
- [x] 3.2 Add second `EncoderCounter steeringEncoder` (PCNT Unit 1)
- [x] 3.3 Initialize steering encoder, controller, attach encoder, load calibration
- [x] 3.4 Add auto-home + move to center in setup sequence

## 4. Update VehicleController
- [x] 4.1 Change `steering_` member from `ServoController&` to `SteeringController&`
- [x] 4.2 Replace `steering_.setAngle(map(...))` with `steering_.setSteeringPercent(pct)` in SBUS and web command paths
- [x] 4.3 Update failsafe to use `steering_.setSteeringPercent(0.0f)` (center)
- [x] 4.4 Update `getSteeringAngle()` to return percentage or encoder position

## 5. Update Telemetry
- [x] 5.1 Update telemetry to report steering percentage or encoder position instead of servo angle

## 6. Add Web Calibration Commands
- [x] 6.1 Add "Set Steering Center" web command (saves current encoder position as center)
- [x] 6.2 Add "Set Steering Right Limit" web command (saves current encoder position as right limit)
- [x] 6.3 Add "Clear Steering Calibration" web command

## 7. Cleanup
- [x] 7.1 Remove `#include "ServoController.h"` from VehicleController if no longer needed
- [x] 7.2 Update `GPIO_PINOUT.md` to reflect new steering pin allocation
- [x] 7.3 Update LEDC channel table (Ch0 freed)

## 8. Validation
- [x] 8.1 Build compiles without errors
- [ ] 8.2 Steering auto-homes and centers on startup
- [ ] 8.3 SBUS steering commands move actuator correctly
- [ ] 8.4 Software limits prevent over-travel
- [ ] 8.5 Calibration persists across reboot
- [ ] 8.6 Failsafe centers steering
