# Replace Steering Servo with BTS7960 + Hall Sensor

## Why
The current steering uses a servo motor (ServoController on LEDC Ch0). Replacing it with a BTS7960 H-bridge motor driver + hall sensor encoder provides:
- More torque for steering a quad bike
- Position feedback via encoder (closed-loop control)
- Auto-homing and calibrated center/limits
- No PWM needed (full speed movement) — frees LEDC Ch0

## What
- Remove ServoController for steering
- Add a new `SteeringController` class using plain GPIO digital write (not LEDC) for BTS7960 direction control at full speed
- Add a second EncoderCounter (PCNT Unit 1) for steering position feedback
- On startup: auto-home to one physical limit, then move to calibrated center
- During operation: map steering percentage (-100% to +100%) to encoder positions within software limits
- Store center position and limits in NVS for persistence across reboots

## Impact
- **Frees**: LEDC Ch0 (GPIO 13) — available for future use
- **Uses**: 4 freed ESP32 GPIOs (from MCP23017 migration) for BTS7960 + encoder
- **Modifies**: VehicleController steering command path, main.cpp initialization, Constants.h, telemetry
- **Spec changes**: vehicle-systems (Steering Control System requirement)
