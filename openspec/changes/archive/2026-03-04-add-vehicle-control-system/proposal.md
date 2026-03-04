# Change: Add Vehicle Control System with S-bus Integration

## Why
This quad bike requires electronic control of steering, throttle, transmission, and braking systems via ESP32-C6, receiving commands from ArduPilot Rover via S-bus protocol. The system must provide safe, reliable control of 2 servos (PWM-based) and 2 linear actuators (BTS7960 motor driver) with fail-safe mechanisms for signal loss.

## What Changes
- Add S-bus protocol decoder for receiving commands from ArduPilot Rover
- Add PWM-based servo control for steering wheel and acceleration throttle
- Add BTS7960 motor driver control for transmission gear selection (R, N, H, L) and brakes
- Implement high-level vehicle control systems with safety features and signal loss detection
- Define pin configurations, S-bus channel mappings, and hardware interfaces for ESP32-C6 DevKit
- Establish control protocols, fail-safe mechanisms, and ArduPilot integration

## Impact
- Affected specs: **sbus-input**, **vehicle-actuators**, **vehicle-systems** (all new)
- Affected code: src/main.cpp, include/Constants.h, new input/control/hardware modules
- Hardware dependencies: ESP32-C6 DevKit, S-bus receiver or ArduPilot connection, 2x servo motors, 2x linear actuators with BTS7960 drivers
- External system: ArduPilot Rover (S-bus command source)
