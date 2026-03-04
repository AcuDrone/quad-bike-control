# Project Context

## Purpose
Electronic control system for a quad bike, receiving commands via S-bus from ArduPilot Rover and controlling steering, throttle, transmission, and braking systems via ESP32-C6 microcontroller.

## Tech Stack
- **Platform**: ESP32-C6 DevKit
- **Framework**: Arduino (via PlatformIO)
- **Language**: C++17
- **Build System**: PlatformIO
- **High-Level Controller**: ArduPilot Rover (via S-bus protocol)
- **Hardware**:
  - S-bus receiver input (16-channel RC protocol)
  - 2x PWM Servo Motors (steering, throttle)
  - 2x Linear Actuators with BTS7960 H-bridge drivers (transmission, brakes)

## Project Conventions

### Code Style
- Follow Arduino/ESP32 naming conventions
- Class names: PascalCase (e.g., `ServoController`, `BrakeSystem`)
- Methods: camelCase (e.g., `setPosition()`, `emergencyStop()`)
- Constants: UPPER_SNAKE_CASE with `PIN_` or `CONFIG_` prefixes
- File organization: One class per .h/.cpp pair in include/src directories
- Comments: Doxygen-style for public APIs

### Architecture Patterns
- **Layered architecture**: S-bus input → Command interpreter → Vehicle systems → Hardware actuators
- **ArduPilot integration**: ESP32 acts as servo/actuator driver, ArduPilot provides high-level control
- **Fail-safe design**: All systems default to safe states on signal loss or errors
- **Configuration over code**: Pin assignments and parameters in Constants.h

### Testing Strategy
- Hardware-in-loop testing required for all actuators
- S-bus signal simulation for testing without ArduPilot
- Integration tests with ArduPilot Rover
- Calibration validation on each deployment

### Git Workflow
- Main branch: stable, deployable code only
- Feature branches: individual implementations
- Commit messages: Clear, imperative mood
- Hardware testing required before merge

## Domain Context

### ArduPilot Rover Integration
- **ArduPilot Rover**: Open-source autonomous vehicle platform
- **S-bus**: Futaba's serial bus protocol, 16 channels, 11-bit resolution (880-2160μs range)
- **Channel mapping** (typical):
  - Ch1: Steering
  - Ch2: Throttle
  - Ch3: Transmission selector
  - Ch4: Brake control
  - Ch5+: Auxiliary/mode switches

### Safety-Critical System
This is a **safety-critical vehicle control system**:
- Fail-safe on S-bus signal loss (brakes on, neutral, center steering)
- Emergency stop must work independently
- Watchdog timer mandatory
- Brake commands always take priority
- Safe state after power loss or reset

## Important Constraints

### Hardware Constraints
- ESP32-C6 limited GPIO pins
- S-bus requires UART with inverted signal (or inverter circuit)
- BTS7960 requires high current supply (separate from logic power)
- Linear actuators may lack position feedback

### Performance Constraints
- S-bus frame rate: 7-14ms (70-140Hz)
- Control loop must match or exceed S-bus rate
- Servo PWM must be 50Hz for stability
- Watchdog timeout: 2-5 seconds

### Safety Constraints
- Signal loss timeout: 500ms maximum before fail-safe
- Never disable watchdog in production
- Brake system independent of other systems

## External Dependencies

### Libraries (via PlatformIO)
- `framework-arduinoespressif32`: Core ESP32 framework
- S-bus library (e.g., `sbus` by bolderflight)
- ESP32 LEDC: Hardware PWM
- ESP32 NVS: Configuration persistence
- ESP32 Watchdog Timer

### Hardware Dependencies
- ArduPilot Rover with S-bus output
- S-bus receiver or direct S-bus connection
- Power supplies: 5V logic, 12-24V motors
- BTS7960 motor driver modules
