# Project Context

## Purpose
Electronic control system for a quad bike, receiving commands via MAVLink from an ArduPilot (Pixhawk) autopilot and controlling steering, throttle, transmission, and braking systems via an ESP32-S3 microcontroller on the custom `Control_v0` control board.

## Tech Stack
- **Platform**: ESP32-S3-WROOM-1U on the custom `Control_v0` control board (see `GPIO_PINOUT_CUSTOM_BOARD_S3.md`)
- **Framework**: Arduino (via PlatformIO)
- **Language**: C++17
- **Build System**: PlatformIO
- **High-Level Controller**: ArduPilot Rover (via MAVLink 2 over UART1)
- **Hardware**:
  - MAVLink serial link to the Pixhawk TELEM port (SERVO_OUTPUT_RAW command channels)
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
- **Layered architecture**: MAVLink input → Command interpreter → Vehicle systems → Hardware actuators
- **ArduPilot integration**: ESP32 acts as servo/actuator driver, ArduPilot provides high-level control
- **Fail-safe design**: All systems default to safe states on signal loss or errors
- **Configuration over code**: Pin assignments and parameters in Constants.h

### Testing Strategy
- Hardware-in-loop testing required for all actuators
- MAVLink command simulation / web portal control for testing without ArduPilot
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
- **MAVLink**: SERVO_OUTPUT_RAW carries up to 16 command channels in microseconds (880-2160µs range)
- **Channel mapping** (typical):
  - Ch1: Steering
  - Ch2: Throttle
  - Ch3: Transmission selector
  - Ch4: Brake control
  - Ch5+: Auxiliary/mode switches

### Safety-Critical System
This is a **safety-critical vehicle control system**:
- Fail-safe on MAVLink link loss (brakes on, neutral, center steering)
- Emergency stop must work independently
- Watchdog timer mandatory
- Brake commands always take priority
- Safe state after power loss or reset

## Important Constraints

### Hardware Constraints
- Limited free ESP32-S3 GPIO: gear switches, brake endstop and relays are behind I2C PCA9557 port expanders
- MAVLink runs at 115200 through BSS138 level shifters — the baud rate must not be raised
- BTS7960 requires high current supply (separate from logic power)
- Linear actuators may lack position feedback

### Performance Constraints
- MAVLink command stream: ~25 Hz SERVO_OUTPUT_RAW
- Control loop must match or exceed the command stream rate
- Servo PWM must be 50Hz for stability
- Watchdog timeout: 2-5 seconds

### Safety Constraints
- Signal loss timeout: 500ms maximum before fail-safe
- Never disable watchdog in production
- Brake system independent of other systems

## External Dependencies

### Libraries (via PlatformIO)
- `framework-arduinoespressif32`: Core ESP32 framework
- `mavlink/c_library_v2`: MAVLink 2 message library
- ESP32 LEDC: Hardware PWM
- ESP32 NVS: Configuration persistence
- ESP32 Watchdog Timer

### Hardware Dependencies
- ArduPilot Rover with a MAVLink TELEM port
- Custom `Control_v0` board (PCA9557 opto-input and relay expanders, 12V→24V boost converter)
- Power supplies: 5V logic, 12-24V motors
- BTS7960 motor driver modules
