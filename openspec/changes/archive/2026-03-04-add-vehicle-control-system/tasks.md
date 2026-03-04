# Implementation Tasks

## 1. Project Setup and Configuration
- [ ] 1.1 Update platformio.ini with required libraries
  - [ ] 1.1.1 Add S-bus library (e.g., `sbus` by bolderflight)
  - [ ] 1.1.2 Add ESP32 PWM and NVS libraries
- [ ] 1.2 Define all GPIO pin assignments in include/Constants.h
  - [ ] 1.2.1 S-bus UART pin and configuration
  - [ ] 1.2.2 Servo PWM pins (2 pins)
  - [ ] 1.2.3 BTS7960 motor driver pins (4 pins total: 2x RPWM, 2x LPWM only - enable pins hardwired)
- [ ] 1.3 Add S-bus channel mapping constants
- [ ] 1.4 Add hardware configuration constants (PWM frequencies, pulse widths, limits)
- [ ] 1.5 Set up serial debug output configuration

## 2. S-bus Input Layer
- [ ] 2.1 Create include/SBusInput.h and src/SBusInput.cpp
  - [ ] 2.1.1 Implement UART initialization for S-bus (100000 baud, 8E2)
  - [ ] 2.1.2 Handle signal inversion (hardware or software)
  - [ ] 2.1.3 Implement frame decoding (25-byte S-bus protocol)
  - [ ] 2.1.4 Add channel value extraction (16 channels, 11-bit)
  - [ ] 2.1.5 Implement signal freshness checking
  - [ ] 2.1.6 Add signal quality monitoring
- [ ] 2.2 Create include/CommandMapper.h and src/CommandMapper.cpp
  - [ ] 2.2.1 Implement mapSteering() with deadband
  - [ ] 2.2.2 Implement mapThrottle() with deadband
  - [ ] 2.2.3 Implement mapGear() with discrete position ranges
  - [ ] 2.2.4 Implement mapBrake()
  - [ ] 2.2.5 Add configurable mapping parameters
- [ ] 2.3 Create test sketch for S-bus decoder
  - [ ] 2.3.1 Display all 16 channels in real-time
  - [ ] 2.3.2 Show signal quality metrics
  - [ ] 2.3.3 Test with ArduPilot or S-bus simulator

## 3. Hardware Abstraction Layer - Actuators
- [ ] 3.1 Create include/ServoController.h and src/ServoController.cpp
  - [ ] 3.1.1 Implement LEDC PWM initialization
  - [ ] 3.1.2 Implement setAngle() and setMicroseconds() methods
  - [ ] 3.1.3 Add angle-to-pulse-width conversion
  - [ ] 3.1.4 Add disable() method
- [ ] 3.2 Create include/BTS7960Controller.h and src/BTS7960Controller.cpp
  - [ ] 3.2.1 Implement GPIO and PWM initialization (RPWM, LPWM only)
  - [ ] 3.2.2 Implement setSpeed() with forward/reverse logic
  - [ ] 3.2.3 Implement stop() and brake() methods
  - [ ] 3.2.4 Note: Enable pins hardwired to 5V, no enable/disable control needed
- [ ] 3.3 Create unit test sketches for each actuator type
  - [ ] 3.3.1 Test ServoController with single servo
  - [ ] 3.3.2 Test BTS7960Controller with single actuator

## 4. Vehicle Systems Layer
- [ ] 4.1 Create include/SteeringSystem.h and src/SteeringSystem.cpp
  - [ ] 4.1.1 Implement percentage-to-angle mapping
  - [ ] 4.1.2 Add center() convenience method
  - [ ] 4.1.3 Implement rate limiting for smooth transitions
- [ ] 4.2 Create include/ThrottleSystem.h and src/ThrottleSystem.cpp
  - [ ] 4.2.1 Implement percentage-to-angle mapping
  - [ ] 4.2.2 Add idle() convenience method
  - [ ] 4.2.3 Add brake override logic hook
- [ ] 4.3 Create include/TransmissionSystem.h and src/TransmissionSystem.cpp
  - [ ] 4.3.1 Define gear enumeration (REVERSE, NEUTRAL, HIGH, LOW)
  - [ ] 4.3.2 Implement setGear() with position control
  - [ ] 4.3.3 Implement calibrate() routine with NVS storage
  - [ ] 4.3.4 Add safe gear transition logic (must pass through NEUTRAL)
- [ ] 4.4 Create include/BrakeSystem.h and src/BrakeSystem.cpp
  - [ ] 4.4.1 Implement percentage-to-extension mapping
  - [ ] 4.4.2 Add release() convenience method
  - [ ] 4.4.3 Implement emergencyStop() method

## 5. Configuration and Storage
- [ ] 5.1 Create include/VehicleConfig.h with configuration structures
  - [ ] 5.1.1 S-bus channel mappings and parameters
  - [ ] 5.1.2 Servo and actuator calibration values
  - [ ] 5.1.3 Safety thresholds and timeouts
- [ ] 5.2 Create include/ConfigManager.h and src/ConfigManager.cpp
  - [ ] 5.2.1 Implement NVS initialization and access
  - [ ] 5.2.2 Implement load/save configuration methods
  - [ ] 5.2.3 Add calibration data storage methods
  - [ ] 5.2.4 Implement factory reset functionality
- [ ] 5.3 Define default configuration values

## 6. Vehicle Coordinator
- [ ] 6.1 Create include/VehicleController.h and src/VehicleController.cpp
  - [ ] 6.1.1 Implement begin() with system initialization sequence
  - [ ] 6.1.2 Integrate SBusInput and CommandMapper
  - [ ] 6.1.3 Add update() method to read S-bus and update systems
  - [ ] 6.1.4 Add getState() for querying all system states
  - [ ] 6.1.5 Implement emergencyStop() coordinating all systems
  - [ ] 6.1.6 Add safety interlock logic (brake priority, throttle limits)
- [ ] 6.2 Implement fail-safe logic
  - [ ] 6.2.1 Detect S-bus signal loss
  - [ ] 6.2.2 Apply fail-safe commands (brakes on, neutral, center, idle)
  - [ ] 6.2.3 Handle signal recovery
- [ ] 6.3 Implement system health monitoring
  - [ ] 6.3.1 Add S-bus signal quality monitoring
  - [ ] 6.3.2 Add actuator timeout detection
  - [ ] 6.3.3 Implement fault logging
  - [ ] 6.3.4 Add getDiagnostics() method

## 7. Safety Features
- [ ] 7.1 Configure and enable ESP32 watchdog timer
- [ ] 7.2 Implement global emergency stop function
- [ ] 7.3 Add input validation and clamping for all S-bus channels
- [ ] 7.4 Implement safe startup state (brakes on, neutral, idle, center)
- [ ] 7.5 Add safe shutdown procedure
- [ ] 7.6 Verify fail-safe timing (signal loss detection within 500ms)

## 8. Main Control Loop
- [ ] 8.1 Update src/main.cpp setup()
  - [ ] 8.1.1 Initialize serial communication for debug output
  - [ ] 8.1.2 Initialize watchdog
  - [ ] 8.1.3 Initialize VehicleController (includes S-bus)
  - [ ] 8.1.4 Load configuration
  - [ ] 8.1.5 Perform startup self-test
  - [ ] 8.1.6 Wait for valid S-bus signal before enabling control
- [ ] 8.2 Update src/main.cpp loop()
  - [ ] 8.2.1 Update VehicleController (reads S-bus, updates systems)
  - [ ] 8.2.2 Check for fail-safe conditions
  - [ ] 8.2.3 Refresh watchdog timer
  - [ ] 8.2.4 Monitor and log system health
  - [ ] 8.2.5 Optional: Output diagnostics to serial

## 9. Testing and Validation
- [ ] 9.1 S-bus integration testing
  - [ ] 9.1.1 Verify S-bus frame decoding with ArduPilot
  - [ ] 9.1.2 Test all 4 control channels (steering, throttle, trans, brake)
  - [ ] 9.1.3 Test signal loss detection and fail-safe activation
  - [ ] 9.1.4 Verify signal recovery behavior
  - [ ] 9.1.5 Monitor signal quality metrics
- [ ] 9.2 Hardware-in-loop testing
  - [ ] 9.2.1 Test steering system through full range via S-bus
  - [ ] 9.2.2 Test throttle system response via S-bus
  - [ ] 9.2.3 Test transmission gear selection and transitions via S-bus
  - [ ] 9.2.4 Test brake system actuation via S-bus
- [ ] 9.3 Integration testing
  - [ ] 9.3.1 Test brake priority over throttle
  - [ ] 9.3.2 Test safe gear transitions
  - [ ] 9.3.3 Test emergency stop from various states
  - [ ] 9.3.4 Test watchdog reset recovery
  - [ ] 9.3.5 Test S-bus signal loss during various operations
- [ ] 9.4 Calibration procedures
  - [ ] 9.4.1 Calibrate S-bus channel ranges and deadbands
  - [ ] 9.4.2 Calibrate servo min/max positions
  - [ ] 9.4.3 Calibrate transmission detent positions
  - [ ] 9.4.4 Calibrate brake engagement points
- [ ] 9.5 Performance validation
  - [ ] 9.5.1 Measure control loop timing (should be >50Hz)
  - [ ] 9.5.2 Verify S-bus reception rate (~70-140Hz)
  - [ ] 9.5.3 Verify PWM signal quality with oscilloscope
  - [ ] 9.5.4 Test under load conditions with full vehicle
  - [ ] 9.5.5 Measure fail-safe response time (<500ms)

## 10. Documentation
- [ ] 10.1 Document S-bus integration with ArduPilot
  - [ ] 10.1.1 S-bus wiring and signal inversion
  - [ ] 10.1.2 ArduPilot output configuration
  - [ ] 10.1.3 Channel mapping table
- [ ] 10.2 Document pin assignments and wiring diagram
- [ ] 10.3 Document calibration procedures
  - [ ] 10.3.1 S-bus channel calibration
  - [ ] 10.3.2 Actuator calibration
- [ ] 10.4 Document fail-safe behavior and testing
- [ ] 10.5 Create troubleshooting guide
  - [ ] 10.5.1 S-bus signal issues
  - [ ] 10.5.2 Actuator problems
  - [ ] 10.5.3 Fail-safe troubleshooting
- [ ] 10.6 Document safety procedures and emergency responses

## Dependencies and Parallelization

**Can be done in parallel:**
- Tasks 2.1 and 2.2 (SBusInput and CommandMapper are independent)
- Tasks 3.1 and 3.2 (ServoController and BTS7960Controller are independent)
- Tasks 4.1, 4.2, 4.3, 4.4 after Task 3 completes (all vehicle systems are independent)
- Task 5 can be done in parallel with Tasks 2, 3, 4

**Sequential dependencies:**
- Task 2 must complete before Task 6 (coordinator needs S-bus input)
- Task 3 must complete before Task 4 (systems depend on actuators)
- Task 4 must complete before Task 6 (coordinator needs vehicle systems)
- Task 5 must complete before Task 6 (coordinator needs configuration)
- Task 6 requires Tasks 2, 3, 4, and 5 to be complete
- Task 7 requires Task 6 to be complete
- Task 8 requires Task 7 to be complete
- Task 9 requires Task 8 to be complete
- Task 10 can be done in parallel with Tasks 8-9

**Critical path:** Tasks 3 → 4 → 6 → 7 → 8 → 9
