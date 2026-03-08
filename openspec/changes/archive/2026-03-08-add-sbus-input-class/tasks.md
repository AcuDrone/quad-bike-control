# Implementation Tasks

## 1. Channel Configuration Structure
- [x] 1.1 Define SBusChannelConfig struct in Constants.h with channel assignments
- [x] 1.2 Define mapping constants for channel value ranges (gear thresholds, deadzones)
- [x] 1.3 Add configuration documentation in comments

## 2. SBusInput Class Implementation
- [x] 2.1 Create include/SBusInput.h header with class declaration
- [x] 2.2 Create src/SBusInput.cpp implementation file
- [x] 2.3 Add begin() method for UART initialization
- [x] 2.4 Add update() method for frame reading and processing
- [x] 2.5 Implement raw channel value getters (getChannel, getRawChannels)
- [x] 2.6 Implement signal validity methods (isSignalValid, getSignalAge)

## 3. Channel Mapping Methods
- [x] 3.1 Implement getSteering() - returns percentage (-100 to +100) with deadband
- [x] 3.2 Implement getThrottle() - returns percentage (0 to 100) with deadband
- [x] 3.3 Implement getGear() - returns gear enum based on channel ranges
- [x] 3.4 Implement getBrake() - returns percentage (0 to 100)
- [x] 3.5 Implement getIgnitionState() - returns ignition enum (OFF/ACC/IGNITION) based on Ch5 ranges
- [x] 3.6 Implement getFrontLight() - returns boolean (on/off) based on Ch6 threshold
- [x] 3.7 Add helper method for applying deadzones to channel values

## 4. Signal Quality Monitoring
- [x] 4.1 Add frame statistics tracking (total frames, errors, loss count)
- [x] 4.2 Implement getFrameRate() method
- [x] 4.3 Implement getSignalQuality() method returning quality metrics
- [x] 4.4 Add error rate calculation and quality warning detection

## 5. Fail-Safe Detection
- [x] 5.1 Implement signal timeout detection (500ms threshold)
- [x] 5.2 Add fail-safe flag tracking from SBUS frames
- [x] 5.3 Implement recovery frame counting (3 consecutive good frames)
- [x] 5.4 Add callback support for signal loss/recovery events (optional)

## 6. Main Application Integration
- [x] 6.1 Instantiate SBusInput in main.cpp
- [x] 6.2 Initialize SBusInput in setup() with UART configuration
- [x] 6.3 Call SBusInput.update() in main loop

## 7. TelemetryManager Integration
- [x] 7.1 Add SBusInput reference to TelemetryManager constructor
- [x] 7.2 Update determineInputSource() to check SBUS validity first
- [x] 7.3 Update collectTelemetry() to set sbus_active flag from SBusInput

## 8. RelayController Class Implementation
- [x] 8.1 Create include/RelayController.h header with class declaration
- [x] 8.2 Create src/RelayController.cpp implementation file
- [x] 8.3 Add begin() method to initialize relay pins (RELAY1, RELAY2, RELAY3)
- [x] 8.4 Implement setIgnitionState(enum) method - controls RELAY1 (ignition) and RELAY2 (accessory)
- [x] 8.5 Implement setFrontLight(bool) method - controls RELAY3
- [x] 8.6 Add getIgnitionState() and getFrontLight() getters
- [x] 8.7 Implement fail-safe method to turn all relays OFF

## 9. VehicleController Integration
- [x] 9.1 Add SBusInput reference to VehicleController constructor
- [x] 9.2 Add RelayController reference to VehicleController constructor
- [x] 9.3 Add processSBusCommands() method to VehicleController
- [x] 9.4 Call processSBusCommands() in update() when input source is SBUS
- [x] 9.5 Map SBUS commands to actuator controls (steering, throttle, gear, brake)
- [x] 9.6 Map SBUS commands to relay controls (ignition state, front light)

## 10. Testing and Validation
- [ ] 10.1 Test UART initialization and SBUS frame reception
- [ ] 10.2 Verify channel value decoding and mapping accuracy
- [ ] 10.3 Test signal loss detection and fail-safe activation
- [ ] 10.4 Verify input source priority (SBUS > WEB > FAILSAFE)
- [ ] 10.5 Test gear selection mapping with all 3 positions (R/N/L)
- [ ] 10.6 Test steering and throttle deadband application
- [ ] 10.7 Validate signal quality monitoring and metrics
- [ ] 10.8 Test ignition state mapping with all 3 positions (OFF/ACC/IGNITION)
- [ ] 10.9 Test front light on/off toggle
- [ ] 10.10 Verify relay switching for ignition and lights
- [ ] 10.11 Test fail-safe behavior for relays (all OFF on signal loss)
