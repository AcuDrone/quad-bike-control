# sbus-input Specification

## Purpose
TBD - created by archiving change add-vehicle-control-system. Update Purpose after archive.
## Requirements
### Requirement: S-bus Protocol Decoding
The system SHALL decode S-bus frames from ArduPilot Rover to receive vehicle control commands using the SBusInput class wrapper around the SBUS library.

#### Scenario: Initialize S-bus receiver
- **WHEN** SBusInput.begin(rxPin, uartNum) is called with valid UART configuration
- **THEN** UART is configured for 100000 baud, 8E2 (8 data bits, even parity, 2 stop bits)
- **AND** RX pin is configured (GPIO 20 on UART1, inverted signal handling)
- **AND** Bolder Flight SBUS library is initialized with UART interface
- **AND** receiver enters listening mode
- **AND** true is returned if successful

#### Scenario: Receive valid S-bus frame
- **WHEN** update() is called and complete 25-byte S-bus frame is available
- **THEN** Bolder Flight library decodes frame header (0x0F) and footer (0x00)
- **AND** 16 channels are decoded from 11-bit packed format
- **AND** channel values are scaled from raw (172-1811) to standard (880-2160μs)
- **AND** frame timestamp is recorded
- **AND** totalFrames counter is incremented

#### Scenario: Decode channel values
- **WHEN** getChannel(n) is called for valid channel (1-16)
- **THEN** most recent decoded value for that channel is returned in microseconds
- **AND** value is within valid range (880-2160μs)
- **AND** value is clamped if library returns out-of-range value

#### Scenario: Check signal freshness
- **WHEN** isSignalValid() is called
- **THEN** true is returned if frame received within timeout period (SBUS_SIGNAL_TIMEOUT, default 500ms)
- **AND** false is returned if no recent frames
- **AND** signal age in milliseconds is available via getSignalAge()

### Requirement: Channel Mapping Configuration
The system SHALL provide configurable mapping of S-bus channels to vehicle control functions using compile-time constants defined in Constants.h.

#### Scenario: Load default channel mapping
- **WHEN** system initializes
- **THEN** channel mapping is loaded from SBusChannelConfig constants:
  - Channel 1 → Steering
  - Channel 2 → Throttle
  - Channel 3 → Transmission selector
  - Channel 4 → Brake control
  - Channel 5 → Ignition state (OFF/ACC/IGNITION)
  - Channel 6 → Front light (on/off)
- **AND** mapping is immutable at runtime (compile-time only)

#### Scenario: Convert channel to vehicle command
- **WHEN** mapChannelToSteering(value) or getSteering() is called with S-bus value (880-2160μs)
- **THEN** value is converted to steering percentage (-100% to +100%)
- **AND** center deadband is applied (configurable via SBUS_STEERING_DEADBAND constant, default ±2%)
- **AND** resulting command is within valid range

#### Scenario: Convert channel to throttle command
- **WHEN** mapChannelToThrottle(value) or getThrottle() is called with S-bus value
- **THEN** value is converted to throttle percentage (0% to 100%)
- **AND** idle deadband is applied at low end (configurable via SBUS_THROTTLE_DEADBAND constant)
- **AND** resulting command is within valid range

#### Scenario: Convert channel to gear selection
- **WHEN** mapChannelToGear(value) or getGear() is called with S-bus value
- **THEN** value ranges are mapped to discrete gears using range constants:
  - SBUS_GEAR_REVERSE_MIN to SBUS_GEAR_REVERSE_MAX → REVERSE
  - SBUS_GEAR_NEUTRAL_MIN to SBUS_GEAR_NEUTRAL_MAX → NEUTRAL
  - SBUS_GEAR_LOW_MIN to SBUS_GEAR_LOW_MAX → LOW
- **AND** gear enum is returned
- **AND** default gear is NEUTRAL if value is out of all ranges

#### Scenario: Convert channel to brake command
- **WHEN** mapChannelToBrake(value) or getBrake() is called with S-bus value
- **THEN** value is converted to brake percentage (0% to 100%)
- **AND** resulting command is within valid range

### Requirement: Signal Loss Detection and Fail-Safe
The system SHALL detect S-bus signal loss and activate fail-safe mode.

#### Scenario: Detect signal timeout
- **WHEN** no valid S-bus frame received for timeout period (default 500ms)
- **THEN** signal loss flag is set
- **AND** fail-safe mode is activated
- **AND** signal loss callback is triggered (if registered)

#### Scenario: Enter fail-safe mode on signal loss
- **WHEN** signal loss is detected
- **THEN** fail-safe commands are issued:
  - Steering → center (0%)
  - Throttle → idle (0%)
  - Transmission → NEUTRAL
  - Brakes → applied (30% parking brake)
- **AND** system remains in fail-safe until signal recovers

#### Scenario: Recover from signal loss
- **WHEN** valid S-bus frames resume after signal loss
- **THEN** fail-safe mode is cleared after N consecutive good frames (default 3)
- **AND** normal control is restored
- **AND** signal recovery callback is triggered (if registered)

#### Scenario: Fail-safe frame detection
- **WHEN** S-bus frame with fail-safe flag set is received
- **THEN** frame is marked as fail-safe
- **AND** fail-safe mode is activated same as signal timeout
- **AND** channel data is ignored

### Requirement: Signal Quality Monitoring
The system SHALL monitor S-bus signal quality and report statistics.

#### Scenario: Track frame statistics
- **WHEN** S-bus frames are being received
- **THEN** system tracks:
  - Total frames received
  - CRC/validation errors
  - Frame loss count
  - Average frame rate (Hz)
  - Signal age histogram

#### Scenario: Query signal quality
- **WHEN** getSignalQuality() is called
- **THEN** signal quality metrics are returned including:
  - Frame success rate (%)
  - Current frame rate (Hz)
  - Time since last frame (ms)
  - Consecutive error count

#### Scenario: Detect signal degradation
- **WHEN** error rate exceeds threshold (e.g., >10% failed frames)
- **THEN** signal quality warning is logged
- **AND** quality warning callback is triggered (if registered)
- **AND** system continues operating but monitors for full signal loss

### Requirement: Input Validation and Sanitization
The system SHALL validate and sanitize all S-bus input data before use.

#### Scenario: Validate channel value range
- **WHEN** channel value is decoded from S-bus frame
- **THEN** value is checked against valid range (880-2160μs)
- **AND** out-of-range values are clamped to limits
- **AND** validation error is logged

#### Scenario: Detect and reject corrupted frames
- **WHEN** S-bus frame is received with invalid header or footer
- **THEN** frame is rejected and not processed
- **AND** error counter is incremented
- **AND** system waits for next valid frame

#### Scenario: Apply rate limiting to commands
- **WHEN** channel values change rapidly
- **THEN** rate limiting is applied to prevent excessive actuator movement
- **AND** maximum change per update cycle is enforced (configurable)
- **AND** smooth transitions are maintained

### Requirement: Diagnostic and Debug Support
The system SHALL provide diagnostic information for S-bus signal debugging.

#### Scenario: Enable raw data logging
- **WHEN** debug mode is enabled
- **THEN** raw S-bus frames are logged to serial output
- **AND** decoded channel values are displayed
- **AND** frame timing and errors are reported

#### Scenario: Display channel monitor
- **WHEN** displayChannelMonitor() is called
- **THEN** real-time display of all 16 channels is shown
- **AND** signal quality indicators are displayed
- **AND** fail-safe status is indicated

#### Scenario: Test without S-bus hardware
- **WHEN** simulation mode is enabled
- **THEN** synthetic S-bus frames are generated internally
- **AND** system behavior can be tested without ArduPilot
- **AND** simulation parameters are configurable (frame rate, channel values)

### Requirement: Input Source Priority Integration
The system SHALL support prioritized input source selection between S-bus and alternative control sources.

#### Scenario: Report S-bus priority status
- **WHEN** getInputPriority() is called on SBusInput
- **THEN** current priority level is returned (PRIMARY, SECONDARY, INACTIVE)
- **AND** S-bus has PRIMARY priority when signal is valid
- **AND** S-bus has INACTIVE priority when signal lost or timed out

#### Scenario: Query if S-bus should control vehicle
- **WHEN** shouldControlVehicle() is called
- **THEN** true is returned if S-bus signal is valid (within 500ms timeout)
- **AND** false is returned if S-bus signal is lost or timed out

#### Scenario: Integrate with input source arbiter
- **WHEN** external input source arbiter queries S-bus status
- **THEN** S-bus provides signal validity status
- **AND** S-bus provides time since last valid frame
- **AND** arbiter can determine if S-bus should have control priority

### Requirement: SBusInput Class Interface
The system SHALL provide a SBusInput class that wraps the SBUS library and provides vehicle-specific command APIs.

#### Scenario: Instantiate and initialize SBusInput
- **WHEN** SBusInput object is created and begin() is called
- **THEN** UART is configured for SBUS protocol (100000 baud, 8E2, inverted signal)
- **AND** internal SBUS library object is initialized
- **AND** signal tracking variables are reset to initial state
- **AND** true is returned if initialization succeeds

#### Scenario: Provide dual API for raw and mapped values
- **WHEN** application code needs channel data
- **THEN** getChannel(n) provides raw microseconds value (880-2160μs)
- **AND** getSteering() provides mapped percentage (-100 to +100)
- **AND** getThrottle() provides mapped percentage (0 to 100)
- **AND** getGear() provides gear enum (REVERSE/NEUTRAL/LOW)
- **AND** getBrake() provides mapped percentage (0 to 100)
- **AND** getIgnitionState() provides ignition enum (OFF/ACC/IGNITION)
- **AND** getFrontLight() provides boolean (true=ON, false=OFF)

#### Scenario: Update frame processing in main loop
- **WHEN** update() is called in main loop
- **THEN** SBUS library reads available frames from UART buffer
- **AND** channel data is decoded and stored
- **AND** frame statistics are updated (total frames, errors)
- **AND** signal validity is checked based on timeout

### Requirement: Channel Configuration Structure
The system SHALL define channel mapping configuration as compile-time constants in Constants.h.

#### Scenario: Define channel assignments
- **WHEN** code references channel assignments
- **THEN** SBusChannelConfig struct provides named constants:
  - STEERING = 1
  - THROTTLE = 2
  - TRANSMISSION = 3
  - BRAKE = 4
  - IGNITION = 5
  - FRONT_LIGHT = 6
- **AND** constants are used throughout codebase instead of magic numbers

#### Scenario: Define mapping parameters
- **WHEN** channel values are mapped to vehicle commands
- **THEN** deadband constants are defined (STEERING_DEADBAND, THROTTLE_DEADBAND)
- **AND** gear selection range constants are defined (min/max for each gear)
- **AND** ignition state range constants are defined (OFF: 880-1200μs, ACC: 1201-1520μs, IGNITION: 1521-2160μs)
- **AND** front light threshold constant is defined (1520μs)
- **AND** constants are documented with units and purpose

### Requirement: Mapped Vehicle Command Methods
The system SHALL convert raw SBUS channel values to vehicle-specific commands with appropriate scaling and deadzones.

#### Scenario: Map steering channel to percentage
- **WHEN** getSteering() is called
- **THEN** STEERING channel value is read (880-2160μs)
- **AND** value is mapped to -100 to +100 percentage
- **AND** center deadband (±2%) is applied around 1520μs center point
- **AND** resulting percentage is clamped to valid range

#### Scenario: Map throttle channel to percentage
- **WHEN** getThrottle() is called
- **THEN** THROTTLE channel value is read (880-2160μs)
- **AND** value is mapped to 0 to 100 percentage
- **AND** idle deadband (2%) is applied at low end
- **AND** resulting percentage is clamped to valid range

#### Scenario: Map transmission channel to gear enum
- **WHEN** getGear() is called
- **THEN** TRANSMISSION channel value is read (880-2160μs)
- **AND** value is compared against gear range constants:
  - 880-1200μs → Gear::REVERSE
  - 1201-1520μs → Gear::NEUTRAL
  - 1521-2160μs → Gear::LOW
- **AND** corresponding gear enum is returned
- **AND** default is NEUTRAL if value is invalid

#### Scenario: Map brake channel to percentage
- **WHEN** getBrake() is called
- **THEN** BRAKE channel value is read (880-2160μs)
- **AND** value is mapped to 0 to 100 percentage linearly
- **AND** resulting percentage is clamped to valid range

#### Scenario: Map ignition channel to state enum
- **WHEN** getIgnitionState() is called
- **THEN** IGNITION channel value is read (880-2160μs)
- **AND** value is compared against ignition range constants:
  - 880-1200μs → IgnitionState::OFF
  - 1201-1520μs → IgnitionState::ACC (accessory power)
  - 1521-2160μs → IgnitionState::IGNITION (full ignition)
- **AND** corresponding ignition state enum is returned
- **AND** default is OFF if value is invalid

#### Scenario: Map front light channel to boolean
- **WHEN** getFrontLight() is called
- **THEN** FRONT_LIGHT channel value is read (880-2160μs)
- **AND** value is compared against threshold (1520μs)
- **AND** true is returned if value > 1520μs (ON)
- **AND** false is returned if value <= 1520μs (OFF)

### Requirement: Relay Controller for Ignition and Lights
The system SHALL provide a RelayController class to manage relay outputs for ignition states and front light control.

#### Scenario: Initialize relay controller
- **WHEN** RelayController.begin() is called with relay pin assignments
- **THEN** GPIO pins for RELAY1, RELAY2, RELAY3 are configured as outputs
- **AND** all relays are initialized to LOW (OFF) state
- **AND** internal state tracking is initialized
- **AND** true is returned if initialization succeeds

#### Scenario: Set ignition state to OFF
- **WHEN** setIgnitionState(IgnitionState::OFF) is called
- **THEN** RELAY1 is set to LOW (ignition off)
- **AND** RELAY2 is set to LOW (accessory off)
- **AND** internal state is updated to OFF

#### Scenario: Set ignition state to ACC
- **WHEN** setIgnitionState(IgnitionState::ACC) is called
- **THEN** RELAY1 is set to LOW (ignition off)
- **AND** RELAY2 is set to HIGH (accessory on)
- **AND** internal state is updated to ACC

#### Scenario: Set ignition state to IGNITION
- **WHEN** setIgnitionState(IgnitionState::IGNITION) is called
- **THEN** RELAY1 is set to HIGH (ignition on)
- **AND** RELAY2 is set to HIGH (accessory on)
- **AND** internal state is updated to IGNITION

#### Scenario: Control front light
- **WHEN** setFrontLight(true) is called
- **THEN** RELAY3 is set to HIGH (light on)
- **AND** internal state is updated
- **WHEN** setFrontLight(false) is called
- **THEN** RELAY3 is set to LOW (light off)

#### Scenario: Fail-safe all relays off
- **WHEN** allOff() is called (fail-safe trigger)
- **THEN** all relays (RELAY1, RELAY2, RELAY3) are set to LOW
- **AND** internal state is reset to safe defaults (OFF, lights off)

### Requirement: Signal Quality Metrics Structure
The system SHALL provide structured signal quality information through a SignalQuality struct.

#### Scenario: Query comprehensive signal quality
- **WHEN** getSignalQuality() is called
- **THEN** SignalQuality struct is returned containing:
  - totalFrames (count since initialization)
  - errorFrames (validation failures)
  - frameRate (Hz, rolling average)
  - errorRate (percentage, 0-100)
  - signalAge (milliseconds since last valid frame)
  - isValid (boolean, within timeout threshold)

#### Scenario: Calculate frame rate
- **WHEN** signal quality metrics are updated
- **THEN** frame rate is calculated from time between frames
- **AND** rolling average is maintained for stability
- **AND** rate is reported in Hz (typical range 70-140Hz for SBUS)

