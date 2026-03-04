# S-bus Input Specification

## ADDED Requirements

### Requirement: S-bus Protocol Decoding
The system SHALL decode S-bus frames from ArduPilot Rover to receive vehicle control commands.

#### Scenario: Initialize S-bus receiver
- **WHEN** SBusInput.begin() is called with valid UART configuration
- **THEN** UART is configured for 100000 baud, 8E2 (8 data bits, even parity, 2 stop bits)
- **AND** RX pin is configured (inverted signal handling as needed)
- **AND** receiver enters listening mode

#### Scenario: Receive valid S-bus frame
- **WHEN** complete 25-byte S-bus frame is received
- **THEN** frame header (0x0F) and footer (0x00) are validated
- **AND** 16 channels are decoded from 11-bit packed format
- **AND** channel values are scaled from raw (172-1811) to standard (880-2160μs)
- **AND** frame timestamp is recorded

#### Scenario: Decode channel values
- **WHEN** getChannel(n) is called for valid channel (1-16)
- **THEN** most recent decoded value for that channel is returned in microseconds
- **AND** value is within valid range (880-2160μs)

#### Scenario: Check signal freshness
- **WHEN** isSignalValid() is called
- **THEN** true is returned if frame received within timeout period (default 500ms)
- **AND** false is returned if no recent frames
- **AND** signal age in milliseconds is available via getSignalAge()

### Requirement: Channel Mapping Configuration
The system SHALL provide configurable mapping of S-bus channels to vehicle control functions.

#### Scenario: Load default channel mapping
- **WHEN** system initializes
- **THEN** default channel mapping is loaded:
  - Channel 1 → Steering
  - Channel 2 → Throttle
  - Channel 3 → Transmission selector
  - Channel 4 → Brake control
- **AND** mapping can be customized via configuration

#### Scenario: Convert channel to vehicle command
- **WHEN** mapChannelToSteering(value) is called with S-bus value (880-2160μs)
- **THEN** value is converted to steering percentage (-100% to +100%)
- **AND** center deadband is applied (configurable, e.g., ±2%)
- **AND** resulting command is within valid range

#### Scenario: Convert channel to throttle command
- **WHEN** mapChannelToThrottle(value) is called with S-bus value
- **THEN** value is converted to throttle percentage (0% to 100%)
- **AND** idle deadband is applied at low end
- **AND** resulting command is within valid range

#### Scenario: Convert channel to gear selection
- **WHEN** mapChannelToGear(value) is called with S-bus value
- **THEN** value ranges are mapped to discrete gears:
  - 880-1200μs → REVERSE
  - 1201-1520μs → NEUTRAL
  - 1521-1840μs → HIGH
  - 1841-2160μs → LOW
- **AND** gear enum is returned

#### Scenario: Convert channel to brake command
- **WHEN** mapChannelToBrake(value) is called with S-bus value
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
