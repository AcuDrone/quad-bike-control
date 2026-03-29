## MODIFIED Requirements
### Requirement: Channel Mapping Configuration
The system SHALL provide configurable mapping of S-bus channels to vehicle control functions using compile-time constants defined in Constants.h.

#### Scenario: Load default channel mapping
- **WHEN** system initializes
- **THEN** channel mapping is loaded from SBusChannelConfig constants:
  - Channel 1 → Steering
  - Channel 2 → Throttle/Brake (combined: above center = throttle, below center = brake)
  - Channel 3 → Transmission selector
  - Channel 4 → Ignition state (OFF/ACC/IGNITION)
  - Channel 5 → Front light (on/off)
- **AND** mapping is immutable at runtime (compile-time only)

#### Scenario: Convert channel to vehicle command
- **WHEN** mapChannelToSteering(value) or getSteering() is called with S-bus value (880-2160μs)
- **THEN** value is converted to steering percentage (-100% to +100%)
- **AND** center deadband is applied (configurable via SBUS_STEERING_DEADBAND constant, default ±2%)
- **AND** resulting command is within valid range

#### Scenario: Convert channel to throttle command
- **WHEN** getThrottle() is called
- **THEN** combined throttle/brake channel (CH2) value is read
- **AND** if value is above center (1500μs), it is mapped to throttle percentage (0% to 100%) where 1500μs=0% and 2000μs=100%
- **AND** if value is at or below center, throttle returns 0%
- **AND** throttle deadband is applied near center

#### Scenario: Convert channel to brake command
- **WHEN** getBrake() is called
- **THEN** combined throttle/brake channel (CH2) value is read
- **AND** if value is below center (1500μs), it is mapped to brake percentage (0% to 100%) where 1500μs=0% and 1000μs=100%
- **AND** if value is at or above center, brake returns 0%
- **AND** brake deadband is applied near center

#### Scenario: Convert channel to gear selection
- **WHEN** mapChannelToGear(value) or getGear() is called with S-bus value
- **THEN** value ranges are mapped to discrete gears using range constants:
  - SBUS_GEAR_REVERSE_MIN to SBUS_GEAR_REVERSE_MAX → REVERSE
  - SBUS_GEAR_NEUTRAL_MIN to SBUS_GEAR_NEUTRAL_MAX → NEUTRAL
  - SBUS_GEAR_LOW_MIN to SBUS_GEAR_LOW_MAX → LOW
- **AND** gear enum is returned
- **AND** default gear is NEUTRAL if value is out of all ranges

### Requirement: Mapped Vehicle Command Methods
The system SHALL provide methods that read S-bus channels and return typed vehicle commands.

#### Scenario: Map steering channel to percentage
- **WHEN** getSteering() is called
- **THEN** channel value from SBusChannelConfig::STEERING is read
- **AND** value is converted using bidirectional mapping (center = 0%, min = -100%, max = +100%)
- **AND** deadband of SBUS_STEERING_DEADBAND is applied around center

#### Scenario: Map throttle from combined channel
- **WHEN** getThrottle() is called
- **THEN** channel value from SBusChannelConfig::THROTTLE is read
- **AND** only the upper half (above SBUS_US_CENTER) is mapped to 0–100%
- **AND** values at or below center return 0%
- **AND** deadband of SBUS_THROTTLE_DEADBAND is applied near 0%

#### Scenario: Map brake from combined channel
- **WHEN** getBrake() is called
- **THEN** channel value from SBusChannelConfig::THROTTLE (same channel) is read
- **AND** only the lower half (below SBUS_US_CENTER) is mapped to 0–100%
- **AND** values at or above center return 0%

#### Scenario: Map transmission channel to gear enum
- **WHEN** getGear() is called
- **THEN** channel value from SBusChannelConfig::TRANSMISSION is read
- **AND** value is mapped to TransmissionController::Gear enum based on defined ranges

#### Scenario: Map ignition channel to state enum
- **WHEN** getIgnitionState() is called
- **THEN** channel value from SBusChannelConfig::IGNITION is read
- **AND** value is mapped to IgnitionState enum (OFF/ACC/IGNITION) based on defined ranges

#### Scenario: Map front light channel to boolean
- **WHEN** getFrontLight() is called
- **THEN** channel value from SBusChannelConfig::FRONT_LIGHT is read
- **AND** value above SBUS_FRONT_LIGHT_THRESHOLD returns true (ON)
- **AND** value at or below threshold returns false (OFF)

### Requirement: Channel Configuration Structure
The system SHALL define channel-to-function mapping via a compile-time SBusChannelConfig structure in Constants.h.

#### Scenario: Define channel assignments
- **WHEN** SBusChannelConfig is defined
- **THEN** the following constants are available:
  - STEERING = 1 (steering servo)
  - THROTTLE = 2 (combined throttle/brake)
  - TRANSMISSION = 3 (gear selector)
  - IGNITION = 4 (ignition state)
  - FRONT_LIGHT = 5 (front light toggle)
- **AND** BRAKE channel constant is removed (brake reads from THROTTLE channel)

#### Scenario: Define mapping parameters
- **WHEN** mapping parameters are defined
- **THEN** the following constants are available:
  - SBUS_US_MIN = 1000 (minimum microseconds)
  - SBUS_US_MAX = 2000 (maximum microseconds)
  - SBUS_US_CENTER = 1500 (center point — split between throttle and brake)
  - SBUS_STEERING_DEADBAND = 2.0 (percent)
  - SBUS_THROTTLE_DEADBAND = 2.0 (percent)
