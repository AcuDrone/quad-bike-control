## MODIFIED Requirements

### Requirement: Relay Controller for Ignition and Lights
The system SHALL provide a `RelayController` class to manage relay outputs for ignition
states, front light and front-wheel lock control, driven by commands decoded from the MAVLink
interface or the web portal. The relays SHALL be driven through the external relay board's PCA9557
expander on `Wire1` at address `0x18` rather than through direct ESP32 GPIO, with the public API
unchanged for callers.

Relay mapping: `Relay_1` = ignition/ECU line, `Relay_2` = starter, `Relay_3` = front light,
`Relay_4` = front-wheel lock, `Relay_5-8` = spare (always off).

#### Scenario: Initialize relay controller
- **WHEN** `RelayController.begin()` is called with the `Wire1` I2C bus
- **THEN** the PCA9557 at `0x18` SHALL be detected and all eight pins configured as outputs
- **AND** the output port SHALL be written to `0x00` (all relays OFF) and verified by read-back
- **AND** internal state tracking is initialized to OFF / light off / lock off
- **AND** true is returned if initialization succeeds, false if the expander does not answer or the
  all-off write does not verify

#### Scenario: Set ignition state
- **WHEN** `setIgnitionState(OFF)` is called, `Relay_1` and `Relay_2` are set LOW
- **WHEN** `setIgnitionState(ACC)` is called, `Relay_1` is HIGH and `Relay_2` is LOW
- **WHEN** `setIgnitionState(IGNITION)` is called, `Relay_1` is HIGH and `Relay_2` is LOW
- **WHEN** `setIgnitionState(CRANKING)` is called, `Relay_1` and `Relay_2` are both HIGH
- **AND** internal state is updated to match in each case
- **AND** each change is applied as a single whole-port write from the output shadow byte

#### Scenario: Control front light
- **WHEN** `setFrontLight(true)` is called, `Relay_3` is set HIGH and state is updated
- **WHEN** `setFrontLight(false)` is called, `Relay_3` is set LOW and state is updated

#### Scenario: Control front-wheel lock
- **WHEN** `setWheelLock(true)` is called, `Relay_4` is set HIGH and state is updated
- **WHEN** `setWheelLock(false)` is called, `Relay_4` is set LOW and state is updated

#### Scenario: Fail-safe all relays off
- **WHEN** `allOff()` is called (fail-safe trigger)
- **THEN** the ignition and front-light relays are set LOW
- **AND** internal state is reset to safe defaults (OFF, lights off)
- **AND** the front-wheel lock relay is intentionally left in its last state
- **AND** the fail-safe write is verified by read-back like any other relay write

#### Scenario: Relay commands are no-ops when the expander is unavailable
- **WHEN** the expander failed to initialize or the driver is faulted
- **AND** a MAVLink- or web-decoded relay command arrives
- **THEN** the command SHALL be logged and ignored rather than silently assumed applied
- **AND** the reported ignition state SHALL be `OFF`
