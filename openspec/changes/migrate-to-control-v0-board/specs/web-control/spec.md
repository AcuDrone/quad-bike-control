## ADDED Requirements

### Requirement: 24V Boost Rail Control via Web Interface
The web portal SHALL provide a guarded control for the 24V boost rail, following the same command
pattern as the existing front light and wheel lock controls, so the rail can be power-cycled on the
bench without removing steering authority from a live vehicle.

#### Scenario: Send a boost command
- **WHEN** the operator toggles the 24V boost control in the web UI
- **THEN** a `set_boost` command with a boolean value SHALL be sent over the WebSocket
- **AND** it SHALL be parsed into `WebPortal::WebCommand` and dispatched by
  `VehicleController::processWebCommand` in the same manner as `set_light` and `set_wheel_lock`
- **AND** it SHALL be accepted regardless of the active input source (MAVLINK / WEB / FAILSAFE)

#### Scenario: Enable the boost rail
- **WHEN** a `set_boost` command with value `true` is received
- **THEN** `PIN_BOOST_EN` SHALL be driven HIGH
- **AND** the startup grace period for the low-rail warning SHALL be restarted
- **AND** a success response SHALL be sent to the client
- **AND** enabling SHALL always be permitted

#### Scenario: Refuse to disable the boost while ignition is not OFF
- **WHEN** a `set_boost` command with value `false` is received
- **AND** the reported ignition state is not `OFF`
- **THEN** the command SHALL be rejected and `PIN_BOOST_EN` SHALL remain HIGH
- **AND** an error response SHALL be sent to the client explaining that the rail powers the steering
  driver and that ignition must be OFF first
- **AND** the refusal SHALL be logged

#### Scenario: Disable the boost when ignition is OFF
- **WHEN** a `set_boost` command with value `false` is received
- **AND** the reported ignition state is `OFF`
- **THEN** `PIN_BOOST_EN` SHALL be driven LOW
- **AND** the low-rail warning SHALL be suppressed while the boost is commanded off
- **AND** a success response SHALL be sent to the client

#### Scenario: Reflect boost state in the UI
- **WHEN** telemetry reporting `boost_on` is received
- **THEN** the boost control SHALL reflect the actual commanded state rather than the last click
- **AND** its label SHALL have i18n keys present in both the `en` and `uk` dictionaries
