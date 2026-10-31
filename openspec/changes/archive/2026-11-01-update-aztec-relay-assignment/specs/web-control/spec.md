## MODIFIED Requirements
### Requirement: Ignition and Light Telemetry Broadcasting
The system SHALL broadcast current ignition state and front light status in telemetry updates.

#### Scenario: Include ignition state in WebSocket telemetry
- **WHEN** telemetry update is broadcast via WebSocket
- **THEN** telemetry includes ignition_state field with current state (OFF/ACC/IGNITION/CRANKING)
- **AND** telemetry includes is_cranking boolean field (true during starter motor operation)

#### Scenario: Include light state in WebSocket telemetry
- **WHEN** telemetry update is broadcast via WebSocket
- **THEN** telemetry includes front_light_on boolean field with current light state
- **AND** light state reflects actual relay output

#### Scenario: Update ignition display in telemetry section
- **WHEN** web page receives telemetry update
- **THEN** ignition state indicator updates to show current state
- **AND** cranking indicator shows animated/highlighted state during START operation
- **AND** front light indicator updates to show ON/OFF state

#### Scenario: Report the front-wheel lock as not fitted
- **WHEN** telemetry update is broadcast via WebSocket
- **THEN** telemetry includes `wheel_lock_fitted` boolean field, `false` on the aztec buggy
- **AND** `wheel_lock_on` SHALL be `false`

#### Scenario: Wheel-lock control disabled when not fitted
- **WHEN** web page receives telemetry with `wheel_lock_fitted` equal to `false`
- **THEN** the wheel-lock button SHALL be disabled and its status SHALL read "not fitted" (localized)
- **AND** clicking it SHALL NOT send a `set_wheel_lock` command
- **AND** telemetry without the `wheel_lock_fitted` field SHALL be treated as fitted

#### Scenario: Wheel-lock command rejected when not fitted
- **WHEN** a `set_wheel_lock` command is received via WebSocket and the lock is not fitted
- **THEN** an error response "Front-wheel lock not fitted on this vehicle" SHALL be sent
- **AND** no relay SHALL change state

## ADDED Requirements
### Requirement: Web Relay Test Control
The web portal SHALL provide a relay test for wiring checks: one on/off toggle per relay
(Relay_1..Relay_8), sent as the WebSocket command
`{"cmd":"set_relay_test","relay":<1..8>,"value":<true|false>}` and handled in
`VehicleController::processWebCommand()` through the queued web-command path into
`RelayController::setRelayTest()`. The command SHALL NOT be gated by input source; its gate is the
relay-test gate (ignition `OFF`, no function relay energized).

#### Scenario: Toggle a relay on from the web UI
- **WHEN** the operator switches a relay's toggle on while the ignition is OFF
- **THEN** a `set_relay_test` command with that relay number and `value: true` SHALL be sent
- **AND** the response SHALL be success ("Relay test: Relay_N ON") and the relay SHALL energize

#### Scenario: Toggle a relay off from the web UI
- **WHEN** the operator switches an energized test relay's toggle off
- **THEN** `set_relay_test` with `value: false` SHALL be sent and the relay SHALL release
- **AND** switching OFF SHALL remain possible whatever the ignition state

#### Scenario: Relay outside 1..8 is rejected
- **WHEN** a `set_relay_test` command carries a missing relay number or one outside 1..8
- **THEN** an error response "Relay test: Relay number must be 1-8" SHALL be sent
- **AND** no relay SHALL change state

#### Scenario: Relay test rejected while the ignition is on
- **WHEN** a `set_relay_test` ON command arrives while the ignition is not OFF or the front light is on
- **THEN** an error response starting with "Relay test:" SHALL be sent and no relay SHALL change

#### Scenario: Relay test card in the UI
- **WHEN** the web page is loaded
- **THEN** a "Relay Test" card in the calibration / diagnostics area SHALL show eight toggles labelled
  with the relay number and its function — 1 ignition, 2 starter (max 2 s), 3 light, 4–6 lock
  (unassigned), 7–8 spare — in English and Ukrainian
- **AND** each toggle's state SHALL follow the telemetry `relay_test` bitfield, not the last click
- **AND** a toggle that is off SHALL be disabled while the ignition is not OFF, the front light is on, the
  relay board is faulted or the WebSocket is disconnected; a toggle that is on SHALL stay clickable while
  connected so it can be switched off
- **AND** the response to a relay test command SHALL be shown in the card
