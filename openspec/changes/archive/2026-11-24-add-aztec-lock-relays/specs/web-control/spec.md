## MODIFIED Requirements
### Requirement: Ignition and Light Telemetry Broadcasting
The system SHALL broadcast current ignition state, front light status and lock status in telemetry
updates.

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
- **THEN** telemetry includes `wheel_lock_fitted` boolean field, `true` on the aztec buggy (the lock
  on Relay_4 + Relay_5 + Relay_6) and `false` only on a build with `RELAY_WHEEL_LOCK_FITTED = 0`
- **AND** `wheel_lock_on` SHALL report the commanded lock state (`true` while Relay_4..Relay_6 are
  energized); it SHALL be `false` whenever the lock is not fitted

#### Scenario: Wheel-lock control disabled when not fitted
- **WHEN** web page receives telemetry with `wheel_lock_fitted` equal to `false`
- **THEN** the lock button SHALL be disabled and its status SHALL read "not fitted" (localized)
- **AND** clicking it SHALL NOT send a `set_wheel_lock` command
- **AND** telemetry without the `wheel_lock_fitted` field SHALL be treated as fitted

#### Scenario: Lock control enabled on the aztec buggy
- **WHEN** web page receives telemetry with `wheel_lock_fitted` equal to `true`
- **THEN** the lock card ("Lock" / "Блокування") SHALL offer an enabled toggle whose status follows
  `wheel_lock_on` (LOCKED / UNLOCKED, localized)
- **AND** clicking it SHALL send `set_wheel_lock` with the inverted state
- **AND** a `set_wheel_lock` command SHALL be answered "Lock LOCKED" / "Lock UNLOCKED" on success

#### Scenario: Wheel-lock command rejected when not fitted
- **WHEN** a `set_wheel_lock` command is received via WebSocket and the lock is not fitted
- **THEN** an error response "Front-wheel lock not fitted on this vehicle" SHALL be sent
- **AND** no relay SHALL change state

### Requirement: Web Relay Test Control
The web portal SHALL provide a relay test for wiring checks: one on/off button per relay
(Relay_1..Relay_8), sent as the WebSocket command
`{"cmd":"set_relay_test","relay":<1..8>,"value":<true|false>}` and handled in
`VehicleController::processWebCommand()` through the queued web-command path into
`RelayController::setRelayTest()`. The command SHALL NOT be gated by input source; its gate is the
relay-test gate (ignition `OFF`, no function relay energized — the lock included).

#### Scenario: Toggle a relay on from the web UI
- **WHEN** the operator clicks a relay's button while it is off, the ignition is OFF and the lock is
  released
- **THEN** a `set_relay_test` command with that relay number and `value: true` SHALL be sent
- **AND** the response SHALL be success ("Relay test: Relay_N ON") and the relay SHALL energize

#### Scenario: Toggle a relay off from the web UI
- **WHEN** the operator clicks an energized test relay's button
- **THEN** `set_relay_test` with `value: false` SHALL be sent and the relay SHALL release
- **AND** switching OFF SHALL remain possible whatever the ignition state

#### Scenario: Relay outside 1..8 is rejected
- **WHEN** a `set_relay_test` command carries a missing relay number or one outside 1..8
- **THEN** an error response "Relay test: Relay number must be 1-8" SHALL be sent
- **AND** no relay SHALL change state

#### Scenario: Relay test rejected while the ignition is on
- **WHEN** a `set_relay_test` ON command arrives while the ignition is not OFF, the front light is on
  or the lock is engaged
- **THEN** an error response starting with "Relay test:" SHALL be sent and no relay SHALL change

#### Scenario: Relay test card in the UI
- **WHEN** the Tests section is shown
- **THEN** a "Relay Test" card SHALL show eight equal-size buttons in a 2-column grid (1 column at
  768 px or less), styled like the gear buttons
- **AND** each button SHALL show "Реле N" / "Relay N" on its first line and the relay's function on
  a smaller second line — on the aztec buggy 1 ignition, 2 starter (max 2 s), 3 light, 4 lock (with
  5+6), 5 lock (with 4+6), 6 lock (with 4+5), 7–8 spare — in English and Ukrainian ("блокування (з
  5+6)" …)
- **AND** the aztec label set SHALL be chosen when the telemetry carries the aztec-only `gear_codes`
  object, and the quad bike's set otherwise (`wheel_lock_fitted` is `true` on both vehicles and no
  longer tells them apart)
- **AND** an energized relay SHALL be shown with the same highlight as the active gear button and
  `aria-pressed="true"`; the button SHALL NOT carry ON/OFF text, and its tooltip SHALL name the
  relay, its function and its ON/OFF state
- **AND** each button's state SHALL follow the telemetry `relay_test` bitfield, not the last click
- **AND** a button that is off SHALL be disabled while the ignition is not OFF, the front light is
  on, the lock is engaged, the relay board is faulted or the WebSocket is disconnected; a button that
  is on SHALL stay clickable while connected so it can be switched off
- **AND** the response to a relay test command SHALL be shown in the card
