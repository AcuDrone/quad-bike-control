## MODIFIED Requirements
### Requirement: Gear Position Default Configuration via Web Interface
The system SHALL allow the user to view and update the default transmission servo position
(0.0–100.0 %) for each gear through the web portal, without requiring a firmware recompile.

#### Scenario: Set a gear's default position via WebSocket
- **WHEN** a `set_gear_default` WebSocket command is received with a valid gear string (`"R"`, `"N"`, `"L"`, `"H"`) and a position value
- **THEN** the system SHALL call `setDefaultPosition()` on the transmission controller
- **AND** SHALL respond with success naming the gear and the stored position
- **AND** this command SHALL be accepted regardless of the active input source (same privilege level as calibration commands)

#### Scenario: Reject invalid gear or out-of-range position
- **WHEN** a `set_gear_default` command is received with an unrecognised gear string or a position outside 0.0–100.0 %
- **THEN** the system SHALL respond with `{"ok": false, "error": "<reason>"}` and leave NVS unchanged

#### Scenario: Display gear position defaults editor in web UI
- **WHEN** the Calibration section is shown
- **THEN** a "Gear position defaults" card SHALL show one "Положення серво" / "Servo position"
  slider (0–100 %, 0.1 % steps) that follows `hall_position` from telemetry while it is not being
  dragged
- **AND** four large buttons "Зберегти як R/N/H/L" / "Save as R/N/H/L" in a 2×2 grid (order R, N,
  H, L), each with the saved position from telemetry on a second line (`--` when unknown)
- **AND** a compact row of "▶ R", "▶ N", "▶ H", "▶ L" buttons
- **AND** the boost RPM target input with its Save button, and beside it the "Перегазовка при
  перемиканні" / "Gear-change RPM boost" on/off toggle
- **AND** there SHALL be no per-gear number inputs, no 📍 capture button and no per-row Move/Save
  buttons

#### Scenario: Start and finish positioning
- **WHEN** the engineer presses "Почати" / "Start"
- **THEN** the slider and the ▶ row SHALL become enabled and "Готово" / "Done" SHALL replace "Почати"
- **WHEN** the engineer presses "Готово"
- **THEN** the slider and the ▶ row SHALL be disabled again
- **AND** this active state SHALL exist only in the page; no WebSocket command SHALL be sent for
  Почати or Готово, and a reloaded page SHALL start locked
- **WHEN** the WebSocket disconnects while the card is active
- **THEN** the slider and the ▶ row SHALL be disabled and "Почати" shown again, and reconnecting
  SHALL NOT unlock them by itself

#### Scenario: Drag the slider to move the servo
- **WHEN** the slider is enabled and the engineer drags it
- **THEN** the existing `move_to_position` command SHALL be sent with the slider value
- **AND** commands SHALL be rate-limited to at most one per 100 ms, and the final value SHALL
  always be sent when the drag ends

#### Scenario: Save a default from the web UI
- **WHEN** the engineer presses "Зберегти як <gear>" (enabled whether or not "Почати" was pressed,
  because saving does not move the servo)
- **THEN** the web UI SHALL send a `set_gear_default` WebSocket command with that gear and the
  current slider value
- **AND** on the next telemetry the button's second line SHALL show the saved value

#### Scenario: Highlight the gear at the current position
- **WHEN** the current slider value is within ±0.3 % of a gear's saved position
- **THEN** that gear's "Зберегти як" button SHALL be highlighted with the active-gear style
- **AND** the other buttons SHALL not be highlighted

#### Scenario: Move to a saved position
- **WHEN** the card is active and the engineer presses "▶ <gear>"
- **THEN** `move_to_position` SHALL be sent with that gear's saved position

#### Scenario: Show live encoder count near gear controls
- **WHEN** the Manual Control card is displayed
- **THEN** the current transmission servo position in percent (from `hall_position` telemetry; formerly an encoder count) SHALL be visible adjacent to the gear buttons
- **AND** it SHALL update in real time

## ADDED Requirements
### Requirement: Gear-Change Boost Enable via Web Interface
The web portal SHALL let the operator switch the gear-change RPM boost on or off per vehicle
without a reflash, from a toggle in the "Gear Position Defaults" card next to the "RPM Boost
Target" row, using the `set_boost_enabled` WebSocket command.

#### Scenario: Switch the boost off or on
- **WHEN** the operator toggles "Перегазовка при перемиканні" / "Gear-change RPM boost"
- **THEN** a `set_boost_enabled` WebSocket command carrying a boolean value SHALL be sent
- **AND** the system SHALL persist the value to NVS `boost/enabled` first and apply it only after
  the write succeeded
- **AND** it SHALL respond with an English acknowledgement naming the new state ("Gear boost
  enabled" / "Gear boost disabled"), shown in the card's status line
- **AND** the command SHALL be accepted regardless of the active input source, like `set_boost_rpm`

#### Scenario: Persisting fails
- **WHEN** a `set_boost_enabled` command is received and the NVS write fails
- **THEN** the system SHALL respond with an English error and the switch state in use SHALL be
  left unchanged

#### Scenario: The toggle follows the stored value
- **WHEN** the portal renders a telemetry frame carrying `boost_enabled`
- **THEN** the toggle SHALL show that state rather than the last click

#### Scenario: Target RPM stays editable while disabled
- **WHEN** the boost is switched off
- **THEN** the "RPM Boost Target" input and its Save button SHALL remain enabled

#### Scenario: Labels and help
- **WHEN** the card is shown
- **THEN** the card help (ⓘ) SHALL include one sentence explaining that the switch turns the
  RPM blip on gear changes on or off, that with it off the throttle is still held at idle during
  the shift, and that the manual boost test is not affected
- **AND** the toggle label and help sentence SHALL be present in both the English and the
  Ukrainian dictionary
