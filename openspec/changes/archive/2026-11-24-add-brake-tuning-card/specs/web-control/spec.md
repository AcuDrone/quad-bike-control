## ADDED Requirements
### Requirement: Brake Tuning Card
The web portal SHALL provide a card "🛑 Гальмо" / "🛑 Brake", placed right after the "Throttle
Calibration" card, to set the brake full travel time and the hold drive and to show the released
endstop state. The commands SHALL be routed through `WebPortal::WebCommand` →
`VehicleController::processWebCommand`, SHALL be accepted regardless of the active input source,
and SHALL persist before applying.

#### Scenario: Set the travel time
- **WHEN** the operator drags the travel time slider (range input 1000–3000 ms, step 50, default
  1500) and releases it
- **THEN** the value label next to the slider SHALL show "N мс" / "N ms" live while dragging, and
  no command SHALL be sent before the release
- **AND** on release (`change`) one `set_brake_travel_ms` command carrying the slider value SHALL be
  sent
- **AND** the firmware SHALL round it to a whole ms, validate 1000–3000, persist, apply, and
  respond "Brake travel time set to N ms"
- **AND** an out-of-range value SHALL be answered "Brake travel time out of range (1000-3000 ms)"

#### Scenario: Set the hold drive
- **WHEN** the operator enters a hold drive and saves it
- **THEN** a `set_brake_hold_pwm` command carrying that value SHALL be sent
- **AND** the firmware SHALL round it to a whole number, validate 0–100, persist, apply, and
  respond "Brake hold set to N", or "Brake hold off" for 0
- **AND** an out-of-range value SHALL be answered "Brake hold out of range (0-100)"

#### Scenario: Save failure
- **WHEN** the NVS write fails on either command
- **THEN** the firmware SHALL respond "Failed to save the brake setting" and keep the value in use
  unchanged

#### Scenario: Card fills from telemetry and validates locally
- **WHEN** telemetry arrives and the slider is not being dragged / the hold input is not being
  edited
- **THEN** the travel slider and its value label SHALL show `brake_travel_ms` and the hold input
  `brake_hold_pwm`
- **AND** the travel time SHALL be a slider with bounds 1000–3000 and step 50 and a value label
  "N мс" / "N ms", the hold drive a number input with bounds 0–100 and step 5, with the hint
  "0 = без утримання" / "0 = no hold push"
- **AND** out-of-range values SHALL be refused locally before sending, with a translated message
- **AND** replies starting with "Brake travel", "Brake hold" or "Failed to save the brake" SHALL be
  shown in the card's status line

#### Scenario: Endstop status line
- **WHEN** telemetry reports `brake_released`
- **THEN** the card SHALL show "Кінцевик: відпущено" / "Endstop: released" when true and
  "Кінцевик: ні" / "Endstop: no" when false

#### Scenario: Help and translations
- **WHEN** the operator opens the card help (ⓘ)
- **THEN** it SHALL explain that the travel time should be set so that the panel's brake
  percentage reaches 100 % just as the actuator reaches its end, and that the hold drive is the
  push that keeps a held brake from creeping back (0 = none, higher values heat the actuator)
- **AND** every new label, hint, message and help text SHALL be present in both the English and
  the Ukrainian dictionary
