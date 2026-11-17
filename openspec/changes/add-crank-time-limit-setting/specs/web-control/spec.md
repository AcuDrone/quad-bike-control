## ADDED Requirements
### Requirement: Starter Time Limit Configuration via Web Interface
The web portal SHALL allow the operator to set the starter time limit without a reflash, from a
slider "Макс. час роботи стартера" / "Max starter time" (2–10 s, step 0.5 s) in the "🚀 Запуск
двигуна" / "🚀 Engine start" card of the Calibration section, under the pre-crank delay, and not in
the Tests section. The command SHALL be routed through `WebPortal::WebCommand` →
`VehicleController::processWebCommand`, SHALL be accepted regardless of the active input source,
and SHALL persist before applying.

#### Scenario: The setting lives in the Engine start card
- **WHEN** the operator opens the Calibration section
- **THEN** the "Engine start" card SHALL show, under the pre-crank delay row, the starter time row
  (label, slider with its value on the right, hint and its own reply line; no Save button and no
  caution line — owner decision 2026-11-17)
- **AND** the card's help button (ⓘ) SHALL show the pre-crank delay help and the starter time help

#### Scenario: Set the limit from the portal
- **WHEN** the operator moves the slider (seconds, min 2, max 10, step 0.5) and releases it
- **THEN** exactly one `set_crank_max_ms` WebSocket command carrying the value × 1000, rounded to a
  whole ms, SHALL be sent on release, and none while dragging (each command is an NVS write)
- **AND** while dragging, the value label SHALL follow the thumb
- **AND** the firmware SHALL round it to a whole ms, validate 2000–10000, persist it, apply it from
  the next start attempt, and respond "Starter time limit set to N ms", shown in the row's status
  line

#### Scenario: Reject an out-of-range limit
- **WHEN** a `set_crank_max_ms` command carries a value that is not-a-number, below 2000 or above
  10000
- **THEN** the system SHALL respond "Starter time limit out of range (2000-10000 ms)"
- **AND** the stored limit SHALL be left unchanged
- **AND** the portal cannot produce such a value: the slider is bounded to 2–10 s

#### Scenario: Save failure
- **WHEN** the NVS write fails
- **THEN** the system SHALL respond "Failed to save the starter time limit" and keep the limit in
  use unchanged

#### Scenario: The slider follows the stored value
- **WHEN** the portal renders a telemetry frame and the operator is not holding the slider
- **THEN** the slider SHALL move to `crank_max_ms` / 1000 (the nearest 0.5 s step for a value off
  the grid) and the value label SHALL show the stored value in seconds with one decimal ("5,0 с" /
  "5.0 s" on a unit without a stored value)
- **AND** while the operator holds the slider, telemetry SHALL NOT move the thumb

#### Scenario: No caution line at any value
- **WHEN** the value shown by the slider is above 5 s
- **THEN** no caution or warning line SHALL be shown in the row (owner decision 2026-11-17); the
  risk of long cranking is explained in the help only

#### Scenario: Hint, help and translations
- **WHEN** the row is shown
- **THEN** a one-line hint SHALL say that the starter runs at most the set time per attempt, stops
  earlier once the engine runs, and that a change applies from the next attempt
- **AND** the help SHALL explain that the limit applies to the RC switch, the ground station and
  the START button, that the engine-running RPM releases the starter earlier, and that a too-long
  value risks starter heat, battery drain and cranking a running engine for the whole limit when
  the RPM reads low; 2–10 s, default 5 s
- **AND** every new label, hint, message and help text SHALL be present in both the English and the
  Ukrainian dictionary
