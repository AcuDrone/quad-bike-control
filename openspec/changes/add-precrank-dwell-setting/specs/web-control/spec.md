## ADDED Requirements
### Requirement: Pre-Crank Delay Configuration via Web Interface
The web portal SHALL allow the operator to set the pre-crank dwell without a reflash, from a slider
"Затримка стартера" / "Pre-crank delay" (1–8 s, step 0.5 s) in its own card "🚀 Запуск двигуна" / "🚀 Engine
start" in the Calibration section ("Калібрування"), next to the other persisted settings, and not
in the Tests section. The command SHALL be
routed through `WebPortal::WebCommand` → `VehicleController::processWebCommand`, SHALL be accepted
regardless of the active input source, and SHALL persist before applying.

#### Scenario: The setting lives in the Calibration section
- **WHEN** the operator opens the Calibration section
- **THEN** the "Engine start" card SHALL show the delay row (label, slider with its value on the right,
  hint and its own reply line; no Save button) with its own help button (ⓘ)
- **AND** the "Vehicle Ignition & Lighting" card in the Tests section SHALL hold only the ignition,
  light, wheel-lock and boost-rail controls, without the delay row

#### Scenario: Set the delay from the portal
- **WHEN** the operator moves the slider (seconds, min 1, max 8, step 0.5) and releases it
- **THEN** exactly one `set_crank_dwell_ms` WebSocket command carrying the value × 1000, rounded to
  a whole ms, SHALL be sent on release, and none while dragging (each command is an NVS write)
- **AND** while dragging, the value label SHALL follow the thumb
- **AND** the firmware SHALL round it to a whole ms, validate 1000–8000, persist it, apply it, and
  respond "Pre-crank delay set to N ms", shown in the row's status line

#### Scenario: Reject an out-of-range delay
- **WHEN** a `set_crank_dwell_ms` command carries a value that is not-a-number, below 1000 or above
  8000
- **THEN** the system SHALL respond "Pre-crank delay out of range (1000-8000 ms)"
- **AND** the stored delay SHALL be left unchanged
- **AND** the portal cannot produce such a value: the slider is bounded to 1–8 s

#### Scenario: Save failure
- **WHEN** the NVS write fails
- **THEN** the system SHALL respond "Failed to save the pre-crank delay" and keep the delay in use
  unchanged

#### Scenario: The slider follows the stored value
- **WHEN** the portal renders a telemetry frame and the operator is not holding the slider
- **THEN** the slider SHALL move to `crank_dwell_ms` / 1000 (the nearest 0.5 s step for a value off
  the grid) and the value label SHALL show the stored value itself in seconds with one decimal
  ("3,0 с" / "3.0 s" on a unit without a stored value; "1,3 с" for 1300 ms)
- **AND** while the operator holds the slider, telemetry SHALL NOT move the thumb

#### Scenario: Hint, help and translations
- **WHEN** the row is shown
- **THEN** a one-line hint SHALL say that the ignition is on for the set time before the starter
  turns on an RC or GCS auto-start, and that the START button in the Tests section has no delay
- **AND** the "Engine start" card help (ⓘ) SHALL explain that the delay lets the ECU and the fuel pump get ready
  and that a longer value is the first thing to try when the engine does not start from the
  switch
- **AND** every new label, hint, message and help text SHALL be present in both the English and the
  Ukrainian dictionary
