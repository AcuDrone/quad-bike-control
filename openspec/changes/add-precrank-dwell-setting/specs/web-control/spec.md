## ADDED Requirements
### Requirement: Pre-Crank Delay Configuration via Web Interface
The web portal SHALL allow the operator to set the pre-crank dwell without a reflash, from a row
"Затримка стартера, с" / "Pre-crank delay, s" placed right under the OFF / ACC / START buttons in
the "🔑 Запалювання та освітлення" / "🔑 Vehicle Ignition & Lighting" card. The command SHALL be
routed through `WebPortal::WebCommand` → `VehicleController::processWebCommand`, SHALL be accepted
regardless of the active input source, and SHALL persist before applying.

#### Scenario: Set the delay from the portal
- **WHEN** the operator enters a delay in seconds (number input, min 1, max 8, step 0.5) and saves
  it
- **THEN** a `set_crank_dwell_ms` WebSocket command carrying the value × 1000, rounded to a whole
  ms, SHALL be sent
- **AND** the firmware SHALL round it to a whole ms, validate 1000–8000, persist it, apply it, and
  respond "Pre-crank delay set to N ms", shown in the row's status line

#### Scenario: Reject an out-of-range delay
- **WHEN** a `set_crank_dwell_ms` command carries a value that is not-a-number, below 1000 or above
  8000
- **THEN** the system SHALL respond "Pre-crank delay out of range (1000-8000 ms)"
- **AND** the stored delay SHALL be left unchanged
- **AND** the portal SHALL also refuse a value outside 1–8 s locally, before sending, with a
  translated message

#### Scenario: Save failure
- **WHEN** the NVS write fails
- **THEN** the system SHALL respond "Failed to save the pre-crank delay" and keep the delay in use
  unchanged

#### Scenario: The input follows the stored value
- **WHEN** the portal renders a telemetry frame and the input is not being edited
- **THEN** the input SHALL show `crank_dwell_ms` / 1000 with one decimal, which is 3.0 on a unit
  without a stored value

#### Scenario: Hint, help and translations
- **WHEN** the row is shown
- **THEN** a one-line hint SHALL say that the ignition is on for the set time before the starter
  turns on an RC or GCS auto-start, and that the START button has no delay
- **AND** the card help (ⓘ) SHALL explain that the delay lets the ECU and the fuel pump get ready
  and that a longer value is the first thing to try when the engine does not start from the
  switch
- **AND** every new label, hint, message and help text SHALL be present in both the English and the
  Ukrainian dictionary
