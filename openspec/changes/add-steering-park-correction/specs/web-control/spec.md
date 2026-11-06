## ADDED Requirements
### Requirement: Steering Park Correction Configuration via Web Interface
The web portal SHALL provide a card "Доводка центру керма" / "Steering park correction", placed right after the "Steering Calibration" card, to switch the park correction on or off, set the three ladder rungs, the done tolerance and the maximum pulses, and show the last parking result. The commands SHALL be routed through `WebPortal::WebCommand` → `VehicleController::processWebCommand`, SHALL be accepted regardless of the active input source, and SHALL persist before applying.

#### Scenario: Toggle the correction
- **WHEN** the operator presses the enable toggle
- **THEN** a `set_steer_park_enabled` command with a boolean value SHALL be sent
- **AND** the reply SHALL be "Park correction enabled" or "Park correction disabled", and the button state SHALL follow telemetry (no optimistic update)

#### Scenario: Set a rung
- **WHEN** the operator saves rung N with duty D and length T ms
- **THEN** a `set_steer_park_rung` command with `value` N (1–3) and `strValue` "D,T" SHALL be sent
- **AND** a valid rung SHALL be answered "Park rung N set to D×T"
- **AND** a duty outside 40–150 or a length outside 40–200 ms SHALL be refused with "Park rung out of range (duty 40-150, 40-200 ms)"
- **AND** a rung that would make the ladder decrease SHALL be refused with "Park ladder must not decrease (rung N)"

#### Scenario: Set the done tolerance and the maximum pulses
- **WHEN** the operator saves a done tolerance in degrees or a maximum pulse count
- **THEN** `set_steer_park_tol` (rounded to 0.1°) or `set_steer_park_max` (whole number) SHALL be sent
- **AND** the replies SHALL be "Park tolerance set to X°" / "Park max pulses set to N", or "Park tolerance out of range (0.1-0.5°)" / "Park max pulses out of range (1-10)"
- **AND** an NVS failure on any park command SHALL be answered "Failed to save the park correction setting"

#### Scenario: Card fills from telemetry and validates locally
- **WHEN** telemetry arrives and an input is not being edited
- **THEN** the card SHALL show the stored values from telemetry
- **AND** it SHALL refuse out-of-range or decreasing values locally before sending

#### Scenario: Last-result line
- **WHEN** telemetry reports the last parking result
- **THEN** the card SHALL show, in uk / en, "Останнє паркування: 3 імпульси, залишок 0.15°, останній 85×90" / "Last parking: 3 pulses, residual 0.15°, last 85×90" for done, "Здалось на 0.6° після 5 імпульсів" / "Gave up at 0.6° after 5 pulses" for gave up, "Скасовано рухом стіка" / "Cancelled by stick movement" for cancelled and "ще не було" / "none yet" for none
- **AND** the card help SHALL explain that the first rung should be lowered if a pulse overshoots and the rungs raised if the correction gives up
