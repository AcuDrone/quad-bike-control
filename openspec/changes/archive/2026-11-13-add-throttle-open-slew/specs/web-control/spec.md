## ADDED Requirements
### Requirement: Throttle Opening Rate Configuration via Web Interface
The web portal SHALL allow the operator to set the throttle opening rate without a reflash, from a
number input under the "Плавність обмежувача (%/с)" / "Limiter throttle rate (%/s)" row in the
"Обмеження швидкості та кермо" / "Speed limit and steering" card, following the same command
pattern as `set_speed_limit_slew`. The command SHALL be routed through `WebPortal::WebCommand` →
`VehicleController::processWebCommand`.

#### Scenario: Set the rate from the portal
- **WHEN** the operator enters a rate in %/s and saves it
- **THEN** a `set_throttle_open_slew` WebSocket command carrying that value SHALL be sent
- **AND** the firmware SHALL round it to a whole %/s, persist it, then apply it, and respond
  "Throttle open rate set to N %/s", or "Throttle open rate off" for 0, shown in the card's status
  line
- **AND** the command SHALL be accepted regardless of the active input source, like
  `set_speed_limit_slew`

#### Scenario: Reject an out-of-range rate
- **WHEN** a `set_throttle_open_slew` command carries a value that is not-a-number, negative,
  between 0 and `THROTTLE_OPEN_SLEW_MIN_PCT_S` exclusive, or above `THROTTLE_OPEN_SLEW_MAX_PCT_S`
- **THEN** the system SHALL respond "Throttle open rate out of range (0 or 30-100 %/s)"
- **AND** the stored rate SHALL be left unchanged
- **AND** the portal SHALL also refuse the value locally, before sending, with a translated message

#### Scenario: Save failure
- **WHEN** the NVS write fails
- **THEN** the system SHALL respond "Failed to save the throttle open rate" and keep the rate in use
  unchanged

#### Scenario: The input follows the stored value
- **WHEN** the portal renders a telemetry frame and the rate input is not being edited
- **THEN** the input SHALL show `throttle_open_slew`

#### Scenario: Live hint and help
- **WHEN** the input holds a value
- **THEN** a one-line hint under it SHALL show the idle-to-full time, "0 → 100 % за X.X с" /
  "0 → 100 % in X.X s" computed as 100 / value, or "0 = вимкнено" / "0 = off" when the value is 0
- **AND** the input SHALL use step 5 and bounds 0–100
- **AND** the card help (ⓘ) SHALL gain one sentence describing the setting: it slows only the
  opening of the throttle to keep the engine from stalling, closing stays instant, 0 is off, 30–100
  %/s, default 60
- **AND** every new label, hint, message and help text SHALL be present in both the English and
  the Ukrainian dictionary
