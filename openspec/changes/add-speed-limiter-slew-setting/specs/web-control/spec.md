## ADDED Requirements
### Requirement: Speed-Limiter Ceiling Rate Configuration via Web Interface
The web portal SHALL allow the operator to set the speed-limiter throttle rate (ceiling slew)
without a reflash, from a number input in the "Обмеження швидкості та кермо" / "Speed limit and
steering" card, following the same command pattern as `set_speed_limit_floor`.

#### Scenario: Set the rate from the portal
- **WHEN** the operator enters a rate in %/s and saves it
- **THEN** a `set_speed_limit_slew` WebSocket command carrying that value SHALL be sent
- **AND** on success the system SHALL persist the value and respond with an acknowledgement naming
  the stored value in %/s, shown in the card's status line
- **AND** the command SHALL be accepted regardless of the active input source, like
  `set_speed_limit_floor`

#### Scenario: Reject an out-of-range rate
- **WHEN** a `set_speed_limit_slew` command carries a value that is not-a-number, below
  `SPEED_LIMIT_SLEW_MIN_PCT_S` or above `SPEED_LIMIT_SLEW_MAX_PCT_S`
- **THEN** the system SHALL respond with an error naming the accepted range (10–200 %/s)
- **AND** the stored rate SHALL be left unchanged
- **AND** the portal SHALL also refuse the value locally, before sending, with a translated message

#### Scenario: The input follows the stored value
- **WHEN** the portal renders a telemetry frame and the rate input is not being edited
- **THEN** the input SHALL show `speed_limit_slew`

#### Scenario: Hint and help explain the rate
- **WHEN** the card is shown
- **THEN** a one-line hint under the input SHALL say that a lower value closes and returns the
  throttle more gently and that 80 is the previous behaviour
- **AND** the card help (ⓘ) SHALL describe the setting and its range
- **AND** every new label, hint, message and help text SHALL be present in both the English and
  the Ukrainian dictionary
