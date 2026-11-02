## ADDED Requirements
### Requirement: Speed-Limiter Floor Configuration via Web Interface
The web portal SHALL allow the operator to set the speed-limiter throttle floor without a reflash,
from the Speed sensor settings card next to the pulses-per-revolution and wheel-circumference
inputs, following the same command pattern as the existing speed calibration setters. The command
SHALL be routed through `WebPortal::WebCommand` → `VehicleController::processWebCommand`.

#### Scenario: Set the floor from the portal
- **WHEN** the operator enters a floor in percent and saves it
- **THEN** a `set_speed_limit_floor` WebSocket command carrying that value SHALL be sent
- **AND** on success the system SHALL persist the value and respond with an acknowledgement naming
  the stored value in percent
- **AND** the command SHALL be accepted regardless of the active input source, at the same
  privilege level as the existing `speed_cal_ppr` and `speed_cal_circ` commands

#### Scenario: Reject an out-of-range floor
- **WHEN** a `set_speed_limit_floor` command carries a value that is not-a-number, below
  `SPEED_LIMIT_FLOOR_MIN_PCT` or above `SPEED_LIMIT_FLOOR_MAX_PCT`
- **THEN** the system SHALL respond with an error naming the accepted range (0–20 %)
- **AND** the stored floor SHALL be left unchanged
- **AND** the portal SHALL also refuse the value locally, before sending, with a translated
  message

#### Scenario: The input follows the stored value
- **WHEN** the portal renders a telemetry frame and the floor input is not being edited
- **THEN** the input SHALL show `speed_limit_floor`, as the pulses-per-revolution and
  circumference inputs show their stored values

#### Scenario: The help explains the floor
- **WHEN** the operator opens the help (ⓘ) of the Speed sensor settings card
- **THEN** it SHALL explain that the floor is the lowest throttle the speed limiter may leave at
  and above the limit, that 0 % means idle and holds the limit most tightly, and that a higher
  floor softens engine braking at the cost of a limit that may not hold
- **AND** every new label, message and help text SHALL be present in both the English and the
  Ukrainian dictionary
