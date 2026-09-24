## ADDED Requirements

### Requirement: Steering Speed-Scaling Configuration via Web Interface
The web portal SHALL allow the operator to set the steering speed-scaling base without a reflash,
and to inject a test speed for bench verification, following the same command pattern as the
existing speed calibration setters. Both commands SHALL be routed through
`WebPortal::WebCommand` → `VehicleController::processWebCommand`.

#### Scenario: Set the scaling base from the portal
- **WHEN** the operator enters a base speed in metres per second and saves it
- **THEN** a `set_steer_sca_base` WebSocket command carrying that value SHALL be sent
- **AND** on success the system SHALL respond with an acknowledgement naming the stored value
- **AND** the command SHALL be accepted regardless of the active input source, at the same
  privilege level as the existing `speed_cal_ppr` and `speed_cal_circ` commands

#### Scenario: Zero clears the local base
- **WHEN** a `set_steer_sca_base` command carries zero
- **THEN** the stored base SHALL be cleared to "not set" and persisted as such
- **AND** the portal SHALL show that the base now comes from the autopilot's `MOT_SPD_SCA_BASE`,
  or that there is none

#### Scenario: Reject an out-of-range base from the portal
- **WHEN** a `set_steer_sca_base` command carries a value that is not-a-number, negative, or above
  `STEER_SCALE_BASE_MAX_MS`
- **THEN** the system SHALL respond with an error naming the accepted range
- **AND** the stored base SHALL be left unchanged

#### Scenario: Inject a bench test speed
- **WHEN** the operator enters a test speed and sends it
- **THEN** a `set_test_speed` WebSocket command carrying that value in metres per second SHALL be
  sent
- **AND** the system SHALL apply it to the steering speed-scaling calculation only
- **AND** the response SHALL state that the override is temporary and when it will expire
- **AND** the command SHALL be accepted regardless of the active input source

#### Scenario: Reject an out-of-range test speed
- **WHEN** a `set_test_speed` command carries a value that is not-a-number, negative, or above
  `STEER_SCALE_TEST_SPEED_MAX_MS`
- **THEN** the system SHALL respond with an error and no override SHALL be applied

#### Scenario: Show the scaling state in the portal
- **WHEN** the web portal renders a telemetry frame
- **THEN** it SHALL show the base currently in use, which source that base came from, and the
  scale being applied
- **AND** a live test-speed override SHALL be shown prominently, with its value and the fact that
  it expires by itself
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary
