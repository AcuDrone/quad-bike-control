## ADDED Requirements
### Requirement: Parking Brake Telemetry
The telemetry stream on the aztec buggy SHALL report the parking-brake state, the run progress, the
throttle hold and the parking-brake settings in use.

#### Scenario: Report the state
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `pbrake_state`, one of `"unknown"`, `"released"`, `"applied"`,
  `"applying"` or `"releasing"`
- **AND** `pbrake_remaining_ms` SHALL be the time left in the current run (including the 200 ms
  reverse dwell and the 500 ms margin), and 0 when no run is in progress

#### Scenario: Report the throttle hold
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `pbrake_thr_hold`, true while the park brake is holding the operator
  throttle at idle: target gear L, H or R, and the state is `APPLIED`, `APPLYING`, `UNKNOWN`, or
  `RELEASING` before the throttle release point
- **AND** `pbrake_thr_hold` SHALL be false in the part of a release run after the release point,
  while `pbrake_state` is still `"releasing"`

#### Scenario: Report why a throttle press did not release
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `pbrake_da_wait`:
  - `"ign_off"` while the state is `APPLIED` or `UNKNOWN`, the target gear is L, H or R and the
    ignition is `OFF`;
  - `"rearm"` while the state is `APPLIED` or `UNKNOWN`, the target gear is L, H or R, the ignition
    is on, the demand is above 10 % and the drive-away latch is not armed;
  - `"none"` otherwise

#### Scenario: Report the settings
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `pbrake_travel_ms` (whole ms), `pbrake_thr_pass_pct` (whole %) and
  `pbrake_invert` (bool), the values in use

#### Scenario: Telemetry document capacity
- **WHEN** the seven keys are added
- **THEN** the worst-case aztec telemetry document SHALL still fit `StaticJsonDocument<9216>` with
  headroom
- **AND** `pbrake_state` and `pbrake_da_wait` SHALL be serialised from string literals, not copied
  `String`s
- **AND** the capacity comment SHALL be updated
