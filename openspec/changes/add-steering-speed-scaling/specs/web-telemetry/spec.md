## ADDED Requirements

### Requirement: Steering Speed-Scaling Telemetry
The telemetry stream SHALL report the steering speed-scaling factor being applied, the base value
that produced it, where that base came from, and whether a bench test speed is overriding the
measured speed — so that "the steering feels weak" can be diagnosed from the portal without a
serial console.

#### Scenario: Report the applied scale
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `steer_scale`, the scale actually applied after rate limiting, in the
  range 0 to 1 with two decimals
- **AND** a value of 1 SHALL mean no authority is being withdrawn

#### Scenario: Report the base and its source
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `steer_sca_base`, the base in use in metres per second, zero when
  there is none
- **AND** it SHALL include `steer_sca_src`, naming the source of that base as the locally stored
  value, the autopilot's `MOT_SPD_SCA_BASE`, or none

#### Scenario: Report a live test-speed override
- **WHEN** a telemetry frame is produced while a `set_test_speed` override is in force
- **THEN** it SHALL include `steer_test_speed`, the overriding speed in metres per second
- **AND** the field SHALL be zero or absent when no override is in force, so a stale bench setting
  cannot be mistaken for a real reading

#### Scenario: Web interface shows why the steering is being scaled
- **WHEN** the web interface renders a telemetry frame
- **THEN** it SHALL display the applied scale, and alongside it the base and its source
- **AND** it SHALL state when the scaling is inactive and why — no base, an invalid speed reading,
  or the autopilot not being in MANUAL
- **AND** a live test-speed override SHALL be shown as a temporary bench state, not as a normal
  reading
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary
