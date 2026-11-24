## ADDED Requirements
### Requirement: Service Brake Status Text
On the aztec buggy the MAVLink interface SHALL tell the ground station when the iBooster service
brake is lost, in fault, or back to normal, using `STATUSTEXT` through the shared
`MAVLINK_STATUSTEXT_MIN_MS` limiter and a brake-specific hold-off `IB_STATUSTEXT_MIN_MS` (5 s). No
other MAVLink message, parameter or field SHALL change.

#### Scenario: Brake lost
- **WHEN** the brake state enters `LOST`
- **THEN** a `STATUSTEXT` with `MAV_SEVERITY_WARNING` and the text "Brake: LOST (no iBooster)" SHALL
  be sent

#### Scenario: Brake fault
- **WHEN** the brake state enters `FAULT`
- **THEN** a `STATUSTEXT` with `MAV_SEVERITY_WARNING` and the text "Brake: FAULT (iBooster)" SHALL be
  sent

#### Scenario: Brake recovered
- **WHEN** the brake state becomes `READY` after a `LOST` or `FAULT` text was sent
- **THEN** a `STATUSTEXT` with `MAV_SEVERITY_INFO` and the text "Brake: OK" SHALL be sent once

#### Scenario: Normal transitions are silent
- **WHEN** the brake state changes between `ASLEEP`, `STARTING`, `READY`, `BRAKING` and `RELEASING`
  without a preceding announced `LOST` or `FAULT`
- **THEN** no brake `STATUSTEXT` SHALL be sent

#### Scenario: Flapping unit cannot flood the link
- **WHEN** the brake state changes again less than `IB_STATUSTEXT_MIN_MS` (5 s) after the last brake
  `STATUSTEXT`
- **THEN** no text SHALL be sent before the hold-off ends; the change SHALL replace any pending brake
  text, so at most one brake text is sent per 5 s and the last one sent describes the current state
- **AND** a pending recovery that cancels a still-unsent `LOST` or `FAULT` text SHALL send nothing
- **AND** a text held back by the shared limiter SHALL stay pending, not be lost
