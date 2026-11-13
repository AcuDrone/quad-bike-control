## ADDED Requirements
### Requirement: Gear-Change Boost Enable Telemetry
The telemetry stream SHALL report whether the gear-change RPM boost is switched on, so the web
portal can show the stored setting.

#### Scenario: Report the switch state
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `boost_enabled` (bool), the switch state in use, beside
  `boost_target_rpm`
- **AND** the key SHALL be present whether or not a gear change or boost test is active
