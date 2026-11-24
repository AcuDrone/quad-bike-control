## ADDED Requirements
### Requirement: Speed Governor Status Text
The MAVLink interface SHALL tell the ground station when the speed governor should be governing
but cannot because the speed reading is invalid, using `STATUSTEXT` through the shared
`MAVLINK_STATUSTEXT_MIN_MS` limiter. No other MAVLink message, parameter or field SHALL change.

#### Scenario: Warning when the governor falls back to direct throttle
- **WHEN** the governor switch is on, an effective limit exists, the speed reading is invalid and
  the throttle input has been above the 5 % dead-band continuously for `GOV_NOSPEED_WARN_DELAY_MS`
  (2000 ms)
- **THEN** a `STATUSTEXT` with `MAV_SEVERITY_WARNING` and the text "Gov: NO SPEED - direct throttle"
  SHALL be sent
- **AND** it SHALL be repeated every `GOV_NOSPEED_WARN_REPEAT_MS` (10 s) while the condition holds
- **AND** a message held back by the shared limiter SHALL stay pending and be sent on the next
  report past the window, not lost

#### Scenario: Recovery notice
- **WHEN** the warning condition ends because the governor became ACTIVE again
- **THEN** a `STATUSTEXT` with `MAV_SEVERITY_INFO` and the text "Gov: speed OK - governing" SHALL
  be sent once
- **AND** when the condition ends for any other reason (stick released, switch off, limit gone) no
  text SHALL be sent

#### Scenario: Nothing while standing with the stick released
- **WHEN** the speed reading is invalid but the throttle input stays in the dead-band
- **THEN** no governor `STATUSTEXT` SHALL be sent
