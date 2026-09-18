## ADDED Requirements

### Requirement: Speed-Limiter Source and Ceiling Telemetry
The telemetry stream SHALL report which ceiling the maximum-speed limiter is actually enforcing,
where that ceiling came from, the autopilot value behind it, and how much throttle authority the
limiter is currently withdrawing. The existing stored-ceiling key SHALL keep reporting the stored
value, so the configuration input continues to edit the local fallback while the autopilot is in
charge.

#### Scenario: Report the effective ceiling and its source
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_eff`, the ceiling the limiter is enforcing in km/h
- **AND** it SHALL include `speed_limit_src`, one of `"off"`, `"local"` or `"mavlink"`
- **AND** `speed_limit_eff` SHALL be zero whenever `speed_limit_src` is `"off"`

#### Scenario: Report the autopilot parameter separately from the enforced ceiling
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `mav_speed_max`, the last `SPEED_MAX` received from the autopilot
  converted to km/h
- **AND** it SHALL report zero when no usable value has been received, so the operator can tell
  "the autopilot never sent one" from "the autopilot sent one and it is being used"
- **AND** this key SHALL be reported independently of `speed_limit_src`, so a value that exists but
  is not being used (limiter off) is still visible

#### Scenario: Report the current throttle ceiling
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_ceil`, the throttle ceiling the taper is currently
  applying as a percentage
- **AND** a value of 100 SHALL mean the limiter is withdrawing no authority

#### Scenario: The stored ceiling keeps its own key
- **WHEN** the limiter ceiling is being supplied by the autopilot
- **THEN** `speed_limit_max` SHALL still report the locally stored ceiling, not the autopilot value
- **AND** the web configuration input SHALL stay bound to `speed_limit_max`, so saving from the UI
  continues to edit the local fallback

#### Scenario: Web interface shows the active limit and its source
- **WHEN** the web interface renders a telemetry frame
- **THEN** it SHALL display the active limit and a label naming its source
- **AND** while the source is `"mavlink"` the stored-ceiling input SHALL be visually de-emphasised
  but SHALL remain editable, because it is still the fallback the operator may need to set
- **AND** the current throttle ceiling SHALL be shown only while the limiter is actually
  withdrawing authority (`speed_limit_ceil` below 100)
- **AND** every added label SHALL be present in both the English and the Ukrainian dictionary
