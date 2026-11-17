## ADDED Requirements
### Requirement: Speed Governor Telemetry
The telemetry stream SHALL report the governor's switch, state, setpoint, output, saturation and
settings in ten scalar keys, within the telemetry document's configured capacity (the capacity
comment SHALL be updated).

#### Scenario: State keys
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `gov_en` (bool), `gov_state` (one of `off`, `nolimit`, `bypass`,
  `nospeed`, `active`, the mode selected in the latest evaluation), `gov_sp` (the ramped setpoint in
  km/h, one decimal, 0 when not ACTIVE), `gov_out` (the governor output in whole %, 0 when not
  ACTIVE) and `gov_sat` (bool)
- **AND** the keys SHALL be present whether or not a limit is in force

#### Scenario: Saturation flag
- **WHEN** the governor output has been at idle while the speed stayed more than
  `SPEED_LIMIT_SAT_MARGIN_MS` above the setpoint for `SPEED_LIMIT_SAT_HOLD_MS`
- **THEN** `gov_sat` SHALL be true, and SHALL stay true for `SPEED_LIMIT_SAT_UI_HOLD_MS` after the
  condition clears
- **AND** no serial log line and no MAVLink message SHALL be produced for it

#### Scenario: Settings keys
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `gov_kp`, `gov_ki`, `gov_acc`, `gov_dec` and `gov_launch`, in firmware
  units, `gov_kp` and `gov_ki` with two decimals (the defaults, 1.0 / 0.35 on the buggy and 3.0 / 1.0
  on the quad bike, sit in the low range) and the others with one decimal

### Requirement: Speed Governor State Display
The Diagnostics "Швидкість" / "Speed" card and the Calibration "Speed limit and steering" card SHALL
show the governor state read-only, rendered by one path from the same telemetry frame.

#### Scenario: Governor row
- **WHEN** a telemetry frame with `gov_state` is rendered
- **THEN** a "Регулятор" / "Governor" row SHALL show "вимкнено" / "off", "немає ліміту — газ
  напряму" / "no limit — direct throttle", "обхід (N / перемикання)" / "bypassed (N / shift)" or
  "активний: NN км/год, газ NN %" / "active: NN km/h, throttle NN %"
- **AND** an unknown or missing value SHALL show `--` without a JavaScript error

#### Scenario: No-speed warning banner
- **WHEN** `gov_en` is true and `gov_state` is `nospeed`
- **THEN** both cards SHALL show a banner in the danger colour "Регулятор не діє: немає сигналу
  швидкості — газ напряму (повний газ = повний газ)" / "Governor not acting: no speed signal —
  direct throttle (full stick = full throttle)"

#### Scenario: At-idle badge
- **WHEN** `gov_sat` is true
- **THEN** the governor row SHALL show "Регулятор на холостому, швидкість вища за задану —
  гальмуйте" / "Governor at idle, speed above setpoint — brake" in the warning colour
- **AND** while the governor is ACTIVE the limiter's "Limiting" row SHALL be hidden, since the
  limiter stands down
