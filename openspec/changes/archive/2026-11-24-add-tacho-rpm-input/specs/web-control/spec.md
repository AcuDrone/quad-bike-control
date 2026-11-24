## ADDED Requirements
### Requirement: Tacho Calibration via Web Interface
The web console SHALL offer a "Оберти (TACHO)" / "Tacho (RPM) calibration" card in the Calibration
section, for calibration only: it SHALL show the live tacho frequency, the ppr in effect, the
tacho-computed RPM and the effective `rpm_source`, and SHALL let the operator set the
pulses-per-revolution from a reference RPM, select it manually or reset it. Command plumbing SHALL
follow the `cal_bat_v` / `reset_bat_v_cal` path (`WebPortal::WebCommand` →
`VehicleController::processWebCommand`), and every command SHALL be accepted regardless of the
active input source.

#### Scenario: Live readout
- **WHEN** telemetry arrives
- **THEN** the card SHALL show `tacho_hz` (one decimal), `tacho_ppr`, `tacho_rpm` and `rpm_source`
- **AND** when `tacho_hz` is 0 it SHALL show "немає імпульсів" / "no pulses"
- **AND** a one-line hint SHALL say the reference comes from the dash, or from the crank sensor:
  "RPM = crank Hz × 60 / 23"

#### Scenario: Calibrate from a reference RPM
- **WHEN** the operator enters "Еталонні оберти" / "Reference RPM" and presses "Калібрувати" /
  "Calibrate"
- **THEN** a `cal_tacho_ppr` command SHALL be sent with `value` = the reference RPM
- **AND** the firmware SHALL compute `raw = Hz × 60 / reference` from the CURRENT averaged frequency
  and SNAP it to the nearest allowed value in {0.5, 1, 2, 3, 4}
- **AND** on acceptance SHALL persist and apply the snapped ppr and reply in the form
  "Tacho: 26.1 Hz × 60 / 1550 = 1.01 → ppr 1"

#### Scenario: Refuse an unusable calibration
- **WHEN** `cal_tacho_ppr` is received and no pulses are present (tacho timed out), or the reference
  is outside 300–8000 RPM, or `raw` differs from the nearest allowed value by more than ±25 % of
  that value, or the NVS write fails
- **THEN** the firmware SHALL keep the previous ppr and reply with a message starting "Tacho:" that
  states the reason (and the raw ratio, where computed)

#### Scenario: Set ppr manually
- **WHEN** the operator picks a ppr from the select and confirms
- **THEN** a `set_tacho_ppr` command SHALL be sent with `value` = the chosen ppr
- **AND** the firmware SHALL accept only a value in {0.5, 1, 2, 3, 4}, persist and apply it, and
  reply "Tacho: ppr set to N"; any other value SHALL be refused with a "Tacho:" message

#### Scenario: Reset ppr
- **WHEN** the operator presses "Скинути" / "Reset"
- **THEN** a `reset_tacho_ppr` command SHALL be sent, and the firmware SHALL remove the stored `ppr`
  key, apply 1.0 and reply "Tacho: ppr reset to 1"
- **AND** the tacho SHALL stay fitted until the next reboot if pulses were seen this boot

#### Scenario: Reply shown in the card
- **WHEN** a command reply whose message starts with "Tacho" arrives
- **THEN** it SHALL be shown in the tacho card's status line, green on success and red on refusal

## MODIFIED Requirements
### Requirement: Service Console Section Layout
Each card SHALL belong to exactly one section. The Diagnostics section SHALL be read-only: apart
from the existing "Probe ECU" diagnostic button in the CAN card it SHALL contain no control that
sends a WebSocket command. Every command-sending control SHALL be in Calibration, Tests or System.

#### Scenario: Diagnostics content
- **WHEN** the Diagnostics section is shown
- **THEN** it SHALL show, in order, the attention box (when not empty), Telemetry, "Кермо: стан" /
  "Steering: state", Vehicle Data (CAN), MAVLink Command Channels and the read-only "Швидкість" /
  "Speed" card

#### Scenario: Calibration content
- **WHEN** the Calibration section is shown
- **THEN** it SHALL show Steering calibration, Throttle calibration, Gear position defaults, Gear
  sensor codes, the "Датчик швидкості" / "Speed sensor" card (pulses per revolution, the
  pulses-per-revolution measurement, wheel circumference) and, directly after it, the "Обмеження
  швидкості та кермо" / "Speed limit and steering" card (limiter floor, speed-limit readout with
  the limiter row, steering authority, steering scaling base, bench test speed), followed by the
  "Оберти (TACHO)" / "Tacho (RPM) calibration" card
- **AND** each of the two speed cards and the tacho card SHALL have its own status line, and the
  reply to a save SHALL be shown in the status line of the card the save came from

#### Scenario: Tests content
- **WHEN** the Tests section is shown
- **THEN** it SHALL show a one-line note "Ручне керування виконавчими механізмами для перевірки на
  стенді" / "Manual actuator control for bench checks", then Manual control, Ignition & lighting
  and Relay test

#### Scenario: System content
- **WHEN** the System section is shown
- **THEN** it SHALL show System maintenance (debug toggle, boost test, firmware upload, and a "Що
  нового" / "What's new" button that opens the existing changelog modal)
- **AND** a read-only "Мережа" / "Network" card showing the access point SSID and IP address from
  telemetry (`wifi_ssid`, `wifi_ip`) with a note that the settings will come later, offering no
  input or command
- **AND** there SHALL be no separate "What's new" card
