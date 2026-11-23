## ADDED Requirements
### Requirement: Service Brake Telemetry
The telemetry stream SHALL report the service brake's command, its measured position, the unit state
and the brake settings in use.

#### Scenario: Keys
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `brake_pct` (measured rod as % of the maximum stroke, 0–100, 0 when not
  available), `brake_target_pct` (commanded), `brake_rod_mm` (two decimals, `null` when unknown),
  `brake_state` (`asleep`, `starting`, `ready`, `braking`, `releasing`, `lost`, `fault`), `ib_st`,
  `ib_is`, `ib_drv` (the unit's status, internal state and driver_brake_apply, `null` when unknown),
  `ib_partner` (bool), `ib_runout` (bar), `brake_max_mm`, `brake_apply_mlps`, `brake_release_mlps`,
  `ib_tx_miss`, `ib_gap_max_ms` and `ib_faults` (session values)
- **AND** it SHALL NOT include `brake_travel_ms`, `brake_hold_pwm` or `brake_released`

#### Scenario: Telemetry document capacity
- **WHEN** the keys are added
- **THEN** the worst-case telemetry document SHALL still fit the aztec `StaticJsonDocument<10240>`
  (grown from 9216 in 1.5.36) with headroom, and the capacity comment SHALL be updated

### Requirement: Service Brake Display and Attention
The web UI SHALL show the service brake's availability wherever the brake is shown, and SHALL raise
it in the Diagnostics attention summary when it is unavailable with the ignition on.

#### Scenario: Brake tile
- **WHEN** the brake state is `ready`, `braking` or `releasing`
- **THEN** the "Гальмо" / "Brake" tile SHALL show the measured percentage and the rod in mm
- **WHEN** the state is `asleep`
- **THEN** the tile SHALL show "спить (запалювання вимкнено)" / "asleep (ignition off)" in the muted
  colour
- **WHEN** the state is `starting`, `lost` or `fault`
- **THEN** the tile SHALL show the state in words in the warning (`starting`) or danger (`lost`,
  `fault`) colour

#### Scenario: Attention items
- **WHEN** the state is `fault` or `lost`
- **THEN** the attention summary SHALL list a danger item naming the state and the action
  ("ignition off and on; if that does not help, remove the iBooster's power")
- **WHEN** the state has been `starting` for more than 20 s
- **THEN** a warning item SHALL be listed
- **AND** `asleep` SHALL NOT be an attention item

#### Scenario: Tests brake slider
- **WHEN** the Tests section brake slider is shown
- **THEN** the brake state SHALL be shown next to its value
- **AND** a web brake command while the brake is unavailable SHALL be accepted as a target and
  answered with a reply that says the brake is unavailable

## REMOVED Requirements
### Requirement: Brake Position and Tuning Telemetry
**Reason**: The open-loop estimate, the tuning values and the In5 endstop no longer exist.
**Migration**: `brake_pct` keeps its name with the measured value ("Service Brake Telemetry");
`brake_travel_ms`, `brake_hold_pwm` and `brake_released` are removed.
