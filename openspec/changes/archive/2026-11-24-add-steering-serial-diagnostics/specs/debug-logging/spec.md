## ADDED Requirements
### Requirement: USB Steering Diagnostics Console
The firmware SHALL accept line commands on the USB-CDC `Serial` console (never UART0 or UART2) to measure the steering actuator: `diag on`, `diag off`, `status`, `stop` / `s`, `help`, `ramp L|R [max= step= stepms= start=]`, `pulse L|R <duty> <ms>`, `hold L|R <duty> <ms>` and `goto <deg>`. Lines SHALL be accepted with CR, LF or CRLF endings, case-insensitively. Output SHALL be machine-readable with fixed prefixes: `OK <cmd>`, `E,<cmd>,<reason>`, `E,abort,<reason>`, `I,...`, `D,<t_ms>,<duty>,<pos>,<rate>,<cur>,<vin>,<state>,<ref>` at 50 Hz while an experiment runs, and `S,<cmd>,key=value,...` summaries. Reading and writing SHALL never block the main loop; sample lines that do not fit the USB transmit buffer SHALL be dropped and counted.

#### Scenario: Standstill only
- **WHEN** `diag on` or an experiment is requested while the trusted vehicle speed exceeds `STEER_DIAG_MAX_SPEED_KMH` (0.5 km/h)
- **THEN** it SHALL be refused with `E,<cmd>,speed`
- **AND** a running experiment SHALL be aborted with the motor cut when speed appears

#### Scenario: Experiment preconditions and aborts
- **WHEN** an experiment is requested
- **THEN** it SHALL require diag mode, both limits captured, a valid LWS angle and the VESC link with no fault, and SHALL NOT start toward a limit closer than `STEER_DIAG_LIMIT_MARGIN` (2°)
- **AND** a running experiment SHALL be aborted (motor cut, `E,abort,<reason>`) on angle loss, VESC link loss or fault, driving to within `STEER_DIAG_LIMIT_MARGIN` of a captured limit, `stop`, or a new command

#### Scenario: Break-away ramp
- **WHEN** `ramp R` is run
- **THEN** the duty SHALL start at `start` and rise by `step` every `stepms` up to `max`
- **AND** when the wheel has moved 0.5° in the drive direction (or turns at ≥ `STEER_DIAG_MOVE_RATE` °/s that way) the duty and the latest motor current SHALL be recorded, the duty cut to 0, and the coast logged for `STEER_DIAG_SETTLE_MS`
- **AND** an `S,ramp` line SHALL report the break-away duty, current and coast; the ramp SHALL end after 1 s at `max` without movement and SHALL NOT exceed 15 s

#### Scenario: Pulse and hold
- **WHEN** `pulse` or `hold` is run
- **THEN** the duty SHALL be applied open loop for the given time (pulse ≤ 1000 ms, hold ≤ 1500 ms) and logged until `STEER_DIAG_SETTLE_MS` after the cut
- **AND** the summary SHALL report the displacement at the cut and after settling, the coast and the peak current; a hold SHALL also report the mean rate and mean current over its second half

#### Scenario: Closed-loop goto trace
- **WHEN** `goto <deg>` is run in diag mode
- **THEN** the normal closed loop SHALL drive to the target while D lines carry the loop state (`drive`, `park`, `idle`) and the target
- **AND** after the loop has stopped driving for `STEER_DIAG_SETTLE_MS` (or after 12 s) an `S,goto` line SHALL report the overshoot past the target and the residual error, and the loop SHALL be dropped to idle

#### Scenario: Automatic release
- **WHEN** diag mode is on and no command line arrives for `STEER_DIAG_IDLE_TIMEOUT_MS` (60 s), or the USB host disappears (`Serial` reads not connected continuously for `STEER_DIAG_HOST_LOST_MS` (500 ms), counted only after it has read connected once since `diag on`)
- **THEN** any experiment SHALL be aborted, the motor stopped and the steering released (`I,diag,off,reason=...`)
