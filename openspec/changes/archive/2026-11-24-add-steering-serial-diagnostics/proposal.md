# Change: USB serial steering diagnostics console

## Why
Two field reports on the steering (Bosch LWS + VESC + 1:40 self-locking worm):
1. after a turn, the RC command back to centre (1500 µs → 0°) leaves a residual error and the
   vehicle pulls to one side;
2. setting exactly 0° by hand is very hard — the web jog / nudge overshoots strongly either way.

The duties and times behind both behaviours — `STEER_PWM_MIN_DUTY` 140 (break-away),
`STEER_PWM_TRACK_MIN_DUTY` 100, `STEER_NUDGE_DUTY` 150, `STEER_NUDGE_MS` 70 — were guessed on the
bench and never measured on the vehicle. Tuning them needs numbers: the real break-away duty per
direction, how far the wheel coasts after the duty is cut, degrees per short pulse, the turning rate
versus duty, and a 50 Hz trace of the closed loop settling at 0°.

## What Changes
- **New USB console `SteerDiag`** (`include/SteerDiag.h`, `src/SteerDiag.cpp`) polled from
  `loop()`, reading line commands from the USB-CDC `Serial` (the firmware had no serial input
  until now). Never UART0 / UART2 (the VESC links). Commands: `diag on|off`, `status`, `stop`/`s`,
  `help`, `ramp L|R [max= step= stepms= start=]`, `pulse L|R <duty> <ms>`, `hold L|R <duty> <ms>`,
  `goto <deg>`. Machine-readable output: `OK`, `E,`, `I,`, `D,` (50 Hz samples), `S,` (summary).
- **Exclusive steering ownership.** `diag on` sets a new diagnostics lock in
  `SteeringController`: `setSteeringPercent()` (MAVLink, web slider, failsafe centre, boot centre),
  `setRateCommand()`, `jog()`, `nudge()`, the limit captures and the sensor zero-set are refused;
  the web commands answer "Steering is held by the USB diagnostics console". Stop paths are never
  gated. The console drives open loop through `diagDrive()` and runs one closed-loop move through
  `diagGoto()`. Auto `diag off` after 60 s without a line or when the USB host goes away.
- **Safety.** `diag on` and every experiment are refused above 0.5 km/h; experiments need both
  limits, a valid LWS angle, the VESC link with no fault. A running experiment is aborted (motor
  cut) on speed, angle loss, VESC link loss or fault, or when driving to within 2° of a captured
  limit. Hard bounds: ramp 15 s, pulse ≤ 1000 ms, hold ≤ 1500 ms, goto 12 s, duty ≤ 255.
- **VESC telemetry rate.** `IMotorDriver` gains `lastSpeed()` and `setTelemetryPeriodMs()`;
  `VescMotorDriver` polls COMM_GET_VALUES every 50 ms while diag is on (300 ms otherwise).
- **Pure logic, host-tested.** The parser, the experiment state machine and the summary values
  live in header-only `include/SteerDiagLaw.h` with a native suite `test/test_steer_diag`.
- **Host script** `tools/steer_diag.py` (python3 + pyserial): `status`, `ramp`, `pulse`, `hold`,
  `goto`, `stop`, `sweep-breakaway`, `sweep-pulse`, `center`, `raw`; CSV logs under `diag-logs/`.
- **Version** 1.5.7 → **1.5.12** on `master` (one version line across branches: `aztec` is at
  1.5.11), with a `Changelog.h` entry (uk/en).
- **Unchanged:** every steering constant and the position law; with diag off the steering behaves
  exactly as before.

## Impact
- Affected specs: `vehicle-actuators` (ADDED "Steering Diagnostics Ownership Lock"),
  `debug-logging` (ADDED "USB Steering Diagnostics Console").
- Affected code: `include/IMotorDriver.h`, `include/VescMotorDriver.h`, `src/VescMotorDriver.cpp`,
  `include/SteeringController.h`, `src/SteeringController.cpp`, `src/VehicleController.cpp`,
  `src/main.cpp`, `include/Constants.h`, `include/Changelog.h`, new `include/SteerDiag.h`,
  `src/SteerDiag.cpp`, `include/SteerDiagLaw.h`, `test/test_steer_diag/`, `platformio.ini`,
  `tools/steer_diag.py`, `.gitignore`.
- Safety: the console can drive the steering motor open loop. It is standstill-only, needs an
  explicit `diag on`, is bounded in time and duty, stops at 2° from the limits, and releases the
  steering by itself after 60 s of silence or when the USB host disappears. While it is on the
  failsafe centre is refused (the vehicle is standing still by precondition).
