## 0. Preconditions
- [x] 0.1 Version slot: `master` 1.5.7, `aztec` 1.5.11 — take **1.5.12** on `master` (patch: diagnostics only, no calibration or wiring change).
- [ ] 0.2 Branch: implement on `master`, then merge `master` into `aztec`.

## 1. Driver hooks
- [x] 1.1 `IMotorDriver`: `lastSpeed()` and `setTelemetryPeriodMs()` with no-op defaults.
- [x] 1.2 `VescMotorDriver`: return the last effort; COMM_GET_VALUES period from `telemPeriodMs_` (default `STEER_VESC_TELEM_MS`).

## 2. Steering controller
- [x] 2.1 Diagnostics lock (`setDiagLock`, `isDiagLocked`) gating `setSteeringPercent`, `setRateCommand`, `jog`, `nudge`, captures and the zero-set; boot centring cancelled.
- [x] 2.2 `diagDrive()` (open loop, no floor) and `diagGoto()` through the shared `commandTarget()` path.
- [x] 2.3 Expose `getLastDriveCommand()`, `getTargetPos()`, `getPosState()`, `posStateToken()`.
- [x] 2.4 `VehicleController`: web steering / jog / nudge / capture / zero answer "held by the USB diagnostics console" while locked.

## 3. Diagnostics console
- [x] 3.1 `include/SteerDiagLaw.h`: parser, ramp / pulse / hold / goto machine, summary values, safety predicates.
- [x] 3.2 `include/SteerDiag.h`, `src/SteerDiag.cpp`: line reader, commands, safety gating and aborts, 60 s idle release, debounced host-lost release (STEER_DIAG_HOST_LOST_MS), D / S / E / I / OK output, non-blocking writes.
- [x] 3.3 `src/main.cpp`: instance + `steerDiag.update()` after `vehicleController.update()`, ready banner.
- [x] 3.4 Constants block "STEERING DIAGNOSTICS (USB serial)" in `include/Constants.h`.

## 4. Tests and host tool
- [x] 4.1 `test/test_steer_diag` (parser, ramp detection / no-move / timeout, pulse, hold averages, goto overshoot / residual / restart / timeout, safety predicates); add to `platformio.ini` `test_filter`.
- [x] 4.2 `tools/steer_diag.py`: status, ramp, pulse, hold, goto, stop, sweep-breakaway, sweep-pulse, center, raw; CSV logs; reset-tolerant connect. `diag-logs/` in `.gitignore`.

- [x] 4.3 Web: `steer_angle` command (degrees, `SteeringController::setSteeringAngleDeg()` via `commandTarget()`, web control + Position-mode gating), "Target angle" input with Go / 0° in the calibration card, jog speed slider 80–255 default 120, nudge ≈ 0.4° hint (en/uk); `steer_diag.py` summary shows park window 0.4°, restart 1.0°.

## 5. Release
- [x] 5.1 `FIRMWARE_VERSION` 1.5.12.
- [x] 5.2 `Changelog.h` entry (uk/en).
- [x] 5.3 `pio run -e esp32-s3-devkitc-1` and `pio test -e native` pass.

## 6. Bench / vehicle
- [ ] 6.1 `status` answers over USB; `diag on` blocks web jog and MAVLink steering; `diag off` restores them.
- [ ] 6.2 Abort checks: `stop` mid-hold, unplugging USB mid-hold, ramp toward a limit stops 2° short.
- [ ] 6.3 Run `sweep-breakaway`, `sweep-pulse`, `center`; record the results for the tuning change.
