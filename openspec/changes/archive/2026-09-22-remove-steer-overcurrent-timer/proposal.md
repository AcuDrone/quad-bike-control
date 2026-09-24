# Change: Remove the firmware VESC over-current stall timer from steering

## Why

The firmware-side over-current monitor in `SteeringController::serviceDriver()` compares a
400 ms wall-clock window (`STEER_VESC_OVERCURRENT_MS`) against motor-current samples that only
refresh every `STEER_VESC_TELEM_MS` (300 ms) and are never zeroed on `stop()`. That mismatch
makes the monitor unsound in two observed ways:

- **Latch flips to the opposite direction right after a trip.** The trip stops the motor but the
  last decoded current sample stays high, so the next telemetry window re-trips and latches
  against whatever `lastDriveDir_` now holds — blocking the escape direction the stall latch is
  supposed to keep open.
- **False "over-current" trips on an idle motor.** A single lost `COMM_GET_VALUES` reply leaves
  the stale high sample in place for another 300 ms; with only two samples inside the 400 ms
  window, one lost reply is enough to trip a motor that is not drawing current at all.

The protection the timer was meant to add is already in place elsewhere:

- **VESC Tool** enforces *Motor Current Max* (~20 A, a clamp — the VESC limits current rather than
  faulting), *Absolute Max Current* (a hard fault, already surfaced to firmware as a nonzero fault
  code and handled by the existing fault-code → stall-stop path) and MOSFET temperature limiting.
- **The AS5600 position stall detector** (`STEER_STALL_TIMEOUT`, 500 ms with no position change)
  already covers mechanical jams for both closed-loop position moves and open-loop jog.

So the firmware timer adds no coverage — only false trips. A separate observation from the same
bench work: with the real protection being a jam detector rather than a thermal/current integrator,
a 1500 ms same-direction cooldown is longer than needed to let the operator retry, and reads as
unresponsive steering.

## What Changes

- Remove the sustained-over-current monitor from `SteeringController::serviceDriver()`, together
  with the `overCurrentStart_` member and its three reset sites (comm-fault path, VESC-fault path,
  constructor).
- Delete the constants `STEER_VESC_OVERCURRENT_A` and `STEER_VESC_OVERCURRENT_MS`.
- Lower `STEER_STALL_COOLDOWN_MS` from 1500 ms to 700 ms.
- Update the `vehicle-actuators` spec: the VESC telemetry requirement no longer claims a firmware
  over-current backstop, and the stall-latch requirement quotes the shorter cooldown.

Explicitly unchanged:

- VESC fault code → stall-stop (`motor_.hasFault()` → `triggerStallStop()`).
- Comm timeout (`STEER_VESC_COMM_TIMEOUT_MS`) → driver fault: stop, reject moves, auto-recover.
- `steer_motor_current` telemetry — motor current is still decoded, published and displayed; only
  the firmware trip logic goes away.
- `STEER_STALL_TIMEOUT` (500 ms), the AS5600 stall detector, and the whole stall-latch mechanism.

## Impact

- Affected specs: `vehicle-actuators`
- Affected code:
  - `include/Constants.h` — drop both over-current constants; `STEER_STALL_COOLDOWN_MS` 1500 → 700
  - `include/SteeringController.h` — drop `overCurrentStart_`; refresh the `serviceDriver()` doc
  - `src/SteeringController.cpp` — drop the over-current block and its resets
- Not touched: `VescMotorDriver`, `TelemetryManager`, `WebPortal`, `data/`.

## Spec delta note

The over-current behaviour lives in the current spec only as the scenario *"Sustained over-current
triggers stall-stop"* inside `### Requirement: VESC Telemetry Fault Monitoring and Communication
Failsafe` — there is no standalone over-current requirement, and OpenSpec's `MODIFIED` operation
refuses to drop a scenario the current spec still has. The delta therefore removes that requirement
whole and re-adds the surviving behaviour as `### Requirement: VESC Fault Monitoring and
Communication Failsafe` (telemetry polling, fault-code stop, comm failsafe, `steer_motor_current`
telemetry — all unchanged in substance). The stall-latch requirement is carried under
`## MODIFIED Requirements` with the 700 ms cooldown.
