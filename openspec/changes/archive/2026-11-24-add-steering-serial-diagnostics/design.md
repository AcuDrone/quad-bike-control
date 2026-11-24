# Design: USB serial steering diagnostics

## D1 — Where the lock lives
The lock is a flag inside `SteeringController`, checked at every external entry point
(`setSteeringPercent`, `setRateCommand`, `jog`, `nudge`, captures, zero-set). One place covers all
callers: MAVLink (`VehicleController::processMavlinkCommands`), web slider / jog / nudge / captures
/ zero (`VehicleController::processSteer*Command`), failsafe centre (`applyFailsafe`) and boot
centring (`serviceBootCenter`). The web handlers additionally answer with an explicit message.
Stop paths (`stop`, `stopRate`, sensor and VESC monitors) are never gated.

## D2 — Driving from the console
- Open loop: `diagDrive(speed)` drops the closed loop and any jog to idle and calls
  `setSpeed(speed)` with no duty floor (the ramp must start below 140), no stall latch and no
  refresh timeout. The console re-sends the duty at least every 100 ms (VESC UART timeout) and
  owns the bounds.
- Closed loop: `diagGoto(target)` uses the same command path as `setSteeringPercent()`
  (`commandTarget()`), so the trace shows the real loop. When the summary is printed the loop is
  dropped to idle (coast; the worm holds) so it does not restart under the lock.

## D3 — Pure logic
`include/SteerDiagLaw.h` holds the parser, the experiment machine (DRIVE → SETTLE → DONE) and the
summary values; `SteerDiag.cpp` only does I/O, safety gating and applies the duty. The machine is
stepped every loop iteration (pulse length accuracy = loop period), the D lines are rate-limited to
50 Hz plus one per duty step / phase change.

## D4 — USB output never blocks the loop
D lines are written only when `Serial.availableForWrite()` has room; otherwise they are counted
(`drop=` in the summary). Host gone aborts and releases the steering, debounced: HWCDC's `operator bool` (core 3.3.7) is a `connected` flag set by RX / TX-drain interrupts and cleared by bus reset, unplug and write timeouts, so it can read false transiently. The check arms after one true reading since `diag on` and trips after `STEER_DIAG_HOST_LOST_MS` (500 ms) of continuous false (`steerDiagHostLost()`).

## D5 — Movement detection
Ramp movement = 0.5° from the start position in the drive direction, or an LWS rate of ≥ 5 °/s in
the drive direction (rate in the wrong direction, e.g. backlash, is ignored). The current at
detection is the latest VESC reply (≤ 50 ms old in diag mode).
