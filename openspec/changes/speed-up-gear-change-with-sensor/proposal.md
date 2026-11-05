# Change: End the gear-change dwells early when the gear sensor confirms the gear

## Why
Every shift step waits a fixed `TRANS_OVERSHOOT_DWELL_MS` (2000 ms) at the overshoot position,
and a multi-step sequence waits a further fixed `TRANS_SEQUENCE_STEP_DWELL_MS` (1000 ms) between
steps, whatever the physical gear sensor says. With the default positions a single step takes
≈2.8 s, N→L ≈6.7 s and R→L ≈10.3 s, most of it waiting for a gear that the sensor already shows
engaged. The RPM boost / shift idle hold (`needsThrottleBoost()`) lasts the whole dwell too.

Depends on `fix-transmission-gear-sync` (1.5.14): the expected gear must already be reconciled
with the sensor, and this change modifies the same Transmission Servo Control requirement on top
of that change's version. Implement and archive it after that change.

## What Changes
- **Overshoot dwell ends on confirmation.** In `OVERSHOOT_DWELL` the dwell ends as soon as
  `getPhysicalGear()` has reported the step's target gear continuously for `TRANS_DWELL_CONFIRM_MS`
  (new, 200 ms; comment: 300–400 ms may be needed if the linkage bounces), measured from the first
  confirming read; any other reading restarts the window. `TRANS_OVERSHOOT_DWELL_MS` (2000) stays
  as the timeout, after which the gear is assumed engaged exactly as today (worst case = today).
  Then RETURN as today. Logs `[TRANS] Dwell confirmed after <n> ms` or
  `[TRANS] Dwell timeout <n> ms, gear assumed`.
- **`needsThrottleBoost()` unchanged** (true for the whole `OVERSHOOT_DWELL`), so the boost PID and
  the shift idle hold release automatically with the shorter dwell.
- **Inter-step pause ends on confirmation.** While a sequence is queued, the pause before the next
  step ends once the sensor has reported the just-completed step's gear (`targetGear_`)
  continuously for `TRANS_DWELL_CONFIRM_MS`; `TRANS_SEQUENCE_STEP_DWELL_MS` (1000) stays as the
  timeout. Logs `[TRANS] Step <gear> confirmed after <n> ms` / `... pause timeout`.
- **Sensor reads during the waits.** Verified in `getPhysicalGear()`: the idle-retry rule
  (`TRANS_GEAR_READ_RETRY_COUNT` deferrals that return the *previous* cached gear) applies only
  when `gearChangePhase_ == NONE`, so reads during `OVERSHOOT_DWELL` are already decoded
  immediately. The inter-step pause runs in phase NONE, so the retry condition is extended to
  "phase NONE **and** no sequence queued" — a stale cached gear can then never count as
  confirmation. The 100 ms read cache (`TRANS_GEAR_READ_INTERVAL_MS`) stays: confirmation is
  sampled on a 100 ms grid (3 matching reads ≈ 200–300 ms after the gear engages), the expander
  snapshot itself is ≤ `BOARD_INPUT_POLL_MS` (25 ms) old.
- **Expected timings** (servo 35 ms/%, min 300 ms per move; positions R 40, N 48, H 58, L 68 %;
  overshoots R 4.5, N 1, H 5, L 5 %; confirmation assumed ≈300 ms after reaching the overshoot,
  pause ≈250 ms):

  | Shift | Today | With sensor confirm | Sensor absent |
  |-------|-------|---------------------|---------------|
  | one step, e.g. N→H (525 + dwell + 300 ms) | ≈2.8 s | ≈1.2 s | ≈2.8 s |
  | N→L (N→H, pause, H→L) | ≈6.7 s | ≈2.6 s | ≈6.7 s |
  | R→L (R→N, N→H, H→L, 2 pauses) | ≈10.3 s | ≈4 s | ≈10.3 s |

- **Telemetry / web:** no change. Debug log only: `[TRANS] Shift <from> -> <to> took <n> ms` when a
  single step or a whole sequence completes.
- **Pure logic** in `include/TransmissionLaw.h` (from `fix-transmission-gear-sync`):
  `ShiftConfirm::step(now, startMs, sensorMatches, confirmMs, timeoutMs)` → `WAIT` / `CONFIRMED` /
  `TIMEOUT`, used for both waits. Tests added to the same native suite `test_transmission_sync`.
- **Unchanged:** overshoot travel and RETURN timing, sequencing R-N-H-L, interlocks, boost PID and
  idle hold logic, init / resync from `fix-transmission-gear-sync`.
- Firmware 1.5.15 (`FIRMWARE_VERSION`) with a changelog bullet (uk + en).

## Impact
- Affected specs: vehicle-actuators (Transmission Servo Control), vehicle-systems (Transmission
  Control System + new Sensor-Confirmed Gear-Change Waits).
- Affected code: `include/TransmissionLaw.h`, `include/TransmissionController.h`,
  `src/TransmissionController.cpp`, `include/Constants.h`, `include/Changelog.h`,
  `test/test_transmission_sync/test_main.cpp`.
- A sensor stuck on the wrong gear or disconnected costs nothing: the timeouts give today's
  behaviour. A sensor that reports the target gear too early (bouncing linkage) is the risk —
  hence the continuous-confirmation window and the 300–400 ms note on `TRANS_DWELL_CONFIRM_MS`.
