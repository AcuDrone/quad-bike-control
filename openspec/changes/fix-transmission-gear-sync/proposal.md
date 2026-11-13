# Change: Reconcile the expected gear with the physical gear sensor

## Why
On 2026-11-05 the firmware believed the gearbox was in N while the gear sensor showed L. The
expected gear (`TransmissionController::targetGear_`) is never reconciled with the sensor:
- the constructor starts `targetGear_` at NEUTRAL;
- `restoreStateIfValid()` runs once on the first `isEngineRunning()` and, without a valid NVS
  state, commands the servo blindly to the N position while the gearbox may sit in any gear (the
  follow-up `setGear(NEUTRAL)` in `VehicleController::update()` is a no-op: target is already N);
- a web `set_gear` before the engine runs starts a shift from `targetGear_` = N and from a servo
  that has never had a pulse (`servo_.begin(..., 0)`, so `getCurrentServoPct()` ≈ −57 %);
- the idle mismatch check (`TRANS_GEAR_CHECK_INTERVAL`, 500 ms) only logs "MISMATCH".

## What Changes
- **Not-initialised state.** `targetGear_` and `fromGear_` start as `GEAR_UNKNOWN` instead of
  NEUTRAL; `targetGear_ == GEAR_UNKNOWN` *is* the "not initialised" state (no separate flag that
  could disagree with it — `startGearChange()` never writes UNKNOWN, so it can only mean "never
  initialised"). New `isInitialised()`. It lasts only until the first initialisation (below),
  which always ends in a known gear. Until then:
  - `setGear()` returns false and does nothing (no servo command, no NVS write, queue untouched).
    This replaces the "route through NEUTRAL when the current gear is unknown" rule;
  - `shouldClipThrottle()` also returns true for a `GEAR_UNKNOWN` target, so throttle stays capped
    at `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (8.5 %);
  - the transmission servo keeps no pulse;
  - `getTargetGearString()` / `getFromGearString()` return `"?"` instead of falling back to `"N"`.
- **Initialisation `TransmissionController::initialise()`** replaces `restoreStateIfValid()`. It
  runs once and picks, in this order:
  1. the sensor (`getPhysicalGear()`) reports exactly one gear → `targetGear_` = that gear, servo
     commanded straight to that gear's configured position `gearPositions_[gear]` (direct move, no
     overshoot, as today's restore), state saved valid; log `[TRANS] Initialised from sensor: L`;
  2. the sensor reports `GEAR_UNKNOWN` → the NVS state if `state_valid`, `state_gear` is a real gear
     and `state_pct` is 0–100 → servo to `state_pct`, `targetGear_` = `state_gear` (as today); log
     `[TRANS] Initialised from NVS: L at 68.0%`;
  3. neither (sensor UNKNOWN, NVS missing or invalid) → **a real shift step into NEUTRAL**
     (overshoot → dwell → return through the normal state machine, so the boost PID / shift idle
     hold run as for any step); log `[TRANS] Init: sensor unknown, stepping to NEUTRAL`.
     - The servo position is unknown, so the overshoot move uses a fixed settle
       `TRANS_INIT_SETTLE_MS` = 1500 ms (new constant: the farthest default gear from N is 20 %,
       + 1 % overshoot ≈ 0.75 s at 35 ms/%, doubled) instead of `computeSettleMs()` from the bogus
       ≈ −57 %. The RETURN move uses `computeSettleMs()` from the overshoot as usual.
     - Overshoot direction comes from the invalid NVS record as a hint: the interrupted step saved
       `state_gear` (its target) and `state_pct` (servo position at its start), so the lever lies
       between them. Direction = sign(N position − `state_pct`) when `state_pct` is within 0–100;
       if `state_pct` equals the N position (the interrupted step started at N), sign(N position −
       position of `state_gear`); no record, a bogus `state_pct` or a tie with no usable gear →
       +1 (toward H).
     - The step marks the state invalid without overwriting the stored `state_pct`, so a power
       loss during the init step leaves the same hint for the next boot.
     - `fromGear_` stays `GEAR_UNKNOWN` for this step; `targetGear_` = NEUTRAL from its start (so
       the controller counts as initialised and a shift is active).
     - On completion: sensor reads N → normal completion (state saved valid), log
       `[TRANS] Gear NEUTRAL engaged ...`; sensor still UNKNOWN → NEUTRAL accepted as the working
       assumption (as today), state saved valid, log `[TRANS] Init: sensor silent, assuming
       NEUTRAL`; the throttle stays capped at 8.5 % by the NEUTRAL target and the idle resync
       corrects the gear later if the sensor comes back.
  No blind direct jump to N and no permanent "waiting for the sensor" state any more.
- **Where it runs.** `VehicleController::update()` calls `initialise()` once on the first
  `isEngineRunning()` while `!transmission_.isInitialised()` (replaces the
  `transmissionInitialized_` block and the `setGear(NEUTRAL)` fallback; the field is removed).
- **Web `set_gear` before the engine runs stays allowed** (needed for calibration with the engine
  off). `processGearCommand()` first calls `initialise()` if not initialised, then `setGear()` as
  today. After cases 1/2 the shift starts from the real servo position and the real `targetGear_`.
  During the case-3 init step `setGear()` behaves as during any active step — verified in the
  current code: it does not refuse; it queues the destination (`queuedGear_`) and returns true, so
  the web reply is the usual `"Moving to <gear> gear"` and the queued gear is shifted to after the
  init step (requesting N while the init step runs is idempotent). MAVLink keeps its
  `isEngineRunning()` gate and the `lastCommandedGear_` retry loop. The "Gear unknown, calibrate
  gear positions first" reply is not needed: the init step replaces it.
- **Post-step resync.** In the idle mismatch check (phase NONE, no queued sequence), when the
  sensor reports one gear different from `targetGear_` on `TRANS_RESYNC_CONFIRM_CHECKS` (2)
  consecutive checks (≈1 s) — the same gear both times — `targetGear_` becomes the sensor gear and
  `saveState(sensorGear, getCurrentServoPct(), true)` is written; log
  `[TRANS] Resynced from sensor: target=N -> L`. The servo is NOT moved. The first-detection
  `MISMATCH` log stays. Resync runs only when initialised.
- **Unchanged:** overshoot / dwell / return phases, sequencing R-N-H-L, speed interlock, boost and
  shift idle hold, gear position and overshoot calibration, MAVLink reporting format (before
  initialisation the assumed gear `"?"` keeps the existing `encodeGear()` fallback of 0.0; the
  physical gear field already reports the truth).
- **Pure logic** in new header-only `include/TransmissionLaw.h` (no Arduino / `Constants.h`):
  `chooseTransmissionInit(sensorGear, gearPositions[4], nvsFound, nvsValid, nvsGear, nvsPct)` →
  `{source SENSOR | NVS | STEP_TO_NEUTRAL, gear, pct, direction}` (NVS plausibility checks and the
  overshoot-direction hint), and `TransmissionResync::step()` (consecutive-mismatch counter).
  Native suite `test/test_transmission_sync`, registered in `platformio.ini` `[env:native]`
  `test_filter`.
- Firmware 1.5.14 (`FIRMWARE_VERSION`) with a changelog bullet (uk + en).

## Impact
- Affected specs: vehicle-systems (Transmission State Persistence + new Transmission Gear Sync
  With the Physical Sensor), vehicle-actuators (Transmission Servo Control), web-control (new Web
  Gear Selection requirement).
- Affected code: `include/TransmissionLaw.h` (new), `include/TransmissionController.h`,
  `src/TransmissionController.cpp`, `include/VehicleController.h`, `src/VehicleController.cpp`,
  `include/Constants.h`, `include/Changelog.h`, `platformio.ini`,
  `test/test_transmission_sync/test_main.cpp` (new).
- No NVS migration: the `"transmission"` keys are unchanged.
- The init step's dwell is the normal overshoot dwell; `speed-up-gear-change-with-sensor` shortens
  it like any other step.
- `speed-up-gear-change-with-sensor` (1.5.15) builds on this change and modifies the same
  Transmission Servo Control requirement; this change is implemented and archived first.
