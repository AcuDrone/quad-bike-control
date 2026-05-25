# Tasks: refactor-transmission-servo

## Ordered Work Items

- [x] 1. **Constants.h** — Remove BTS7960/encoder transmission constants; add servo pin, channel, µs range, `TRANS_SERVO_MS_PER_PCT` (20 @ 24V), `TRANS_SERVO_SETTLE_MIN_MS` (300), overshoot %, tolerance %, and four default gear percent constants. (See design.md §10–11.)

- [x] 2. **TransmissionController.h** — Drop `BTS7960Controller` inheritance; add `ServoController servo_` member; change `gearPositions_[4]` from `int32_t` to `float`; change `finalGearPosition_` from `int32_t` to `float`; add `float currentServoPct_` (last commanded servo position), `uint32_t gearPhaseStartTime_`, `uint32_t settleMs_` members; update signatures: `getGearPosition()` returns `float`, `setDefaultPosition()` takes `float positionPct`; remove methods that delegated to BTS7960 (`moveToPosition`, `stopPositionControl`, `getPosition`, `recalibrateEncoder`, `isPositionControlActive`, `autoHome`, `setSpeed`, `stop`); keep `needsThrottleBoost()`, `getCurrentGear()`, `isAtGear()`, `setGear()`, `getTargetGear()`, `getPhysicalGear()`, `isGearPositionValid()`, `update()`, `restoreStateIfValid()`, `loadDefaultPositions()`, `saveState()`, `loadState()`.

- [x] 3. **TransmissionController.cpp** — Rewrite implementation:
  - `begin()`: initialise `servo_` with `PIN_TRANS_SERVO`, `LEDC_CH_TRANS_SERVO`, `TRANS_SERVO_MIN_US`, `TRANS_SERVO_MAX_US`; move to neutral default; call `initGearSensors()`.
  - `setGear()`: compute `overshootPct`, command servo to overshoot, set `gearChangePhase_ = OVERSHOOT`, record `gearPhaseStartTime_`, save `state_valid=false` to NVS.
  - `update()`: time-based phase transitions (OVERSHOOT→RETURN→NONE) per design.md §4; physical switch confirmation on RETURN completion; save state on NONE transition.
  - `getCurrentGear()`: return `getPhysicalGear()`.
  - `isAtGear()`: return `getPhysicalGear() == gear`.
  - `getGearPosition()`: return `gearPositions_[(int)gear]`.
  - `setDefaultPosition()`: validate 0.0–100.0; save to NVS using `pct_*` keys; update array.
  - `loadDefaultPositions()`: load float `pct_r/n/l/h` keys; fall back to `TRANS_GEAR_DEFAULT_*_PCT` constants.
  - `saveState()` / `loadState()`: use `state_pct` float key instead of `state_pos` int key.
  - `restoreStateIfValid()`: load float position, command servo, no encoder restore.
  - Helper `pctToUs(float pct)`: maps 0–100 → `TRANS_SERVO_MIN_US`–`TRANS_SERVO_MAX_US`.

- [x] 4. **main.cpp** — Remove: `transmissionEncoder` declaration and `begin()` call, `transmissionActuator.begin(PIN_TRANS_RPWM, ...)`, `transmissionActuator.attachEncoder(...)`, `transmissionActuator.stop()` (BTS7960 stop). Replace with: `transmissionActuator.begin()` (new servo-based begin, no arguments needed if pin is in Constants.h). Remove `autoHome` call or guard that referenced it.

- [x] 5. **VehicleController.h** — Remove: `autoHomeActive_`, `autoHomeStartTime_`, `autoHomeLastChangeTime_`, `autoHomeLastPosition_` members; `updateAutoHome()` method declaration.

- [x] 6. **VehicleController.cpp** — Remove: `updateAutoHome()` implementation; `autoHome*` init in constructor; `updateAutoHome()` call in `update()`; `"auto_home"` command handler in `processWebCommand()`; `!autoHomeActive_` guard in `processSBusCommands()`; `autoHomeActive_` guard in `processGearCommand()`.

- [x] 7. **WebPortal.h** — Change `gear_default_r`, `gear_default_n`, `gear_default_l`, `gear_default_h` telemetry fields from `int32_t` to `float`.

- [x] 8. **WebPortal.cpp** — In `createTelemetryJSON()`, serialise the four gear-default fields with one decimal precision (e.g., `doc["gear_default_r"] = roundf(telemetry.gear_default_r * 10.0f) / 10.0f`).

- [x] 9. **data/index.html** — Gear Position Defaults card: change inputs to accept float `0.0`–`100.0`; display with one decimal; update `saveGearDefault()` JS to parse float and validate range; update label/placeholder to show "%" units; remove "Transmission Auto-Home" button.

- [x] 10. **Build validation** — Compile with PlatformIO; confirm zero errors and zero relevant warnings; confirm `EncoderCounter` for transmission is no longer instantiated.

- [x] 11. **Hardware validation** — Flash to device; confirm:
  - Servo moves to each gear on web command.
  - Physical gear switches confirm each gear.
  - Overshoot-and-return motion visible (brief over-travel before settling).
  - NVS gear default saved and restored across reboot.
  - Throttle boost PID activates during gear change and deactivates on completion.

## Dependencies

Tasks 1–2 are prerequisites for all others.  
Tasks 3–6 are independent and can be worked in parallel once 1–2 are done.  
Tasks 7–9 are independent of 3–6.  
Task 10 requires all of 1–9.  
Task 11 requires task 10.
