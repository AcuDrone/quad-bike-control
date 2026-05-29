# Tasks: update-transmission-shift-logic

## Status: complete

- [x] 1. **Constants.h** — add `TRANS_OVERSHOOT_DWELL_MS` (2000) and `TRANS_ROLLBACK_DWELL_MS` (1000)
- [x] 2. **TransmissionController.h** — add `OVERSHOOT_DWELL` and `PULLBACK_DWELL` to `GearChangePhase` enum
- [x] 3. **TransmissionController.h** — add `Gear queuedGear_` member and `startGearChange(Gear)` private method declaration
- [x] 4. **TransmissionController.cpp** — extract `startGearChange(Gear)` from `setGear()` body
- [x] 5. **TransmissionController.cpp** — update `setGear()` to compute sequence step, set `queuedGear_`, call `startGearChange()`
- [x] 6. **TransmissionController.cpp** — add `OVERSHOOT` → `OVERSHOOT_DWELL` transition (replace direct → RETURN)
- [x] 7. **TransmissionController.cpp** — implement `OVERSHOOT_DWELL` phase: poll switch, early-exit to RETURN, timeout to PULLBACK
- [x] 8. **TransmissionController.cpp** — update `RETURN` phase: remove switch check, transition to NONE on settle (save state)
- [x] 9. **TransmissionController.cpp** — update `PULLBACK` phase: transition to `PULLBACK_DWELL` on settle (was: direct retry OVERSHOOT)
- [x] 10. **TransmissionController.cpp** — implement `PULLBACK_DWELL` phase: wait `TRANS_ROLLBACK_DWELL_MS`, then retry OVERSHOOT or give up
- [x] 11. **TransmissionController.cpp** — update `NONE` phase in `update()`: advance gear queue when `queuedGear_ != GEAR_UNKNOWN`
- [x] 12. **TransmissionController.cpp** — update status log string to include `OVERSHOOT_DWELL` and `PULLBACK_DWELL` phase names
- [x] 13. **TransmissionController.cpp** — initialise `queuedGear_` in constructor (`GEAR_UNKNOWN`)
- [x] 14. **Build** — verify project compiles without errors (`pio run`)
