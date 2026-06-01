# Tasks: use-time-based-gear-confirmation

- [x] 1. Remove PULLBACK and PULLBACK_DWELL from `GearChangePhase` enum in `TransmissionController.h`
- [x] 2. Remove `overshootRetryCount_`, `pullbackPcts_[4]` fields from `TransmissionController.h`
- [x] 3. Remove `getGearPullback()`, `setGearPullback()`, `loadGearPullbacks()` declarations from `TransmissionController.h`
- [x] 4. Delete `TRANS_GEAR_PULLBACK_*_PCT`, `TRANS_ROLLBACK_DWELL_MS` constants from `Constants.h`
- [x] 5. Update `OVERSHOOT_DWELL` phase in `update()`: remove sensor early-exit; after timeout always transition to RETURN
- [x] 6. Remove PULLBACK and PULLBACK_DWELL phase handlers from `update()`
- [x] 7. Update NONE-phase sequence advancement to use `targetGear_` instead of `effectiveGear` (sensor fallback)
- [x] 8. Remove `overshootRetryCount_` initialisation and usage from constructor and `startGearChange()`
- [x] 9. Remove `pullbackPcts_` initialisation from constructor body
- [x] 10. Delete `getGearPullback()`, `setGearPullback()`, `loadGearPullbacks()` implementations from `TransmissionController.cpp`
- [x] 11. Remove `loadGearPullbacks()` call from `main.cpp` setup
- [x] 12. Build and verify no compile errors; confirm PULLBACK log lines no longer appear
