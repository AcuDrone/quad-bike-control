# Change: Add PID-Controlled RPM Boost During Gear Changes

## Why
When engaging a gear under load, the engine needs to be at the right RPM for a smooth mechanical engagement. A fixed open-loop throttle percentage overshoots badly. A PID loop targeting a configurable RPM setpoint gives precise, repeatable rev-matching without driver input.

## What Changes
- Replace the existing open-loop `updateThrottleBoost()` (currently disabled via zero constants) with a closed-loop `updateGearBoostPID()` that holds engine RPM at `TRANS_GEAR_BOOST_TARGET_RPM` for the duration of the gear change
- PID gains (`Kp`, `Ki`, `Kd`) and target RPM are tunable constants in `Constants.h`
- CAN RPM poll interval drops to `CAN_POLL_INTERVAL_RPM_BOOST` (e.g. 50 ms) while gear change is active, then reverts to `CAN_POLL_INTERVAL_RPM` (200 ms) when idle
- PID overrides SBUS/web throttle command while active; control returns to normal source immediately after gear change ends or timeout expires
- Safety timeout `TRANS_GEAR_BOOST_TIMEOUT` releases the PID if gear change takes too long

## Impact
- Affected specs: `can-controller` (RPM polling rate), `vehicle-systems` (throttle boost behaviour)
- Affected code:
  - `include/Constants.h` — new PID/RPM/timeout constants, remove dead boost constants
  - `include/CANController.h` / `src/CANController.cpp` — add `setRPMPollInterval(uint32_t ms)`
  - `include/VehicleController.h` — replace boost members with PID state members, rename method
  - `src/VehicleController.cpp` — implement `updateGearBoostPID()`, integrate into `update()`
