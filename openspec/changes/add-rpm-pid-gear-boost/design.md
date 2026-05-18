# Design: PID-Controlled RPM Boost During Gear Changes

## Context
The transmission is a mechanical gear selector driven by a linear actuator with an encoder. When `setGear()` is called the actuator moves toward the target encoder position. For smooth mechanical gear engagement (especially L/H under load) the engine should be at a specific RPM when the gear engages. A simple fixed throttle boost causes RPM overshoot; a PID loop avoids this.

## Goals / Non-Goals
- **Goals**: closed-loop RPM hold during gear transition; faster CAN RPM feedback; clean handoff back to driver after gear engages
- **Non-Goals**: gear-specific RPM targets (single setpoint for all gears); full auto rev-match on approach to gear (that needs upshift/downshift awareness)

## Decisions

### PID lives in VehicleController
`VehicleController` already owns both `canController_` (RPM source) and `throttle_` (output). Adding PID state there avoids coupling `TransmissionController` to throttle or CAN.

### Trigger: `transmission_.needsThrottleBoost()`
`needsThrottleBoost()` returns `true` while `!isStopped() && targetGear_ != GEAR_NEUTRAL`. This is the existing activation predicate — no new API needed on `TransmissionController`.

### PID output → throttle angle (degrees)
Output is clamped to `[THROTTLE_MIN_ANGLE, THROTTLE_MAX_ANGLE]`. Integral wind-up is clamped to ±20% of full throttle range to prevent accumulation during stale CAN conditions.

### CAN polling rate change
`CANController` exposes `setRPMPollInterval(uint32_t ms)`. `VehicleController` calls it on PID activation (`CAN_POLL_INTERVAL_RPM_BOOST`) and deactivation (`CAN_POLL_INTERVAL_RPM`). This keeps the fast-poll concern isolated to the call site, not baked into `CANController` logic.

### Safety fallback
If CAN data is invalid (stale/disconnected) the PID output is frozen at the last valid value. If the boost exceeds `TRANS_GEAR_BOOST_TIMEOUT` the PID is forcibly deactivated and throttle returns to SBUS/web command.

## Risks / Trade-offs
- **PID tuning**: defaults (`Kp=0.05`, `Ki=0.01`, `Kd=0.005`) are a starting point; hardware tuning required. All gains are compile-time constants in `Constants.h`, so re-flash needed for tuning iterations.
- **SBUS override**: while PID is active, driver throttle is ignored. If gear engagement hangs, the `TRANS_GEAR_BOOST_TIMEOUT` is the escape hatch.
- **Faster CAN polling**: 50 ms instead of 200 ms adds ~4× CAN bus load during shifts, which are brief (<2 s typical). Acceptable.

## Open Questions
- None — user confirmed: PID overrides SBUS; faster poll only during gear change.
