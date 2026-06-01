# Proposal: use-time-based-gear-confirmation

## Summary
The physical gear-position hall-effect sensors are unreliable in practice. The current
`OVERSHOOT_DWELL` phase polls the sensor for an early-exit and falls back to a pullback/retry
cycle when the sensor does not confirm. Because the sensor cannot be trusted, this retry
cycle never converges reliably. The fix is to remove all sensor-gated logic from the gear-change
state machine and rely solely on `TRANS_OVERSHOOT_DWELL_MS` as the confirmation signal:
after dwelling at the overshoot position for the configured time, the gear is assumed engaged
and the servo returns to the final position.

## Problem
- Hall-effect sensors intermittently read UNKNOWN or the wrong gear while the lever is engaged.
- The `OVERSHOOT_DWELL` early-exit on switch confirmation never fires reliably, so the full
  dwell time always elapses.
- The PULLBACK+retry path compounds the problem: it moves the servo away from the detent,
  then back, adding latency without improving engagement.
- Sequence advancement in the NONE phase uses `effectiveGear` (sensor or `targetGear_`
  fallback), which is noisy post-RETURN.

## Proposed Change
1. **OVERSHOOT_DWELL** — remove the sensor early-exit check. After `TRANS_OVERSHOOT_DWELL_MS`
   elapses, always transition directly to RETURN and mark the gear assumed-engaged.
2. **Remove PULLBACK and PULLBACK_DWELL phases** — the retry path is eliminated entirely.
3. **NONE-phase sequence advancement** — use `targetGear_` (the last assumed-engaged gear)
   instead of the physical sensor reading to determine the current gear for next-step
   computation. The sensor is kept for telemetry and idle mismatch warnings only.

## Out of Scope
- Removing `getPhysicalGear()` — it is still used for telemetry, status display, and
  idle mismatch detection.
- Changing `TRANS_OVERSHOOT_DWELL_MS` value — tuning is a separate concern.
- Changing the overshoot motion profile (direction, per-gear overshoot percentages).

## Acceptance Criteria
- A gear change completes in exactly OVERSHOOT settle + TRANS_OVERSHOOT_DWELL_MS + RETURN
  settle, regardless of sensor state.
- No PULLBACK or PULLBACK_DWELL log lines are emitted.
- Multi-step sequences (e.g. R→L) advance using `targetGear_`, not the sensor.
- Pullback constants, NVS keys, and related public API are removed.
