---
id: add-gear-overshoot-engagement
title: Add Gear Overshoot Engagement
status: proposed
type: enhancement
created: 2026-05-14
---

## Summary

Change the transmission gear-change motion profile from a single move to a two-phase
**overshoot-and-return** pattern. The actuator first drives past the target gear position
by a configurable encoder-count offset, then returns to the exact target. This ensures
the mechanical detent is fully engaged before the actuator settles.

## Problem

The current `setGear()` moves directly to the stored gear position and stops.
On the physical quad bike transmission, a direct stop at the target position sometimes
leaves the shift mechanism partially engaged — the detent has not snapped fully into the
gear slot. The physical switch may read the gear as selected while the mechanical engagement
is marginal, causing the gear to disengage under load.

## Proposed Solution

**Phase 1 — overshoot:** when `setGear(gear)` is called, compute:

```
overshootOffset = sign(targetPosition − currentPosition) × TRANS_GEAR_OVERSHOOT
overshootPosition = targetPosition + overshootOffset
```

Move the actuator to `overshootPosition`.

**Phase 2 — return:** once the actuator reaches `overshootPosition`, move back to
`gearPositions_[targetGear]` (the stored default position). This is the final resting
position.

Physical-switch confirmation and `saveState()` happen only after Phase 2 completes,
unchanged from the current flow.

## Scope

- `include/Constants.h` — new constant `TRANS_GEAR_OVERSHOOT`
- `include/TransmissionController.h` — new private state (`gearChangePhase_`,
  `finalGearPosition_`)
- `src/TransmissionController.cpp` — `setGear()` starts Phase 1; `update()` detects
  Phase 1 completion and launches Phase 2
- No changes to `BTS7960Controller`, web portal, or telemetry

## Out of Scope

- Per-gear overshoot tuning (single global constant is sufficient)
- Applying overshoot to `moveToPosition()` called directly (e.g., from web Move button)
- Changes to physical-switch confirmation logic or `saveState()` timing

## Constraints

- `TRANS_GEAR_OVERSHOOT` must default to a value that cannot push the actuator past the
  physical end-stop. A safe default is 30–50 encoder counts.
- Phase 1 must be skipped if the actuator is already within `TRANS_POSITION_TOLERANCE`
  of the target (no movement needed, no overshoot).
- Direction is determined from `targetPosition − currentPosition` at the moment
  `setGear()` is called.
