---
id: update-transmission-shift-logic
title: Update Transmission Shift Logic
status: proposed
type: enhancement
created: 2026-05-25
---

## Summary

Two improvements to `TransmissionController`:

1. **Sequential gear shifting** — enforce the physical gear order `R → N → H → L`.
   When a target gear is not adjacent to the current gear, the controller automatically
   steps through the intermediate gears in sequence.

2. **Overshoot dwell & rollback dwell** — replace the current immediate RETURN transition
   with a configurable dwell at the overshoot position (`TRANS_OVERSHOOT_DWELL_MS`), and
   add a configurable dwell after pullback (`TRANS_ROLLBACK_DWELL_MS`) before retrying.

## Why

### Sequential gear shifting
The physical transmission is a sequential gearbox — gears can only be engaged one step at
a time. Commanding a jump from REVERSE to LOW without passing through NEUTRAL and HIGH
risks mechanical damage (detent ratchet skipped or jammed). The sequence must be enforced
in firmware.

### Overshoot dwell
The current OVERSHOOT phase transitions to RETURN as soon as servo movement time elapses.
In practice, the detent mechanism may need additional time to snap in after the servo
arrives. Waiting up to 2 s at the overshoot position gives the detent time to engage
before the servo returns, reducing the retry rate. Early exit (as soon as the switch
fires) keeps the gear change fast on easy shifts.

### Rollback dwell
The current PULLBACK phase moves the servo immediately and proceeds to retry OVERSHOOT as
soon as movement completes. A short dwell at the pullback position gives the detent time
to fully disengage before the next overshoot attempt.

## What Changes

### Constants.h
- Add `TRANS_OVERSHOOT_DWELL_MS 2000` — max wait at overshoot position
- Add `TRANS_ROLLBACK_DWELL_MS 1000`  — wait after pullback before retry

### TransmissionController.h
- Expand `GearChangePhase` enum: add `OVERSHOOT_DWELL` and `PULLBACK_DWELL`
- Add `Gear queuedGear_` member — `GEAR_UNKNOWN` = no sequence pending, else = final destination gear
- Add private `startGearChange(Gear)` — extracted overshoot-initiation logic shared by `setGear()` and the sequence-advance path in `update()`

### TransmissionController.cpp
**`setGear(Gear finalGear)`**
- Compute the immediate next step toward `finalGear` from `getPhysicalGear()` using the
  static sequence table `[R, N, H, L]`
- If physical gear is UNKNOWN, route through NEUTRAL first
- Set `queuedGear_` to `finalGear`; if already adjacent, `queuedGear_ = GEAR_UNKNOWN`
- Call `startGearChange(nextStep)` to begin motion

**`update()` — NONE phase**
- When `gearChangePhase_ == NONE` and `queuedGear_ != GEAR_UNKNOWN`:
  - Read `physicalGear`; if `== queuedGear_`, clear queue and finish
  - If `physicalGear == GEAR_UNKNOWN`, abort queue with warning
  - Otherwise call `startGearChange(nextStep)` toward `queuedGear_`

**Phase state machine** (new flow):
```
OVERSHOOT (servo moving)
  → settle time elapsed → OVERSHOOT_DWELL

OVERSHOOT_DWELL (waiting at overshoot, max TRANS_OVERSHOOT_DWELL_MS)
  → switch confirms gear early → RETURN immediately
  → TRANS_OVERSHOOT_DWELL_MS elapsed, switch confirms → RETURN
  → TRANS_OVERSHOOT_DWELL_MS elapsed, switch NOT confirmed → PULLBACK

RETURN (servo moving back to target)
  → settle time elapsed → NONE  (gear already confirmed; save state)

PULLBACK (servo moving away from target)
  → settle time elapsed → PULLBACK_DWELL

PULLBACK_DWELL (waiting TRANS_ROLLBACK_DWELL_MS)
  → elapsed → increment overshootRetryCount_, retry OVERSHOOT with 1.5×/2.0×
  → retryCount >= 100 → NONE (give up, log warning)
```

Note: the switch check is removed from the RETURN phase — confirmation now happens exclusively in OVERSHOOT_DWELL.

## Out of Scope
- Per-gear dwell values (single global constant per dwell type is sufficient)
- Exposing queue state in web telemetry (existing `targetGear` field covers it)
- Changes to `canChangeGear()`, speed interlock, or CAN integration
