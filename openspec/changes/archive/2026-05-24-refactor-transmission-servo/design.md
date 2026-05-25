# Design: refactor-transmission-servo

## 1. Class Hierarchy Change

**Current:** `TransmissionController` extends `BTS7960Controller`.  
**New:** `TransmissionController` is a standalone class that **owns** a `ServoController` member.

Rationale: inheritance was justified when `TransmissionController` needed all of
`BTS7960Controller`'s position-control and speed-control methods. A servo has none of that
complexity — the only operation is "set microseconds". Composition is more appropriate and
eliminates a large inherited surface area that is no longer meaningful.

`VehicleController` currently calls several inherited BTS7960 methods directly
(`stopPositionControl`, `setSpeed`, `stop`, `getPosition`, `recalibrateEncoder`,
`isPositionControlActive`, `moveToPosition`). All of these disappear. The auto-home state
machine in `VehicleController` is also removed (see §5).

## 2. Gear Positions as Float Percent

Gear positions are stored as `float` in the range `[0.0, 100.0]` where:
- `0.0%` → 800 µs (servo minimum)
- `100.0%` → 2200 µs (servo maximum)
- The mapping is linear: `µs = 800 + pct/100.0 * 1400`

**One decimal digit** is enforced in the web UI and telemetry (e.g. `"45.2"`). In C++ the
value is a regular `float`; rounding to one decimal is the UI's responsibility.

**Factory defaults** (compile-time constants):
| Gear | Default % |
|------|----------|
| R    | 10.0     |
| N    | 35.0     |
| L    | 65.0     |
| H    | 90.0     |

These are tunable via web UI and persisted in NVS.

## 3. NVS Key Migration

The existing NVS keys store `int32_t` encoder counts. Reading them as `float` would return
garbage. New keys are introduced:

| Purpose | Old key (int32_t) | New key (float) |
|---------|-------------------|-----------------|
| Reverse default | `def_reverse` | `pct_r` |
| Neutral default | `def_neutral` | `pct_n` |
| Low default | `def_low` | `pct_l` |
| High default | `def_high` | `pct_h` |
| State position | `state_pos` | `state_pct` |
| State gear | `state_gear` | `state_gear` (unchanged, int) |
| State valid | `state_valid` | `state_valid` (unchanged, bool) |

Old keys are never read or written by new code; they remain in NVS inert and are cleaned up on
next NVS clear. No migration step is required.

## 4. Servo Hardware — HappyModel Super400 Plus

| Parameter | Value |
|-----------|-------|
| Model | HappyModel Super400 Plus |
| Torque | 400 kg·cm max, 140 kg·cm rated |
| Speed @ 12V | 0.74 sec / 60° |
| Speed @ 24V | 0.37 sec / 60° |
| Total rotation | 300° (±150°) |
| PWM range | 800 – 2200 µs ✓ |
| Voltage | 12 V – 24 V |

Full 300° range at 12V takes **3 700 ms**. Full range at 24V takes **1 850 ms**.  
A fixed settle constant would either time out too early or add unnecessary delay for short moves.

## 5. Overshoot Phase — Dynamic Settle Time

Since there is no encoder feedback, phase transitions are **time-based using movement distance**.

```
setGear(target)
  → remember currentServoPct_  (last commanded position)
  → compute overshootPct = targetPct ± TRANS_GEAR_OVERSHOOT_PCT (clamped 0–100)
  → settleMs = max(TRANS_SERVO_SETTLE_MIN_MS,
                   (uint32_t)(abs(overshootPct - currentServoPct_) * TRANS_SERVO_MS_PER_PCT))
  → command servo to overshootPct, update currentServoPct_
  → phase = OVERSHOOT, record gearPhaseStartTime_, store settleMs_

update() — OVERSHOOT phase:
  → if millis() - gearPhaseStartTime_ >= settleMs_:
      overshootSettleMs = max(TRANS_SERVO_SETTLE_MIN_MS,
                              (uint32_t)(TRANS_GEAR_OVERSHOOT_PCT * TRANS_SERVO_MS_PER_PCT))
      command servo to finalGearPositionPct, update currentServoPct_
      phase = RETURN, reset gearPhaseStartTime_, settleMs_ = overshootSettleMs

update() — RETURN phase:
  → if millis() - gearPhaseStartTime_ >= settleMs_:
      if getPhysicalGear() == targetGear_:
          phase = NONE, save state
      else if millis() - gearPhaseStartTime_ >= 2 × settleMs_:
          log warning "[TRANS] Gear mismatch after return"
          phase = NONE
```

`TRANS_SERVO_MS_PER_PCT` encodes servo speed per 1% of range:
- Full range = 300°, mapped to 100% (800–2200 µs).
- At 12V: 3 700 ms / 100% = **37 ms/%** → use 40 ms/% (adds ~10% safety margin).
- At 24V: 1 850 ms / 100% = 18.5 ms/% → same constant still works, just waits a bit longer.

`TRANS_SERVO_SETTLE_MIN_MS` = 300 ms (minimum even for very short moves — accounts for
mechanical backlash and gear lash settling).

`TransmissionController` must track `currentServoPct_` (float) — the last position commanded
to the servo — so the distance calculation is always available.

Direction of overshoot is determined from the sign of `targetPct - currentServoPct_`. If the
difference is zero (already at target), no movement is issued and phase stays NONE.

## 6. Auto-Home Removal

Auto-home (`autoHome()`) and the non-blocking auto-home state machine in `VehicleController`
(`updateAutoHome()`, `autoHomeActive_` etc.) are removed entirely. Justification:

- A servo always knows its commanded position — there is no encoder drift.
- On first power-on with no NVS state, the servo is commanded to the neutral default (35%)
  and the system is operational immediately.
- The web UI "Transmission Auto-Home" button is removed.

## 7. `needsThrottleBoost()` Redefinition

Currently returns `gearChangePhase_ != NONE` (based on encoder position control being active).  
New: same condition, still `gearChangePhase_ != NONE`. The semantics are unchanged — boost is
active for the entire duration from `setGear()` call until physical switch confirms target gear.

## 8. `getCurrentGear()` Redefinition

Current implementation compares encoder position against calibrated positions within tolerance.
New implementation: returns `getPhysicalGear()` directly. The physical hall-effect switches are
the authoritative source for gear state; servo position is the command, not confirmation.

## 9. `restoreStateIfValid()` Adaptation

On valid NVS state:
1. Load `state_gear` (int) and `state_pct` (float).
2. Command servo to `state_pct` immediately (no encoder restore needed).
3. Return true immediately — no need to wait for physical switch (servo will reach position within `TRANS_SERVO_MS_PER_PCT × distance` ms; system is usable immediately).

## 10. Constants Removed

These constants become dead code and are deleted:
- `PIN_TRANS_ENCODER_A`, `PIN_TRANS_ENCODER_B`, `PCNT_UNIT_TRANS`
- `PIN_TRANS_RPWM`, `PIN_TRANS_LPWM`, `LEDC_CH_TRANS_RPWM`, `LEDC_CH_TRANS_LPWM`
- `TRANS_ENCODER_MAX_COUNT`, `TRANS_ENCODER_MIN_COUNT`
- `TRANS_POSITION_TOLERANCE` → replaced by `TRANS_SERVO_POSITION_TOLERANCE_PCT`
- `TRANS_GEAR_OVERSHOOT` → replaced by `TRANS_GEAR_OVERSHOOT_PCT`
- `TRANS_HOMING_SPEED`, `TRANS_HOMING_TIMEOUT`, `TRANS_STALL_THRESHOLD`, `TRANS_STALL_TIMEOUT`

## 11. Constants Added

```c
#define PIN_TRANS_SERVO                     GPIO_NUM_9
#define LEDC_CH_TRANS_SERVO                 2           // freed from RPWM
#define TRANS_SERVO_MIN_US                  800
#define TRANS_SERVO_MAX_US                  2200
#define TRANS_SERVO_MS_PER_PCT              12          // ms per 1% travel @ 24V, 180° range (HappyModel Super400 Plus: 0.37s/60°)
#define TRANS_SERVO_SETTLE_MIN_MS           300         // minimum settle time regardless of distance
#define TRANS_SERVO_POSITION_TOLERANCE_PCT  1.0f        // % — tolerance for "at gear" check
#define TRANS_GEAR_OVERSHOOT_PCT            3.0f        // % past target (overshoot phase)
#define TRANS_GEAR_DEFAULT_REVERSE_PCT      10.0f
#define TRANS_GEAR_DEFAULT_NEUTRAL_PCT      35.0f
#define TRANS_GEAR_DEFAULT_LOW_PCT          65.0f
#define TRANS_GEAR_DEFAULT_HIGH_PCT         90.0f
```

`TRANS_SERVO_MS_PER_PCT` is the dominant tuning knob for servo timing. System runs at 24V with
180° configured travel: 180°/60° × 370ms = 1110ms for full range → 11.1 ms/% → 12 ms/%
(~8% safety margin). Worst-case R→H = 80% = 960 ms. If gear changes time out or the physical
switch is not yet active when checked, increase this value.
