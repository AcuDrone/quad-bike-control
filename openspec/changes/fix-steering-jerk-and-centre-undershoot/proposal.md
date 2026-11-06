# Change: Steering — no stop-go jerks on a move, park on centre instead of 0.3–0.4° short

## Why
USB steering diagnostics on the quad (2026-11-05) show two faults of the closed-loop position law
(`include/SteerPositionLaw.h`, `duty = clamp(|err| × STEER_KP + boost, floor, 255)`,
`STEER_KP` 1.15 per 0.1°, floor `STEER_PWM_MIN_DUTY` 80 from rest / after a reversal until 0.3° of
progress, then `STEER_PWM_TRACK_MIN_DUTY` 70; boost every `STEER_BOOST_CHECK_MS` 100 ms:
progress < 0.3° → `+STEER_BOOST_STEP` 30, else −30 down to 0):

1. **Jerks on a 10° move.** Below ≈ 6° of error the P term (6° × 11.5 = 69) falls under the
   tracking floor, so the last ≈ 6° run at a flat duty 70. Away from centre the tyre scrub raises
   the running friction above 70: the wheel stops. The first boost step is hidden under the floor
   (P ≈ 30 + boost 30 = 60 < 70), so the wheel stands for 200–300 ms; at boost 60 the duty jumps to
   ≈ 90–100, the wheel jumps 0.5–1°, the very next check sees progress and removes the whole 30, the
   duty falls back to 70 and the wheel stops again. Logs: 5–6 stop/go plateaus per 10° move,
   17–27 A drawn while standing at duty 70, 2.0–2.5 s per move; the plateaus broke free at duty
   89–107. The return to centre at duty 70 is smooth (less scrub near centre).
2. **Centre undershoot.** The loop parks as soon as `|err| ≤ STEER_POSITION_TOLERANCE` (4 = 0.4°)
   and the 1:40 self-locking worm stops the shaft at once (coast ≈ 0), so the wheel systematically
   rests 0.3–0.4° short of the target on the approach side. The park correction
   (`add-steering-park-correction`, `SteerParkLaw`) fires only when `|err| > 0.4°`, so this residual
   sits in its blind zone. The operator sees "always 0.3–0.4° short, the side depends on where it
   came from".

## What Changes
- **Boost on top of the floor** (`SteerPositionLaw.h`): `duty = clamp(max(|err| × STEER_KP, floor)
  + boost, floor, STEER_PWM_MAX_DUTY)`, signed toward the target. The first stalled check
  (+30) already raises the duty above the floor; no boost step is ever hidden.
- **Gentle boost decay**: per check, stalled (progress < `STEER_BOOST_MIN_PROGRESS`) →
  `+STEER_BOOST_STEP` (30, unchanged); moving → `−STEER_BOOST_DECAY` (new constant, 10), never
  below 0. The boost that broke the wheel free is shed over ≈ 300 ms instead of in one check, so
  the duty does not drop straight back to the floor that just stalled. Boost still resets to 0 on a
  start from rest and on a direction flip; the boost cap (`< STEER_PWM_MAX_DUTY`) is unchanged.
- **Boost hold band** (`STEER_BOOST_HOLD_ERR` 2.0°): while moving and `|err| > 2.0°` the boost is held, it decays only inside the band (`STEER_BOOST_DECAY` 30 there) — a loaded wheel (35–44 A stalls, bench 2026-11-06) otherwise stalls again once the boost decays (design.md D7).
- **Quad duty floors**: `STEER_PWM_TRACK_MIN_DUTY` 70 → 90 (bench-tune; plateaus broke free at
  89–107), `STEER_PWM_MIN_DUTY` 80 → 90 so the break-away floor is not below the tracking floor.
  `STEER_PWM_MIN_DUTY` is also the jog / nudge clamp and the rate-drive floor, so a web jog at
  < 90 duty now runs at 90 on the quad (see design.md D3).
- **Quad park window**: `STEER_POSITION_TOLERANCE` 4 → 2 (0.2°). The restart hysteresis
  `STEER_RESTART_TOLERANCE` stays 10 (1.0°).
- **Park correction trigger unchanged at 0.4°**: today it is `max(STEER_POSITION_TOLERANCE, done
  tolerance)`; to keep it at 0.4° when the park window shrinks, the law's trigger gets its own
  constant `STEER_PARK_TRIGGER_TOL` 4 (0.4°), so `max(0.4°, done tolerance)` as before. With the
  loop now parking within 0.2°, the 0.2–0.4° blind zone disappears from the approach itself.
- **Unchanged**: continuous tracking, stop hysteresis states, stall-stop and latch, move timeout,
  range guard, park correction logic and settings, interlocks, sensorless rate drive law, web UI,
  telemetry.
- **Native tests** (`test/test_steer_position`): boost raises the duty above the floor on the first
  stalled check; boost decays by 10 per moving check and never below 0; parks at ≤ 0.2° and not at
  0.3°; a parked loop restarts only beyond 1.0°. The test parameter block mirrors the new
  constants (it still carries the stale 10 / 30 / 140 / 100 values of 1.5.5).
- **Firmware 1.5.20** (patch: behaviour fix, no recalibration, no wiring change), changelog bullet
  uk + en.

## Impact
- Affected specs: `vehicle-actuators` — MODIFIED "Steering Break-away and Tracking Duty Floors",
  MODIFIED "Steering Stop Hysteresis". Both are ADDED by the not-yet-archived
  `fix-steering-continuous-tracking`; archive this change after it (and after
  `add-steering-park-correction`, whose trigger rule is restated here). The modified texts also
  bring the numbers in line with the firmware (the pending text still says 140 / 100 and
  1.0° / 3.0°).
- Affected code: `include/SteerPositionLaw.h` (law, `SteerPosParams.boostDecay`, header comment),
  `src/SteeringController.cpp` (`POS_PARAMS`, `PARK_DEFAULTS.triggerTol`), `include/Constants.h`
  (values + comments, `STEER_BOOST_DECAY`, `STEER_PARK_TRIGGER_TOL`, `FIRMWARE_VERSION`),
  `include/SteeringController.h` (doc comments), `include/Changelog.h`,
  `test/test_steer_position/test_main.cpp`.
- Behaviour: a 10° move should run without plateaus and faster; the wheel should park within 0.2°
  of centre from both sides. Risk: at duty 90 into a 0.2° window the wheel may overshoot or hunt —
  mitigated by the bench-tune of the tracking floor (step down 90 → 85 → 80) and, if needed,
  widening the window back to 0.3°; the 1.0° restart band means an overshoot below 1.0° is left to
  the park correction, never a restart loop.
- Aztec (rack steering): **unaffected until the next `master` → `aztec` merge**. The merge takes the
  law change (boost on top, decay 10) but MUST keep aztec's own `STEER_PWM_MIN_DUTY` /
  `STEER_PWM_TRACK_MIN_DUTY` 30 / 30 and `STEER_POSITION_TOLERANCE` 4 (0.4°) — see design.md.
