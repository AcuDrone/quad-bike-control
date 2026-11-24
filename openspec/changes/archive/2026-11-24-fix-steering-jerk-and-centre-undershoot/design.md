# Design: Steering — boost on top of the floor, gentle decay, 0.2° park window

## Context
- Drive train: brushed motor → 1:40 self-locking worm → steering shaft; Bosch LWS (0.1°) on the
  shaft. Coast after the duty is cut ≈ 0: the worm stops the shaft at once.
- Law today (`include/SteerPositionLaw.h`, 1.5.12 values on the quad):
  `duty = clamp(|err| × 1.15 + boost, floor, 255)`, floor 80 (break-away, until 0.3° of progress)
  then 70 (tracking); boost per 100 ms: progress < 0.3° → +30, else −30, ≥ 0. Park at
  `|err| ≤ 4` (0.4°), restart beyond 10 (1.0°).
- Measured 2026-11-05 on the quad (USB diag, `goto ±10`): 5–6 plateaus per 10° move, 17–27 A while
  standing at duty 70, 2.0–2.5 s per move; each plateau broke free at duty 89–107; parked 0.3–0.4°
  short on the approach side; return to centre at duty 70 smooth.

## Goals / Non-Goals
- Goals: no stop-go plateaus on a move; the wheel parks within 0.2° of the target from either
  side; both proven by native tests on the pure law plus a bench run.
- Non-Goals: a new controller structure (PID, velocity loop), web-tunable duties, changes to the
  park correction ladder, the stall / timeout / range guard or the rate drive law.

## Decisions
- **D1 — Boost adds to the floor, not to the P term under the floor.**
  `duty = clamp(max(|err| × kp, floor) + boost, floor, maxDuty)`. Today a boost step is invisible
  while `P + boost < floor` (e.g. P 30 + 30 = 60 < 70), so a stalled wheel waits one or two extra
  checks (200–300 ms) and then gets a large jump. With D1 the first stalled check gives
  floor + 30 = 120 on the quad. Far from the target (P > floor) the law is identical to today.
  Alternative considered: raise `STEER_KP` so P never falls under the floor — rejected, it makes
  the approach to the target faster (overshoot) and does not fix the hidden step near the lock.
- **D2 — Asymmetric boost: +30 when stalled, −10 when moving (`STEER_BOOST_DECAY`).** Today the
  check right after break-away removes the whole 30, so the duty falls straight back to the floor
  that just stalled the wheel: the plateau repeats. Decaying by 10 per 100 ms keeps the wheel
  moving for ≈ 300 ms on the boost that freed it, which carries it past the friction peak. If it
  stalls again the boost climbs again from where it is, not from 0. Boost still resets to 0 on a
  start from rest and on a direction flip, and is still capped below `maxDuty`.
- **D3 — Quad floors 90 / 90.** `STEER_PWM_TRACK_MIN_DUTY` 70 → 90: the measured breaks happened
  at 89–107, so 90 is the lowest value that clears the running friction at 10° without boost;
  bench-tune (step 90 → 85 → 80 if the wheel overshoots the 0.2° window). `STEER_PWM_MIN_DUTY`
  80 → 90 keeps break-away ≥ tracking (no `static_assert` enforces it today; the task adds one,
  `STEER_PWM_MIN_DUTY >= STEER_PWM_TRACK_MIN_DUTY`). Side effect: `STEER_PWM_MIN_DUTY` is also the
  jog / nudge clamp in `driveJog()` and the rate-drive floor (`steerRateDuty`), so jogs and the
  sensorless rate drive on the quad start at 90 instead of 80 — acceptable (jogs are hand-held,
  the quad does not use the rate drive with a healthy LWS). The park correction pulses bypass the
  clamp and are not affected.
- **D4 — Park window 0.2°, restart band 1.0°.** `STEER_POSITION_TOLERANCE` 4 → 2. Because coast is
  ≈ 0, the wheel stops where the loop cuts the duty; cutting at 0.2° instead of 0.4° halves the
  residual and puts it at the sensor's resolution scale. The restart tolerance stays 10 (1.0°):
  the band between 0.2° and 1.0° never drives the main loop, so a parked wheel does not chatter.
- **D5 — Park correction trigger decoupled.** The correction's trigger was
  `max(STEER_POSITION_TOLERANCE, doneTol)`. With D4 it would silently drop to 0.2° and start
  pulsing on 0.3° residuals. A new `STEER_PARK_TRIGGER_TOL` 4 (0.4°) is passed as
  `SteerParkParams.triggerTol`, so the correction behaves exactly as in 1.5.19.
- **D6 — Pure law, new parameter.** `SteerPosParams` gets `int16_t boostDecay`
  (`STEER_BOOST_DECAY`); every caller (`POS_PARAMS` in `SteeringController.cpp`, the native test
  block) is updated. No Arduino dependency is added.
- **D7 — Boost hold band (`STEER_BOOST_HOLD_ERR` 20 = 2.0°).** Bench 2026-11-06 on the quad
  (wheels on the ground, ~200 kg load, dry steering): outward 10° moves still showed 2–3 stop/go
  plateaus at duty 103–150 with any floor (70 / 85 / 90 / 100). The wheel stalls at 35–44 A, the
  +30 boost breaks it free, the −10 / 100 ms decay removes the boost within ≈ 300 ms and the wheel
  stalls again — the running friction of a loaded tyre needs the boost for the whole move, not just
  for break-away. So on a moving check the boost is held while `|err| > STEER_BOOST_HOLD_ERR` and
  decays by `STEER_BOOST_DECAY` only inside the band; a stalled check adds `STEER_BOOST_STEP`
  anywhere; the clamp to `maxDuty` is unchanged. Returns to 0 at floor 85 already parked with
  ≤ 0.2° residual and no hunting; the band keeps the in-band approach identical to D2. If the boost
  arriving at the band edge causes overshoot at centre, raise `STEER_BOOST_DECAY` (e.g. 30) to shed
  it faster inside the band. `SteerPosParams` gets `int32_t boostHoldErr`. Floors settled at 85 / 85.
  Bench 2026-11-06 (`center -n 6 --offset 10`, quad loaded):
  | hold / decay | outward plateaus R / L | outward t_stop | peak A | return residual / overshoot / restarts |
  |---|---|---|---|---|
  | none / 10 (ref 140216) | 3 / 5 | 1.71–2.45 s | 37–41 | ≤ 0.2° / ≤ 0.2° / 0 |
  | 2.0° / 10 (141134) | 0 / 0 | 1.32–1.55 s | 40–41 | ≤ 0.1° except one 0.7° / 0.7° / 0 |
  | 2.0° / 30 (141305) | 0 / 2 (≈ 250 ms at 0.3–0.4° from the target) | 1.32–1.86 s | 38–44 | ≤ 0.1° / ≤ 0.1° / 0 |
  With decay 10 a +30 earned on the first check of a return (break-away latency) was held all the
  way to centre and arrived at 115 duty: one 0.7° overshoot in six. `STEER_BOOST_DECAY` 10 → 30
  sheds it in one in-band check; the price is a short shed / stall / re-add at the very end of a
  loaded outward move. Quad values: hold 2.0°, decay 30.

## Risks / Trade-offs
- **Overshoot / hunting into a 0.2° window at duty 90.** Near the target the duty is
  floor + boost ≥ 90; if the wheel coasts or the loop reacts a sample late, it may pass the 0.2°
  window, reverse (break-away re-armed at 90) and hunt. The move timeout does not stop a hunt
  inside 1.0° (it is re-armed there) and a moving wheel does not stall. Mitigation: bench check
  with `goto ±10` and `goto 0` from both sides (USB log); if it overshoots, lower
  `STEER_PWM_TRACK_MIN_DUTY` (90 → 85 → 80) first, then widen `STEER_POSITION_TOLERANCE` to 3. A
  hunt guard (park after N reversals within the restart band) is out of scope unless the bench
  shows hunting.
- **More current per move.** Duty 90 + boost draws more than 70, but for far less time than the
  17–27 A plateaus did; the VESC current clamp is unchanged.
- **Gentle decay keeps the duty higher after break-away** → a slightly faster approach; the
  0.2° window and the P term (which dominates beyond ≈ 8° at floor 90) bound it.

## Merge note — aztec branch
The aztec rack has its own measured constants (break-away 14–18, 100 × 100 ms = 5.6°, the rack
coasts): `STEER_PWM_MIN_DUTY` 30, `STEER_PWM_TRACK_MIN_DUTY` 30, `STEER_POSITION_TOLERANCE` 4
(0.4°) and its own park ladder 40×50 / 40×60 / 50×60. When `master` (1.5.20) is merged into
`aztec`:
- take the law change (D1, D2, D5, D6), `STEER_BOOST_DECAY` 10 and `STEER_PARK_TRIGGER_TOL` 4;
- KEEP aztec's 30 / 30 duties and the 0.4° park window (a coasting rack would overshoot a 0.2°
  window); resolve the `Constants.h` conflict in favour of aztec for these three lines;
- bench-check a 10° move on the buggy: with boost on top of a 30 floor the first stalled check
  gives 60 instead of being hidden — confirm it does not jump.
This proposal changes nothing on `aztec`.

## Open Questions
- Final quad tracking floor (85 or 90) — from the bench run.
