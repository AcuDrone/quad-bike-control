// SteerParkLaw.h — steering park correction: finish a parking that ended outside the park
// window with a bounded ladder of short open-loop pulses (change add-steering-park-correction).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit tests
// (test/test_steer_park). Callers pass the constants and the web settings in through
// SteerParkParams. Positions and tolerances are in 0.1° (repo unit), times in ms.
//
//  Phases: ARMED   — waiting for |err| > max(triggerTol, doneTol) to send the first pulse;
//          WAIT    — a pulse was sent; nothing is judged until pulse ms + gapMs have passed;
//          DONE    — |err| ≤ doneTol after a pulse; a later |err| > trigger resumes at rung 1
//                    with the REMAINING budget (DONE does not refill it);
//          GAVE_UP — maxPulses sent and the evaluation after the last one was not done,
//                    or a pulse overshot beyond the error it started from (TOO_STRONG).
//  The budget and the ladder re-arm only through reset() — the caller's settle clock reset
//  (target move beyond the stable band, new park entry), seen here as settledMs < settleMs.
//  Evaluation after a pulse, in this order (moved = progress toward the target, signed):
//   1. |err| ≤ doneTol                     → DONE;
//   2. err changed sign and |err| > |err before the pulse| + doneTol
//                                          → GIVE_UP, note TOO_STRONG (the pulse was clearly
//                                            too strong: reversing would only shake the wheel);
//   3. err changed sign, |err| > doneTol   → OVERSHOOT, next pulse rung 1 (new direction)
//                                            — small overshoots only (|err| ≲ |err before|);
//   4. moved ≤ −moveMin                    → WRONG_WAY, next pulse rung 1;
//   5. moved ≥ moveMin                     → SAME rung;
//   6. otherwise                           → ESCALATE (next rung, stays at rung 3);
//   then pulses ≥ maxPulses → GIVE_UP, else PULSE toward sign(err).
// The stall latch, the motor and the logs stay in SteeringController.
#pragma once

#include <stdint.h>

static constexpr uint8_t STEER_PARK_RUNGS = 3;

/** @brief One ladder rung: open-loop duty (0–255) for ms */
struct SteerParkRung {
  uint8_t duty;
  uint16_t ms;
};

/** @brief Tunables (0.1° for positions, ms for times) */
struct SteerParkParams {
  int32_t triggerTol;    // STEER_POSITION_TOLERANCE: no correction within this
  int32_t doneTol;       // done tolerance (web setting park_tol)
  int32_t moveMin;       // STEER_PARK_MOVE_MIN: smaller position change = "did not move"
  uint32_t settleMs;     // STEER_PARK_SETTLE_MS: parked + target stable this long before a pulse
  uint32_t gapMs;        // STEER_PARK_PULSE_GAP_MS: wait after a pulse's end before judging it
  uint8_t maxPulses;     // budget per parking (web setting park_max)
  SteerParkRung rung[STEER_PARK_RUNGS];
};

enum class SteerParkPhase : uint8_t { ARMED, WAIT, DONE, GAVE_UP };
enum class SteerParkAction : uint8_t { NONE, PULSE, DONE, GIVE_UP };
// TOO_STRONG: the law gave up after a pulse that overshot beyond the error it started from.
// OVERSHOOT_RESTART: set by the caller (not the law) when the main loop restarted after a pulse.
enum class SteerParkNote : uint8_t { NONE, SAME, ESCALATE, OVERSHOOT, WRONG_WAY, TOO_STRONG,
                                     OVERSHOOT_RESTART };

/** @brief Per-parking bookkeeping; reset() re-arms the ladder and the budget */
struct SteerParkState {
  SteerParkPhase phase = SteerParkPhase::ARMED;
  uint8_t rung = 0;            // index 0..2 of the rung of the next / running pulse
  uint8_t pulses = 0;          // pulses sent in this parking (the budget used)
  uint32_t pulseStartMs = 0;
  uint16_t pulseMs = 0;        // length of the running / last pulse
  int32_t posBefore = 0;       // position when the last pulse was sent
  int32_t absErrBefore = 0;    // |err| when the last pulse was sent
  int8_t dir = 0;              // direction of the last pulse (+1 / −1)
  uint8_t lastDuty = 0;        // duty × ms of the last pulse (0 = none yet)
  uint16_t lastMs = 0;

  void reset() { *this = SteerParkState(); }
};

/** @brief What steerParkStep() decided this iteration (the caller applies the side effects) */
struct SteerParkResult {
  SteerParkAction action = SteerParkAction::NONE;
  uint8_t duty = 0;            // PULSE: duty and length of the new pulse
  uint16_t ms = 0;
  int8_t dir = 0;              // PULSE: +1 / −1
  bool evaluated = false;      // a pulse was judged this step (log line "park nudge …")
  uint8_t evalDuty = 0;        // evaluated: duty × ms of the judged pulse
  uint16_t evalMs = 0;
  int32_t moved = 0;           // evaluated: progress toward the target since the pulse (0.1°, signed)
  SteerParkNote note = SteerParkNote::NONE;
};

static inline int32_t steerParkAbs(int32_t v) { return v < 0 ? -v : v; }
static inline int8_t steerParkSign(int32_t v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

/** @brief The error beyond which a correction starts: max(triggerTol, doneTol) */
static inline int32_t steerParkTrigger(const SteerParkParams& p) {
  return p.triggerTol > p.doneTol ? p.triggerTol : p.doneTol;
}

/** @brief Non-decreasing ladder in both duty and ms (the bounds of each rung are the caller's) */
static inline bool steerParkLadderValid(const SteerParkParams& p) {
  for (uint8_t i = 1; i < STEER_PARK_RUNGS; i++) {
    if (p.rung[i].duty < p.rung[i - 1].duty) return false;
    if (p.rung[i].ms < p.rung[i - 1].ms) return false;
  }
  return true;
}

/** @brief Fill r as a PULSE at s.rung toward sign(err) and book it in s */
static inline void steerParkIssue(SteerParkState& s, const SteerParkParams& p, uint32_t now,
                                  int32_t err, int32_t pos, SteerParkResult& r) {
  const SteerParkRung& g = p.rung[s.rung];
  s.phase = SteerParkPhase::WAIT;
  s.pulses++;
  s.pulseStartMs = now;
  s.pulseMs = g.ms;
  s.posBefore = pos;
  s.absErrBefore = steerParkAbs(err);
  s.dir = steerParkSign(err);
  s.lastDuty = g.duty;
  s.lastMs = g.ms;
  r.action = SteerParkAction::PULSE;
  r.duty = g.duty;
  r.ms = g.ms;
  r.dir = s.dir;
}

/**
 * @brief One iteration while the closed loop is PARKED.
 * @param err       target − position (0.1°)
 * @param pos       position (0.1°)
 * @param settledMs time the loop has been parked with a stable target (the caller's settle clock)
 */
static inline SteerParkResult steerParkStep(SteerParkState& s, const SteerParkParams& p,
                                            uint32_t now, int32_t err, int32_t pos,
                                            uint32_t settledMs) {
  SteerParkResult r;
  if (settledMs < p.settleMs) {
    s.reset();
    return r;
  }
  const int32_t absErr = steerParkAbs(err);

  switch (s.phase) {
    case SteerParkPhase::GAVE_UP:
      return r;

    case SteerParkPhase::ARMED:
    case SteerParkPhase::DONE:
      if (absErr <= steerParkTrigger(p)) return r;
      if (s.pulses >= p.maxPulses) {   // DONE with the budget spent: nothing left to try
        s.phase = SteerParkPhase::GAVE_UP;
        r.action = SteerParkAction::GIVE_UP;
        return r;
      }
      s.rung = 0;
      steerParkIssue(s, p, now, err, pos, r);
      return r;

    case SteerParkPhase::WAIT:
      break;
  }

  // WAIT: judge the pulse only once its length and the gap have passed
  if (now - s.pulseStartMs < (uint32_t)s.pulseMs + p.gapMs) return r;

  r.evaluated = true;
  r.evalDuty = s.lastDuty;
  r.evalMs = s.lastMs;
  r.moved = (pos - s.posBefore) * (int32_t)s.dir;

  if (absErr <= p.doneTol) {
    s.phase = SteerParkPhase::DONE;
    r.action = SteerParkAction::DONE;
    return r;
  }
  if (steerParkSign(err) != s.dir && absErr > s.absErrBefore + p.doneTol) {
    // Overshot further than it started from: a reverse pulse would overshoot again.
    r.note = SteerParkNote::TOO_STRONG;
    s.phase = SteerParkPhase::GAVE_UP;
    r.action = SteerParkAction::GIVE_UP;
    return r;
  }
  if (steerParkSign(err) != s.dir) {
    r.note = SteerParkNote::OVERSHOOT;
    s.rung = 0;
  } else if (r.moved <= -p.moveMin) {
    r.note = SteerParkNote::WRONG_WAY;
    s.rung = 0;
  } else if (r.moved >= p.moveMin) {
    r.note = SteerParkNote::SAME;
  } else {
    r.note = SteerParkNote::ESCALATE;
    if (s.rung + 1 < STEER_PARK_RUNGS) s.rung++;
  }

  if (s.pulses >= p.maxPulses) {
    s.phase = SteerParkPhase::GAVE_UP;
    r.action = SteerParkAction::GIVE_UP;
    return r;
  }
  steerParkIssue(s, p, now, err, pos, r);
  return r;
}
