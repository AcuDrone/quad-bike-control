// SteerPositionLaw.h — closed-loop steering position law: continuous target tracking,
// stop hysteresis and the break-away / tracking duty floors
// (change fix-steering-continuous-tracking, design D1–D7; boost on top of the floor, gentle
// decay: change fix-steering-jerk-and-centre-undershoot, design D1/D2/D6).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit
// tests (test/test_steer_position). Callers pass the constants in through SteerPosParams.
//
//  States: IDLE (coast, the loop owns nothing), DRIVING (P duty toward the target),
//          PARKED (coast after one stop() on entry; the self-locking worm holds the shaft).
//  DRIVING → PARKED at |err| ≤ posTol;  PARKED → DRIVING at |err| > restartTol (start from rest).
//  A target update never touches the timers: only a start from rest (startFromRest()) resets
//  the move timer, the stall window, the progress baseline, the boost and arms break-away.
//  While DRIVING, per step:
//   1. direction flip (sign of err changed) → stall window restarts, boost = 0, break-away re-armed;
//   2. stall: |pos − stallPos| < stallThreshold over stallWindowMs → STALL (state IDLE);
//   3. |err| ≤ posTol → PARKED;
//   4. move timer re-armed while |err| ≤ restartTol; else after moveTimeoutMs → TIMEOUT (IDLE);
//   5. anti-stall boost: < boostMinProgress per boostCheckMs → +boostStep (stalled, anywhere);
//      moving and |err| > boostHoldErr → boost held (hold band: a loaded wheel that broke free
//      far from the target keeps the extra duty instead of stalling again); moving inside
//      boostHoldErr → −boostDecay, never below 0;
//   6. break-away disarmed once |pos − breakawayStartPos| ≥ boostMinProgress;
//   7. duty = clamp(max(|err| × kp, floor) + boost, floor, maxDuty), floor = breakawayMinDuty
//      while break-away is armed, else trackMinDuty; signed toward the target. The boost always
//      adds on top of the floor, so a stalled check is never hidden under it.
// The range guard, sensor / driver health and the stall latch stay in SteeringController.
#pragma once

#include <stdint.h>

/** @brief Closed-loop position state */
enum class SteerPosState : uint8_t { IDLE, DRIVING, PARKED };

/** @brief What steerPosStep() did this iteration (the caller applies the side effects) */
enum class SteerPosEvent : uint8_t { NONE, PARKED, RESTARTED, STALL, TIMEOUT };

/** @brief Tunables (0.1° for positions, ms for times, 0–255 for duties) */
struct SteerPosParams {
  int32_t posTol;            // park window (STEER_POSITION_TOLERANCE)
  int32_t restartTol;        // drive again from parked beyond this (STEER_RESTART_TOLERANCE)
  float kp;                  // duty per 0.1° of error (STEER_KP)
  uint8_t breakawayMinDuty;  // floor from rest / after a reversal (STEER_PWM_MIN_DUTY)
  uint8_t trackMinDuty;      // floor once moving (STEER_PWM_TRACK_MIN_DUTY)
  uint8_t maxDuty;           // STEER_PWM_MAX_DUTY
  int32_t stallThreshold;    // STEER_STALL_THRESHOLD
  uint32_t stallWindowMs;    // STEER_STALL_TIMEOUT
  uint32_t moveTimeoutMs;    // STEER_MOVE_TIMEOUT
  uint32_t boostCheckMs;     // STEER_BOOST_CHECK_MS
  int32_t boostMinProgress;  // STEER_BOOST_MIN_PROGRESS (also the break-away distance)
  int16_t boostStep;         // STEER_BOOST_STEP: added per stalled check
  int16_t boostDecay;        // STEER_BOOST_DECAY: removed per moving check (never below 0)
  int32_t boostHoldErr;      // STEER_BOOST_HOLD_ERR: no decay while |err| > this (0.1°)
};

/** @brief Per-drive bookkeeping; reset only by startFromRest() */
struct SteerPosTimers {
  uint32_t moveStartMs = 0;      // move timer (re-armed while within restartTol)
  int32_t stallPos = 0;          // stall window baseline
  uint32_t stallMs = 0;
  int32_t progressPos = 0;       // anti-stall boost baseline
  uint32_t progressMs = 0;
  int16_t boost = 0;             // anti-stall duty boost
  int8_t lastDir = 0;            // sign of the error on the last DRIVING step (0 = none yet)
  bool breakaway = false;        // break-away floor armed
  int32_t breakawayStartPos = 0; // position where the drive / reversal started

  void startFromRest(int32_t pos, uint32_t now) {
    moveStartMs = now;
    stallPos = pos;
    stallMs = now;
    progressPos = pos;
    progressMs = now;
    boost = 0;
    lastDir = 0;
    breakaway = true;
    breakawayStartPos = pos;
  }
};

static inline int32_t steerPosAbs(int32_t v) { return v < 0 ? -v : v; }

/** @brief The duty floor currently in force */
static inline uint8_t steerPosFloor(const SteerPosTimers& t, const SteerPosParams& p) {
  return t.breakaway ? p.breakawayMinDuty : p.trackMinDuty;
}

/**
 * @brief Accept a command while IDLE: start a drive from rest, or park at once when already
 * within posTol (the caller coasts the motor on PARKED). The stall latch is the caller's.
 * @return the new state (DRIVING or PARKED)
 */
static inline SteerPosState steerPosStart(SteerPosState& state, SteerPosTimers& t,
                                          const SteerPosParams& p, int32_t target,
                                          int32_t pos, uint32_t now) {
  if (steerPosAbs(target - pos) <= p.posTol) {
    state = SteerPosState::PARKED;
  } else {
    t.startFromRest(pos, now);
    state = SteerPosState::DRIVING;
  }
  return state;
}

/**
 * @brief One control-loop iteration.
 * @param dutyOut signed duty to apply while the returned state is DRIVING, else 0
 * @return NONE / PARKED (coast once) / RESTARTED (left PARKED, now DRIVING) /
 *         STALL (state IDLE, latch the error's direction) / TIMEOUT (state IDLE, coast)
 */
static inline SteerPosEvent steerPosStep(SteerPosState& state, SteerPosTimers& t,
                                         const SteerPosParams& p, int32_t target,
                                         int32_t pos, uint32_t now, int16_t* dutyOut) {
  if (dutyOut) *dutyOut = 0;
  if (state == SteerPosState::IDLE) return SteerPosEvent::NONE;

  const int32_t err = target - pos;
  const int32_t absErr = steerPosAbs(err);
  SteerPosEvent ev = SteerPosEvent::NONE;

  if (state == SteerPosState::PARKED) {
    if (absErr <= p.restartTol) return SteerPosEvent::NONE;   // hysteresis band: stay parked
    t.startFromRest(pos, now);
    state = SteerPosState::DRIVING;
    ev = SteerPosEvent::RESTARTED;
  }

  // DRIVING
  const int8_t dir = err > 0 ? 1 : (err < 0 ? -1 : 0);
  if (dir != 0 && t.lastDir != 0 && dir != t.lastDir) {
    // Reversal: passes through zero speed — fresh stall window, boost earned in the old
    // direction dropped, the worm is at rest again so break-away is re-armed.
    t.stallPos = pos;
    t.stallMs = now;
    t.boost = 0;
    t.breakaway = true;
    t.breakawayStartPos = pos;
  }
  if (dir != 0) t.lastDir = dir;

  if (now - t.stallMs >= p.stallWindowMs) {
    if (steerPosAbs(pos - t.stallPos) < p.stallThreshold) {
      state = SteerPosState::IDLE;
      return SteerPosEvent::STALL;
    }
    t.stallPos = pos;
    t.stallMs = now;
  }

  if (absErr <= p.posTol) {
    state = SteerPosState::PARKED;
    return SteerPosEvent::PARKED;
  }

  if (absErr <= p.restartTol) {
    t.moveStartMs = now;   // keeping up: the timeout measures time continuously behind
  } else if (now - t.moveStartMs >= p.moveTimeoutMs) {
    state = SteerPosState::IDLE;
    return SteerPosEvent::TIMEOUT;
  }

  if (now - t.progressMs >= p.boostCheckMs) {
    if (steerPosAbs(pos - t.progressPos) < p.boostMinProgress) {
      if (t.boost < (int16_t)p.maxDuty) t.boost += p.boostStep;
    } else if (t.boost > 0 && absErr <= p.boostHoldErr) {
      t.boost -= (t.boost < p.boostDecay) ? t.boost : p.boostDecay;
    }
    t.progressPos = pos;
    t.progressMs = now;
  }

  if (t.breakaway && steerPosAbs(pos - t.breakawayStartPos) >= p.boostMinProgress) {
    t.breakaway = false;
  }

  const float minDuty = (float)steerPosFloor(t, p);
  float duty = (float)absErr * p.kp;
  if (duty < minDuty) duty = minDuty;   // boost adds on top of the floor, never hidden under it
  duty += (float)t.boost;               // boost ≥ 0, so the result stays ≥ floor
  if (duty > (float)p.maxDuty) duty = (float)p.maxDuty;
  const int16_t d = (int16_t)duty;
  if (dutyOut) *dutyOut = err > 0 ? d : (int16_t)-d;
  return ev;
}
