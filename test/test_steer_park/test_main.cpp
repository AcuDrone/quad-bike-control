// Native unit tests for the steering park correction law: settle gating, pulse ladder,
// done / give-up, overshoot and wrong-way handling, budget per parking
// (include/SteerParkLaw.h). Run: pio test -e native
#include <unity.h>
#include "SteerParkLaw.h"

// Mirror of the defaults in include/Constants.h (which needs Arduino.h and cannot be built natively).
static const SteerParkParams P = {
    4,      // triggerTol  STEER_POSITION_TOLERANCE (0.4°)
    2,      // doneTol     STEER_PARK_DONE_TOL_DEFAULT (0.2°)
    1,      // moveMin     STEER_PARK_MOVE_MIN (0.1°)
    1000,   // settleMs    STEER_PARK_SETTLE_MS
    500,    // gapMs       STEER_PARK_PULSE_GAP_MS
    5,      // maxPulses   STEER_PARK_MAX_PULSES_DEFAULT
    {{70, 80}, {85, 90}, {100, 100}},
};

static SteerParkState st;

void setUp() { st.reset(); }
void tearDown() {}

// The tests drive a parked wheel with target 0: err = −pos. The settle clock started at t = 0.
static SteerParkResult stepAt(const SteerParkParams& p, uint32_t now, int32_t pos) {
  return steerParkStep(st, p, now, 0 - pos, pos, now);
}
static SteerParkResult step(uint32_t now, int32_t pos) { return stepAt(P, now, pos); }

static void assertPulse(const SteerParkResult& r, uint8_t duty, uint16_t ms, int8_t dir) {
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::PULSE, (uint8_t)r.action);
  TEST_ASSERT_EQUAL_UINT8(duty, r.duty);
  TEST_ASSERT_EQUAL_UINT16(ms, r.ms);
  TEST_ASSERT_EQUAL_INT8(dir, r.dir);
}

// Time at which the pulse sent at `start` with length `ms` is judged
static uint32_t evalAt(uint32_t start, uint16_t ms) { return start + ms + P.gapMs; }

// ─── Gating ───────────────────────────────────────────────────────────────

static void test_no_pulse_before_settle() {
  for (uint32_t t = 0; t < 1000; t += 10) {
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::NONE, (uint8_t)step(t, 8).action);
  }
  assertPulse(step(1000, 8), 70, 80, -1);
}

static void test_no_pulse_within_trigger() {
  for (int32_t pos = -4; pos <= 4; pos++) {
    setUp();
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::NONE, (uint8_t)step(2000, pos).action);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkPhase::ARMED, (uint8_t)st.phase);
  }
  // A done tolerance above the trigger raises the trigger
  SteerParkParams p = P;
  p.triggerTol = 2;
  p.doneTol = 5;
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::NONE, (uint8_t)stepAt(p, 2000, 5).action);
  assertPulse(stepAt(p, 2000, 6), 70, 80, -1);
}

static void test_first_pulse_rung1_toward_error_both_signs() {
  assertPulse(step(1500, 7), 70, 80, -1);    // wheel right of target → drive left
  TEST_ASSERT_EQUAL_UINT8(1, st.pulses);
  TEST_ASSERT_EQUAL_INT32(7, st.posBefore);
  setUp();
  assertPulse(step(1500, -7), 70, 80, +1);   // wheel left of target → drive right
}

static void test_settle_counts_from_caller_clock() {
  // settledMs is the caller's: a target move resets it to 0 → reset, no pulse
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::NONE,
                          (uint8_t)steerParkStep(st, P, 50000, -8, 8, 999).action);
  assertPulse(steerParkStep(st, P, 50001, -8, 8, 1000), 70, 80, -1);
}

// ─── Evaluation timing and the ladder ─────────────────────────────────────

static void test_nothing_evaluated_before_gap() {
  assertPulse(step(1000, 8), 70, 80, -1);
  for (uint32_t t = 1000; t < evalAt(1000, 80); t += 10) {
    SteerParkResult r = step(t, 8);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::NONE, (uint8_t)r.action);
    TEST_ASSERT_FALSE(r.evaluated);
  }
  SteerParkResult r = step(evalAt(1000, 80), 8);
  TEST_ASSERT_TRUE(r.evaluated);
  TEST_ASSERT_EQUAL_UINT8(70, r.evalDuty);
  TEST_ASSERT_EQUAL_UINT16(80, r.evalMs);
}

static void test_no_move_escalates_and_stays_at_rung3() {
  uint32_t t = 1000;
  assertPulse(step(t, 8), 70, 80, -1);
  t = evalAt(t, 80);
  SteerParkResult r = step(t, 8);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkNote::ESCALATE, (uint8_t)r.note);
  TEST_ASSERT_EQUAL_INT32(0, r.moved);
  assertPulse(r, 85, 90, -1);
  t = evalAt(t, 90);
  r = step(t, 8);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkNote::ESCALATE, (uint8_t)r.note);
  assertPulse(r, 100, 100, -1);
  t = evalAt(t, 100);
  r = step(t, 8);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkNote::ESCALATE, (uint8_t)r.note);
  assertPulse(r, 100, 100, -1);   // capped at rung 3
  TEST_ASSERT_EQUAL_UINT8(4, st.pulses);
}

static void test_move_keeps_rung() {
  uint32_t t = 1000;
  assertPulse(step(t, 9), 70, 80, -1);
  t = evalAt(t, 80);
  SteerParkResult r = step(t, 8);   // moved 0.1° toward the target
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkNote::SAME, (uint8_t)r.note);
  TEST_ASSERT_EQUAL_INT32(1, r.moved);
  assertPulse(r, 70, 80, -1);
  // Escalate once, then a move keeps rung 2
  t = evalAt(t, 80);
  r = step(t, 8);
  assertPulse(r, 85, 90, -1);
  t = evalAt(t, 90);
  r = step(t, 6);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkNote::SAME, (uint8_t)r.note);
  TEST_ASSERT_EQUAL_INT32(2, r.moved);
  assertPulse(r, 85, 90, -1);
}

static void test_done_at_default_tolerance() {
  uint32_t t = 1000;
  assertPulse(step(t, 7), 70, 80, -1);
  t = evalAt(t, 80);
  SteerParkResult r = step(t, 3);   // 0.3° > 0.2°: continue (not the 0.4° trigger)
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkNote::SAME, (uint8_t)r.note);
  assertPulse(r, 70, 80, -1);
  t = evalAt(t, 80);
  r = step(t, 2);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::DONE, (uint8_t)r.action);
  TEST_ASSERT_TRUE(r.evaluated);
  TEST_ASSERT_EQUAL_INT32(1, r.moved);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkPhase::DONE, (uint8_t)st.phase);
  TEST_ASSERT_EQUAL_UINT8(2, st.pulses);
  // Done stays quiet inside the trigger
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::NONE, (uint8_t)step(t + 5000, 4).action);
}

static void test_done_at_custom_tolerance() {
  SteerParkParams p = P;
  p.doneTol = 4;
  uint32_t t = 1000;
  assertPulse(stepAt(p, t, 8), 70, 80, -1);
  t = evalAt(t, 80);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::DONE, (uint8_t)stepAt(p, t, 4).action);
  setUp();
  p.doneTol = 1;
  t = 1000;
  assertPulse(stepAt(p, t, 8), 70, 80, -1);
  t = evalAt(t, 80);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::PULSE, (uint8_t)stepAt(p, t, 2).action);
  t = evalAt(t, 80);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::DONE, (uint8_t)stepAt(p, t, 1).action);
}

static void test_give_up_after_default_max() {
  uint32_t t = 1000;
  SteerParkResult r = step(t, 8);
  uint16_t ms = r.ms;
  for (int i = 1; i < 5; i++) {
    t = evalAt(t, ms);
    r = step(t, 8);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::PULSE, (uint8_t)r.action);
    ms = r.ms;
  }
  TEST_ASSERT_EQUAL_UINT8(5, st.pulses);
  t = evalAt(t, ms);
  r = step(t, 8);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::GIVE_UP, (uint8_t)r.action);
  TEST_ASSERT_TRUE(r.evaluated);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkPhase::GAVE_UP, (uint8_t)st.phase);
  // No pulse again until reset
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::NONE, (uint8_t)step(t + 10000, 9).action);
  TEST_ASSERT_EQUAL_UINT8(5, st.pulses);
}

static void test_give_up_after_custom_max() {
  SteerParkParams p = P;
  p.maxPulses = 2;
  uint32_t t = 1000;
  assertPulse(stepAt(p, t, 8), 70, 80, -1);
  t = evalAt(t, 80);
  assertPulse(stepAt(p, t, 8), 85, 90, -1);
  t = evalAt(t, 90);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::GIVE_UP, (uint8_t)stepAt(p, t, 8).action);
  TEST_ASSERT_EQUAL_UINT8(2, st.pulses);
}

static void test_overshoot_reverses_at_rung1_and_counts() {
  uint32_t t = 1000;
  assertPulse(step(t, 8), 70, 80, -1);
  t = evalAt(t, 80);
  assertPulse(step(t, 8), 85, 90, -1);     // escalated to rung 2
  t = evalAt(t, 90);
  SteerParkResult r = step(t, -3);         // went past 0 to −0.3°
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkNote::OVERSHOOT, (uint8_t)r.note);
  assertPulse(r, 70, 80, +1);              // rung 1, reversed
  TEST_ASSERT_EQUAL_UINT8(3, st.pulses);
  // Overshoot within the done tolerance is done, not reversed
  setUp();
  t = 1000;
  assertPulse(step(t, 8), 70, 80, -1);
  t = evalAt(t, 80);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::DONE, (uint8_t)step(t, -2).action);
}

static void test_wrong_way_resets_to_rung1() {
  uint32_t t = 1000;
  assertPulse(step(t, 6), 70, 80, -1);
  t = evalAt(t, 80);
  assertPulse(step(t, 6), 85, 90, -1);
  t = evalAt(t, 90);
  SteerParkResult r = step(t, 7);          // moved 0.1° away
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkNote::WRONG_WAY, (uint8_t)r.note);
  TEST_ASSERT_EQUAL_INT32(-1, r.moved);
  assertPulse(r, 70, 80, -1);
}

// ─── Budget per parking ───────────────────────────────────────────────────

static void test_settle_reset_cancels_and_rearms_budget() {
  SteerParkParams p = P;
  p.maxPulses = 2;
  uint32_t t = 1000;
  assertPulse(stepAt(p, t, 8), 70, 80, -1);
  t = evalAt(t, 80);
  assertPulse(stepAt(p, t, 8), 85, 90, -1);
  // Target moved: the caller's settle clock restarts → reset mid-cycle
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::NONE,
                          (uint8_t)steerParkStep(st, p, t + 10, -8, 8, 0).action);
  TEST_ASSERT_EQUAL_UINT8(0, st.pulses);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkPhase::ARMED, (uint8_t)st.phase);
  // Settled again: a full budget at rung 1
  assertPulse(steerParkStep(st, p, t + 2000, -8, 8, 1000), 70, 80, -1);
  TEST_ASSERT_EQUAL_UINT8(1, st.pulses);
}

static void test_done_does_not_refill_budget() {
  SteerParkParams p = P;
  p.maxPulses = 3;
  uint32_t t = 1000;
  assertPulse(stepAt(p, t, 7), 70, 80, -1);
  t = evalAt(t, 80);
  assertPulse(stepAt(p, t, 4), 70, 80, -1);   // moved 0.3°, same rung
  t = evalAt(t, 80);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::DONE, (uint8_t)stepAt(p, t, 1).action);
  // The wheel creeps out again: resumes at rung 1 with 1 pulse left
  t += 3000;
  assertPulse(stepAt(p, t, 6), 70, 80, -1);
  TEST_ASSERT_EQUAL_UINT8(3, st.pulses);
  t = evalAt(t, 80);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::GIVE_UP, (uint8_t)stepAt(p, t, 6).action);
  // Budget spent after a done: a creep gives up without a pulse
  setUp();
  p.maxPulses = 1;
  t = 1000;
  assertPulse(stepAt(p, t, 7), 70, 80, -1);
  t = evalAt(t, 80);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::DONE, (uint8_t)stepAt(p, t, 0).action);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerParkAction::GIVE_UP, (uint8_t)stepAt(p, t + 3000, 6).action);
  TEST_ASSERT_EQUAL_UINT8(1, st.pulses);
}

// ─── Custom ladder and validity ───────────────────────────────────────────

static void test_custom_ladder_is_used() {
  SteerParkParams p = P;
  p.rung[0] = {60, 80};
  p.rung[1] = {85, 90};
  p.rung[2] = {120, 150};
  uint32_t t = 1000;
  assertPulse(stepAt(p, t, -9), 60, 80, +1);
  t = evalAt(t, 80);
  assertPulse(stepAt(p, t, -9), 85, 90, +1);
  t = evalAt(t, 90);
  assertPulse(stepAt(p, t, -9), 120, 150, +1);
  t += 150 + 499;
  TEST_ASSERT_FALSE(stepAt(p, t, -9).evaluated);   // the 150 ms rung's own gap
  SteerParkResult r = stepAt(p, t + 1, -9);
  TEST_ASSERT_TRUE(r.evaluated);
  TEST_ASSERT_EQUAL_UINT8(120, r.evalDuty);
  TEST_ASSERT_EQUAL_UINT16(150, r.evalMs);
  assertPulse(r, 120, 150, +1);
}

static void test_ladder_validity() {
  TEST_ASSERT_TRUE(steerParkLadderValid(P));
  SteerParkParams p = P;
  p.rung[0] = {85, 90};   // equal rungs are allowed
  TEST_ASSERT_TRUE(steerParkLadderValid(p));
  p = P;
  p.rung[1].duty = 69;    // duty decreases 1 → 2
  TEST_ASSERT_FALSE(steerParkLadderValid(p));
  p = P;
  p.rung[2].ms = 89;      // ms decreases 2 → 3
  TEST_ASSERT_FALSE(steerParkLadderValid(p));
  p = P;
  p.rung[0].duty = 101;   // rung 1 above rung 2 and 3
  TEST_ASSERT_FALSE(steerParkLadderValid(p));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_no_pulse_before_settle);
  RUN_TEST(test_no_pulse_within_trigger);
  RUN_TEST(test_first_pulse_rung1_toward_error_both_signs);
  RUN_TEST(test_settle_counts_from_caller_clock);
  RUN_TEST(test_nothing_evaluated_before_gap);
  RUN_TEST(test_no_move_escalates_and_stays_at_rung3);
  RUN_TEST(test_move_keeps_rung);
  RUN_TEST(test_done_at_default_tolerance);
  RUN_TEST(test_done_at_custom_tolerance);
  RUN_TEST(test_give_up_after_default_max);
  RUN_TEST(test_give_up_after_custom_max);
  RUN_TEST(test_overshoot_reverses_at_rung1_and_counts);
  RUN_TEST(test_wrong_way_resets_to_rung1);
  RUN_TEST(test_settle_reset_cancels_and_rearms_budget);
  RUN_TEST(test_done_does_not_refill_budget);
  RUN_TEST(test_custom_ladder_is_used);
  RUN_TEST(test_ladder_validity);
  return UNITY_END();
}
