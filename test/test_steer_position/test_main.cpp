// Native unit tests for the closed-loop steering position law: continuous tracking, stop
// hysteresis, break-away / tracking duty floors, stall and move timeout
// (include/SteerPositionLaw.h). Boost on top of the floor, gentle decay, 0.2° park window:
// change fix-steering-jerk-and-centre-undershoot. Run: pio test -e native
#include <unity.h>
#include "SteerPositionLaw.h"

// Mirror of include/Constants.h (which needs Arduino.h and cannot be built natively).
static const SteerPosParams P = {
    2,       // posTol            STEER_POSITION_TOLERANCE (0.2°)
    10,      // restartTol        STEER_RESTART_TOLERANCE (1.0°)
    1.15f,   // kp                STEER_KP
    85,      // breakawayMinDuty  STEER_PWM_MIN_DUTY
    85,      // trackMinDuty      STEER_PWM_TRACK_MIN_DUTY
    255,     // maxDuty           STEER_PWM_MAX_DUTY
    5,       // stallThreshold    STEER_STALL_THRESHOLD
    500,     // stallWindowMs     STEER_STALL_TIMEOUT
    7500,    // moveTimeoutMs     STEER_MOVE_TIMEOUT
    100,     // boostCheckMs      STEER_BOOST_CHECK_MS
    3,       // boostMinProgress  STEER_BOOST_MIN_PROGRESS
    30,      // boostStep         STEER_BOOST_STEP
    30,      // boostDecay        STEER_BOOST_DECAY
    20,      // boostHoldErr      STEER_BOOST_HOLD_ERR (2.0°)
};

// The quad's two floors are equal (85 / 85); the floor-switching tests use a variant with a
// higher break-away floor so the switch is visible in the duty.
static const SteerPosParams PF = {
    P.posTol, P.restartTol, P.kp,
    100,     // breakawayMinDuty (test only)
    P.trackMinDuty,  // trackMinDuty
    P.maxDuty, P.stallThreshold, P.stallWindowMs, P.moveTimeoutMs,
    P.boostCheckMs, P.boostMinProgress, P.boostStep, P.boostDecay, P.boostHoldErr,
};
// Decay-step variant: the law's per-check decay with a smaller step than the quad's 30, so the
// step-by-step shedding is visible.
static const SteerPosParams PD = {
    P.posTol, P.restartTol, P.kp, P.breakawayMinDuty, P.trackMinDuty,
    P.maxDuty, P.stallThreshold, P.stallWindowMs, P.moveTimeoutMs,
    P.boostCheckMs, P.boostMinProgress, P.boostStep,
    10,      // boostDecay (test only)
    P.boostHoldErr,
};
static const SteerPosParams* cur = &P;

static SteerPosState st;
static SteerPosTimers tm;
static int16_t duty;

void setUp() {
    st = SteerPosState::IDLE;
    tm = SteerPosTimers();
    duty = 0;
    cur = &P;
}
void tearDown() {}

static SteerPosEvent step(int32_t target, int32_t pos, uint32_t now) {
    return steerPosStep(st, tm, *cur, target, pos, now, &duty);
}

static void start(int32_t target, int32_t pos, uint32_t now) {
    steerPosStart(st, tm, *cur, target, pos, now);
}

// ─── States and hysteresis ────────────────────────────────────────────────

static void test_idle_does_nothing() {
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(500, 0, 1000));
    TEST_ASSERT_EQUAL_INT16(0, duty);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::IDLE, (uint8_t)st);
}

static void test_start_from_idle_drives_or_parks() {
    start(200, 0, 1000);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::DRIVING, (uint8_t)st);
    TEST_ASSERT_TRUE(tm.breakaway);
    TEST_ASSERT_EQUAL_UINT32(1000, tm.moveStartMs);

    setUp();
    start(2, 0, 1000);    // within posTol → park without driving
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::PARKED, (uint8_t)st);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(2, 0, 1010));
    TEST_ASSERT_EQUAL_INT16(0, duty);
}

static void test_drive_parks_within_pos_tol() {
    start(200, 0, 1000);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(200, 0, 1010));
    TEST_ASSERT_TRUE(duty > 0);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(200, 197, 1020));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::PARKED, (uint8_t)step(200, 198, 1030));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::PARKED, (uint8_t)st);
    TEST_ASSERT_EQUAL_INT16(0, duty);
}

static void test_parked_holds_inside_band() {
    start(2, 0, 1000);    // parked at 0
    for (int32_t tgt = 3; tgt <= 10; tgt++) {
        TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(tgt, 0, 1000 + (uint32_t)tgt * 10));
        TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::PARKED, (uint8_t)st);
        TEST_ASSERT_EQUAL_INT16(0, duty);
    }
    for (int32_t tgt = -10; tgt <= -3; tgt++) {
        TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(tgt, 0, 2000));
        TEST_ASSERT_EQUAL_INT16(0, duty);
    }
}

static void test_parked_restarts_beyond_band() {
    start(200, 0, 1000);
    step(200, 50, 1500);                // drive a while: boost/progress state accumulates
    tm.boost = 60;                      // pretend boost was earned
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::PARKED, (uint8_t)step(200, 198, 2000));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::RESTARTED, (uint8_t)step(209, 198, 3000));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::DRIVING, (uint8_t)st);
    TEST_ASSERT_EQUAL_UINT32(3000, tm.moveStartMs);
    TEST_ASSERT_EQUAL_UINT32(3000, tm.stallMs);
    TEST_ASSERT_EQUAL_INT32(198, tm.stallPos);
    TEST_ASSERT_EQUAL_UINT32(3000, tm.progressMs);
    TEST_ASSERT_EQUAL_INT32(198, tm.progressPos);
    TEST_ASSERT_EQUAL_INT16(0, tm.boost);
    TEST_ASSERT_TRUE(tm.breakaway);
    TEST_ASSERT_EQUAL_INT32(198, tm.breakawayStartPos);
    TEST_ASSERT_EQUAL_INT16(85, duty);    // 11 × 1.15 = 12 → break-away floor
}

// ─── Continuous tracking: a target update is not a new move ────────────────

static void test_target_update_keeps_timers() {
    start(300, 0, 1000);
    step(300, 0, 1010);
    step(300, 10, 1110);   // moved 1.0°: break-away disarmed, progress baseline at 1110
    TEST_ASSERT_FALSE(tm.breakaway);
    const SteerPosTimers before = tm;
    // New targets on both sides and tiny deltas (0.1°) — all just replace the target.
    step(301, 11, 1120);
    TEST_ASSERT_TRUE(duty > 0);
    step(250, 12, 1130);
    TEST_ASSERT_EQUAL_UINT32(before.moveStartMs, tm.moveStartMs);
    TEST_ASSERT_EQUAL_UINT32(before.stallMs, tm.stallMs);
    TEST_ASSERT_EQUAL_INT32(before.stallPos, tm.stallPos);
    TEST_ASSERT_EQUAL_UINT32(before.progressMs, tm.progressMs);
    TEST_ASSERT_EQUAL_INT32(before.progressPos, tm.progressPos);
    TEST_ASSERT_EQUAL_INT16(before.boost, tm.boost);
    TEST_ASSERT_FALSE(tm.breakaway);
    TEST_ASSERT_EQUAL_INT32(before.breakawayStartPos, tm.breakawayStartPos);
    // Large target change in the same direction: duty grows, timers still untouched.
    step(2000, 13, 1140);
    TEST_ASSERT_EQUAL_INT16(255, duty);
    TEST_ASSERT_EQUAL_UINT32(before.stallMs, tm.stallMs);
    TEST_ASSERT_FALSE(tm.breakaway);
}

// ─── Duty floors ───────────────────────────────────────────────────────────

static void test_breakaway_then_track_floor() {
    cur = &PF;
    start(20, 0, 1000);                 // 2° error from rest
    step(20, 0, 1010);
    TEST_ASSERT_EQUAL_INT16(100, duty);
    step(20, 2, 1020);                  // 0.2° moved: still break-away
    TEST_ASSERT_EQUAL_INT16(100, duty);
    step(23, 3, 1030);                  // 0.3° moved, 2° error: track floor
    TEST_ASSERT_EQUAL_INT16(85, duty);
    TEST_ASSERT_EQUAL_UINT8(85, steerPosFloor(tm, PF));
    step(25, 5, 1040);                  // target update: never re-arms the break-away floor
    TEST_ASSERT_EQUAL_INT16(85, duty);
    step(-300, 6, 1050);                // 30.6° error (reversal re-arms, but saturated anyway)
    TEST_ASSERT_EQUAL_INT16(-255, duty);
}

static void test_saturates_on_either_floor() {
    start(300, 0, 1000);
    step(300, 0, 1010);
    TEST_ASSERT_EQUAL_INT16(255, duty);   // break-away armed
    step(305, 5, 1020);
    TEST_ASSERT_FALSE(tm.breakaway);
    TEST_ASSERT_EQUAL_INT16(255, duty);   // tracking floor, 30° error
    start(-300, 0, 2000);
    st = SteerPosState::DRIVING;
    step(-300, 0, 2010);
    TEST_ASSERT_EQUAL_INT16(-255, duty);
}

static void test_direction_flip_rearms_breakaway() {
    cur = &PF;
    start(100, 0, 1000);
    step(100, 0, 1010);
    step(100, 20, 1100);
    TEST_ASSERT_FALSE(tm.breakaway);
    step(5, 25, 1110);                   // stick reversed: err −2°
    TEST_ASSERT_TRUE(tm.breakaway);
    TEST_ASSERT_EQUAL_INT32(25, tm.breakawayStartPos);
    TEST_ASSERT_EQUAL_INT16(-100, duty);
    step(0, 23, 1120);                   // 0.2° back: still break-away
    TEST_ASSERT_EQUAL_INT16(-100, duty);
    step(-5, 22, 1130);                  // 0.3° back from the reversal: track floor
    TEST_ASSERT_EQUAL_INT16(-85, duty);
}

static void test_restart_from_parked_rearms_breakaway() {
    cur = &PF;
    start(50, 0, 1000);
    step(50, 0, 1010);
    step(50, 20, 1100);
    TEST_ASSERT_FALSE(tm.breakaway);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::PARKED, (uint8_t)step(50, 49, 1200));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::RESTARTED, (uint8_t)step(60, 49, 1500));
    TEST_ASSERT_EQUAL_INT16(100, duty);
}

// Boost on top of the floor (fix-steering-jerk-and-centre-undershoot D1): the first stalled
// check already raises the duty above the floor, then it climbs by 30 per stalled check.
static void test_first_stalled_check_raises_duty_above_floor() {
    start(100, 0, 1000);
    step(100, 0, 1010);
    step(100, 5, 1050);                  // moved 0.5°: track floor
    TEST_ASSERT_FALSE(tm.breakaway);
    // Keep the error small via the target, but the wheel stops.
    // progress baseline: 1000 @0. At 1100: |5−0| ≥ 3 → boost stays 0, baseline 5 @1100.
    step(25, 5, 1100);
    TEST_ASSERT_EQUAL_INT16(0, tm.boost);
    TEST_ASSERT_EQUAL_INT16(85, duty);   // 2° × 1.15 = 23 < 85: floor
    step(25, 5, 1200);                   // first stalled check
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    TEST_ASSERT_EQUAL_INT16(115, duty);  // floor 85 + 30, not hidden under the floor
    step(25, 5, 1300);
    TEST_ASSERT_EQUAL_INT16(145, duty);
    step(25, 5, 1400);
    TEST_ASSERT_EQUAL_INT16(175, duty);
    step(25, 5, 1500);
    // stall window 1000 @0 → at 1500 |5−0| ≥ 5 ok, new window @1500 pos 5
    TEST_ASSERT_EQUAL_INT16(120, tm.boost);
    TEST_ASSERT_EQUAL_INT16(205, duty);  // 85 + 120
}

// Gentle decay (D2): −boostDecay (10 here) per moving check, never below 0; a new stall adds on
// top of what is left.
static void test_boost_decays_gently() {
    cur = &PD;
    start(100, 0, 1000);
    step(100, 0, 1010);
    step(100, 5, 1050);
    step(25, 5, 1100);                   // moving check: boost 0, baseline 5 @1100
    step(25, 5, 1200);                   // stalled: boost 30
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    step(26, 8, 1300);                   // moved 0.3°: −10
    TEST_ASSERT_EQUAL_INT16(20, tm.boost);
    TEST_ASSERT_EQUAL_INT16(105, duty);  // 1.8° × 1.15 = 20 < 85 → 85 + 20
    step(30, 11, 1400);
    TEST_ASSERT_EQUAL_INT16(10, tm.boost);
    TEST_ASSERT_EQUAL_INT16(95, duty);
    step(34, 14, 1500);
    TEST_ASSERT_EQUAL_INT16(0, tm.boost);
    TEST_ASSERT_EQUAL_INT16(85, duty);
    step(37, 17, 1600);                  // still moving: stays 0, never negative
    TEST_ASSERT_EQUAL_INT16(0, tm.boost);
    TEST_ASSERT_EQUAL_INT16(85, duty);
    step(37, 17, 1700);                  // stalled again: +30 from where it is
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    step(40, 20, 1800);                  // moving: 20
    step(40, 20, 1900);                  // stalled: 20 + 30
    TEST_ASSERT_EQUAL_INT16(50, tm.boost);
    tm.boost = 5;                        // a remainder smaller than the decay
    step(44, 24, 2000);
    TEST_ASSERT_EQUAL_INT16(0, tm.boost);
}

// Far from the target the P term rules (unchanged behaviour): duty = P + boost, clamped.
static void test_far_from_target_p_term_rules() {
    start(150, 0, 1000);                 // 15° error
    step(150, 0, 1010);
    TEST_ASSERT_EQUAL_INT16(172, duty);  // 150 × 1.15 = 172.5 > floor 85
    step(150, 0, 1100);                  // stalled check: +30
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    TEST_ASSERT_EQUAL_INT16(202, duty);  // 172.5 + 30
    step(200, 0, 1150);                  // 20°: 230 + 30 → clamped
    TEST_ASSERT_EQUAL_INT16(255, duty);
    setUp();
    start(-200, 0, 2000);                // 20° from rest, no boost
    step(-200, 0, 2010);
    TEST_ASSERT_EQUAL_INT16(-230, duty);
}

// Boost hold band (D7): while the wheel moves and |err| > 2.0° the boost is kept; inside 2.0°
// it decays by 30 per moving check; a stalled check inside the band still adds.
static void test_boost_held_far_from_target() {
    start(150, 0, 1000);                 // 15° error
    step(150, 0, 1010);
    step(150, 0, 1100);                  // stalled check: +30
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    step(150, 10, 1200);                 // moving, |err| 14° > 2°: held
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    TEST_ASSERT_EQUAL_INT16(191, duty);  // 140 × 1.15 = 161 + 30
    step(150, 60, 1300);                 // moving, 9°: held
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    step(150, 120, 1400);                // moving, 3°: still held
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    step(150, 129, 1500);                // moving, |err| 2.1° > 2.0°: held at the band edge
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
}

static void test_boost_decays_inside_hold_band() {
    start(150, 0, 1000);
    step(150, 0, 1010);
    step(150, 0, 1100);                  // stalled: 30
    step(150, 0, 1200);                  // stalled: 60
    TEST_ASSERT_EQUAL_INT16(60, tm.boost);
    step(150, 130, 1300);                // moving, |err| exactly 2.0°: decays by 30
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    TEST_ASSERT_EQUAL_INT16(115, duty);  // 20 × 1.15 = 23 < 85 → 85 + 30
    step(150, 134, 1400);                // moving, 1.6°: decays to 0
    TEST_ASSERT_EQUAL_INT16(0, tm.boost);
    TEST_ASSERT_EQUAL_INT16(85, duty);
}

static void test_stalled_check_inside_hold_band_adds() {
    start(150, 0, 1000);
    step(150, 0, 1010);
    step(150, 135, 1100);                // moving into the band (1.5°), no boost yet
    TEST_ASSERT_EQUAL_INT16(0, tm.boost);
    step(150, 135, 1200);                // stalled inside the band: +30
    TEST_ASSERT_EQUAL_INT16(30, tm.boost);
    TEST_ASSERT_EQUAL_INT16(115, duty);  // 15 × 1.15 = 17 < 85 → 85 + 30
    step(150, 135, 1300);                // stalled again: +30
    TEST_ASSERT_EQUAL_INT16(60, tm.boost);
}

// Park window 0.2° (D4): parks at |err| = 2, keeps driving at |err| = 3, from both sides.
static void test_parks_at_0p2_not_0p3_both_sides() {
    start(100, 0, 1000);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(100, 97, 1010));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::DRIVING, (uint8_t)st);
    TEST_ASSERT_EQUAL_INT16(85, duty);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::PARKED, (uint8_t)step(100, 98, 1020));
    TEST_ASSERT_EQUAL_INT16(0, duty);

    setUp();
    start(0, 100, 2000);                 // approach centre from the right
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(0, 3, 2010));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::DRIVING, (uint8_t)st);
    TEST_ASSERT_EQUAL_INT16(-85, duty);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::PARKED, (uint8_t)step(0, 2, 2020));

    setUp();
    start(0, -3, 3000);                  // from idle: 0.3° off drives, 0.2° parks at once
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::DRIVING, (uint8_t)st);
    setUp();
    start(0, -2, 3000);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::PARKED, (uint8_t)st);
}

// Restart hysteresis unchanged at 1.0°: parked stays at |err| = 10, restarts at 11.
static void test_restart_only_beyond_1p0() {
    start(0, 0, 1000);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::PARKED, (uint8_t)st);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(10, 0, 1010));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(-10, 0, 1020));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::PARKED, (uint8_t)st);
    TEST_ASSERT_EQUAL_INT16(0, duty);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::RESTARTED, (uint8_t)step(-11, 0, 1030));
    TEST_ASSERT_EQUAL_INT16(-85, duty);
    setUp();
    start(0, 0, 2000);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::RESTARTED, (uint8_t)step(11, 0, 2010));
    TEST_ASSERT_EQUAL_INT16(85, duty);
}

// ─── Stall ─────────────────────────────────────────────────────────────────

static void test_stall_with_breakaway_armed_under_stream() {
    start(100, 0, 1000);
    SteerPosEvent ev = SteerPosEvent::NONE;
    uint32_t t = 1000;
    int32_t tgt = 100;
    while (ev == SteerPosEvent::NONE && t < 3000) {
        t += 40;                          // 25 Hz stream, targets beyond the jam
        tgt += 3;
        ev = step(tgt, 0, t);
        if (ev == SteerPosEvent::NONE) TEST_ASSERT_TRUE(tm.breakaway);
    }
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::STALL, (uint8_t)ev);
    TEST_ASSERT_UINT32_WITHIN(40, 1500, t);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::IDLE, (uint8_t)st);
    TEST_ASSERT_EQUAL_INT16(0, duty);
}

static void test_stall_after_breakaway_at_track_floor() {
    start(50, 0, 1000);
    uint32_t t = 1000;
    int32_t pos = 0;
    // Track for 1 s: the wheel moves 1° per 100 ms behind a moving target.
    for (int i = 0; i < 100; i++) {
        t += 10;
        if (i % 10 == 0) pos += 10;
        TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)step(pos + 25, pos, t));
    }
    TEST_ASSERT_FALSE(tm.breakaway);
    // Jam at pos: targets keep arriving beyond it.
    const uint32_t jamAt = t;
    SteerPosEvent ev = SteerPosEvent::NONE;
    int32_t tgt = pos + 25;
    while (ev == SteerPosEvent::NONE && t < jamAt + 3000) {
        t += 40;
        tgt += 2;
        ev = step(tgt, pos, t);
    }
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::STALL, (uint8_t)ev);
    // The last window baseline was taken at or before the jam: trips within one window + a step.
    TEST_ASSERT_TRUE(t - jamAt <= 500 + 40);
    TEST_ASSERT_TRUE(t - jamAt >= 40);
}

static void test_slow_wheel_never_stalls() {
    start(5000, 0, 1000);
    uint32_t t = 1000;
    int32_t pos = 0;
    for (int i = 0; i < 1000; i++) {   // 10 s at 0.6° per 500 ms
        t += 10;
        if (i % 50 == 0) pos += 6;
        SteerPosEvent ev = step(pos + 8, pos, t);    // target just ahead, within restartTol
        TEST_ASSERT_TRUE(ev != SteerPosEvent::STALL);
        TEST_ASSERT_TRUE(ev != SteerPosEvent::TIMEOUT);
    }
}

static void test_direction_flip_restarts_window_and_zeroes_boost() {
    start(200, 0, 1000);
    step(200, 0, 1010);
    step(200, 4, 1400);
    tm.boost = 90;
    // Reversal at 1450: window restarts at pos 4, boost 0.
    step(-100, 4, 1450);
    TEST_ASSERT_EQUAL_UINT32(1450, tm.stallMs);
    TEST_ASSERT_EQUAL_INT32(4, tm.stallPos);
    TEST_ASSERT_EQUAL_INT16(0, tm.boost);
    // Through zero speed: the wheel barely moves until 1900, then picks up — no stall at 1500.
    SteerPosEvent ev = step(-100, 3, 1500);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)ev);
    ev = step(-100, -10, 1900);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)ev);
    ev = step(-100, -20, 1950);   // window 1450 @4 → |−20 − 4| ≥ 5: no stall
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::NONE, (uint8_t)ev);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::DRIVING, (uint8_t)st);
}

// ─── Move timeout ──────────────────────────────────────────────────────────

static void test_timeout_when_held_behind() {
    start(1000, 0, 1000);
    uint32_t t = 1000;
    int32_t pos = 0;
    SteerPosEvent ev = SteerPosEvent::NONE;
    while (ev == SteerPosEvent::NONE && t < 20000) {
        t += 10;
        if (t % 500 == 0) pos += 6;          // dragging: 0.6° per 500 ms keeps the stall detector quiet
        ev = step(pos + 100, pos, t);        // 10° behind, always
    }
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::TIMEOUT, (uint8_t)ev);
    TEST_ASSERT_EQUAL_UINT32(8500, t);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosState::IDLE, (uint8_t)st);
}

static void test_ramp_followed_never_times_out() {
    start(40, 0, 1000);
    uint32_t t = 1000;
    int32_t pos = 0;
    int32_t tgt = 40;
    int dir = 1;
    int drivingSteps = 0;
    for (int i = 0; i < 3000; i++) {   // 30 s, back and forth over ±300°
        t += 10;
        if (i % 4 == 0) {              // 25 Hz stream, 0.3°/frame (7.5°/s)
            tgt += dir * 3;
            if (tgt > 3000 || tgt < -3000) dir = -dir;
        }
        SteerPosEvent ev = step(tgt, pos, t);
        TEST_ASSERT_TRUE(ev != SteerPosEvent::TIMEOUT);
        TEST_ASSERT_TRUE(ev != SteerPosEvent::STALL);
        TEST_ASSERT_TRUE(st != SteerPosState::IDLE);
        // Wheel model: 0.2° per 10 ms (20°/s) while driven, held by the worm otherwise.
        if (duty > 0) pos += 2; else if (duty < 0) pos -= 2;
        if (st == SteerPosState::DRIVING) drivingSteps++;
        TEST_ASSERT_TRUE(tgt - pos <= 50 && pos - tgt <= 50);   // follows within 5°
    }
    TEST_ASSERT_TRUE(drivingSteps > 0);
}

static void test_lock_to_lock_step_times_out() {
    start(5000, -5000, 1000);
    uint32_t t = 1000;
    int32_t pos = -5000;
    SteerPosEvent ev = SteerPosEvent::NONE;
    while (ev == SteerPosEvent::NONE && t < 20000) {
        t += 10;
        pos += 1;                          // 10°/s: 1000° needs 100 s
        ev = step(5000, pos, t);
    }
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerPosEvent::TIMEOUT, (uint8_t)ev);
    TEST_ASSERT_EQUAL_UINT32(8500, t);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_idle_does_nothing);
    RUN_TEST(test_start_from_idle_drives_or_parks);
    RUN_TEST(test_drive_parks_within_pos_tol);
    RUN_TEST(test_parked_holds_inside_band);
    RUN_TEST(test_parked_restarts_beyond_band);
    RUN_TEST(test_target_update_keeps_timers);
    RUN_TEST(test_breakaway_then_track_floor);
    RUN_TEST(test_saturates_on_either_floor);
    RUN_TEST(test_direction_flip_rearms_breakaway);
    RUN_TEST(test_restart_from_parked_rearms_breakaway);
    RUN_TEST(test_first_stalled_check_raises_duty_above_floor);
    RUN_TEST(test_boost_decays_gently);
    RUN_TEST(test_far_from_target_p_term_rules);
    RUN_TEST(test_boost_held_far_from_target);
    RUN_TEST(test_boost_decays_inside_hold_band);
    RUN_TEST(test_stalled_check_inside_hold_band_adds);
    RUN_TEST(test_parks_at_0p2_not_0p3_both_sides);
    RUN_TEST(test_restart_only_beyond_1p0);
    RUN_TEST(test_stall_with_breakaway_armed_under_stream);
    RUN_TEST(test_stall_after_breakaway_at_track_floor);
    RUN_TEST(test_slow_wheel_never_stalls);
    RUN_TEST(test_direction_flip_restarts_window_and_zeroes_boost);
    RUN_TEST(test_timeout_when_held_behind);
    RUN_TEST(test_ramp_followed_never_times_out);
    RUN_TEST(test_lock_to_lock_step_times_out);
    return UNITY_END();
}
