// Native unit tests for the USB steering diagnostics logic: command parser, ramp / pulse /
// hold / goto state machine, summary values and safety predicates (include/SteerDiagLaw.h).
// Run: pio test -e native
#include <unity.h>
#include "SteerDiagLaw.h"

// Mirror of include/Constants.h (which needs Arduino.h and cannot be built natively).
static const SteerDiagParams P = {
    5,      // movePos        STEER_DIAG_MOVE_POS
    5,      // moveRate       STEER_DIAG_MOVE_RATE
    600,    // settleMs       STEER_DIAG_SETTLE_MS
    15000,  // rampTimeoutMs  STEER_DIAG_RAMP_TIMEOUT_MS
    1000,   // rampMaxHoldMs  STEER_DIAG_RAMP_MAXHOLD_MS
    12000,  // gotoTimeoutMs  STEER_DIAG_GOTO_TIMEOUT_MS
};
static const SteerDiagLimits L = {255, 1000, 1500, 60.0f};
static const int32_t UNSET = INT32_MIN;

static SteerDiagExp e;

void setUp() { e = SteerDiagExp(); }
void tearDown() {}

static SteerDiagSample smp(uint32_t now, int32_t pos, float cur = 0.0f, int32_t rate = 0, bool rateValid = false,
                           SteerDiagLoop loop = SteerDiagLoop::IDLE) {
    SteerDiagSample s;
    s.now = now;
    s.pos = pos;
    s.currentA = cur;
    s.rate = rate;
    s.rateValid = rateValid;
    s.loop = loop;
    return s;
}

// ---------------------------------------------------------------- parser

static void test_parse_basic_commands() {
    TEST_ASSERT_EQUAL(SteerDiagCmdType::DIAG_ON, steerDiagParse("diag on", L).type);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::DIAG_OFF, steerDiagParse("  DIAG   Off\r\n", L).type);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::STATUS, steerDiagParse("status\r", L).type);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::STOP, steerDiagParse("s", L).type);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::STOP, steerDiagParse("stop", L).type);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::HELP, steerDiagParse("?", L).type);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::NONE, steerDiagParse("  \r\n", L).type);
    SteerDiagCmd c = steerDiagParse("fly away", L);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::INVALID, c.type);
    TEST_ASSERT_EQUAL_STRING("unknown_cmd", c.error);
    TEST_ASSERT_EQUAL_STRING("bad_arg", steerDiagParse("diag maybe", L).error);
}

static void test_parse_ramp_defaults_and_keys() {
    SteerDiagCmd c = steerDiagParse("ramp L", L);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::RAMP, c.type);
    TEST_ASSERT_EQUAL(-1, c.dir);
    TEST_ASSERT_EQUAL(220, c.maxDuty);
    TEST_ASSERT_EQUAL(2, c.step);
    TEST_ASSERT_EQUAL(150, c.stepMs);
    TEST_ASSERT_EQUAL(40, c.startDuty);
    c = steerDiagParse("RAMP r max=200 step=4 stepMs=100 start=60", L);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::RAMP, c.type);
    TEST_ASSERT_EQUAL(1, c.dir);
    TEST_ASSERT_EQUAL(200, c.maxDuty);
    TEST_ASSERT_EQUAL(4, c.step);
    TEST_ASSERT_EQUAL(100, c.stepMs);
    TEST_ASSERT_EQUAL(60, c.startDuty);
    TEST_ASSERT_EQUAL_STRING("range", steerDiagParse("ramp r max=300", L).error);
    TEST_ASSERT_EQUAL_STRING("range", steerDiagParse("ramp r max=50 start=60", L).error);
    TEST_ASSERT_EQUAL_STRING("bad_arg", steerDiagParse("ramp r foo=1", L).error);
    TEST_ASSERT_EQUAL_STRING("bad_arg", steerDiagParse("ramp r max=abc", L).error);
    TEST_ASSERT_EQUAL_STRING("bad_dir", steerDiagParse("ramp up", L).error);
    TEST_ASSERT_EQUAL_STRING("missing_arg", steerDiagParse("ramp", L).error);
}

static void test_parse_pulse_hold_goto() {
    SteerDiagCmd c = steerDiagParse("pulse R 150 70", L);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::PULSE, c.type);
    TEST_ASSERT_EQUAL(1, c.dir);
    TEST_ASSERT_EQUAL(150, c.duty);
    TEST_ASSERT_EQUAL(70, c.ms);
    TEST_ASSERT_EQUAL_STRING("range", steerDiagParse("pulse r 150 2000", L).error);
    TEST_ASSERT_EQUAL_STRING("range", steerDiagParse("pulse r 256 50", L).error);
    TEST_ASSERT_EQUAL_STRING("missing_arg", steerDiagParse("pulse r 150", L).error);
    c = steerDiagParse("hold l 200 1500", L);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::HOLD, c.type);
    TEST_ASSERT_EQUAL(-1, c.dir);
    TEST_ASSERT_EQUAL_STRING("range", steerDiagParse("hold l 200 1501", L).error);
    c = steerDiagParse("goto -12.34", L);
    TEST_ASSERT_EQUAL(SteerDiagCmdType::GOTO, c.type);
    TEST_ASSERT_EQUAL(-123, c.target);
    TEST_ASSERT_EQUAL(0, steerDiagParse("goto 0", L).target);
    TEST_ASSERT_EQUAL_STRING("range", steerDiagParse("goto 61", L).error);
    TEST_ASSERT_EQUAL_STRING("bad_arg", steerDiagParse("goto nan", L).error);
    TEST_ASSERT_EQUAL_STRING("bad_arg", steerDiagParse("goto 1x", L).error);
}

// ---------------------------------------------------------------- safety

static void test_safety_predicates() {
    TEST_ASSERT_FALSE(steerDiagMoving(false, 10.0f, 0.5f));   // untrusted reading
    TEST_ASSERT_FALSE(steerDiagMoving(true, 0.13f, 0.5f));    // 0.47 km/h
    TEST_ASSERT_TRUE(steerDiagMoving(true, 0.15f, 0.5f));     // 0.54 km/h
    // Limits -280 / +280, margin 20
    TEST_ASSERT_TRUE(steerDiagNearLimit(260, 1, -280, 280, 20, UNSET));
    TEST_ASSERT_FALSE(steerDiagNearLimit(259, 1, -280, 280, 20, UNSET));
    TEST_ASSERT_FALSE(steerDiagNearLimit(270, -1, -280, 280, 20, UNSET));   // driving away
    TEST_ASSERT_FALSE(steerDiagNearLimit(270, 0, -280, 280, 20, UNSET));    // motor off
    TEST_ASSERT_TRUE(steerDiagNearLimit(-265, -1, -280, 280, 20, UNSET));
    TEST_ASSERT_FALSE(steerDiagNearLimit(500, 1, -280, UNSET, 20, UNSET));  // unset side not checked
    TEST_ASSERT_TRUE(steerDiagTargetOk(0, -280, 280, 20));
    TEST_ASSERT_FALSE(steerDiagTargetOk(260, -280, 280, 20));
    TEST_ASSERT_FALSE(steerDiagTargetOk(-260, -280, 280, 20));
}

static void test_host_lost_debounce() {
    SteerDiagHostWatch w;
    // Never trips before it has seen the host once
    TEST_ASSERT_FALSE(steerDiagHostLost(w, false, 0, 500));
    TEST_ASSERT_FALSE(steerDiagHostLost(w, false, 5000, 500));
    TEST_ASSERT_FALSE(steerDiagHostLost(w, true, 6000, 500));   // armed
    TEST_ASSERT_FALSE(steerDiagHostLost(w, false, 6100, 500));  // low starts
    TEST_ASSERT_FALSE(steerDiagHostLost(w, false, 6599, 500));
    TEST_ASSERT_FALSE(steerDiagHostLost(w, true, 6599, 500));   // true again: timer reset
    TEST_ASSERT_FALSE(steerDiagHostLost(w, false, 6600, 500));
    TEST_ASSERT_FALSE(steerDiagHostLost(w, false, 7099, 500));
    TEST_ASSERT_TRUE(steerDiagHostLost(w, false, 7100, 500));   // 500 ms continuous false
}

static void test_keepalive_window() {
    // No line ever received and diag off: never (boot, host absent)
    TEST_ASSERT_FALSE(steerDiagKeepaliveDue(false, false, false, 0, 0, 1000, 30000, 100));
    // Line 5 s ago, last keepalive 100 ms ago: due; 99 ms ago: not yet
    TEST_ASSERT_TRUE(steerDiagKeepaliveDue(false, false, true, 10000, 14900, 15000, 30000, 100));
    TEST_ASSERT_FALSE(steerDiagKeepaliveDue(false, false, true, 10000, 14901, 15000, 30000, 100));
    // Window closed 30 s after the last line (diag off)
    TEST_ASSERT_FALSE(steerDiagKeepaliveDue(false, false, true, 10000, 0, 40000, 30000, 100));
    // Diag on keeps the window open regardless of the last line
    TEST_ASSERT_TRUE(steerDiagKeepaliveDue(true, false, true, 10000, 0, 90000, 30000, 100));
    // D-lines flowing: skipped
    TEST_ASSERT_FALSE(steerDiagKeepaliveDue(true, true, true, 10000, 0, 90000, 30000, 100));
    // millis() wrap: line just before wrap, now just after
    TEST_ASSERT_TRUE(steerDiagKeepaliveDue(false, false, true, 0xFFFFFF00u, 0xFFFFFF00u, 0x100u, 30000, 100));
}

// ---------------------------------------------------------------- ramp

static void test_ramp_steps_detects_and_coasts() {
    steerDiagBeginRamp(e, 1, 40, 220, 2, 150, 100, 1000);
    SteerDiagOut o = steerDiagStep(e, P, smp(1000, 100));
    TEST_ASSERT_EQUAL(40, o.duty);
    o = steerDiagStep(e, P, smp(1149, 100));
    TEST_ASSERT_EQUAL(40, o.duty);
    o = steerDiagStep(e, P, smp(1150, 100));
    TEST_ASSERT_EQUAL(42, o.duty);
    TEST_ASSERT_TRUE(o.phaseChanged);
    // Run on to duty 130 (45 steps) without movement
    uint32_t t = 1150;
    for (int i = 0; i < 44; ++i) { t += 150; o = steerDiagStep(e, P, smp(t, 102)); }
    TEST_ASSERT_EQUAL(130, o.duty);
    // 0.4° is not movement, 0.5° is
    o = steerDiagStep(e, P, smp(t + 10, 104, 3.5f));
    TEST_ASSERT_EQUAL(130, o.duty);
    o = steerDiagStep(e, P, smp(t + 20, 105, 4.25f));
    TEST_ASSERT_EQUAL(0, o.duty);
    TEST_ASSERT_TRUE(o.phaseChanged);
    TEST_ASSERT_EQUAL(SteerDiagPhase::SETTLE, e.phase);
    TEST_ASSERT_TRUE(e.moved);
    TEST_ASSERT_EQUAL(130, e.breakDuty);
    TEST_ASSERT_EQUAL_FLOAT(4.25f, e.breakCurrentA);
    TEST_ASSERT_EQUAL(t + 20 - 1000, e.breakMs);
    // Coast: keeps turning 1.2° after the cut
    o = steerDiagStep(e, P, smp(t + 300, 113));
    TEST_ASSERT_FALSE(o.done);
    o = steerDiagStep(e, P, smp(t + 620, 117));
    TEST_ASSERT_TRUE(o.done);
    TEST_ASSERT_EQUAL(0, o.duty);
    TEST_ASSERT_EQUAL_STRING("moved", e.endReason);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.2f, steerDiagCoastDeg(e));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, steerDiagMovedAtCutDeg(e));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.7f, steerDiagMovedEndDeg(e));
    TEST_ASSERT_FALSE(e.running());
    // After DONE the machine is inert
    o = steerDiagStep(e, P, smp(t + 700, 117));
    TEST_ASSERT_FALSE(o.done);
}

static void test_ramp_left_rate_detection_ignores_wrong_direction() {
    steerDiagBeginRamp(e, -1, 100, 220, 2, 150, 0, 0);
    SteerDiagOut o = steerDiagStep(e, P, smp(10, 0, 1.0f, 8, true));   // rate to the right: ignored
    TEST_ASSERT_EQUAL(-100, o.duty);
    o = steerDiagStep(e, P, smp(20, -1, 1.0f, -6, true));               // 6 °/s to the left
    TEST_ASSERT_EQUAL(0, o.duty);
    TEST_ASSERT_TRUE(e.moved);
    TEST_ASSERT_EQUAL(100, e.breakDuty);
    // Position movement in the wrong direction never counts
    steerDiagBeginRamp(e, -1, 100, 220, 2, 150, 0, 0);
    o = steerDiagStep(e, P, smp(10, 20));
    TEST_ASSERT_EQUAL(-100, o.duty);
}

static void test_ramp_no_move_and_timeout() {
    steerDiagBeginRamp(e, 1, 200, 210, 5, 100, 0, 0);
    SteerDiagOut o = steerDiagStep(e, P, smp(100, 0));
    TEST_ASSERT_EQUAL(205, o.duty);
    o = steerDiagStep(e, P, smp(200, 0));
    TEST_ASSERT_EQUAL(210, o.duty);   // capped at max
    o = steerDiagStep(e, P, smp(1199, 0));
    TEST_ASSERT_EQUAL(210, o.duty);
    o = steerDiagStep(e, P, smp(1200, 0));   // 1 s at max
    TEST_ASSERT_EQUAL(0, o.duty);
    TEST_ASSERT_EQUAL_STRING("no_move", e.endReason);
    TEST_ASSERT_FALSE(e.moved);
    // Hard timeout with a slow ramp
    steerDiagBeginRamp(e, 1, 0, 255, 1, 2000, 0, 0);
    o = steerDiagStep(e, P, smp(14999, 0));
    TEST_ASSERT_TRUE(o.duty > 0);
    o = steerDiagStep(e, P, smp(15000, 0));
    TEST_ASSERT_EQUAL(0, o.duty);
    TEST_ASSERT_EQUAL_STRING("timeout", e.endReason);
}

// ---------------------------------------------------------------- pulse / hold

static void test_pulse_length_and_summary() {
    steerDiagBeginTimed(e, SteerDiagKind::PULSE, -1, 150, 70, 50, 500);
    SteerDiagOut o = steerDiagStep(e, P, smp(500, 50, -2.0f));
    TEST_ASSERT_EQUAL(-150, o.duty);
    o = steerDiagStep(e, P, smp(569, 47, -5.5f));
    TEST_ASSERT_EQUAL(-150, o.duty);
    o = steerDiagStep(e, P, smp(570, 46, -3.0f));
    TEST_ASSERT_EQUAL(0, o.duty);
    TEST_ASSERT_EQUAL(70, e.cutMs);
    o = steerDiagStep(e, P, smp(1170, 42));
    TEST_ASSERT_TRUE(o.done);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.4f, steerDiagMovedAtCutDeg(e));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.8f, steerDiagMovedEndDeg(e));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, steerDiagCoastDeg(e));   // kept going left
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.5f, e.peakCurrentA);
}

static void test_hold_second_half_averages() {
    steerDiagBeginTimed(e, SteerDiagKind::HOLD, 1, 200, 1000, 0, 0);
    // 30 °/s = 3 (0.1°) per 10 ms; current 2 A in the 1st half, 4 A in the 2nd
    SteerDiagOut o;
    for (uint32_t t = 0; t < 1000; t += 10) {
        o = steerDiagStep(e, P, smp(t, (int32_t)(t * 3 / 10), t < 500 ? 2.0f : 4.0f, 30, true));
        TEST_ASSERT_EQUAL(200, o.duty);
    }
    o = steerDiagStep(e, P, smp(1000, 300, 0.5f, 30, true));
    TEST_ASSERT_EQUAL(0, o.duty);
    TEST_ASSERT_TRUE(e.halfSeen);
    TEST_ASSERT_EQUAL(500, e.halfMs);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, steerDiagHoldPosRate(e));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, steerDiagHoldLwsRate(e));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f, steerDiagHoldMeanCurrent(e));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f, e.peakCurrentA);
    o = steerDiagStep(e, P, smp(1600, 320));
    TEST_ASSERT_TRUE(o.done);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, steerDiagCoastDeg(e));
}

// ---------------------------------------------------------------- goto

static void test_goto_overshoot_and_residual() {
    steerDiagBeginGoto(e, 0, 100, 0);   // from +10° to 0°: travel left
    TEST_ASSERT_EQUAL(-1, e.dir);
    SteerDiagOut o = steerDiagStep(e, P, smp(20, 90, 3.0f, 0, false, SteerDiagLoop::DRIVE));
    TEST_ASSERT_FALSE(o.phaseChanged);
    o = steerDiagStep(e, P, smp(400, -15, 3.0f, 0, false, SteerDiagLoop::DRIVE));   // 1.5° past
    o = steerDiagStep(e, P, smp(420, -8, 1.0f, 0, false, SteerDiagLoop::PARK));
    TEST_ASSERT_TRUE(o.phaseChanged);
    TEST_ASSERT_EQUAL(SteerDiagPhase::SETTLE, e.phase);
    TEST_ASSERT_EQUAL(420, e.firstStopMs);
    o = steerDiagStep(e, P, smp(1019, -9, 0, 0, false, SteerDiagLoop::PARK));
    TEST_ASSERT_FALSE(o.done);
    o = steerDiagStep(e, P, smp(1020, -9, 0, 0, false, SteerDiagLoop::PARK));
    TEST_ASSERT_TRUE(o.done);
    TEST_ASSERT_EQUAL_STRING("park", e.endReason);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5f, steerDiagOvershootDeg(e));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.9f, steerDiagResidualDeg(e));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, e.peakCurrentA);
    TEST_ASSERT_EQUAL(1020, e.endMs);
}

static void test_goto_undershoot_restart_and_timeout() {
    steerDiagBeginGoto(e, 0, -100, 0);   // from −10° to 0°: travel right
    steerDiagStep(e, P, smp(100, -50, 0, 0, false, SteerDiagLoop::DRIVE));
    steerDiagStep(e, P, smp(200, -40, 0, 0, false, SteerDiagLoop::PARK));
    TEST_ASSERT_EQUAL(SteerDiagPhase::SETTLE, e.phase);
    // Restart (drive again) within the settle window
    SteerDiagOut o = steerDiagStep(e, P, smp(500, -40, 0, 0, false, SteerDiagLoop::DRIVE));
    TEST_ASSERT_TRUE(o.phaseChanged);
    TEST_ASSERT_EQUAL(1, e.restarts);
    steerDiagStep(e, P, smp(700, -5, 0, 0, false, SteerDiagLoop::PARK));
    o = steerDiagStep(e, P, smp(1300, -5, 0, 0, false, SteerDiagLoop::PARK));
    TEST_ASSERT_TRUE(o.done);
    TEST_ASSERT_EQUAL(200, e.firstStopMs);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, steerDiagOvershootDeg(e));   // never passed the target
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, steerDiagResidualDeg(e));
    // Timeout while still driving
    steerDiagBeginGoto(e, 200, 0, 0);
    o = steerDiagStep(e, P, smp(11999, 50, 0, 0, false, SteerDiagLoop::DRIVE));
    TEST_ASSERT_FALSE(o.done);
    o = steerDiagStep(e, P, smp(12000, 50, 0, 0, false, SteerDiagLoop::DRIVE));
    TEST_ASSERT_TRUE(o.done);
    TEST_ASSERT_EQUAL_STRING("timeout", e.endReason);
    // Loop dropped to idle (stall / guard)
    steerDiagBeginGoto(e, 200, 0, 0);
    steerDiagStep(e, P, smp(100, 20, 0, 0, false, SteerDiagLoop::DRIVE));
    steerDiagStep(e, P, smp(600, 21, 0, 0, false, SteerDiagLoop::IDLE));
    o = steerDiagStep(e, P, smp(1200, 21, 0, 0, false, SteerDiagLoop::IDLE));
    TEST_ASSERT_TRUE(o.done);
    TEST_ASSERT_EQUAL_STRING("idle", e.endReason);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 17.9f, steerDiagResidualDeg(e));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_parse_basic_commands);
    RUN_TEST(test_parse_ramp_defaults_and_keys);
    RUN_TEST(test_parse_pulse_hold_goto);
    RUN_TEST(test_safety_predicates);
    RUN_TEST(test_host_lost_debounce);
    RUN_TEST(test_keepalive_window);
    RUN_TEST(test_ramp_steps_detects_and_coasts);
    RUN_TEST(test_ramp_left_rate_detection_ignores_wrong_direction);
    RUN_TEST(test_ramp_no_move_and_timeout);
    RUN_TEST(test_pulse_length_and_summary);
    RUN_TEST(test_hold_second_half_averages);
    RUN_TEST(test_goto_overshoot_and_residual);
    RUN_TEST(test_goto_undershoot_restart_and_timeout);
    return UNITY_END();
}
