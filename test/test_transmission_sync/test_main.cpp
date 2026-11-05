// Native unit tests for the transmission gear sync logic: initialisation source choice, the
// initialisation-step overshoot direction hint and the idle resync counter
// (include/TransmissionLaw.h).
// Run: pio test -e native
#include <unity.h>
#include <math.h>
#include "TransmissionLaw.h"

// Mirror of the default gear positions in include/Constants.h (which needs Arduino.h and
// cannot be built natively): H=58, L=68, N=48, R=40, indexed by TransGear.
static const float POS[4] = {58.0f, 68.0f, 48.0f, 40.0f};

static const TransGear H = TransGear::GEAR_HIGH;
static const TransGear L = TransGear::GEAR_LOW;
static const TransGear N = TransGear::GEAR_NEUTRAL;
static const TransGear R = TransGear::GEAR_REVERSE;
static const TransGear U = TransGear::GEAR_UNKNOWN;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- source choice

static void test_sensor_wins_over_valid_nvs() {
    TransInitDecision d = chooseTransmissionInit(L, POS, true, true, N, 48.0f);
    TEST_ASSERT_EQUAL(TransInitSource::SENSOR, d.source);
    TEST_ASSERT_EQUAL(L, d.gear);
    TEST_ASSERT_EQUAL_FLOAT(68.0f, d.pct);

    d = chooseTransmissionInit(R, POS, false, false, U, NAN);
    TEST_ASSERT_EQUAL(TransInitSource::SENSOR, d.source);
    TEST_ASSERT_EQUAL(R, d.gear);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, d.pct);
}

static void test_sensor_unknown_valid_nvs_used() {
    TransInitDecision d = chooseTransmissionInit(U, POS, true, true, H, 58.3f);
    TEST_ASSERT_EQUAL(TransInitSource::NVS, d.source);
    TEST_ASSERT_EQUAL(H, d.gear);
    TEST_ASSERT_EQUAL_FLOAT(58.3f, d.pct);

    // Boundary positions are plausible.
    TEST_ASSERT_EQUAL(TransInitSource::NVS, chooseTransmissionInit(U, POS, true, true, R, 0.0f).source);
    TEST_ASSERT_EQUAL(TransInitSource::NVS, chooseTransmissionInit(U, POS, true, true, L, 100.0f).source);
}

static void test_nvs_unusable_steps_to_neutral() {
    // invalid flag
    TransInitDecision d = chooseTransmissionInit(U, POS, true, false, L, 68.0f);
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL, d.source);
    TEST_ASSERT_EQUAL(N, d.gear);
    TEST_ASSERT_EQUAL_FLOAT(48.0f, d.pct);
    // missing record
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL, chooseTransmissionInit(U, POS, false, false, U, NAN).source);
    // valid flag but UNKNOWN / out-of-enum gear
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL, chooseTransmissionInit(U, POS, true, true, U, 48.0f).source);
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL,
                      chooseTransmissionInit(U, POS, true, true, (TransGear)7, 48.0f).source);
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL,
                      chooseTransmissionInit(U, POS, true, true, (TransGear)-1, 48.0f).source);
    // valid flag but out-of-range / NaN pct
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL, chooseTransmissionInit(U, POS, true, true, L, -57.1f).source);
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL, chooseTransmissionInit(U, POS, true, true, L, 100.5f).source);
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL, chooseTransmissionInit(U, POS, true, true, L, NAN).source);
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL, chooseTransmissionInit(U, POS, true, true, L, INFINITY).source);
}

// ---------------------------------------------------------------- step direction

static float stepDir(bool found, TransGear gear, float pct) {
    TransInitDecision d = chooseTransmissionInit(U, POS, found, false, gear, pct);
    TEST_ASSERT_EQUAL(TransInitSource::STEP_TO_NEUTRAL, d.source);
    return d.direction;
}

static void test_step_direction_from_r_side() {
    // Interrupted R -> N step started at R (40 %): lever on the R side, overshoot toward H.
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(true, N, 40.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(true, R, 45.0f));
}

static void test_step_direction_from_hl_side() {
    // Interrupted H -> L or L -> H step: lever on the H/L side, overshoot toward R.
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, stepDir(true, L, 58.0f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, stepDir(true, H, 68.0f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, stepDir(true, N, 58.0f));
}

static void test_step_direction_no_record() {
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(false, U, NAN));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(false, L, 68.0f));  // fields ignored without a record
}

static void test_step_direction_bogus_pct() {
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(true, L, NAN));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(true, L, -57.1f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(true, L, 100.1f));
}

static void test_step_direction_pct_at_neutral_uses_state_gear() {
    // The interrupted step started at N: the lever lies between N and state_gear.
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, stepDir(true, H, 48.0f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, stepDir(true, L, 48.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(true, R, 48.0f));
    // Read back from servo µs: within the tolerance still counts as N.
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, stepDir(true, H, 47.93f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(true, R, 48.05f));
    // No usable gear for the tie: +1.
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(true, N, 48.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, stepDir(true, U, 48.0f));
}

// ---------------------------------------------------------------- resync

static void test_resync_fires_on_second_identical_mismatch() {
    TransmissionResync r;
    TEST_ASSERT_FALSE(r.step(L, N, 2));
    TEST_ASSERT_TRUE(r.step(L, N, 2));
    // Count reset after firing.
    TEST_ASSERT_EQUAL_UINT8(0, r.count);
}

static void test_resync_resets_on_match() {
    TransmissionResync r;
    TEST_ASSERT_FALSE(r.step(L, N, 2));
    TEST_ASSERT_FALSE(r.step(N, N, 2));
    TEST_ASSERT_FALSE(r.step(L, N, 2));
    TEST_ASSERT_TRUE(r.step(L, N, 2));
}

static void test_resync_resets_on_unknown() {
    TransmissionResync r;
    TEST_ASSERT_FALSE(r.step(L, N, 2));
    TEST_ASSERT_FALSE(r.step(U, N, 2));
    TEST_ASSERT_FALSE(r.step(L, N, 2));
    TEST_ASSERT_TRUE(r.step(L, N, 2));
}

static void test_resync_restarts_on_different_gear() {
    TransmissionResync r;
    TEST_ASSERT_FALSE(r.step(L, N, 2));
    TEST_ASSERT_FALSE(r.step(H, N, 2));
    TEST_ASSERT_FALSE(r.step(L, N, 2));
    TEST_ASSERT_TRUE(r.step(L, N, 2));
}

static void test_resync_never_when_sensor_matches() {
    TransmissionResync r;
    for (int i = 0; i < 10; i++) TEST_ASSERT_FALSE(r.step(N, N, 2));
    for (int i = 0; i < 10; i++) TEST_ASSERT_FALSE(r.step(U, N, 2));
}

static void test_resync_reset_clears_count() {
    TransmissionResync r;
    TEST_ASSERT_FALSE(r.step(L, N, 2));
    r.reset();  // a gear change started
    TEST_ASSERT_FALSE(r.step(L, N, 2));
    TEST_ASSERT_TRUE(r.step(L, N, 2));
}

// ── ShiftConfirm (speed-up-gear-change-with-sensor) ─────────────────────────

static const uint32_t CONFIRM_MS = 200;
static const uint32_t DWELL_TIMEOUT_MS = 2000;
static const uint32_t PAUSE_TIMEOUT_MS = 1000;

// Feeds a read every `stepMs` from `from` to `to` (inclusive) with the given match; returns
// the first non-WAIT result and its time in *atMs (or WAIT / to+1 if none).
static ShiftConfirmResult runReads(ShiftConfirm& c, uint32_t startMs, uint32_t from, uint32_t to,
                                   uint32_t stepMs, bool match, uint32_t timeoutMs,
                                   uint32_t* atMs) {
    for (uint32_t t = from; t <= to; t += stepMs) {
        ShiftConfirmResult r = c.step(t, startMs, match, CONFIRM_MS, timeoutMs);
        if (r != ShiftConfirmResult::WAIT) { *atMs = t; return r; }
    }
    *atMs = to + 1;
    return ShiftConfirmResult::WAIT;
}

void test_confirm_exactly_at_window() {
    ShiftConfirm c;
    uint32_t start = 1000, at = 0;
    // Match from the first read: CONFIRMED exactly 200 ms after it, not before.
    ShiftConfirmResult r = runReads(c, start, start, start + DWELL_TIMEOUT_MS, 10, true,
                                    DWELL_TIMEOUT_MS, &at);
    TEST_ASSERT_EQUAL(ShiftConfirmResult::CONFIRMED, r);
    TEST_ASSERT_EQUAL_UINT32(start + CONFIRM_MS, at);
}

void test_confirm_window_from_first_matching_read() {
    ShiftConfirm c;
    uint32_t start = 0;
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(100, start, false, CONFIRM_MS, DWELL_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(500, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(699, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(ShiftConfirmResult::CONFIRMED, c.step(700, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
}

void test_confirm_mismatch_restarts_window() {
    ShiftConfirm c;
    uint32_t start = 0, at = 0;
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(0, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(100, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
    // Mismatch (other gear or UNKNOWN) at 150 ms restarts the window.
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(150, start, false, CONFIRM_MS, DWELL_TIMEOUT_MS));
    TEST_ASSERT_FALSE(c.armed);
    // 200 ms after the original start is no longer enough.
    ShiftConfirmResult r = runReads(c, start, 200, DWELL_TIMEOUT_MS, 50, true, DWELL_TIMEOUT_MS, &at);
    TEST_ASSERT_EQUAL(ShiftConfirmResult::CONFIRMED, r);
    TEST_ASSERT_EQUAL_UINT32(400, at);   // window from the read at 200 ms
}

void test_confirm_timeout_dwell_without_match() {
    ShiftConfirm c;
    uint32_t start = 5000, at = 0;
    ShiftConfirmResult r = runReads(c, start, start, start + 3000, 100, false, DWELL_TIMEOUT_MS, &at);
    TEST_ASSERT_EQUAL(ShiftConfirmResult::TIMEOUT, r);
    TEST_ASSERT_EQUAL_UINT32(start + DWELL_TIMEOUT_MS, at);
}

void test_confirm_timeout_pause_without_match() {
    ShiftConfirm c;
    uint32_t start = 5000, at = 0;
    ShiftConfirmResult r = runReads(c, start, start, start + 3000, 100, false, PAUSE_TIMEOUT_MS, &at);
    TEST_ASSERT_EQUAL(ShiftConfirmResult::TIMEOUT, r);
    TEST_ASSERT_EQUAL_UINT32(start + PAUSE_TIMEOUT_MS, at);
}

void test_confirm_late_match_still_times_out() {
    ShiftConfirm c;
    uint32_t start = 0;
    for (uint32_t t = 0; t < 1900; t += 100) {
        TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(t, start, false, CONFIRM_MS, DWELL_TIMEOUT_MS));
    }
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(1900, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(ShiftConfirmResult::TIMEOUT, c.step(2000, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
}

void test_confirm_millis_wrap() {
    ShiftConfirm c;
    uint32_t start = 0xFFFFFF00u;   // wraps 256 ms later
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(start + 50, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(start + 249, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(ShiftConfirmResult::CONFIRMED, c.step(start + 250, start, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
}

void test_confirm_reset_clears_window() {
    ShiftConfirm c;
    c.step(0, 0, true, CONFIRM_MS, DWELL_TIMEOUT_MS);
    c.reset();
    TEST_ASSERT_FALSE(c.armed);
    // New wait starting at 1000: the old window (from 0) must not count.
    TEST_ASSERT_EQUAL(ShiftConfirmResult::WAIT, c.step(1000, 1000, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(ShiftConfirmResult::CONFIRMED, c.step(1200, 1000, true, CONFIRM_MS, DWELL_TIMEOUT_MS));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_sensor_wins_over_valid_nvs);
    RUN_TEST(test_sensor_unknown_valid_nvs_used);
    RUN_TEST(test_nvs_unusable_steps_to_neutral);
    RUN_TEST(test_step_direction_from_r_side);
    RUN_TEST(test_step_direction_from_hl_side);
    RUN_TEST(test_step_direction_no_record);
    RUN_TEST(test_step_direction_bogus_pct);
    RUN_TEST(test_step_direction_pct_at_neutral_uses_state_gear);
    RUN_TEST(test_resync_fires_on_second_identical_mismatch);
    RUN_TEST(test_resync_resets_on_match);
    RUN_TEST(test_resync_resets_on_unknown);
    RUN_TEST(test_resync_restarts_on_different_gear);
    RUN_TEST(test_resync_never_when_sensor_matches);
    RUN_TEST(test_resync_reset_clears_count);
    RUN_TEST(test_confirm_exactly_at_window);
    RUN_TEST(test_confirm_window_from_first_matching_read);
    RUN_TEST(test_confirm_mismatch_restarts_window);
    RUN_TEST(test_confirm_timeout_dwell_without_match);
    RUN_TEST(test_confirm_timeout_pause_without_match);
    RUN_TEST(test_confirm_late_match_still_times_out);
    RUN_TEST(test_confirm_millis_wrap);
    RUN_TEST(test_confirm_reset_clears_window);
    return UNITY_END();
}
