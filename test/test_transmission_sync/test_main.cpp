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
    return UNITY_END();
}
