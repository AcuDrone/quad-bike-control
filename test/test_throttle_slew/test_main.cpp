// Native unit tests for the throttle opening rate limit (include/ThrottleSlewLaw.h).
// Run: pio test -e native
#include <unity.h>
#include <math.h>
#include "ThrottleSlewLaw.h"

// Mirror of include/Constants.h (which needs Arduino.h and cannot be built natively).
static const float RATE_DEFAULT = 60.0f;   // THROTTLE_OPEN_SLEW_DEFAULT_PCT_S

void setUp() {}
void tearDown() {}

// Rising is limited to lastPct + rate × dt.
static void test_rising_is_limited() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.0f, throttleSlewStep(100.0f, 0.0f, 0.1f, RATE_DEFAULT));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 26.0f, throttleSlewStep(100.0f, 20.0f, 0.1f, RATE_DEFAULT));
    // A target within reach is reached exactly, never overshot.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 23.0f, throttleSlewStep(23.0f, 20.0f, 0.1f, RATE_DEFAULT));
}

// Falling (and holding) is immediate, whatever the rate.
static void test_falling_is_immediate() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, throttleSlewStep(10.0f, 80.0f, 0.001f, RATE_DEFAULT));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, throttleSlewStep(0.0f, 100.0f, 0.0f, 30.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 50.0f, throttleSlewStep(50.0f, 50.0f, 0.0f, RATE_DEFAULT));
}

// Rate 0 (or negative) is a passthrough: exactly the old behaviour.
static void test_rate_zero_is_passthrough() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, throttleSlewStep(100.0f, 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, throttleSlewStep(100.0f, 0.0f, 0.1f, -5.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 37.0f, throttleSlewStep(37.0f, 90.0f, 0.1f, 0.0f));
}

// Many small steps add up like one step: 1 ms steps for 1.0 s at 60 %/s give 60 %,
// and the full idle → 100 % ramp takes ≈1.67 s.
static void test_small_steps_accumulate() {
    ThrottleSlewState st;
    for (int i = 0; i < 1000; i++) st.step(100.0f, 0.001f, RATE_DEFAULT);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 60.0f, st.lastPct);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, throttleSlewStep(100.0f, 0.0f, 1.0f, RATE_DEFAULT), st.lastPct);

    ThrottleSlewState ramp;
    int steps = 0;
    while (ramp.lastPct < 100.0f && steps < 10000) { ramp.step(100.0f, 0.001f, RATE_DEFAULT); steps++; }
    TEST_ASSERT_INT_WITHIN(5, 1667, steps);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, ramp.lastPct);
}

// reset(0) (what idle() does) makes the next opening ramp from 0 again.
static void test_reset_on_idle() {
    ThrottleSlewState st;
    for (int i = 0; i < 100; i++) st.step(100.0f, 0.01f, RATE_DEFAULT);   // 1 s → 60 %
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 60.0f, st.lastPct);
    st.reset(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, st.lastPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.0f, st.step(100.0f, 0.1f, RATE_DEFAULT));
    // Re-seed from a direct pulse write (setThrottleUs): ramps from there, not from 0.
    st.reset(40.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 46.0f, st.step(100.0f, 0.1f, RATE_DEFAULT));
}

// The target is clamped to 0..100 %; NaN is treated as closed; dt < 0 adds nothing.
static void test_target_clamped() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, throttleSlewStep(150.0f, 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, throttleSlewStep(-20.0f, 50.0f, 0.1f, RATE_DEFAULT));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, throttleSlewStep(NAN, 50.0f, 0.1f, RATE_DEFAULT));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, throttleSlewStep(150.0f, 98.0f, 0.1f, RATE_DEFAULT));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, throttleSlewStep(100.0f, 10.0f, -1.0f, RATE_DEFAULT));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_rising_is_limited);
    RUN_TEST(test_falling_is_immediate);
    RUN_TEST(test_rate_zero_is_passthrough);
    RUN_TEST(test_small_steps_accumulate);
    RUN_TEST(test_reset_on_idle);
    RUN_TEST(test_target_clamped);
    return UNITY_END();
}
