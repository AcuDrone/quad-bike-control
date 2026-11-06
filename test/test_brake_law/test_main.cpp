// Native unit tests for the brake travel time / hold drive law (include/BrakeLaw.h).
// Run: pio test -e native
#include <unity.h>
#include "BrakeLaw.h"

void setUp() {}
void tearDown() {}

// 1500 ms → 0.0667 %/ms; 150 steps of 10 ms take the estimate from 0 to 100 %.
static void test_rate_default() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0667f, brakeRatePctPerMs(1500));
    float pos = 0.0f;
    int steps = 0;
    while (pos < 100.0f && steps < 1000) {
        pos += brakeRatePctPerMs(1500) * 10.0f;
        if (pos > 100.0f) pos = 100.0f;
        steps++;
    }
    TEST_ASSERT_INT_WITHIN(1, 150, steps);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, pos);
}

static void test_rate_bounds() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, brakeRatePctPerMs(1000));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0333f, brakeRatePctPerMs(3000));
    // Never divides by zero: a non-positive travel time uses the default.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0667f, brakeRatePctPerMs(0));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0667f, brakeRatePctPerMs(-5));
}

static void test_travel_validation() {
    TEST_ASSERT_FALSE(brakeTravelMsValid(999));
    TEST_ASSERT_FALSE(brakeTravelMsValid(3001));
    TEST_ASSERT_FALSE(brakeTravelMsValid(0));
    TEST_ASSERT_TRUE(brakeTravelMsValid(1000));
    TEST_ASSERT_TRUE(brakeTravelMsValid(1500));
    TEST_ASSERT_TRUE(brakeTravelMsValid(3000));
}

static void test_hold_validation() {
    TEST_ASSERT_FALSE(brakeHoldPwmValid(-1));
    TEST_ASSERT_FALSE(brakeHoldPwmValid(101));
    TEST_ASSERT_TRUE(brakeHoldPwmValid(0));
    TEST_ASSERT_TRUE(brakeHoldPwmValid(30));
    TEST_ASSERT_TRUE(brakeHoldPwmValid(100));
}

static void test_or_default() {
    TEST_ASSERT_EQUAL_INT32(1500, brakeTravelMsOrDefault(999));
    TEST_ASSERT_EQUAL_INT32(1500, brakeTravelMsOrDefault(3001));
    TEST_ASSERT_EQUAL_INT32(1500, brakeTravelMsOrDefault(0));
    TEST_ASSERT_EQUAL_INT32(2000, brakeTravelMsOrDefault(2000));
    TEST_ASSERT_EQUAL_INT32(1000, brakeTravelMsOrDefault(1000));
    TEST_ASSERT_EQUAL_INT32(30, brakeHoldPwmOrDefault(-1));
    TEST_ASSERT_EQUAL_INT32(30, brakeHoldPwmOrDefault(101));
    TEST_ASSERT_EQUAL_INT32(30, brakeHoldPwmOrDefault(255));
    TEST_ASSERT_EQUAL_INT32(0, brakeHoldPwmOrDefault(0));
    TEST_ASSERT_EQUAL_INT32(60, brakeHoldPwmOrDefault(60));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_rate_default);
    RUN_TEST(test_rate_bounds);
    RUN_TEST(test_travel_validation);
    RUN_TEST(test_hold_validation);
    RUN_TEST(test_or_default);
    return UNITY_END();
}
