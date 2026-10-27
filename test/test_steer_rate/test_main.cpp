// Native unit tests for the steering mode selection and the sensorless rate mapping
// (include/SteerRate.h). Run: pio test -e native
#include <unity.h>
#include "SteerRate.h"

// Mirror of include/Constants.h (which needs Arduino.h and cannot be built natively).
static const float RC_STEERING_DEADBAND = 2.0f;
static const uint8_t STEER_PWM_MIN_DUTY = 140;
static const uint8_t STEER_PWM_MAX_DUTY = 255;

static int16_t duty(float pct) {
    return steerRateDuty(pct, RC_STEERING_DEADBAND, STEER_PWM_MIN_DUTY, STEER_PWM_MAX_DUTY);
}

void setUp() {}
void tearDown() {}

static void test_deadband_edge() {
    TEST_ASSERT_EQUAL_INT16(0, duty(0.0f));
    TEST_ASSERT_EQUAL_INT16(0, duty(1.99f));
    TEST_ASSERT_EQUAL_INT16(0, duty(-1.99f));
    TEST_ASSERT_EQUAL_INT16(140, duty(2.0f));
    TEST_ASSERT_EQUAL_INT16(-140, duty(-2.0f));
}

static void test_reference_points() {
    TEST_ASSERT_EQUAL_INT16(146, duty(25.0f));
    TEST_ASSERT_EQUAL_INT16(168, duty(50.0f));
    TEST_ASSERT_EQUAL_INT16(204, duty(75.0f));
    TEST_ASSERT_EQUAL_INT16(255, duty(100.0f));
}

static void test_sign_symmetry_and_clamp() {
    const float pts[] = {2.0f, 10.0f, 25.0f, 33.3f, 50.0f, 75.0f, 99.0f, 100.0f};
    for (float p : pts) {
        TEST_ASSERT_EQUAL_INT16(-duty(p), duty(-p));
        TEST_ASSERT_TRUE(duty(p) > 0);
    }
    TEST_ASSERT_EQUAL_INT16(255, duty(150.0f));
    TEST_ASSERT_EQUAL_INT16(-255, duty(-150.0f));
}

static void test_monotonic() {
    int16_t prev = 0;
    for (int i = 2; i <= 100; i++) {
        int16_t d = duty((float)i);
        TEST_ASSERT_TRUE(d >= prev);
        prev = d;
    }
}

static void test_mode_select_healthy_sensor_is_position() {
    for (int src = 0; src < 2; src++)
        for (int known = 0; known < 2; known++)
            for (int manual = 0; manual < 2; manual++)
                TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerMode::POSITION,
                    (uint8_t)steerModeSelect(true, src, known, manual));
}

static void test_mode_select_sensorless() {
    // MAVLink + MANUAL → RATE
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerMode::RATE, (uint8_t)steerModeSelect(false, true, true, true));
    // MAVLink + unknown mode → RATE (whatever the manual flag says)
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerMode::RATE, (uint8_t)steerModeSelect(false, true, false, false));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerMode::RATE, (uint8_t)steerModeSelect(false, true, false, true));
    // MAVLink + known non-MANUAL → BLOCKED
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerMode::BLOCKED, (uint8_t)steerModeSelect(false, true, true, false));
    // Web / failsafe source → BLOCKED in every autopilot state
    for (int known = 0; known < 2; known++)
        for (int manual = 0; manual < 2; manual++)
            TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerMode::BLOCKED,
                (uint8_t)steerModeSelect(false, false, known, manual));
}

static void test_mode_tokens() {
    TEST_ASSERT_EQUAL_STRING("position", steerModeToken(SteerMode::POSITION));
    TEST_ASSERT_EQUAL_STRING("rate", steerModeToken(SteerMode::RATE));
    TEST_ASSERT_EQUAL_STRING("blocked", steerModeToken(SteerMode::BLOCKED));
    TEST_ASSERT_EQUAL_UINT8(0, (uint8_t)SteerMode::POSITION);
    TEST_ASSERT_EQUAL_UINT8(1, (uint8_t)SteerMode::RATE);
    TEST_ASSERT_EQUAL_UINT8(2, (uint8_t)SteerMode::BLOCKED);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_deadband_edge);
    RUN_TEST(test_reference_points);
    RUN_TEST(test_sign_symmetry_and_clamp);
    RUN_TEST(test_monotonic);
    RUN_TEST(test_mode_select_healthy_sensor_is_position);
    RUN_TEST(test_mode_select_sensorless);
    RUN_TEST(test_mode_tokens);
    return UNITY_END();
}
