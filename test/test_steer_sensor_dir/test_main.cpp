// Native unit tests for the steering sensor direction (include/SteerSensorDir.h).
// Run: pio test -e native
#include <unity.h>
#include "SteerSensorDir.h"

// Mirror of include/Constants.h (which needs Arduino.h and cannot be built natively).
static const int32_t STEER_CAL_MIN_SPAN = 300;

static const bool LEFT = true;
static const bool RIGHT = false;

void setUp() {}
void tearDown() {}

static void assertResult(SteerDirAction action, bool inv, int32_t pos, bool clearOpposite,
                         const SteerDirResult& r) {
    TEST_ASSERT_EQUAL_UINT8((uint8_t)action, (uint8_t)r.action);
    TEST_ASSERT_EQUAL(inv, r.inv);
    TEST_ASSERT_EQUAL_INT32(pos, r.pos);
    TEST_ASSERT_EQUAL(clearOpposite, r.clearOpposite);
}

static void test_normal_mount_stores() {
    assertResult(SteerDirAction::STORE, false, -400, false,
                 steerDirCapture(LEFT, -400, false, false, STEER_CAL_MIN_SPAN));
    assertResult(SteerDirAction::STORE, false, 400, false,
                 steerDirCapture(RIGHT, 400, false, false, STEER_CAL_MIN_SPAN));
    // Second limit of a normal calibration
    assertResult(SteerDirAction::STORE, false, 400, false,
                 steerDirCapture(RIGHT, 400, false, true, STEER_CAL_MIN_SPAN));
}

static void test_mirrored_detected_left_first() {
    assertResult(SteerDirAction::TOGGLE_AND_STORE, true, -400, true,
                 steerDirCapture(LEFT, 400, false, false, STEER_CAL_MIN_SPAN));
}

static void test_mirrored_detected_right_first() {
    assertResult(SteerDirAction::TOGGLE_AND_STORE, true, 400, true,
                 steerDirCapture(RIGHT, -400, false, false, STEER_CAL_MIN_SPAN));
}

static void test_toggle_back_to_normal() {
    // Flag says mirrored, sensor re-mounted normally, limits erased by a zero-set
    assertResult(SteerDirAction::TOGGLE_AND_STORE, false, -400, true,
                 steerDirCapture(LEFT, 400, true, false, STEER_CAL_MIN_SPAN));
}

static void test_second_capture_confirms() {
    assertResult(SteerDirAction::STORE, true, 400, false,
                 steerDirCapture(RIGHT, 400, true, true, STEER_CAL_MIN_SPAN));
}

static void test_second_capture_contradicts() {
    assertResult(SteerDirAction::REFUSE_CONTRADICTS, true, -400, false,
                 steerDirCapture(RIGHT, -400, true, true, STEER_CAL_MIN_SPAN));
}

static void test_no_toggle_when_opposite_set() {
    assertResult(SteerDirAction::REFUSE_CONTRADICTS, false, 400, false,
                 steerDirCapture(LEFT, 400, false, true, STEER_CAL_MIN_SPAN));
}

static void test_below_span_refused() {
    const int32_t n = STEER_CAL_MIN_SPAN - 1;
    const int32_t positions[] = {n, -n, 0};
    const bool buttons[] = {LEFT, RIGHT};
    const bool invs[] = {false, true};
    for (bool b : buttons) {
        for (bool inv : invs) {
            for (int32_t p : positions) {
                assertResult(SteerDirAction::REFUSE_TOO_CLOSE, inv, p, false,
                             steerDirCapture(b, p, inv, false, STEER_CAL_MIN_SPAN));
                assertResult(SteerDirAction::REFUSE_TOO_CLOSE, inv, p, false,
                             steerDirCapture(b, p, inv, true, STEER_CAL_MIN_SPAN));
            }
        }
    }
}

static void test_exact_span_on_expected_side() {
    assertResult(SteerDirAction::STORE, false, -STEER_CAL_MIN_SPAN, false,
                 steerDirCapture(LEFT, -STEER_CAL_MIN_SPAN, false, false, STEER_CAL_MIN_SPAN));
    assertResult(SteerDirAction::STORE, false, STEER_CAL_MIN_SPAN, false,
                 steerDirCapture(RIGHT, STEER_CAL_MIN_SPAN, false, false, STEER_CAL_MIN_SPAN));
    // Exactly the span on the wrong side may toggle
    assertResult(SteerDirAction::TOGGLE_AND_STORE, true, -STEER_CAL_MIN_SPAN, true,
                 steerDirCapture(LEFT, STEER_CAL_MIN_SPAN, false, false, STEER_CAL_MIN_SPAN));
}

static void test_position_sign_table() {
    TEST_ASSERT_EQUAL_INT32(-123, steerSensorPosition(123, false));
    TEST_ASSERT_EQUAL_INT32(123, steerSensorPosition(-123, false));
    TEST_ASSERT_EQUAL_INT32(123, steerSensorPosition(123, true));
    TEST_ASSERT_EQUAL_INT32(-123, steerSensorPosition(-123, true));
    TEST_ASSERT_EQUAL_INT32(32768, steerSensorPosition(INT16_MIN, false));
    TEST_ASSERT_EQUAL_INT32(0, steerSensorPosition(0, true));
}

static void test_rate_sign_table() {
    TEST_ASSERT_EQUAL_INT32(40, steerSensorRate(40, true, false));
    TEST_ASSERT_EQUAL_INT32(-40, steerSensorRate(40, false, false));
    TEST_ASSERT_EQUAL_INT32(-40, steerSensorRate(40, true, true));
    TEST_ASSERT_EQUAL_INT32(40, steerSensorRate(40, false, true));
    TEST_ASSERT_EQUAL_INT32(0, steerSensorRate(0, true, true));
}

static void test_toggle_negates_position_of_same_angle() {
    // −pos is exact: the toggled position equals the position derived under the new flag
    const int16_t raw = -4123;
    const int32_t before = steerSensorPosition(raw, false);
    const SteerDirResult r = steerDirCapture(LEFT, before, false, false, STEER_CAL_MIN_SPAN);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)SteerDirAction::TOGGLE_AND_STORE, (uint8_t)r.action);
    TEST_ASSERT_EQUAL_INT32(steerSensorPosition(raw, r.inv), r.pos);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_normal_mount_stores);
    RUN_TEST(test_mirrored_detected_left_first);
    RUN_TEST(test_mirrored_detected_right_first);
    RUN_TEST(test_toggle_back_to_normal);
    RUN_TEST(test_second_capture_confirms);
    RUN_TEST(test_second_capture_contradicts);
    RUN_TEST(test_no_toggle_when_opposite_set);
    RUN_TEST(test_below_span_refused);
    RUN_TEST(test_exact_span_on_expected_side);
    RUN_TEST(test_position_sign_table);
    RUN_TEST(test_rate_sign_table);
    RUN_TEST(test_toggle_negates_position_of_same_angle);
    return UNITY_END();
}
