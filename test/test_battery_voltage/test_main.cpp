// Native unit tests for the 12 V battery sense on X10 / GPIO9: scaling, calibration factor,
// low-voltage hysteresis and calibration rejection (include/BatteryVoltage.h).
// Run: pio test -e native
#include <unity.h>
#include "BatteryVoltage.h"

// Mirror of include/Constants.h (which needs Arduino.h and cannot be built natively).
static const BatParams P = {
    9.33f,   // dividerRatio  BAT12_DIVIDER_RATIO
    5.0f,    // presentMinV   BAT12_PRESENT_MIN_V
    11.8f,   // lowOnV        BAT12_LOW_V
    12.1f,   // lowOffV       BAT12_LOW_CLEAR_V
    0.80f,   // calMin        BAT12_CAL_MIN
    1.20f,   // calMax        BAT12_CAL_MAX
    8.0f,    // measuredMinV  BAT12_CAL_MEASURED_MIN_V
    16.0f,   // measuredMaxV  BAT12_CAL_MEASURED_MAX_V
};

void setUp() {}
void tearDown() {}

static void test_raw_scaling() {
    // 12.6 V / 9.33 = 1350.5 mV at the pin
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.6f, batRawVolts(1350.5f, P));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, batRawVolts(0.0f, P));
}

static void test_cal_applied() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, batVolts(12.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.6f, batVolts(12.0f, 1.05f));
}

static void test_present_threshold() {
    TEST_ASSERT_FALSE(batPresent(0.3f, P));
    TEST_ASSERT_FALSE(batPresent(5.0f, P));
    TEST_ASSERT_TRUE(batPresent(5.01f, P));
    TEST_ASSERT_TRUE(batPresent(12.4f, P));
}

static void test_low_sets_below_on_threshold() {
    TEST_ASSERT_FALSE(batLowStep(false, 11.85f, P));
    TEST_ASSERT_TRUE(batLowStep(false, 11.79f, P));
}

static void test_low_hysteresis_band_holds() {
    // inside (11.8, 12.1]: the flag keeps whatever it was
    TEST_ASSERT_TRUE(batLowStep(true, 11.9f, P));
    TEST_ASSERT_TRUE(batLowStep(true, 12.1f, P));
    TEST_ASSERT_FALSE(batLowStep(false, 11.9f, P));
    TEST_ASSERT_FALSE(batLowStep(false, 12.05f, P));
}

static void test_low_clears_above_off_threshold() {
    TEST_ASSERT_FALSE(batLowStep(true, 12.11f, P));
    TEST_ASSERT_FALSE(batLowStep(true, 13.8f, P));
}

static void test_low_sequence_no_chatter() {
    // noisy reading around 11.8 V: one transition down, one up at 12.1 V
    const float seq[] = {12.4f, 11.79f, 11.81f, 11.79f, 11.95f, 12.08f, 12.15f, 12.0f};
    const bool exp[]  = {false, true,   true,   true,   true,   true,   false,  false};
    bool low = false;
    for (unsigned i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        low = batLowStep(low, seq[i], P);
        TEST_ASSERT_EQUAL_MESSAGE(exp[i], low, "step");
    }
}

static void test_low_nan_keeps_state() {
    TEST_ASSERT_TRUE(batLowStep(true, NAN, P));
    TEST_ASSERT_FALSE(batLowStep(false, NAN, P));
}

static void test_cal_computation_ok() {
    float cal = 1.0f;
    TEST_ASSERT_EQUAL((int)BatCalResult::OK, (int)batComputeCal(12.60f, 12.00f, P, cal));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.05f, cal);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.60f, batVolts(12.00f, cal));
    TEST_ASSERT_EQUAL((int)BatCalResult::OK, (int)batComputeCal(11.40f, 12.00f, P, cal));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.95f, cal);
}

static void test_cal_rejects_measured_out_of_range() {
    float cal = 1.0f;
    TEST_ASSERT_EQUAL((int)BatCalResult::MEASURED_OUT_OF_RANGE, (int)batComputeCal(7.99f, 12.0f, P, cal));
    TEST_ASSERT_EQUAL((int)BatCalResult::MEASURED_OUT_OF_RANGE, (int)batComputeCal(16.01f, 12.0f, P, cal));
    TEST_ASSERT_EQUAL((int)BatCalResult::MEASURED_OUT_OF_RANGE, (int)batComputeCal(NAN, 12.0f, P, cal));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, cal);   // untouched
}

static void test_cal_rejects_not_connected() {
    float cal = 1.0f;
    TEST_ASSERT_EQUAL((int)BatCalResult::NOT_CONNECTED, (int)batComputeCal(12.6f, 0.2f, P, cal));
    TEST_ASSERT_EQUAL((int)BatCalResult::NOT_CONNECTED, (int)batComputeCal(12.6f, 5.0f, P, cal));
    TEST_ASSERT_EQUAL((int)BatCalResult::NOT_CONNECTED, (int)batComputeCal(12.6f, NAN, P, cal));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, cal);
}

static void test_cal_rejects_factor_out_of_range() {
    float cal = 1.0f;
    // 12.6 / 9.0 = 1.4 > 1.20
    TEST_ASSERT_EQUAL((int)BatCalResult::FACTOR_OUT_OF_RANGE, (int)batComputeCal(12.6f, 9.0f, P, cal));
    // 8.5 / 12.6 = 0.675 < 0.80
    TEST_ASSERT_EQUAL((int)BatCalResult::FACTOR_OUT_OF_RANGE, (int)batComputeCal(8.5f, 12.6f, P, cal));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, cal);
}

static void test_cal_valid_range() {
    TEST_ASSERT_TRUE(batCalValid(1.0f, P));
    TEST_ASSERT_TRUE(batCalValid(0.80f, P));
    TEST_ASSERT_TRUE(batCalValid(1.20f, P));
    TEST_ASSERT_FALSE(batCalValid(0.79f, P));
    TEST_ASSERT_FALSE(batCalValid(1.21f, P));
    TEST_ASSERT_FALSE(batCalValid(NAN, P));
    TEST_ASSERT_FALSE(batCalValid(0.0f, P));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_raw_scaling);
    RUN_TEST(test_cal_applied);
    RUN_TEST(test_present_threshold);
    RUN_TEST(test_low_sets_below_on_threshold);
    RUN_TEST(test_low_hysteresis_band_holds);
    RUN_TEST(test_low_clears_above_off_threshold);
    RUN_TEST(test_low_sequence_no_chatter);
    RUN_TEST(test_low_nan_keeps_state);
    RUN_TEST(test_cal_computation_ok);
    RUN_TEST(test_cal_rejects_measured_out_of_range);
    RUN_TEST(test_cal_rejects_not_connected);
    RUN_TEST(test_cal_rejects_factor_out_of_range);
    RUN_TEST(test_cal_valid_range);
    return UNITY_END();
}
