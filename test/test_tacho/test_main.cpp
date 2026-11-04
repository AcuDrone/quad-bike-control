// Native unit tests for the ECU tacho input (include/TachoLaw.h): period → RPM, averaging,
// timeout, plausibility, ppr validation / snapping, calibration and RPM source selection;
// fw 1.5.10: median period, lock-window edge filter and diagnostics formatting.
// Run: pio test -e native
#include <unity.h>
#include <string.h>
#include "TachoLaw.h"

// Mirror of include/Constants.h (which needs Arduino.h and cannot be built natively).
static const TachoCalParams CAL = {
    300.0f,   // refMinRpm  TACHO_CAL_REF_MIN_RPM
    8000.0f,  // refMaxRpm  TACHO_CAL_REF_MAX_RPM
    0.25f,    // tolerance  TACHO_CAL_TOLERANCE
};
static const float MAX_RPM = 10000.0f;   // TACHO_MAX_RPM

void setUp() {}
void tearDown() {}

static void test_period_to_rpm_ppr_half_one_two() {
    // 40 000 µs = 25 Hz
    const float hz = tachoHzFromPeriodUs(40000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 25.0f, hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, tachoRpm(hz, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3000.0f, tachoRpm(hz, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 750.0f, tachoRpm(hz, 2.0f));
}

static void test_zero_and_bad_inputs() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tachoHzFromPeriodUs(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tachoHzFromPeriodUs(-5.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tachoRpm(0.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tachoRpm(25.0f, 0.0f));
}

static void test_mean_of_last_four_periods() {
    // 6 edges → 5 periods; only the last 4 (40, 38, 42, 40 ms) are averaged
    const uint32_t e[6] = {0, 100000, 140000, 178000, 220000, 260000};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40000.0f, tachoMeanPeriod(e, 6, 4));
    // two edges → one period
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100000.0f, tachoMeanPeriod(e, 2, 4));
    // fewer than two edges → 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tachoMeanPeriod(e, 1, 4));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tachoMeanPeriod(e, 0, 4));
}

static void test_mean_period_across_micros_wrap() {
    const uint32_t e[3] = {0xFFFFFFFFu - 19999u, 20000u, 60000u};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40000.0f, tachoMeanPeriod(e, 3, 4));
}

static void test_timeout_and_millis_wrap() {
    TEST_ASSERT_FALSE(tachoTimedOut(1499, 1000, 500));
    TEST_ASSERT_TRUE(tachoTimedOut(1500, 1000, 500));
    // last edge just before the millis() wrap, now just after
    TEST_ASSERT_FALSE(tachoTimedOut(100u, 0xFFFFFFFFu - 200u, 500));
    TEST_ASSERT_TRUE(tachoTimedOut(400u, 0xFFFFFFFFu - 200u, 500));
}

static void test_implausible_rpm() {
    TEST_ASSERT_TRUE(tachoPlausible(1500.0f, MAX_RPM));
    TEST_ASSERT_TRUE(tachoPlausible(10000.0f, MAX_RPM));
    TEST_ASSERT_FALSE(tachoPlausible(10001.0f, MAX_RPM));
    TEST_ASSERT_FALSE(tachoPlausible(NAN, MAX_RPM));
    TEST_ASSERT_FALSE(tachoPlausible(-1.0f, MAX_RPM));
    // a 1.1 ms glitch pair at ppr 1 → 54 545 RPM: rejected
    TEST_ASSERT_FALSE(tachoPlausible(tachoRpm(tachoHzFromPeriodUs(1100.0f), 1.0f), MAX_RPM));
}

static void test_allowed_ppr_set() {
    TEST_ASSERT_TRUE(tachoPprAllowed(0.5f));
    TEST_ASSERT_TRUE(tachoPprAllowed(1.0f));
    TEST_ASSERT_TRUE(tachoPprAllowed(2.0f));
    TEST_ASSERT_TRUE(tachoPprAllowed(3.0f));
    TEST_ASSERT_TRUE(tachoPprAllowed(4.0f));
    TEST_ASSERT_FALSE(tachoPprAllowed(0.0f));
    TEST_ASSERT_FALSE(tachoPprAllowed(1.5f));
    TEST_ASSERT_FALSE(tachoPprAllowed(5.0f));
    TEST_ASSERT_FALSE(tachoPprAllowed(NAN));
}

static void test_snapping() {
    TEST_ASSERT_EQUAL_FLOAT(1.0f, tachoSnapPpr(1.01f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, tachoSnapPpr(0.55f));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, tachoSnapPpr(1.9f));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, tachoSnapPpr(3.2f));
    TEST_ASSERT_EQUAL_FLOAT(4.0f, tachoSnapPpr(7.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, tachoSnapPpr(0.1f));
    TEST_ASSERT_TRUE(tachoSnapWithin(1.25f, 1.0f, 0.25f));
    TEST_ASSERT_FALSE(tachoSnapWithin(1.3f, 1.0f, 0.25f));
}

static void test_cal_ok_dash_example() {
    // 26.1 Hz × 60 / 1550 = 1.0103 → ppr 1
    float raw = 0.0f, ppr = -1.0f;
    TEST_ASSERT_EQUAL(TachoCalResult::OK, tachoComputeCal(26.1f, 1550.0f, CAL, raw, ppr));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0103f, raw);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, ppr);
    // ppr 2 ECU: 52 Hz at 1560 RPM → raw 2.0
    TEST_ASSERT_EQUAL(TachoCalResult::OK, tachoComputeCal(52.0f, 1560.0f, CAL, raw, ppr));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, ppr);
}

static void test_cal_refused_no_pulses() {
    float raw = -1.0f, ppr = -1.0f;
    TEST_ASSERT_EQUAL(TachoCalResult::NO_PULSES, tachoComputeCal(0.0f, 1500.0f, CAL, raw, ppr));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, ppr);
}

static void test_cal_refused_ref_out_of_range() {
    float raw = -1.0f, ppr = -1.0f;
    TEST_ASSERT_EQUAL(TachoCalResult::REF_OUT_OF_RANGE, tachoComputeCal(25.0f, 299.0f, CAL, raw, ppr));
    TEST_ASSERT_EQUAL(TachoCalResult::REF_OUT_OF_RANGE, tachoComputeCal(25.0f, 8001.0f, CAL, raw, ppr));
    TEST_ASSERT_EQUAL(TachoCalResult::REF_OUT_OF_RANGE, tachoComputeCal(25.0f, NAN, CAL, raw, ppr));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, ppr);
}

static void test_cal_refused_ratio_off() {
    // 25 Hz × 60 / 1000 = 1.5 → nearest 1 (tie → smaller), 50 % off
    float raw = 0.0f, ppr = -1.0f;
    TEST_ASSERT_EQUAL(TachoCalResult::RATIO_OFF, tachoComputeCal(25.0f, 1000.0f, CAL, raw, ppr));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, raw);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, ppr);
    // 1.3 → nearest 1, 30 % off
    TEST_ASSERT_EQUAL(TachoCalResult::RATIO_OFF, tachoComputeCal(26.0f, 1200.0f, CAL, raw, ppr));
}

static void test_select_live_tacho_wins() {
    EffectiveRpm r = tachoSelectRpm(true, true, 1520, 1000, true, 3000, 900);
    TEST_ASSERT_EQUAL(RpmSource::TACHO, r.source);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(1520, r.rpm);
    TEST_ASSERT_EQUAL_UINT32(1000, r.sampleMs);
    r = tachoSelectRpm(true, true, 1520, 1000, false, 0, 0);
    TEST_ASSERT_EQUAL(RpmSource::TACHO, r.source);
}

static void test_select_silent_tacho_falls_back_to_can_rpm() {
    EffectiveRpm r = tachoSelectRpm(true, false, 0, 1000, true, 1480, 2000);
    TEST_ASSERT_EQUAL(RpmSource::CAN, r.source);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(1480, r.rpm);
    TEST_ASSERT_EQUAL_UINT32(2000, r.sampleMs);
}

static void test_select_silent_fitted_tacho_reads_zero() {
    EffectiveRpm r = tachoSelectRpm(true, false, 0, 1000, true, 0, 2000);   // CAN says 0 (MT05)
    TEST_ASSERT_EQUAL(RpmSource::TACHO, r.source);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(0, r.rpm);
    r = tachoSelectRpm(true, false, 0, 1000, false, 0, 0);                 // CAN down
    TEST_ASSERT_EQUAL(RpmSource::TACHO, r.source);
    TEST_ASSERT_TRUE(r.valid);
}

static void test_select_not_fitted() {
    EffectiveRpm r = tachoSelectRpm(false, false, 0, 0, true, 0, 2000);
    TEST_ASSERT_EQUAL(RpmSource::CAN, r.source);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(0, r.rpm);
    r = tachoSelectRpm(false, false, 0, 0, false, 1234, 2000);
    TEST_ASSERT_EQUAL(RpmSource::NONE, r.source);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(0, r.rpm);
    TEST_ASSERT_EQUAL_STRING("NONE", rpmSourceName(r.source));
    TEST_ASSERT_EQUAL_STRING("TACHO", rpmSourceName(RpmSource::TACHO));
    TEST_ASSERT_EQUAL_STRING("CAN", rpmSourceName(RpmSource::CAN));
}

// ---- fw 1.5.10: median period, lock window, diagnostics formatting ----------------------

// Build chronological edges from a list of periods (µs), starting at t0.
static uint8_t edgesFromPeriods(const uint32_t* periods, uint8_t n, uint32_t t0, uint32_t* out) {
    out[0] = t0;
    for (uint8_t i = 0; i < n; i++) out[i + 1] = out[i] + periods[i];
    return (uint8_t)(n + 1);
}

static void test_median_of_8_clean() {
    const uint32_t p[8] = {42800, 42900, 43000, 42900, 42800, 43000, 42900, 42900};
    uint32_t e[9];
    const uint8_t n = edgesFromPeriods(p, 8, 1000, e);
    // sorted: 42800 42800 42900 42900 42900 42900 43000 43000 -> middle pair 42900
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 42900.0f, tachoMedianPeriod(e, n, 8));
}

static void test_median_ignores_one_spurious_half_period() {
    // A spurious edge produced one half-length period among eight
    const uint32_t p[8] = {42900, 42900, 21400, 42900, 42900, 42900, 42900, 42900};
    uint32_t e[9];
    const uint8_t n = edgesFromPeriods(p, 8, 5000, e);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 42900.0f, tachoMedianPeriod(e, n, 8));
    // the old mean is pulled down well away from the true period
    TEST_ASSERT_TRUE(tachoMeanPeriod(e, n, 8) < 40500.0f);
}

static void test_median_ignores_two_spurious_half_periods() {
    // One extra edge mid-period splits it into two halves: 6 true + 2 half periods
    const uint32_t p[8] = {42900, 21400, 21500, 42900, 42900, 42900, 42900, 42900};
    uint32_t e[9];
    const uint8_t n = edgesFromPeriods(p, 8, 5000, e);
    const float med = tachoMedianPeriod(e, n, 8);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 42900.0f, med);
    // ppr 1 -> ≈1399 RPM, the dash idle
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 1398.6f, tachoRpm(tachoHzFromPeriodUs(med), 1.0f));
}

static void test_median_ignores_one_missed_edge() {
    // A missed edge merges two periods into one double-length period
    const uint32_t p[8] = {42900, 42900, 42900, 85800, 42900, 42900, 42900, 42900};
    uint32_t e[9];
    const uint8_t n = edgesFromPeriods(p, 8, 0xFFFFFFFFu - 100000u, e);   // across the µs wrap
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 42900.0f, tachoMedianPeriod(e, n, 8));
}

static void test_median_uses_only_last_n_periods() {
    // 10 edges -> 9 periods; the oldest (1 000 000 µs) must be outside the last 8
    const uint32_t p[9] = {1000000, 40000, 40000, 40000, 40000, 40000, 40000, 40000, 40000};
    uint32_t e[10];
    const uint8_t n = edgesFromPeriods(p, 9, 0, e);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 40000.0f, tachoMedianPeriod(e, n, 8));
    // odd count: the middle value
    const uint32_t q[5] = {40000, 20000, 41000, 39000, 90000};
    uint32_t f[6];
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 40000.0f, tachoMedianPeriod(f, edgesFromPeriods(q, 5, 0, f), 8));
}

static void test_median_falls_back_to_mean_below_three_periods() {
    const uint32_t e2[3] = {0, 40000, 60000};          // 2 periods: 40 000, 20 000 -> mean 30 000
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30000.0f, tachoMedianPeriod(e2, 3, 8));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, tachoMeanPeriod(e2, 3, 8), tachoMedianPeriod(e2, 3, 8));
    const uint32_t e1[2] = {100, 40100};               // 1 period
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 40000.0f, tachoMedianPeriod(e1, 2, 8));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tachoMedianPeriod(e1, 1, 8));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tachoMedianPeriod(nullptr, 5, 8));
}

static void test_edge_filter_unlocked_floor_only() {
    // lock 0: only the absolute floor (1500 µs) applies
    TEST_ASSERT_FALSE(tachoEdgeAccepted(1499, 0, 0.5f, 1500));
    TEST_ASSERT_TRUE(tachoEdgeAccepted(1500, 0, 0.5f, 1500));
    TEST_ASSERT_TRUE(tachoEdgeAccepted(5000, 0, 0.5f, 1500));
    TEST_ASSERT_EQUAL_UINT32(0u, tachoLockMinUs(0, 0.5f));
}

static void test_edge_filter_lock_window() {
    // locked on a 42 900 µs period: threshold 21 450 µs
    TEST_ASSERT_EQUAL_UINT32(21450u, tachoLockMinUs(42900, 0.5f));
    TEST_ASSERT_FALSE(tachoEdgeAccepted(3000, 42900, 0.5f, 1500));    // ignition spike
    TEST_ASSERT_FALSE(tachoEdgeAccepted(21449, 42900, 0.5f, 1500));   // just inside the window
    TEST_ASSERT_TRUE(tachoEdgeAccepted(21450, 42900, 0.5f, 1500));    // edge of the window
    TEST_ASSERT_TRUE(tachoEdgeAccepted(42900, 42900, 0.5f, 1500));    // the real pulse
    TEST_ASSERT_TRUE(tachoEdgeAccepted(30000, 42900, 0.5f, 1500));    // RPM rising up to 2x passes
    // a tiny lock never undercuts the floor
    TEST_ASSERT_FALSE(tachoEdgeAccepted(1200, 2000, 0.5f, 1500));
    // the ISR's integer core agrees with the float wrapper
    TEST_ASSERT_TRUE(tachoEdgeAcceptedMin(21450, tachoLockMinUs(42900, 0.5f), 1500));
    TEST_ASSERT_FALSE(tachoEdgeAcceptedMin(21449, tachoLockMinUs(42900, 0.5f), 1500));
}

static void test_format_periods_ms() {
    const uint32_t p[4] = {42940, 43050, 21400, 999};
    char buf[64];
    tachoFormatPeriodsMs(p, 4, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("42.9 43.1 21.4 1.0", buf);
    tachoFormatPeriodsMs(p, 0, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("", buf);
    // eight worst-case (499.9 ms) values fit the 64 B telemetry buffer
    const uint32_t w[8] = {499900, 499900, 499900, 499900, 499900, 499900, 499900, 499900};
    tachoFormatPeriodsMs(w, 8, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("499.9 499.9 499.9 499.9 499.9 499.9 499.9 499.9", buf);
    // a too-small buffer stays NUL-terminated
    char small[6];
    tachoFormatPeriodsMs(p, 4, small, sizeof(small));
    TEST_ASSERT_TRUE(strlen(small) < sizeof(small));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_period_to_rpm_ppr_half_one_two);
    RUN_TEST(test_zero_and_bad_inputs);
    RUN_TEST(test_mean_of_last_four_periods);
    RUN_TEST(test_mean_period_across_micros_wrap);
    RUN_TEST(test_timeout_and_millis_wrap);
    RUN_TEST(test_implausible_rpm);
    RUN_TEST(test_allowed_ppr_set);
    RUN_TEST(test_snapping);
    RUN_TEST(test_cal_ok_dash_example);
    RUN_TEST(test_cal_refused_no_pulses);
    RUN_TEST(test_cal_refused_ref_out_of_range);
    RUN_TEST(test_cal_refused_ratio_off);
    RUN_TEST(test_select_live_tacho_wins);
    RUN_TEST(test_select_silent_tacho_falls_back_to_can_rpm);
    RUN_TEST(test_select_silent_fitted_tacho_reads_zero);
    RUN_TEST(test_select_not_fitted);
    RUN_TEST(test_median_of_8_clean);
    RUN_TEST(test_median_ignores_one_spurious_half_period);
    RUN_TEST(test_median_ignores_two_spurious_half_periods);
    RUN_TEST(test_median_ignores_one_missed_edge);
    RUN_TEST(test_median_uses_only_last_n_periods);
    RUN_TEST(test_median_falls_back_to_mean_below_three_periods);
    RUN_TEST(test_edge_filter_unlocked_floor_only);
    RUN_TEST(test_edge_filter_lock_window);
    RUN_TEST(test_format_periods_ms);
    return UNITY_END();
}
