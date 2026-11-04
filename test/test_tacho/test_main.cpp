// Native unit tests for the ECU tacho input (include/TachoLaw.h): period → RPM, averaging,
// timeout, plausibility, ppr validation / snapping, calibration and RPM source selection;
// fw 1.5.10: median period and diagnostics formatting; fw 1.5.11: pulse-width edge validator.
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

// ---- fw 1.5.10: median period, diagnostics formatting ----------------------

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


// ---- fw 1.5.11: pulse-width edge validator (tachoEdgeStep) --------------------------------

static const uint32_t MIN_LEVEL = 2000;   // TACHO_MIN_LEVEL_US

// Simple pin-change simulator: feeds one edge, collects accepted rise times and glitch count.
struct Sim {
    TachoEdgeState st;
    uint32_t accepted[64];
    uint8_t  nAcc;
    uint16_t glitches;
    void init(uint8_t level, uint32_t t) { tachoEdgeInit(st, level, t); nAcc = 0; glitches = 0; }
    TachoEdgeEvent edge(uint8_t level, uint32_t t) {
        const TachoEdgeEvent ev = tachoEdgeStep(st, level, t, MIN_LEVEL);
        if (ev == TachoEdgeEvent::ACCEPTED_RISE && nAcc < 64) accepted[nAcc++] = st.riseUs;
        if (ev == TachoEdgeEvent::GLITCH) glitches++;
        return ev;
    }
};

static void test_edge_clean_square_23hz() {
    // 23 Hz, 50 % duty: period 43 478 µs, high 21 739 µs
    Sim s; s.init(0, 0);
    const uint32_t P = 43478, H = 21739;
    for (uint32_t k = 0; k < 10; k++) {
        const uint32_t t0 = 100000 + k * P;
        TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(1, t0));            // candidate only
        TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, t0 + H));
    }
    TEST_ASSERT_EQUAL_UINT8(10, s.nAcc);
    TEST_ASSERT_EQUAL_UINT16(0, s.glitches);
    for (uint8_t i = 1; i < s.nAcc; i++) TEST_ASSERT_EQUAL_UINT32(P, s.accepted[i] - s.accepted[i - 1]);
    TEST_ASSERT_EQUAL_UINT32(H, s.st.lastHighUs);
    TEST_ASSERT_EQUAL_UINT32(P - H, s.st.lastLowUs);
    // the median over the accepted rises is the true period
    TEST_ASSERT_FLOAT_WITHIN(0.5f, (float)P, tachoMedianPeriod(s.accepted + 1, 9, 8));
}

static void test_edge_bounce_on_falling_edge_rejected() {
    // Each falling edge rings: fall, 0.3 ms low dip, back high 0.1 ms, fall again (hypothesis B)
    Sim s; s.init(0, 0);
    const uint32_t P = 43478, H = 21739;
    for (uint32_t k = 0; k < 6; k++) {
        const uint32_t t0 = 50000 + k * P;
        s.edge(1, t0);
        TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, t0 + H));
        TEST_ASSERT_EQUAL(TachoEdgeEvent::GLITCH, s.edge(1, t0 + H + 300));   // spurious rise
        TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(0, t0 + H + 400));     // no second accept
    }
    TEST_ASSERT_EQUAL_UINT8(6, s.nAcc);
    TEST_ASSERT_EQUAL_UINT16(6, s.glitches);
    for (uint8_t i = 1; i < s.nAcc; i++) TEST_ASSERT_EQUAL_UINT32(P, s.accepted[i] - s.accepted[i - 1]);
    // the low after the bounce is measured from the LAST fall
    TEST_ASSERT_EQUAL_UINT32(P - H - 400, s.st.lastLowUs);
}

static void test_edge_glitch_low_inside_high_ignored() {
    // 0.2 ms low glitch early inside the high pulse (before minLevel has elapsed)
    Sim s; s.init(0, 0);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(1, 10000));
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(0, 11000));     // high only 1 ms: pending
    TEST_ASSERT_EQUAL(TachoEdgeEvent::GLITCH, s.edge(1, 11200));   // low 0.2 ms: pulse continues
    TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, 31500));
    TEST_ASSERT_EQUAL_UINT8(1, s.nAcc);
    TEST_ASSERT_EQUAL_UINT32(10000, s.accepted[0]);                // the ORIGINAL rise time
    TEST_ASSERT_EQUAL_UINT32(21500, s.st.lastHighUs);              // the whole high span
    // and late inside the high pulse (after minLevel): accepted once, never twice
    s.edge(1, 53000);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, 63000));
    TEST_ASSERT_EQUAL(TachoEdgeEvent::GLITCH, s.edge(1, 63200));
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(0, 74500));
    TEST_ASSERT_EQUAL_UINT8(2, s.nAcc);
    TEST_ASSERT_EQUAL_UINT32(53000, s.accepted[1]);
}

static void test_edge_narrow_high_pulse_rejected() {
    // A 1 ms high spike in the middle of a low, then a real pulse
    Sim s; s.init(0, 0);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(1, 20000));
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(0, 21000));     // high 1 ms: not accepted
    TEST_ASSERT_EQUAL(TachoEdgeEvent::GLITCH, s.edge(1, 40000));   // low 19 ms: the spike is dropped
    TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, 61500));
    TEST_ASSERT_EQUAL_UINT8(1, s.nAcc);
    TEST_ASSERT_EQUAL_UINT32(40000, s.accepted[0]);
    TEST_ASSERT_EQUAL_UINT16(1, s.glitches);
}

static void test_edge_first_edge_after_silence() {
    // Boot with the pin low: the first rise is a candidate (the low counts as long)
    Sim s; s.init(0, 1000);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(1, 1500));      // only 0.5 ms after init, still OK
    TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, 23000));
    TEST_ASSERT_EQUAL_UINT32(1500, s.accepted[0]);
    // 5 s of silence (low), then the engine restarts: the first rise is accepted again
    tachoEdgeSilence(s.st);
    s.edge(1, 5023000);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, 5044000));
    TEST_ASSERT_EQUAL_UINT8(2, s.nAcc);
    TEST_ASSERT_EQUAL_UINT32(5023000, s.accepted[1]);
}

static void test_edge_boot_high_and_stuck_high() {
    // Boot with the pin high: that pulse has an unknown start and is never accepted
    Sim s; s.init(1, 0);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(0, 15000));
    TEST_ASSERT_EQUAL_UINT8(0, s.nAcc);
    TEST_ASSERT_FALSE(s.st.highMeasured);
    s.edge(1, 36000);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, 57000));
    // Engine stopped with the signal high: after the silence the stale rise is dropped
    s.edge(1, 80000);
    tachoEdgeSilence(s.st);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(0, 3000000));
    TEST_ASSERT_EQUAL_UINT8(1, s.nAcc);
}

static void test_edge_micros_wrap() {
    // A clean 46 Hz square (2 ppr idle) straddling the 32-bit µs wrap
    Sim s;
    const uint32_t P = 21400, H = 10700;
    const uint32_t start = 0xFFFFFFFFu - 50000u;
    s.init(0, start - 30000u);
    for (uint32_t k = 0; k < 6; k++) {
        const uint32_t t0 = start + k * P;   // wraps naturally
        s.edge(1, t0);
        TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, t0 + H));
    }
    TEST_ASSERT_EQUAL_UINT8(6, s.nAcc);
    for (uint8_t i = 1; i < s.nAcc; i++) TEST_ASSERT_EQUAL_UINT32(P, s.accepted[i] - s.accepted[i - 1]);
    TEST_ASSERT_EQUAL_UINT32(H, s.st.lastHighUs);
    TEST_ASSERT_EQUAL_UINT32(P - H, s.st.lastLowUs);
    // a glitch across the wrap is still a glitch
    s.init(1, 0xFFFFFFFFu - 30000u);
    s.edge(0, 0xFFFFFFFFu - 100u);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::GLITCH, s.edge(1, 100u));    // low 201 µs across the wrap
}

static void test_edge_same_level_and_thresholds() {
    Sim s; s.init(0, 0);
    // a repeated level (both edges of a glitch elapsed before the ISR read the pin) is ignored
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(0, 5000));
    s.edge(1, 10000);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(1, 10500));
    TEST_ASSERT_EQUAL_UINT32(10000, s.st.riseUs);
    // a high of exactly minLevel is accepted; a low one µs short is a glitch
    TEST_ASSERT_EQUAL(TachoEdgeEvent::ACCEPTED_RISE, s.edge(0, 10000 + MIN_LEVEL));
    TEST_ASSERT_EQUAL(TachoEdgeEvent::GLITCH, s.edge(1, 10000 + 2 * MIN_LEVEL - 1));
    s.init(0, 0);
    s.edge(1, 10000);
    TEST_ASSERT_EQUAL(TachoEdgeEvent::NONE, s.edge(0, 10000 + MIN_LEVEL - 1));   // pending
    TEST_ASSERT_EQUAL(TachoEdgeEvent::GLITCH, s.edge(1, 10000 + 2 * MIN_LEVEL));  // narrow dropped
    // sanity floor and the diagnostics conversion
    TEST_ASSERT_FALSE(tachoPeriodAboveFloor(1499, 1500));
    TEST_ASSERT_TRUE(tachoPeriodAboveFloor(1500, 1500));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.7f, tachoUsToMs1(10690));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, tachoUsToMs1(0));
}

static void test_edge_half_frequency_signal_reads_true_period() {
    // Hypothesis A data (46 Hz, 2 ppr) vs B (23 Hz + falling-edge bounce): both give the TRUE
    // period, and the high time tells them apart (≈10.7 ms vs ≈21.7 ms).
    Sim a; a.init(0, 0);
    for (uint32_t k = 0; k < 9; k++) { a.edge(1, 1000 + k * 21400); a.edge(0, 1000 + k * 21400 + 10700); }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 21400.0f, tachoMedianPeriod(a.accepted, a.nAcc, 8));
    TEST_ASSERT_EQUAL_UINT32(10700, a.st.lastHighUs);
    Sim b; b.init(0, 0);
    for (uint32_t k = 0; k < 9; k++) {
        const uint32_t t0 = 1000 + k * 42800;
        b.edge(1, t0); b.edge(0, t0 + 21400); b.edge(1, t0 + 21600); b.edge(0, t0 + 21650);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 42800.0f, tachoMedianPeriod(b.accepted, b.nAcc, 8));
    TEST_ASSERT_EQUAL_UINT16(9, b.glitches);
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
    RUN_TEST(test_edge_clean_square_23hz);
    RUN_TEST(test_edge_bounce_on_falling_edge_rejected);
    RUN_TEST(test_edge_glitch_low_inside_high_ignored);
    RUN_TEST(test_edge_narrow_high_pulse_rejected);
    RUN_TEST(test_edge_first_edge_after_silence);
    RUN_TEST(test_edge_boot_high_and_stuck_high);
    RUN_TEST(test_edge_micros_wrap);
    RUN_TEST(test_edge_same_level_and_thresholds);
    RUN_TEST(test_edge_half_frequency_signal_reads_true_period);
    RUN_TEST(test_format_periods_ms);
    return UNITY_END();
}
