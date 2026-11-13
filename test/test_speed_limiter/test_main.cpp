// Native unit tests for the maximum-speed limiter law (include/SpeedLimiterLaw.h).
// Run: pio test -e native
#include <unity.h>
#include <math.h>
#include "SpeedLimiterLaw.h"

// Mirror of include/Constants.h (which needs Arduino.h and cannot be built natively).
static SpeedLimiterParams params(float floorPct = 0.0f) {
    SpeedLimiterParams p;
    p.taperFraction = 0.20f;   // SPEED_LIMIT_TAPER_FRACTION
    p.taperMinMs = 0.5f;       // SPEED_LIMIT_TAPER_MIN_MS
    p.taperMaxMs = 1.4f;       // SPEED_LIMIT_TAPER_MAX_MS
    p.floorPct = floorPct;     // SPEED_LIMIT_FLOOR_DEFAULT_PCT = 0
    p.kiPctSPerMs = 30.0f;     // SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS
    p.risePctS = 10.0f;        // SPEED_LIMIT_TRIM_RISE_PCT_S
    p.slewPctS = 80.0f;        // SPEED_LIMIT_SLEW_DEFAULT_PCT_S (per-vehicle setting)
    p.bindMarginPct = 1.0f;    // SPEED_LIMIT_BIND_MARGIN_PCT
    return p;
}

static const float L10 = 2.78f;   // 10 km/h
static const float DT = 0.02f;    // 20 ms control loop

void setUp() {}
void tearDown() {}

// ── band ──────────────────────────────────────────────────────────────────────
static void test_band() {
    const SpeedLimiterParams p = params();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.556f, speedLimiterBand(2.78f, p));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.112f, speedLimiterBand(5.56f, p));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.4f, speedLimiterBand(11.1f, p));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, speedLimiterBand(1.39f, p));
}

// ── taper ─────────────────────────────────────────────────────────────────────
static void checkTaper(float floorPct) {
    const float B = speedLimiterBand(L10, params(floorPct));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, speedLimiterTaper(L10 - B, L10, B, floorPct));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, speedLimiterTaper(0.0f, L10, B, floorPct));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, floorPct, speedLimiterTaper(L10, L10, B, floorPct));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (100.0f + floorPct) / 2.0f,
                             speedLimiterTaper(L10 - B / 2.0f, L10, B, floorPct));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, floorPct, speedLimiterTaper(L10 + 5.0f, L10, B, floorPct));
}
static void test_taper_floor0() { checkTaper(0.0f); }
static void test_taper_floor10() { checkTaper(10.0f); }

// ── first call ────────────────────────────────────────────────────────────────
static void test_first_call_adopts_taper() {
    const SpeedLimiterParams p = params();
    SpeedLimiterState s;
    s.trimPct = 42.0f;  // stale value must be discarded
    const float B = speedLimiterBand(L10, p);
    const float v = L10 - B / 4.0f;
    const float out = speedLimiterStep(s, p, 100.0f, v, L10, 0.5f);
    const float taper = speedLimiterTaper(v, L10, B, 0.0f);
    TEST_ASSERT_TRUE(s.primed);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, taper, s.ceilPct);   // no slew from 100
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, s.trimPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, taper, out);
}

// ── over the limit ────────────────────────────────────────────────────────────
static void test_over_limit_trim_falls_in_2_4_s() {
    const SpeedLimiterParams p = params();
    SpeedLimiterState s;
    s.primed = true; s.trimPct = 100.0f; s.ceilPct = 100.0f;
    float t = 0.0f;
    while (s.trimPct > 0.0f && t < 10.0f) {
        speedLimiterStep(s, p, 100.0f, L10 + 1.4f, L10, DT);
        t += DT;
    }
    TEST_ASSERT_TRUE(t >= 2.3f && t <= 2.5f);
}

static void test_over_limit_large_excess_capped_at_slew() {
    const SpeedLimiterParams p = params();
    SpeedLimiterState s;
    s.primed = true; s.trimPct = 100.0f; s.ceilPct = 100.0f;
    speedLimiterStep(s, p, 100.0f, L10 + 5.6f, L10, DT);   // 168 %/s requested
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f - 80.0f * DT, s.trimPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f - 80.0f * DT, s.ceilPct);
}

static void test_over_limit_falls_even_when_not_binding() {
    const SpeedLimiterParams p = params();
    SpeedLimiterState s;
    s.primed = true; s.trimPct = 50.0f; s.ceilPct = 50.0f;
    speedLimiterStep(s, p, 0.0f, L10 + 1.0f, L10, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f - 30.0f * 1.0f * DT, s.trimPct);
}

// ── under the limit ───────────────────────────────────────────────────────────
static void test_under_binding_rise_capped() {
    const SpeedLimiterParams p = params();
    const float B = speedLimiterBand(L10, p);
    SpeedLimiterState s;
    s.primed = true; s.trimPct = 20.0f; s.ceilPct = 20.0f;
    // Deficit 0.5 m/s → 15 %/s requested, capped at 10 %/s.
    speedLimiterStep(s, p, 100.0f, L10 - 0.5f, L10, DT);
    TEST_ASSERT_TRUE(L10 - 0.5f > L10 - B);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f + 10.0f * DT, s.trimPct);
    // Deficit 0.1 m/s → 3 %/s, below the cap.
    s.trimPct = 20.0f;
    speedLimiterStep(s, p, 100.0f, L10 - 0.1f, L10, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f + 3.0f * DT, s.trimPct);
}

static void test_under_not_binding_holds() {
    const SpeedLimiterParams p = params();
    SpeedLimiterState s;
    s.primed = true; s.trimPct = 20.0f; s.ceilPct = 60.0f;
    for (int i = 0; i < 100; i++) {
        speedLimiterStep(s, p, 30.0f, L10 - 0.2f, L10, DT);   // demand < ceil − 1 % throughout
        TEST_ASSERT_TRUE(30.0f < s.ceilPct - 1.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20.0f, s.trimPct);
}

static void test_below_band_resets_trim() {
    const SpeedLimiterParams p = params();
    const float B = speedLimiterBand(L10, p);
    SpeedLimiterState s;
    s.primed = true; s.trimPct = 40.0f; s.ceilPct = 70.0f;
    const float out = speedLimiterStep(s, p, 100.0f, L10 - B, L10, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, s.trimPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 70.0f + 80.0f * DT, s.ceilPct);   // slews back to 100
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, s.ceilPct, out);
}

// ── clamps ────────────────────────────────────────────────────────────────────
static void test_clamps_and_floor_change() {
    SpeedLimiterParams p = params(10.0f);
    SpeedLimiterState s;
    s.primed = true; s.trimPct = 95.0f; s.ceilPct = 100.0f;
    // Trim must not exceed 100 − floor even while rising.
    speedLimiterStep(s, p, 100.0f, L10 - 0.1f, L10, DT);
    TEST_ASSERT_TRUE(s.trimPct <= 90.0f + 1e-4f);
    TEST_ASSERT_TRUE(s.ceilPct <= 100.0f && s.ceilPct >= 10.0f);
    // Long over-limit run: trim to 0, ceiling to the floor, never below.
    for (int i = 0; i < 1000; i++) speedLimiterStep(s, p, 100.0f, L10 + 3.0f, L10, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, s.trimPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, s.ceilPct);
    // Floor raised mid-run to 20: ceiling re-clamps up to the new floor.
    p.floorPct = 20.0f;
    speedLimiterStep(s, p, 100.0f, L10 + 3.0f, L10, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20.0f, s.ceilPct);
    // Floor lowered back to 0 while held at a high trim: trim re-clamps to 100 − floor.
    p.floorPct = 15.0f;
    s.trimPct = 95.0f;
    speedLimiterStep(s, p, 100.0f, L10 - 0.1f, L10, DT);
    TEST_ASSERT_TRUE(s.trimPct <= 85.0f + 1e-4f);
}

// ── slew ──────────────────────────────────────────────────────────────────────
static void test_slew_both_directions() {
    const SpeedLimiterParams p = params();
    SpeedLimiterState s;
    speedLimiterStep(s, p, 100.0f, 0.0f, L10, DT);   // primed at 100
    float prev = s.ceilPct;
    // Jump far above the limit, then far below, then oscillate; never > 80 %/s × dt.
    const float speeds[] = {L10 + 6.0f, L10 + 6.0f, L10 + 6.0f, 0.0f, 0.0f, L10, L10 - 0.1f,
                            L10 + 1.0f, 0.0f, L10 + 3.0f};
    for (int k = 0; k < 10; k++) {
        for (int i = 0; i < 20; i++) {
            speedLimiterStep(s, p, 100.0f, speeds[k], L10, DT);
            TEST_ASSERT_TRUE(fabsf(s.ceilPct - prev) <= 80.0f * DT + 1e-4f);
            prev = s.ceilPct;
        }
    }
    // dt is clamped to 1 s.
    SpeedLimiterState s2;
    speedLimiterStep(s2, p, 100.0f, 0.0f, L10, DT);
    speedLimiterStep(s2, p, 100.0f, L10 + 6.0f, L10, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f, s2.ceilPct);
}

// A lower per-vehicle slew (web setting set_speed_limit_slew) closes the throttle more gently:
// over the same 0.5 s far above the limit, 20 %/s takes 10 % off the ceiling, 80 %/s takes 40 %.
static void test_lower_slew_falls_slower() {
    SpeedLimiterParams fast = params();
    SpeedLimiterParams slow = params();
    slow.slewPctS = 20.0f;
    SpeedLimiterState sf, ss;
    sf.primed = true; sf.trimPct = 100.0f; sf.ceilPct = 100.0f;
    ss.primed = true; ss.trimPct = 100.0f; ss.ceilPct = 100.0f;
    for (int i = 0; i < 25; i++) {   // 25 × 20 ms = 0.5 s
        speedLimiterStep(sf, fast, 100.0f, L10 + 6.0f, L10, DT);
        speedLimiterStep(ss, slow, 100.0f, L10 + 6.0f, L10, DT);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 60.0f, sf.ceilPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 90.0f, ss.ceilPct);
    TEST_ASSERT_TRUE(ss.ceilPct > sf.ceilPct);
}

// ── toy plant ─────────────────────────────────────────────────────────────────
// First-order speed response: dv/dt = (k·u − v) / tau, k chosen so that u* holds L.
static float simulate(const SpeedLimiterParams& p, float uStar, float seconds,
                      SpeedLimiterState& s) {
    const float k = L10 / uStar;
    const float tau = 1.5f;
    float v = 0.0f;
    for (float t = 0.0f; t < seconds; t += DT) {
        const float u = speedLimiterStep(s, p, 100.0f, v, L10, DT);
        v += (k * u - v) / tau * DT;
    }
    return v;
}

static void test_steady_state_converges_to_limit() {
    const SpeedLimiterParams p = params();
    SpeedLimiterState s;
    const float v = simulate(p, 30.0f, 120.0f, s);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, L10, v);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, s.ceilPct);
}

static void test_steady_state_saturates_at_floor() {
    const SpeedLimiterParams p = params(10.0f);
    SpeedLimiterState s;
    const float v = simulate(p, 5.0f, 120.0f, s);
    TEST_ASSERT_TRUE(v > L10);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, s.ceilPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, s.trimPct);
}

// ── multiple calls per loop ───────────────────────────────────────────────────
static void checkSplit(float v, float ceil, float trim, float demand) {
    const SpeedLimiterParams p = params();
    SpeedLimiterState a; a.primed = true; a.ceilPct = ceil; a.trimPct = trim;
    SpeedLimiterState b = a;
    speedLimiterStep(a, p, demand, v, L10, DT);
    speedLimiterStep(b, p, demand, v, L10, DT / 2.0f);
    speedLimiterStep(b, p, demand, v, L10, DT / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, a.trimPct, b.trimPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, a.ceilPct, b.ceilPct);
}
static void test_two_half_steps_equal_one_step() {
    checkSplit(L10 - 0.2f, 50.0f, 10.0f, 100.0f);   // under, binding, slewing down
    checkSplit(L10 - 0.2f, 46.0f, 10.0f, 100.0f);   // under, binding, tracking the target
    checkSplit(L10 + 0.5f, 40.0f, 40.0f, 100.0f);   // over, trim falling
    checkSplit(L10 + 0.5f, 40.0f, 40.0f, 0.0f);     // over, not binding
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_band);
    RUN_TEST(test_taper_floor0);
    RUN_TEST(test_taper_floor10);
    RUN_TEST(test_first_call_adopts_taper);
    RUN_TEST(test_over_limit_trim_falls_in_2_4_s);
    RUN_TEST(test_over_limit_large_excess_capped_at_slew);
    RUN_TEST(test_over_limit_falls_even_when_not_binding);
    RUN_TEST(test_under_binding_rise_capped);
    RUN_TEST(test_under_not_binding_holds);
    RUN_TEST(test_below_band_resets_trim);
    RUN_TEST(test_clamps_and_floor_change);
    RUN_TEST(test_slew_both_directions);
    RUN_TEST(test_lower_slew_falls_slower);
    RUN_TEST(test_steady_state_converges_to_limit);
    RUN_TEST(test_steady_state_saturates_at_floor);
    RUN_TEST(test_two_half_steps_equal_one_step);
    return UNITY_END();
}
