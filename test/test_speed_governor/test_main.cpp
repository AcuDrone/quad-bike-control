// Native unit tests for the speed governor law (include/SpeedGovernorLaw.h), change
// add-speed-governor. Run: pio test -e native
//
// The closed-loop tests drive the law against a test-only vehicle plant (design D13): first-order
// speed response toward a static throttle → speed map with a clutch take-up, a pure dead time,
// the throttle quantised exactly as ThrottleController::percentToUs() + the 14-bit LEDC duty do,
// and the speed measured as SpeedSensor does (28.4 mm pulses counted over ≈ 200 ms windows).
// It mirrors the design's scratchpad/twin_sim.py.
#include <unity.h>
#include <math.h>
#include <stdint.h>
#include <vector>
#include "SpeedGovernorLaw.h"

// ── Both vehicles' default sets, as literals ──────────────────────────────────
// MUST match GOV_*_DEFAULT in aztec's (buggy) and master's (quad bike) include/Constants.h.
// The suite runs on every branch, so both vehicles' plant tests run everywhere (design D12).
struct GovSet { float kp, ki, acc, dec, start; };
static const GovSet kBuggy = {1.0f, 0.35f, 1.0f, 2.5f, 4.0f};    // aztec
static const GovSet kQuad  = {3.0f, 1.0f, 1.0f, 2.5f, 10.0f};    // master, old-board

// Fixed constants mirrored from Constants.h (GOV_STICK_DEADBAND_PCT, GOV_MIN_SETPOINT_MS,
// GOV_STANDSTILL_MS) and the per-vehicle limiter throttle rate default (thr_slew 80 %/s).
static SpeedGovernorParams params(const GovSet& g, float slew = 80.0f) {
    SpeedGovernorParams p;
    p.kpPctPerMs = g.kp;
    p.kiPctSPerMs = g.ki;
    p.accelMs2 = g.acc;
    p.decelMs2 = g.dec;
    p.startCapPct = g.start;
    p.stickDeadbandPct = 5.0f;
    p.minSetpointMs = 1.4f;
    p.standstillMs = 0.05f;
    p.outSlewPctS = slew;
    return p;
}

static const float L10 = 10.0f / 3.6f;

void setUp() {}
void tearDown() {}

// ── Target mapping ────────────────────────────────────────────────────────────
static void test_target_mapping() {
    const SpeedGovernorParams p = params(kQuad);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, govTargetMs(0.0f, L10, p));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, govTargetMs(5.0f, L10, p));
    // 6 % → just above vMin = 1.4 m/s
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.4f + (L10 - 1.4f) / 95.0f, govTargetMs(6.0f, L10, p));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, L10, govTargetMs(100.0f, L10, p));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, L10, govTargetMs(150.0f, L10, p));   // never above L
    // Low limit: vMin halves to 0.5 × L
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, govTargetMs(5.01f, 2.0f, p));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, govTargetMs(100.0f, 2.0f, p));
    // No limit → 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, govTargetMs(100.0f, 0.0f, p));
}

// ── Setpoint ramp ─────────────────────────────────────────────────────────────
static void test_ramp_up_down() {
    SpeedGovernorParams p = params(kQuad);
    p.kpPctPerMs = 0.2f; p.kiPctSPerMs = 0.0f;
    SpeedGovernorState s;
    govPrime(s, 0.0f, 2.0f, p.startCapPct);
    govStep(s, p, 100.0f, false, 2.0f, L10, 0.2f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.2f, s.spMs);            // +acc × dt
    govPrime(s, 0.0f, 2.78f, p.startCapPct);
    govStep(s, p, 50.0f, false, 2.78f, L10, 0.2f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.28f, s.spMs);           // −dec × dt
    // A lowered limit only lowers the target; the setpoint follows at dec.
    govPrime(s, 0.0f, 5.0f, p.startCapPct);
    govStep(s, p, 100.0f, false, 5.0f, L10, 0.2f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.5f, s.spMs);
    for (int i = 0; i < 20; i++) govStep(s, p, 100.0f, false, 5.0f, L10, 0.2f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, L10, s.spMs);
}

// ── Bumpless prime ────────────────────────────────────────────────────────────
static void test_prime_bumpless() {
    const SpeedGovernorParams p = params(kQuad);
    SpeedGovernorState s;
    govPrime(s, 5.0f, 4.0f, p.startCapPct);
    TEST_ASSERT_TRUE(s.primed);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f, s.spMs);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, s.integPct);
    // First evaluation (dt = 0): output equals the throttle at take-over.
    const float out = govStep(s, p, 100.0f, false, 4.0f, 6.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, out);
}

// ── Stop ──────────────────────────────────────────────────────────────────────
static void test_stop_immediate() {
    const SpeedGovernorParams p = params(kQuad);
    SpeedGovernorState s;
    govPrime(s, 40.0f, 2.0f, p.startCapPct);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, govStep(s, p, 3.0f, false, 2.0f, L10, 0.04f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.integPct);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.spMs);
    govPrime(s, 40.0f, 2.0f, p.startCapPct);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, govStep(s, p, 100.0f, true, 2.0f, L10, 0.04f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.integPct);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.spMs);
}

// ── P and I ───────────────────────────────────────────────────────────────────
static void test_p_and_i() {
    const GovSet g = {1.5f, 0.5f, 1.0f, 2.5f, 10.0f};   // test values
    const SpeedGovernorParams p = params(g, 1000.0f);
    SpeedGovernorState s;
    govPrime(s, 0.0f, 1.78f, p.startCapPct);
    s.spMs = L10;   // setpoint at the target: constant error 1 m/s
    const float v = L10 - 1.0f;
    // First step: output = Kp × e at once (the output rate 1000 %/s does not bind)
    float out = govStep(s, p, 100.0f, false, v, L10, 0.04f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.5f, out);
    for (int i = 0; i < 24; i++) out = govStep(s, p, 100.0f, false, v, L10, 0.04f);
    // After 1 s: I = 0.5 (the output uses the integrator before each step: one step behind)
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 1.5f + 0.5f, out);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, s.integPct);
}

// ── Speed quantum ─────────────────────────────────────────────────────────────
static void quantumCheck(const GovSet& g, float maxDelta) {
    const SpeedGovernorParams p = params(g);
    SpeedGovernorState a, b;
    govPrime(a, 5.0f, 2.70f, p.startCapPct); a.spMs = L10;
    b = a;
    const float oa = govStep(a, p, 100.0f, false, 2.70f, L10, 0.04f);
    const float ob = govStep(b, p, 100.0f, false, 2.70f + 0.142f, L10, 0.04f);
    TEST_ASSERT_TRUE(fabsf(oa - ob) <= maxDelta);
}
static void test_quantum_step() {
    quantumCheck(kBuggy, 0.15f);
    quantumCheck(kQuad, 0.45f);
}

// ── Anti-windup ───────────────────────────────────────────────────────────────
static void test_antiwindup_high() {
    const SpeedGovernorParams p = params(kQuad, 1000.0f);
    SpeedGovernorState s;
    govPrime(s, 0.0f, 1.0f, p.startCapPct);
    s.integPct = 95.0f; s.spMs = 20.0f;
    float out = 0.0f;
    for (int i = 0; i < 100; i++) out = govStep(s, p, 100.0f, false, 1.0f, 20.0f, 0.04f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, out);
    const float i0 = s.integPct;
    for (int i = 0; i < 100; i++) govStep(s, p, 100.0f, false, 1.0f, 20.0f, 0.04f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, i0, s.integPct);
    TEST_ASSERT_TRUE(s.integPct <= 100.0f);
}

static void test_antiwindup_low() {
    const SpeedGovernorParams p = params(kQuad, 1000.0f);
    SpeedGovernorState s;
    govPrime(s, 0.0f, L10, p.startCapPct);
    s.spMs = L10;
    // Downhill: 1.5 m/s too fast for 10 s → output 0, integrator not below 0
    float out = 1.0f;
    for (int i = 0; i < 250; i++) out = govStep(s, p, 100.0f, false, L10 + 1.5f, L10, 0.04f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.integPct);
    // Error turns positive (0.5 m/s): output back at once to Kp × e (one Kp step)
    out = govStep(s, p, 100.0f, false, L10 - 0.5f, L10, 0.04f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f * 0.5f, out);
}

// ── Start throttle limit ──────────────────────────────────────────────────────
static void startCapCheck(const GovSet& g) {
    const SpeedGovernorParams p = params(g);
    SpeedGovernorState s;
    govPrime(s, 0.0f, 0.0f, p.startCapPct);
    float out = 0.0f;
    for (int i = 0; i < 250; i++) {   // 10 s, speed 0, large error
        out = govStep(s, p, 100.0f, false, 0.0f, 30.0f / 3.6f, 0.04f);
        TEST_ASSERT_TRUE(out <= g.start + 1e-4f);
        TEST_ASSERT_TRUE(s.integPct <= g.start + 1e-4f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, g.start, out);
    // The wheels turn (0.2 m/s): the cap lifts
    for (int i = 0; i < 25; i++) out = govStep(s, p, 100.0f, false, 0.2f, 30.0f / 3.6f, 0.04f);
    TEST_ASSERT_TRUE(out > g.start + 1.0f);
}
static void test_start_cap_standstill() {
    startCapCheck(kBuggy);
    startCapCheck(kQuad);
}

static void primeCappedCheck(const GovSet& g) {
    const SpeedGovernorParams p = params(g);
    SpeedGovernorState s;
    govPrime(s, 100.0f, 0.3f, p.startCapPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, g.start, s.integPct);
    float prev = govStep(s, p, 100.0f, false, 0.3f, L10, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 100.0f, prev);
    for (int i = 0; i < 50; i++) {
        const float out = govStep(s, p, 100.0f, false, 0.3f, L10, 0.04f);
        TEST_ASSERT_TRUE(prev - out <= 80.0f * 0.04f + 1e-3f);   // falls at ≤ thr_slew
        prev = out;
    }
    // ...and arrives at P + I (well below 100 %)
    TEST_ASSERT_TRUE(prev < 40.0f);
}
static void test_prime_capped() {
    primeCappedCheck(kBuggy);
    primeCappedCheck(kQuad);
}

static void test_output_slew() {
    SpeedGovernorParams p = params(kQuad);
    SpeedGovernorState s;
    govPrime(s, 10.0f, 1.0f, p.startCapPct);
    s.spMs = L10;
    govStep(s, p, 100.0f, false, 1.0f, L10, 0.04f);
    const float before = s.outPct;
    p.kpPctPerMs = 10.0f;   // Kp jump → P jumps by ~12 %
    const float after = govStep(s, p, 100.0f, false, 1.0f, L10, 0.04f);
    TEST_ASSERT_TRUE(after - before <= 80.0f * 0.04f + 1e-4f);
    TEST_ASSERT_TRUE(after > before + 3.0f);
}

static void test_dt_cap() {
    const SpeedGovernorParams p = params(kQuad);
    SpeedGovernorState a, b;
    govPrime(a, 0.0f, 2.0f, p.startCapPct);
    b = a;
    govStep(a, p, 100.0f, false, 2.0f, L10, 5.0f);
    govStep(b, p, 100.0f, false, 2.0f, L10, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, b.spMs, a.spMs);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, b.integPct, a.integPct);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, b.outPct, a.outPct);
}

// ── Modes ─────────────────────────────────────────────────────────────────────
static void test_mode_precedence() {
    for (int en = 0; en < 2; en++)
    for (int lim = 0; lim < 2; lim++)
    for (int byp = 0; byp < 2; byp++)
    for (int val = 0; val < 2; val++) {
        const GovMode m = govModeSelect(en, lim ? 2.78f : 0.0f, byp, val);
        GovMode want;
        if (!en) want = GovMode::OFF;
        else if (!lim) want = GovMode::NOLIMIT;
        else if (byp) want = GovMode::BYPASS;
        else if (!val) want = GovMode::NOSPEED;
        else want = GovMode::ACTIVE;
        TEST_ASSERT_EQUAL_UINT8((uint8_t)want, (uint8_t)m);
    }
    TEST_ASSERT_EQUAL_STRING("off", govModeToken(GovMode::OFF));
    TEST_ASSERT_EQUAL_STRING("nolimit", govModeToken(GovMode::NOLIMIT));
    TEST_ASSERT_EQUAL_STRING("bypass", govModeToken(GovMode::BYPASS));
    TEST_ASSERT_EQUAL_STRING("nospeed", govModeToken(GovMode::NOSPEED));
    TEST_ASSERT_EQUAL_STRING("active", govModeToken(GovMode::ACTIVE));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)GovMode::NOLIMIT, (uint8_t)govModeSelect(true, NAN, false, true));
}

// Switch OFF = today's path untouched: with the switch off the mode is OFF whatever the limit,
// bypass and speed, and once it is OFF with no blend pending the caller takes the pass-through
// (VehicleController::commandThrottlePct() returns applySpeedLimit(demand) and evaluates nothing
// else). Only a blend out of an earlier ACTIVE, or the first evaluation after the switch went off
// while governing, keeps the governor path.
static void test_switch_off_pass_through() {
    for (int lim = 0; lim < 3; lim++)
    for (int byp = 0; byp < 2; byp++)
    for (int val = 0; val < 2; val++) {
        const float L = (lim == 0) ? 0.0f : (lim == 1 ? 2.78f : NAN);
        TEST_ASSERT_EQUAL_UINT8((uint8_t)GovMode::OFF, (uint8_t)govModeSelect(false, L, byp, val));
    }
    TEST_ASSERT_TRUE(govPassThrough(false, GovMode::OFF, false));     // untouched board: always
    TEST_ASSERT_FALSE(govPassThrough(false, GovMode::OFF, true));     // blend after switching off
    TEST_ASSERT_FALSE(govPassThrough(false, GovMode::ACTIVE, false)); // first call after switch-off
    TEST_ASSERT_FALSE(govPassThrough(true, GovMode::OFF, false));     // switched on
    for (int m = 1; m <= 4; m++) TEST_ASSERT_FALSE(govPassThrough(false, (GovMode)m, false));
}

// ── Hand-over blend ───────────────────────────────────────────────────────────
static float blendTime(float from, float to) {
    float b = from; float t = 0.0f;
    while (!govBlendDone(b, to) && t < 10.0f) { b = govBlendStep(b, to, 80.0f, 0.005f); t += 0.005f; }
    return t;
}
static void test_blend_both_directions() {
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.875f, blendTime(30.0f, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, blendTime(60.0f, 20.0f));
    TEST_ASSERT_TRUE(govBlendDone(50.0f, 50.4f));
    TEST_ASSERT_FALSE(govBlendDone(50.0f, 50.6f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 33.2f, govBlendStep(30.0f, 100.0f, 80.0f, 0.04f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 56.8f, govBlendStep(60.0f, 20.0f, 80.0f, 0.04f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 30.0f + 80.0f, govBlendStep(30.0f, 200.0f, 80.0f, 5.0f));   // dt cap
}

static void test_sat_raw() {
    TEST_ASSERT_TRUE(govSatRaw(0.0f, 3.2f, 2.8f, 0.3f));
    TEST_ASSERT_FALSE(govSatRaw(0.0f, 3.0f, 2.8f, 0.3f));
    TEST_ASSERT_FALSE(govSatRaw(5.0f, 3.2f, 2.8f, 0.3f));
}

static void test_validators() {
    TEST_ASSERT_TRUE(govKpValid(0.2f));  TEST_ASSERT_TRUE(govKpValid(10.0f));
    TEST_ASSERT_FALSE(govKpValid(0.19f)); TEST_ASSERT_FALSE(govKpValid(10.01f)); TEST_ASSERT_FALSE(govKpValid(NAN));
    TEST_ASSERT_TRUE(govKiValid(0.0f));  TEST_ASSERT_TRUE(govKiValid(5.0f));
    TEST_ASSERT_FALSE(govKiValid(-0.01f)); TEST_ASSERT_FALSE(govKiValid(5.01f)); TEST_ASSERT_FALSE(govKiValid(NAN));
    TEST_ASSERT_TRUE(govAccValid(0.3f)); TEST_ASSERT_TRUE(govAccValid(3.0f));
    TEST_ASSERT_FALSE(govAccValid(0.29f)); TEST_ASSERT_FALSE(govAccValid(3.01f)); TEST_ASSERT_FALSE(govAccValid(NAN));
    TEST_ASSERT_TRUE(govDecValid(0.5f)); TEST_ASSERT_TRUE(govDecValid(5.0f));
    TEST_ASSERT_FALSE(govDecValid(0.49f)); TEST_ASSERT_FALSE(govDecValid(5.01f)); TEST_ASSERT_FALSE(govDecValid(NAN));
    TEST_ASSERT_TRUE(govLaunchValid(1.0f)); TEST_ASSERT_TRUE(govLaunchValid(40.0f));
    TEST_ASSERT_FALSE(govLaunchValid(0.5f)); TEST_ASSERT_FALSE(govLaunchValid(41.0f)); TEST_ASSERT_FALSE(govLaunchValid(NAN));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, govOrDefault(NAN, govKpValid, 3.0f));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, govOrDefault(11.0f, govKpValid, 3.0f));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, govOrDefault(2.0f, govKpValid, 3.0f));
}

// ── Defaults at a 10 km/h limit (spec scenario "Default gains at a 10 km/h limit") ──
static void defaultsAt10(const GovSet& g, float pWant) {
    const SpeedGovernorParams p = params(g, 1000.0f);
    SpeedGovernorState s;
    govPrime(s, 0.0f, 0.0f, p.startCapPct);
    s.spMs = L10;
    const float out = govStep(s, p, 100.0f, false, 0.0f, L10, 0.04f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, pWant, g.kp * L10);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, (pWant < g.start) ? pWant : g.start, out);
}
static void test_defaults_at_10() {
    defaultsAt10(kBuggy, 2.78f);
    defaultsAt10(kQuad, 8.33f);
}

// ══ Closed-loop plant (test only, design D13; mirrors scratchpad/twin_sim.py) ══════════════
struct Pt { float u, v; };
struct Plant { Pt p[6]; };
// take-up, throttle % for 10 / 20 / 30 km/h, then flattening to 25 m/s at 100 %
static Plant mkPlant(float u0, float u10, float u20, float u30) {
    float u45 = u30 + 2.0f * (u30 - u20);
    if (u45 > 90.0f) u45 = 90.0f;
    Plant pl = {{{u0, 0.0f}, {u10, 2.78f}, {u20, 5.56f}, {u30, 8.33f}, {u45, 12.5f}, {100.0f, 25.0f}}};
    return pl;
}
static const Plant BUGGY_DESIGN = mkPlant(1, 3, 6, 10);
static const Plant BUGGY_SHARP  = mkPlant(1, 2, 3.5f, 5.5f);
static const Plant QUAD_DESIGN  = mkPlant(3, 9, 16, 25);
static const Plant QUAD_STEEP   = mkPlant(2, 6, 12, 20);
static const Plant QUAD_SOFT    = mkPlant(3, 15, 26, 38);

static float interp(const Plant& pl, float u) {
    if (u <= pl.p[0].u) return 0.0f;
    for (int i = 0; i < 5; i++) {
        if (u <= pl.p[i + 1].u) {
            return pl.p[i].v + (pl.p[i + 1].v - pl.p[i].v) * (u - pl.p[i].u) / (pl.p[i + 1].u - pl.p[i].u);
        }
    }
    return pl.p[5].v;
}

struct SimCfg {
    float limitKmh = 10.0f;
    GovSet set = kQuad;
    Plant plant = QUAD_DESIGN;
    float tau = 3.0f;
    float theta = 0.6f;
    float T = 60.0f;
    float load = 0.0f;            // slope: extra throttle %, whole run
    float stepAtS = -1.0f;        // stick 100 → stepTo at this time (< 0 = never)
    float stepTo = 50.0f;
    float primePct = NAN;         // NaN = start from idle; else take-over throttle
    float v0 = 0.0f;
};
struct Sample { float t, v, meas, sp, out; };
struct SimResult {
    std::vector<Sample> s;   // every 0.1 s
    float maxOutWhileStill = 0.0f;
};

static SimResult simulate(const SimCfg& c) {
    const float MMPP = 1990.0f / 70.0f;
    const float L = c.limitKmh / 3.6f;
    const double dt = 0.001;
    const SpeedGovernorParams p = params(c.set);
    SpeedGovernorState gs;
    if (isnan(c.primePct)) { gs.reset(); gs.primed = true; }
    bool needPrime = !isnan(c.primePct);
    double v = c.v0, posMm = 0.0;
    long lastTotal = 0;
    double nextSample = 0.2, lastPulseT = 0.0, ctrlNext = 0.0;
    float meas = c.v0, lastMoving = c.v0;
    bool stale = (c.v0 == 0.0f);
    const int delayN = (int)(c.theta / dt + 1);
    std::vector<float> buf(delayN, 0.0f);
    size_t head = 0;
    float servoUs = NAN, uq = 0.0f;
    uint32_t rng = 1;
    SimResult r;
    const long steps = (long)(c.T / dt);
    for (long k = 0; k < steps; k++) {
        const double t = k * dt;
        if (t >= ctrlNext) {
            ctrlNext += 0.04;
            const float stick = (c.stepAtS >= 0.0f && t >= c.stepAtS) ? c.stepTo : 100.0f;
            if (needPrime) { govPrime(gs, c.primePct, meas, p.startCapPct); needPrime = false; }
            const float out = govStep(gs, p, stick, false, meas, L, 0.04f);
            const int us = (int)(443.0f * out / 100.0f);
            const int duty = (int)((901 + us) * 16383L / 20000L) - (int)(901 * 16383L / 20000L);
            const float cmdUs = duty * 20000.0f / 16383.0f;
            if (isnan(servoUs)) servoUs = cmdUs;
            servoUs = cmdUs;   // servo dead-band 0 µs
            uq = servoUs / 443.0f * 100.0f;
        }
        // dead time: ring buffer of delayN samples
        const float ua = buf[head];
        buf[head] = (k > 0) ? uq : 0.0f;
        head = (head + 1) % buf.size();
        const float vss = interp(c.plant, ua - c.load);
        v += (vss - v) / c.tau * dt;
        if (v < 0) v = 0;
        posMm += v * dt * 1000.0;
        const long cnt = (long)(posMm / MMPP);
        if (t >= nextSample) {
            const long delta = cnt - lastTotal; lastTotal = cnt;
            if (delta > 0) {
                double w = t - lastPulseT;
                if (stale || w <= 0) w = 0.2;
                meas = (float)(delta * MMPP / 1000.0 / w); lastPulseT = t; stale = false; lastMoving = meas;
            } else if (!stale) {
                const double sil = t - lastPulseT;
                float dec = lastMoving - 2.2f * (float)sil; if (dec < 0) dec = 0;
                if (dec < meas) meas = dec;
                if (sil >= 2.0) { stale = true; meas = 0.0f; }
            }
            rng = rng * 1103515245u + 12345u;
            nextSample = t + 0.2 + 0.01 * ((rng >> 16) & 0x7FFF) / 32768.0;
        }
        if (v <= 0.0 && gs.outPct > r.maxOutWhileStill) r.maxOutWhileStill = gs.outPct;
        if (k % 100 == 0) r.s.push_back({(float)t, (float)v, meas, gs.spMs, gs.outPct});
    }
    return r;
}

struct Summary { float peak, rip, mean, thrpp, t90; };
static Summary summarise(const SimResult& r, float limitKmh, float tail = 25.0f) {
    const float T = r.s.back().t;
    Summary m = {0, 0, 0, 0, -1};
    float vmin = 1e9f, vmax = -1e9f, umin = 1e9f, umax = -1e9f, sum = 0; int n = 0;
    for (const Sample& x : r.s) {
        const float kmh = x.v * 3.6f;
        if (kmh > m.peak) m.peak = kmh;
        if (m.t90 < 0 && kmh >= 0.9f * limitKmh) m.t90 = x.t;
        if (x.t >= T - tail) {
            if (kmh < vmin) vmin = kmh;
            if (kmh > vmax) vmax = kmh;
            if (x.out < umin) umin = x.out;
            if (x.out > umax) umax = x.out;
            sum += kmh; n++;
        }
    }
    m.rip = vmax - vmin; m.mean = sum / n; m.thrpp = umax - umin;
    return m;
}

static Summary run(const SimCfg& c, const char* name) {
    const Summary m = summarise(simulate(c), c.limitKmh);
    printf("  %-40s L%2.0f: peak %5.2f mean %5.2f rip %4.2f thrpp %4.2f t90 %5.1f\n",
           name, c.limitKmh, m.peak, m.mean, m.rip, m.thrpp, m.t90);
    return m;
}

static void designCheck(const GovSet& g, const Plant& pl, float L, float tau, float ripMax,
                        float thrppMax, bool checkMean, const char* name) {
    SimCfg c; c.set = g; c.plant = pl; c.limitKmh = L; c.tau = tau;
    const Summary m = run(c, name);
    TEST_ASSERT_TRUE_MESSAGE(m.peak <= L + 1.0f, name);
    TEST_ASSERT_TRUE_MESSAGE(m.rip <= ripMax, name);
    if (checkMean) TEST_ASSERT_TRUE_MESSAGE(fabsf(m.mean - L) <= 0.3f, name);
    if (thrppMax > 0) TEST_ASSERT_TRUE_MESSAGE(m.thrpp <= thrppMax, name);
}

static void test_cl_buggy_design_10() { designCheck(kBuggy, BUGGY_DESIGN, 10, 3, 1.0f, 1.0f, true, "buggy/buggy design"); }
static void test_cl_buggy_design_20() { designCheck(kBuggy, BUGGY_DESIGN, 20, 3, 1.0f, 1.0f, true, "buggy/buggy design"); }
static void test_cl_buggy_design_30() { designCheck(kBuggy, BUGGY_DESIGN, 30, 3, 1.0f, 1.0f, true, "buggy/buggy design"); }
static void test_cl_buggy_sharp_10() { designCheck(kBuggy, BUGGY_SHARP, 10, 3, 1.0f, -1, false, "buggy/buggy sharp"); }
static void test_cl_buggy_sharp_30() { designCheck(kBuggy, BUGGY_SHARP, 30, 3, 1.0f, -1, false, "buggy/buggy sharp"); }
static void test_cl_buggy_sharp_fast_10() { designCheck(kBuggy, BUGGY_SHARP, 10, 1.5f, 1.5f, -1, false, "buggy/buggy sharp tau1.5"); }
static void test_cl_quad_design_10() { designCheck(kQuad, QUAD_DESIGN, 10, 3, 1.0f, 1.5f, true, "quad/quad design"); }
static void test_cl_quad_design_20() { designCheck(kQuad, QUAD_DESIGN, 20, 3, 1.0f, 1.5f, true, "quad/quad design"); }
static void test_cl_quad_design_30() { designCheck(kQuad, QUAD_DESIGN, 30, 3, 1.0f, 1.5f, true, "quad/quad design"); }

static void test_cl_quad_steep_fast_10() {
    SimCfg c; c.set = kQuad; c.plant = QUAD_STEEP; c.tau = 1.5f;
    const Summary m = run(c, "quad/quad steep tau1.5");
    TEST_ASSERT_TRUE(m.peak <= 11.0f);
    TEST_ASSERT_TRUE(m.rip <= 1.0f);
}

static void test_cl_quad_soft_10() {
    SimCfg c; c.set = kQuad; c.plant = QUAD_SOFT;
    const Summary m = run(c, "quad/quad soft");
    TEST_ASSERT_TRUE(fabsf(m.mean - 10.0f) <= 0.3f);
    TEST_ASSERT_TRUE(m.t90 >= 0.0f && m.t90 <= 20.0f);
}

static void test_cl_dead_time_0_9() {
    SimCfg c; c.theta = 0.9f;
    c.set = kBuggy; c.plant = BUGGY_DESIGN;
    Summary m = run(c, "buggy/buggy design theta0.9");
    TEST_ASSERT_TRUE(m.peak <= 11.0f);
    TEST_ASSERT_TRUE(m.rip <= 1.0f);
    c.set = kQuad; c.plant = QUAD_DESIGN;
    m = run(c, "quad/quad design theta0.9");
    TEST_ASSERT_TRUE(m.peak <= 11.0f);
    TEST_ASSERT_TRUE(m.rip <= 1.0f);
}

// Cross-defaults: the quad's numbers on a sharp buggy hunt — why the D12 build guard exists.
static void test_cl_quad_defaults_on_sharp_buggy() {
    SimCfg c; c.set = kQuad; c.plant = BUGGY_SHARP;
    const Summary m = run(c, "quad set on buggy sharp (cross)");
    TEST_ASSERT_TRUE(m.rip >= 2.5f);
}

// Cross-defaults, the safe direction: the buggy's numbers on the quad are only sluggish.
static void test_cl_buggy_defaults_on_quad() {
    SimCfg c; c.set = kBuggy; c.plant = QUAD_DESIGN;
    const Summary m = run(c, "buggy set on quad design (cross)");
    TEST_ASSERT_TRUE(m.peak <= 11.0f);
    TEST_ASSERT_TRUE(m.rip <= 1.0f);
    TEST_ASSERT_TRUE(fabsf(m.mean - 10.0f) <= 0.5f);
}

// Regression documentation: the first draft's Kp 10 / Ki 5 hunts on the buggy design plant.
static void test_cl_old_defaults_hunt() {
    SimCfg c; c.set = {10.0f, 5.0f, 1.0f, 2.5f, 30.0f}; c.plant = BUGGY_DESIGN;
    const Summary m = run(c, "old draft 10/5 on buggy design");
    TEST_ASSERT_TRUE(m.rip >= 3.0f);
}

static void takeoverCheck(const GovSet& g, const Plant& pl, float peakMax, const char* name) {
    SimCfg c; c.set = g; c.plant = pl; c.primePct = 100.0f; c.v0 = 0.3f;
    const SimResult r = simulate(c);
    const Summary m = summarise(r, 10.0f);
    printf("  %-40s L10: peak %5.2f\n", name, m.peak);
    TEST_ASSERT_TRUE_MESSAGE(m.peak <= peakMax, name);
    for (const Sample& x : r.s) {
        if (x.t >= 15.0f) TEST_ASSERT_TRUE_MESSAGE(fabsf(x.v * 3.6f - 10.0f) <= 1.0f, name);
    }
}
static void test_cl_takeover_high_throttle() {
    takeoverCheck(kBuggy, BUGGY_DESIGN, 25.0f, "take-over 100 % buggy/buggy design");
    takeoverCheck(kQuad, QUAD_DESIGN, 20.0f, "take-over 100 % quad/quad design");
}

static void slopeCheck(const GovSet& g, const Plant& pl, float load, bool moves, const char* name) {
    SimCfg c; c.set = g; c.plant = pl; c.load = load; c.T = 40.0f;
    const SimResult r = simulate(c);
    const Summary m = summarise(r, 10.0f);
    float vmax = 0; for (const Sample& x : r.s) if (x.v > vmax) vmax = x.v;
    printf("  %-40s load +%.0f%%: peak %5.2f, max out still %4.2f\n", name, load, m.peak, r.maxOutWhileStill);
    if (moves) {
        TEST_ASSERT_TRUE_MESSAGE(vmax > 0.05f, name);
        TEST_ASSERT_TRUE_MESSAGE(m.peak <= 11.0f, name);
    } else {
        TEST_ASSERT_TRUE_MESSAGE(vmax == 0.0f, name);
        TEST_ASSERT_TRUE_MESSAGE(r.maxOutWhileStill <= g.start + 1e-3f, name);
    }
}
static void test_cl_slope_start_cap() {
    slopeCheck(kBuggy, BUGGY_DESIGN, 2.0f, true, "slope buggy/buggy design");
    slopeCheck(kBuggy, BUGGY_DESIGN, 3.0f, false, "slope buggy/buggy design");
    slopeCheck(kQuad, QUAD_DESIGN, 6.0f, true, "slope quad/quad design");
    slopeCheck(kQuad, QUAD_DESIGN, 9.0f, false, "slope quad/quad design");
}

static void stepDownCheck(const GovSet& g, const Plant& pl, const char* name) {
    SimCfg c; c.set = g; c.plant = pl; c.stepAtS = 30.0f; c.stepTo = 50.0f;
    const SimResult r = simulate(c);
    const float L = 10.0f / 3.6f;
    const float vMin = (1.4f < 0.5f * L) ? 1.4f : 0.5f * L;
    const float tgtKmh = (vMin + 45.0f / 95.0f * (L - vMin)) * 3.6f;
    float lo = 1e9f;
    for (const Sample& x : r.s) if (x.t > 30.0f && x.v * 3.6f < lo) lo = x.v * 3.6f;
    printf("  %-40s step 100->50: target %5.2f min %5.2f\n", name, tgtKmh, lo);
    TEST_ASSERT_TRUE_MESSAGE(lo >= tgtKmh - 0.5f, name);
}
static void test_cl_step_down() {
    stepDownCheck(kBuggy, BUGGY_DESIGN, "buggy/buggy design");
    stepDownCheck(kQuad, QUAD_DESIGN, "quad/quad design");
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_target_mapping);
    RUN_TEST(test_ramp_up_down);
    RUN_TEST(test_prime_bumpless);
    RUN_TEST(test_stop_immediate);
    RUN_TEST(test_p_and_i);
    RUN_TEST(test_quantum_step);
    RUN_TEST(test_antiwindup_high);
    RUN_TEST(test_antiwindup_low);
    RUN_TEST(test_start_cap_standstill);
    RUN_TEST(test_prime_capped);
    RUN_TEST(test_output_slew);
    RUN_TEST(test_dt_cap);
    RUN_TEST(test_mode_precedence);
    RUN_TEST(test_switch_off_pass_through);
    RUN_TEST(test_blend_both_directions);
    RUN_TEST(test_sat_raw);
    RUN_TEST(test_validators);
    RUN_TEST(test_defaults_at_10);
    RUN_TEST(test_cl_buggy_design_10);
    RUN_TEST(test_cl_buggy_design_20);
    RUN_TEST(test_cl_buggy_design_30);
    RUN_TEST(test_cl_buggy_sharp_10);
    RUN_TEST(test_cl_buggy_sharp_30);
    RUN_TEST(test_cl_buggy_sharp_fast_10);
    RUN_TEST(test_cl_quad_design_10);
    RUN_TEST(test_cl_quad_design_20);
    RUN_TEST(test_cl_quad_design_30);
    RUN_TEST(test_cl_quad_steep_fast_10);
    RUN_TEST(test_cl_quad_soft_10);
    RUN_TEST(test_cl_dead_time_0_9);
    RUN_TEST(test_cl_quad_defaults_on_sharp_buggy);
    RUN_TEST(test_cl_buggy_defaults_on_quad);
    RUN_TEST(test_cl_old_defaults_hunt);
    RUN_TEST(test_cl_takeover_high_throttle);
    RUN_TEST(test_cl_slope_start_cap);
    RUN_TEST(test_cl_step_down);
    return UNITY_END();
}
