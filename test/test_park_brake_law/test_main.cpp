// Native unit tests for the aztec parking-brake law (include/ParkBrakeLaw.h).
// Run: pio test -e native
#include <unity.h>
#include "ParkBrakeLaw.h"

void setUp() {}
void tearDown() {}

typedef ParkBrakeState S;
typedef ParkBrakeRequest Q;
typedef ParkBrakeResult R;
typedef ParkBrakeAction A;
typedef ParkBrakeSpeedCheck V;
typedef ParkBrakeGear G;

// ---- stored-code decode ----
static void test_decode_stored() {
    TEST_ASSERT_TRUE(parkBrakeDecodeStored(1) == S::RELEASED);
    TEST_ASSERT_TRUE(parkBrakeDecodeStored(2) == S::APPLIED);
    TEST_ASSERT_TRUE(parkBrakeDecodeStored(0) == S::UNKNOWN);    // also the "missing key" default
    TEST_ASSERT_TRUE(parkBrakeDecodeStored(3) == S::UNKNOWN);    // applying when power was lost
    TEST_ASSERT_TRUE(parkBrakeDecodeStored(4) == S::UNKNOWN);    // releasing when power was lost
    TEST_ASSERT_TRUE(parkBrakeDecodeStored(5) == S::UNKNOWN);
    TEST_ASSERT_TRUE(parkBrakeDecodeStored(255) == S::UNKNOWN);
    TEST_ASSERT_TRUE(parkBrakeDecodeStored(-1) == S::UNKNOWN);
    TEST_ASSERT_EQUAL_STRING("unknown", parkBrakeStateToken(S::UNKNOWN));
    TEST_ASSERT_EQUAL_STRING("released", parkBrakeStateToken(S::RELEASED));
    TEST_ASSERT_EQUAL_STRING("applied", parkBrakeStateToken(S::APPLIED));
    TEST_ASSERT_EQUAL_STRING("applying", parkBrakeStateToken(S::APPLYING));
    TEST_ASSERT_EQUAL_STRING("releasing", parkBrakeStateToken(S::RELEASING));
}

// ---- run length / travel validation ----
static void test_run_ms() {
    TEST_ASSERT_EQUAL_UINT32(2500, parkBrakeRunMs(2000));
    TEST_ASSERT_EQUAL_UINT32(3700, parkBrakeRunMs(3200));
    TEST_ASSERT_EQUAL_UINT32(1500, parkBrakeRunMs(1000));
    TEST_ASSERT_EQUAL_UINT32(5500, parkBrakeRunMs(5000));
}

static void test_travel_validation() {
    TEST_ASSERT_FALSE(parkBrakeTravelMsValid(999));
    TEST_ASSERT_FALSE(parkBrakeTravelMsValid(5001));
    TEST_ASSERT_FALSE(parkBrakeTravelMsValid(0));
    TEST_ASSERT_TRUE(parkBrakeTravelMsValid(1000));
    TEST_ASSERT_TRUE(parkBrakeTravelMsValid(2000));
    TEST_ASSERT_TRUE(parkBrakeTravelMsValid(5000));
    TEST_ASSERT_EQUAL_INT32(2000, parkBrakeTravelMsOrDefault(999));
    TEST_ASSERT_EQUAL_INT32(2000, parkBrakeTravelMsOrDefault(5001));
    TEST_ASSERT_EQUAL_INT32(2000, parkBrakeTravelMsOrDefault(65535));
    TEST_ASSERT_EQUAL_INT32(3200, parkBrakeTravelMsOrDefault(3200));
}

// ---- full D4 transition table ----
static void expect(S st, Q q, V spd, R r, A a) {
    ParkBrakeTransition t = parkBrakeTransition(st, q, spd, true);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)r, (uint8_t)t.result);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)a, (uint8_t)t.action);
}

static void test_transition_table() {
    // UNKNOWN
    expect(S::UNKNOWN, Q::APPLY, V::OK, R::STARTED, A::START);
    expect(S::UNKNOWN, Q::APPLY, V::REFUSED, R::REFUSED_MOVING, A::NONE);
    expect(S::UNKNOWN, Q::RELEASE, V::OK, R::STARTED, A::START);
    // RELEASED
    expect(S::RELEASED, Q::APPLY, V::OK, R::STARTED, A::START);
    expect(S::RELEASED, Q::APPLY, V::REFUSED, R::REFUSED_MOVING, A::NONE);
    expect(S::RELEASED, Q::RELEASE, V::OK, R::STARTED, A::START);           // re-assert
    // APPLIED
    expect(S::APPLIED, Q::APPLY, V::OK, R::STARTED, A::START);              // re-assert
    expect(S::APPLIED, Q::APPLY, V::REFUSED, R::REFUSED_MOVING, A::NONE);
    expect(S::APPLIED, Q::RELEASE, V::OK, R::STARTED, A::START);
    // APPLYING
    expect(S::APPLYING, Q::APPLY, V::OK, R::ALREADY_RUNNING, A::NONE);
    expect(S::APPLYING, Q::APPLY, V::REFUSED, R::ALREADY_RUNNING, A::NONE); // ignored
    expect(S::APPLYING, Q::RELEASE, V::OK, R::REVERSING, A::REVERSE);
    // RELEASING
    expect(S::RELEASING, Q::APPLY, V::OK, R::REVERSING, A::REVERSE);
    expect(S::RELEASING, Q::APPLY, V::REFUSED, R::REFUSED_MOVING, A::NONE); // release continues
    expect(S::RELEASING, Q::RELEASE, V::OK, R::ALREADY_RUNNING, A::NONE);
    // Never pulsed since boot: APPLY allowed, flagged
    expect(S::RELEASED, Q::APPLY, V::UNKNOWN_SPEED, R::STARTED_SPEED_UNKNOWN, A::START);
    expect(S::UNKNOWN, Q::APPLY, V::UNKNOWN_SPEED, R::STARTED_SPEED_UNKNOWN, A::START);
    expect(S::RELEASING, Q::APPLY, V::UNKNOWN_SPEED, R::REVERSING, A::REVERSE);
    // Release is accepted whatever the speed argument says
    expect(S::APPLIED, Q::RELEASE, V::REFUSED, R::STARTED, A::START);
}

static void test_transition_driver_down() {
    const S states[] = {S::UNKNOWN, S::RELEASED, S::APPLIED, S::APPLYING, S::RELEASING};
    for (S st : states) {
        ParkBrakeTransition a = parkBrakeTransition(st, Q::APPLY, V::OK, false);
        ParkBrakeTransition r = parkBrakeTransition(st, Q::RELEASE, V::OK, false);
        TEST_ASSERT_TRUE(a.result == R::REFUSED_DRIVER && a.action == A::NONE);
        TEST_ASSERT_TRUE(r.result == R::REFUSED_DRIVER && r.action == A::NONE);
    }
}

// ---- apply speed check ----
static void test_apply_speed() {
    TEST_ASSERT_TRUE(parkBrakeApplySpeedOk(0.0f, true, false) == V::OK);
    TEST_ASSERT_TRUE(parkBrakeApplySpeedOk(0.83f, true, false) == V::OK);
    TEST_ASSERT_TRUE(parkBrakeApplySpeedOk(0.833f, true, false) == V::OK);   // at the limit
    TEST_ASSERT_TRUE(parkBrakeApplySpeedOk(0.84f, true, false) == V::REFUSED);
    TEST_ASSERT_TRUE(parkBrakeApplySpeedOk(0.0f, false, false) == V::UNKNOWN_SPEED);   // never pulsed
    TEST_ASSERT_TRUE(parkBrakeApplySpeedOk(5.0f, false, false) == V::UNKNOWN_SPEED);
    TEST_ASSERT_TRUE(parkBrakeApplySpeedOk(2.0f, false, true) == V::REFUSED);   // suspicious upper bound
    TEST_ASSERT_TRUE(parkBrakeApplySpeedOk(0.5f, false, true) == V::OK);
}

// ---- throttle release point ----
static void test_thr_pass() {
    TEST_ASSERT_EQUAL_UINT32(1000, parkBrakeThrPassMs(2000, 50));
    TEST_ASSERT_EQUAL_UINT32(1400, parkBrakeThrPassMs(2000, 70));
    TEST_ASSERT_EQUAL_UINT32(400, parkBrakeThrPassMs(2000, 20));
    TEST_ASSERT_EQUAL_UINT32(2500, parkBrakeThrPassMs(2000, 100));   // whole run, margin included
    TEST_ASSERT_EQUAL_UINT32(5500, parkBrakeThrPassMs(5000, 100));
    TEST_ASSERT_FALSE(parkBrakeThrPassPctValid(19));
    TEST_ASSERT_FALSE(parkBrakeThrPassPctValid(101));
    TEST_ASSERT_TRUE(parkBrakeThrPassPctValid(20));
    TEST_ASSERT_TRUE(parkBrakeThrPassPctValid(100));
    TEST_ASSERT_EQUAL_INT32(50, parkBrakeThrPassPctOrDefault(19));
    TEST_ASSERT_EQUAL_INT32(50, parkBrakeThrPassPctOrDefault(255));
    TEST_ASSERT_EQUAL_INT32(70, parkBrakeThrPassPctOrDefault(70));
}

// ---- hold predicate: every state × N/L/H/R/UNKNOWN ----
static void test_hold_matrix() {
    const G gears[] = {G::Neutral, G::Low, G::High, G::Reverse, G::Unknown};
    for (G g : gears) {
        const bool engaged = (g == G::Low || g == G::High || g == G::Reverse);
        TEST_ASSERT_EQUAL(engaged, parkBrakeHoldsThrottle(S::APPLIED, g, 0, 1000));
        TEST_ASSERT_EQUAL(engaged, parkBrakeHoldsThrottle(S::APPLYING, g, 0, 1000));
        TEST_ASSERT_EQUAL(engaged, parkBrakeHoldsThrottle(S::UNKNOWN, g, 0, 1000));
        TEST_ASSERT_EQUAL(engaged, parkBrakeHoldsThrottle(S::RELEASING, g, 0, 1000));      // before the point
        TEST_ASSERT_FALSE(parkBrakeHoldsThrottle(S::RELEASING, g, 1000, 1000));            // past the point
        TEST_ASSERT_FALSE(parkBrakeHoldsThrottle(S::RELEASED, g, 0, 1000));
    }
}

static void test_hold_release_point() {
    const uint32_t p50 = parkBrakeThrPassMs(2000, 50);
    TEST_ASSERT_TRUE(parkBrakeHoldsThrottle(S::RELEASING, G::Low, 999, p50));
    TEST_ASSERT_FALSE(parkBrakeHoldsThrottle(S::RELEASING, G::Low, 1000, p50));
    const uint32_t p100 = parkBrakeThrPassMs(2000, 100);
    TEST_ASSERT_TRUE(parkBrakeHoldsThrottle(S::RELEASING, G::High, 2000, p100));
    TEST_ASSERT_TRUE(parkBrakeHoldsThrottle(S::RELEASING, G::High, 2499, p100));   // held until the run ends
    TEST_ASSERT_FALSE(parkBrakeHoldsThrottle(S::RELEASED, G::High, 0, p100));       // run ended → RELEASED
}

// ---- ignition and the drive-away latch ----
static ParkBrakeLatchStep step(bool& armed, S st, bool ign, G g, bool busy, float d) {
    ParkBrakeLatchStep r = parkBrakeLatchStep(armed, st, ign, g, busy, d);
    armed = r.armed;
    return r;
}

static void test_ignition_off_never_fires() {
    bool armed = true;
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, false, G::Low, false, 0.0f).fire);
    TEST_ASSERT_FALSE(armed);
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, false, G::Low, false, 30.0f).fire);
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::APPLIED, G::Low, false, 30.0f, armed) == ParkBrakeDaWait::IGN_OFF);
    TEST_ASSERT_EQUAL_STRING("ign_off", parkBrakeDaWaitToken(ParkBrakeDaWait::IGN_OFF));
}

static void test_ignition_on_with_throttle_held() {
    bool armed = false;
    // Ignition OFF, demand held at 30 %
    step(armed, S::APPLIED, false, G::Low, false, 30.0f);
    // Ignition ON (ACC / IGNITION / CRANKING all count) with the demand still at 30 %: no fire
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Low, false, 30.0f).fire);
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::APPLIED, G::Low, true, 30.0f, armed) == ParkBrakeDaWait::REARM);
    TEST_ASSERT_EQUAL_STRING("rearm", parkBrakeDaWaitToken(ParkBrakeDaWait::REARM));
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Low, false, 30.0f).fire);
    // Dip to 10 % (≤ threshold) arms, then the first sample above fires — no debounce
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Low, false, 10.0f).fire);
    TEST_ASSERT_TRUE(armed);
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::APPLIED, G::Low, true, 10.0f, armed) == ParkBrakeDaWait::NONE);
    TEST_ASSERT_TRUE(step(armed, S::APPLIED, true, G::Low, false, 10.5f).fire);
    TEST_ASSERT_FALSE(armed);
}

static void test_latch_fires_on_rising_edge() {
    bool armed = false;
    step(armed, S::APPLIED, true, G::Low, false, 0.0f);
    TEST_ASSERT_TRUE(step(armed, S::APPLIED, true, G::Low, false, 30.0f).fire);
    // UNKNOWN behaves as possibly applied, in R too
    armed = false;
    step(armed, S::UNKNOWN, true, G::Reverse, false, 0.0f);
    TEST_ASSERT_TRUE(step(armed, S::UNKNOWN, true, G::Reverse, false, 30.0f).fire);
}

static void test_latch_no_fire_cases() {
    bool armed = false;
    // 8 % in L: armed, never fires
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Low, false, 8.0f).fire);
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Low, false, 8.0f).fire);
    TEST_ASSERT_TRUE(armed);
    // Not in N (and the N clip keeps the demand ≤ 8.5 % anyway)
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Neutral, false, 30.0f).fire);
    // Not while the gear is unknown
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Unknown, false, 30.0f).fire);
    // Not during a gear change / shift lock
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Low, true, 30.0f).fire);
    TEST_ASSERT_TRUE(armed);
    // Not while released or running
    TEST_ASSERT_FALSE(step(armed, S::RELEASED, true, G::Low, false, 30.0f).fire);
    TEST_ASSERT_FALSE(step(armed, S::RELEASING, true, G::Low, false, 30.0f).fire);
    TEST_ASSERT_FALSE(step(armed, S::APPLYING, true, G::Low, false, 30.0f).fire);
}

static void test_latch_shift_from_neutral_with_stick_up() {
    // N clips the demand to ≤ 8.5 %: the latch arms; after the shift into L the 30 % fires.
    bool armed = false;
    step(armed, S::APPLIED, true, G::Neutral, false, 8.5f);
    TEST_ASSERT_TRUE(armed);
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Low, true, 8.5f).fire);   // shift running, clipped
    TEST_ASSERT_TRUE(step(armed, S::APPLIED, true, G::Low, false, 30.0f).fire);
}

static void test_latch_applied_with_throttle_up() {
    // Released (latch disarmed by the release), apply while holding 30 % in L.
    bool armed = false;
    TEST_ASSERT_FALSE(step(armed, S::RELEASED, true, G::Low, false, 30.0f).fire);
    TEST_ASSERT_FALSE(step(armed, S::APPLYING, true, G::Low, false, 30.0f).fire);
    // APPLIED settles with the demand still above 10 %: no ping-pong release
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Low, false, 30.0f).fire);
    TEST_ASSERT_FALSE(armed);
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::APPLIED, G::Low, true, 30.0f, armed) == ParkBrakeDaWait::REARM);
    // Dip to 5, then press again
    TEST_ASSERT_FALSE(step(armed, S::APPLIED, true, G::Low, false, 5.0f).fire);
    TEST_ASSERT_TRUE(step(armed, S::APPLIED, true, G::Low, false, 30.0f).fire);
}

static void test_da_wait_none_cases() {
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::RELEASED, G::Low, false, 30.0f, false) == ParkBrakeDaWait::NONE);
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::RELEASING, G::Low, true, 30.0f, false) == ParkBrakeDaWait::NONE);
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::APPLIED, G::Neutral, false, 0.0f, false) == ParkBrakeDaWait::NONE);
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::APPLIED, G::Low, true, 30.0f, true) == ParkBrakeDaWait::NONE);
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::APPLIED, G::Low, true, 0.0f, false) == ParkBrakeDaWait::NONE);
    TEST_ASSERT_TRUE(parkBrakeDaWait(S::UNKNOWN, G::High, false, 0.0f, false) == ParkBrakeDaWait::IGN_OFF);
    TEST_ASSERT_EQUAL_STRING("none", parkBrakeDaWaitToken(ParkBrakeDaWait::NONE));
}

// ---- direction mapping ----
static void test_direction_mapping() {
    TEST_ASSERT_EQUAL_INT8(1, parkBrakeDriveSign(Q::APPLY, false));     // RPWM, GPIO5
    TEST_ASSERT_EQUAL_INT8(-1, parkBrakeDriveSign(Q::RELEASE, false));  // LPWM, GPIO4
    TEST_ASSERT_EQUAL_INT8(-1, parkBrakeDriveSign(Q::APPLY, true));
    TEST_ASSERT_EQUAL_INT8(1, parkBrakeDriveSign(Q::RELEASE, true));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_decode_stored);
    RUN_TEST(test_run_ms);
    RUN_TEST(test_travel_validation);
    RUN_TEST(test_transition_table);
    RUN_TEST(test_transition_driver_down);
    RUN_TEST(test_apply_speed);
    RUN_TEST(test_thr_pass);
    RUN_TEST(test_hold_matrix);
    RUN_TEST(test_hold_release_point);
    RUN_TEST(test_ignition_off_never_fires);
    RUN_TEST(test_ignition_on_with_throttle_held);
    RUN_TEST(test_latch_fires_on_rising_edge);
    RUN_TEST(test_latch_no_fire_cases);
    RUN_TEST(test_latch_shift_from_neutral_with_stick_up);
    RUN_TEST(test_latch_applied_with_throttle_up);
    RUN_TEST(test_da_wait_none_cases);
    RUN_TEST(test_direction_mapping);
    return UNITY_END();
}
