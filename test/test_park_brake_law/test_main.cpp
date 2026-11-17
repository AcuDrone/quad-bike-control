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

// ---- release by a service-brake press (add-park-brake-release-by-service-brake) ----
typedef ParkBrakeSvcSource SS;

struct SvcSim {
    ParkBrakeSvcRelease st = parkBrakeSvcInitial();
    int fires = 0;
    // One loop; returns true when this step fires. Every step is checked for "fire ⇒ RELEASE only":
    // the step has no apply output at all (2.9) — the type carries only `fire`.
    bool step(SS src, float target, float est, S state) {
        ParkBrakeSvcStep r = parkBrakeSvcStep(st, src, target, est, state);
        st = r.next;
        if (r.fire) {
            fires++;
            TEST_ASSERT_FALSE(st.pending);   // firing always clears the wait
        }
        return r.fire;
    }
};

// Seed the sim at source `src`, target 0, state `state` (two loops: the source change, then steady).
static SvcSim svcSeeded(SS src, S state, float target = 0.0f, float est = 0.0f) {
    SvcSim m;
    m.step(src, target, est, state);
    m.step(src, target, est, state);
    TEST_ASSERT_EQUAL_INT(0, m.fires);
    return m;
}

// 2.1 / spec "RC press releases after the hand-over"
static void test_svc_rising_edge_applied() {
    SvcSim m = svcSeeded(SS::MAVLINK, S::APPLIED);
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 0.0f, S::APPLIED));
    TEST_ASSERT_TRUE(m.st.pending);
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 20.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 48.9f, S::APPLIED));
    TEST_ASSERT_TRUE(m.step(SS::MAVLINK, 50.0f, 49.0f, S::APPLIED));
    // Held: no second fire (the run is RELEASING, then RELEASED; the state would also block)
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 50.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 50.0f, S::RELEASING));
    TEST_ASSERT_EQUAL_INT(1, m.fires);
}

// 2.2
static void test_svc_from_unknown() {
    SvcSim m = svcSeeded(SS::MAVLINK, S::UNKNOWN);
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 0.0f, S::UNKNOWN));
    TEST_ASSERT_TRUE(m.step(SS::MAVLINK, 50.0f, 49.5f, S::UNKNOWN));
    TEST_ASSERT_EQUAL_INT(1, m.fires);
}

// 2.3 / spec "Brake held when the parking brake is applied"
static void test_svc_held_at_apply_time() {
    SvcSim m = svcSeeded(SS::WEB, S::RELEASED);
    m.step(SS::WEB, 60.0f, 10.0f, S::RELEASED);       // the press while RELEASED: nothing
    m.step(SS::WEB, 60.0f, 60.0f, S::RELEASED);
    m.step(SS::WEB, 60.0f, 60.0f, S::APPLYING);
    m.step(SS::WEB, 60.0f, 60.0f, S::APPLYING);
    for (int i = 0; i < 20; i++) m.step(SS::WEB, 60.0f, 60.0f, S::APPLIED);
    TEST_ASSERT_EQUAL_INT(0, m.fires);
    TEST_ASSERT_FALSE(m.st.pending);
    // Let go, press again
    m.step(SS::WEB, 0.0f, 30.0f, S::APPLIED);
    m.step(SS::WEB, 0.0f, 0.0f, S::APPLIED);
    TEST_ASSERT_FALSE(m.step(SS::WEB, 60.0f, 0.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::WEB, 60.0f, 40.0f, S::APPLIED));
    TEST_ASSERT_TRUE(m.step(SS::WEB, 60.0f, 59.0f, S::APPLIED));
    TEST_ASSERT_EQUAL_INT(1, m.fires);
}

// 2.4 / spec "Frozen target under the firmware fail-safe"
static void test_svc_failsafe_frozen_target() {
    SvcSim m = svcSeeded(SS::MAVLINK, S::APPLIED);
    // FAILSAFE begins; the target freezes / is non-zero — never a press under NONE
    m.step(SS::NONE, 25.0f, 25.0f, S::APPLIED);
    m.step(SS::NONE, 25.0f, 25.0f, S::APPLIED);
    // Even a 0 → 25 change seen under NONE is not a press
    m.step(SS::NONE, 0.0f, 0.0f, S::APPLIED);
    m.step(SS::NONE, 25.0f, 25.0f, S::APPLIED);
    TEST_ASSERT_FALSE(m.st.pending);
    // MAVLink returns with the same 25 %
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 25.0f, 25.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 25.0f, 25.0f, S::APPLIED));
    // 25 → 40: non-zero → non-zero, no press
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 40.0f, 30.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 40.0f, 40.0f, S::APPLIED));
    TEST_ASSERT_EQUAL_INT(0, m.fires);
    // A pending wait is cancelled by FAILSAFE
    m.step(SS::MAVLINK, 0.0f, 0.0f, S::APPLIED);
    m.step(SS::MAVLINK, 50.0f, 0.0f, S::APPLIED);
    TEST_ASSERT_TRUE(m.st.pending);
    TEST_ASSERT_FALSE(m.step(SS::NONE, 50.0f, 49.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.st.pending);
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 50.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 50.0f, S::APPLIED));
    TEST_ASSERT_EQUAL_INT(0, m.fires);
}

// 2.5 / spec "Takeover with an inherited target"
static void test_svc_takeover_inherited() {
    SvcSim m = svcSeeded(SS::MAVLINK, S::APPLIED, 80.0f, 80.0f);
    TEST_ASSERT_FALSE(m.step(SS::WEB, 80.0f, 80.0f, S::APPLIED));   // takeover
    TEST_ASSERT_FALSE(m.step(SS::WEB, 80.0f, 80.0f, S::APPLIED));   // web 80 unchanged
    TEST_ASSERT_FALSE(m.step(SS::WEB, 0.0f, 79.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::WEB, 80.0f, 10.0f, S::APPLIED));   // web 0 → 80
    TEST_ASSERT_TRUE(m.step(SS::WEB, 80.0f, 79.0f, S::APPLIED));
    TEST_ASSERT_EQUAL_INT(1, m.fires);

    // A pending wait across MAVLINK → WEB is cancelled; the source change itself is no edge
    SvcSim n = svcSeeded(SS::MAVLINK, S::APPLIED);
    n.step(SS::MAVLINK, 50.0f, 0.0f, S::APPLIED);
    TEST_ASSERT_TRUE(n.st.pending);
    TEST_ASSERT_FALSE(n.step(SS::WEB, 50.0f, 49.0f, S::APPLIED));
    TEST_ASSERT_FALSE(n.st.pending);
    TEST_ASSERT_FALSE(n.step(SS::WEB, 50.0f, 50.0f, S::APPLIED));
    // A source change in the same loop as a 0 → x change is not a press either
    SvcSim k = svcSeeded(SS::WEB, S::APPLIED);
    TEST_ASSERT_FALSE(k.step(SS::MAVLINK, 50.0f, 0.0f, S::APPLIED));
    TEST_ASSERT_FALSE(k.step(SS::MAVLINK, 50.0f, 50.0f, S::APPLIED));
    TEST_ASSERT_EQUAL_INT(0, n.fires + k.fires);
}

// 2.6 / 2.7 / spec "Press dropped before the hand-over"
static void test_svc_dropped_then_second_press() {
    SvcSim m = svcSeeded(SS::MAVLINK, S::APPLIED);
    m.step(SS::MAVLINK, 60.0f, 0.0f, S::APPLIED);
    m.step(SS::MAVLINK, 60.0f, 30.0f, S::APPLIED);
    TEST_ASSERT_TRUE(m.st.pending);
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 0.0f, 35.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.st.pending);
    // The estimate decays past the old target region: nothing
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 0.0f, 10.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 0.0f, 0.0f, S::APPLIED));
    TEST_ASSERT_EQUAL_INT(0, m.fires);
    // 2.7 second press
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 60.0f, 0.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 60.0f, 58.0f, S::APPLIED));
    TEST_ASSERT_TRUE(m.step(SS::MAVLINK, 60.0f, 59.0f, S::APPLIED));
    TEST_ASSERT_EQUAL_INT(1, m.fires);
}

// 2.8 / spec "No apply and no effect when not applied"
static void test_svc_not_applied_states() {
    const S states[] = {S::APPLYING, S::RELEASING, S::RELEASED};
    for (S s : states) {
        SvcSim m = svcSeeded(SS::MAVLINK, s);
        TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 0.0f, s));
        TEST_ASSERT_FALSE(m.st.pending);
        TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 50.0f, s));
        // The press happened in this state: becoming APPLIED with the brake held is no edge
        TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 50.0f, 50.0f, S::APPLIED));
        TEST_ASSERT_EQUAL_INT(0, m.fires);
    }
    // A pending wait is cancelled when the state leaves APPLIED/UNKNOWN (e.g. a web Release / Apply)
    const S leave[] = {S::RELEASING, S::APPLYING, S::RELEASED};
    for (S s : leave) {
        SvcSim m = svcSeeded(SS::WEB, S::APPLIED);
        m.step(SS::WEB, 50.0f, 0.0f, S::APPLIED);
        TEST_ASSERT_TRUE(m.st.pending);
        TEST_ASSERT_FALSE(m.step(SS::WEB, 50.0f, 20.0f, s));
        TEST_ASSERT_FALSE(m.st.pending);
        TEST_ASSERT_FALSE(m.step(SS::WEB, 50.0f, 50.0f, S::APPLIED));
        TEST_ASSERT_EQUAL_INT(0, m.fires);
    }
}

// 2.9: the step has no apply output — a press can only fire a RELEASE, and never from a state that
// is not APPLIED/UNKNOWN. Exhaustive sweep over sources, states and target/estimate pairs.
static void test_svc_never_applies() {
    const SS srcs[] = {SS::NONE, SS::MAVLINK, SS::WEB};
    const S sts[] = {S::UNKNOWN, S::RELEASED, S::APPLIED, S::APPLYING, S::RELEASING};
    const float vals[] = {0.0f, 5.0f, 25.0f, 50.0f, 100.0f};
    for (SS ps : srcs) for (SS src : srcs) for (S st : sts)
    for (float prev : vals) for (float tgt : vals) for (float est : vals)
    for (int pend = 0; pend < 2; pend++) {
        ParkBrakeSvcRelease s{prev, pend != 0, ps};
        ParkBrakeSvcStep r = parkBrakeSvcStep(s, src, tgt, est, st);
        TEST_ASSERT_EQUAL_FLOAT(tgt, r.next.prevTargetPct);            // always sampled
        TEST_ASSERT_TRUE(r.next.prevSource == src);
        if (r.fire) {
            TEST_ASSERT_TRUE(parkBrakeMaybeApplied(st));
            TEST_ASSERT_TRUE(src == ps && src != SS::NONE);
            TEST_ASSERT_TRUE(tgt > 0.0f);
            TEST_ASSERT_FALSE(r.next.pending);
        }
        if (src != ps || src == SS::NONE || !parkBrakeMaybeApplied(st)) {
            TEST_ASSERT_FALSE(r.next.pending);
            TEST_ASSERT_FALSE(r.fire);
        }
    }
}

// 2.10 / spec "Automatic fail-safe brake releases (accepted)": the Copilot's constant 25 % brake
// (channel 2 at 1400 µs) after an RC loss is a new press — documents the accepted behaviour.
static void test_svc_copilot_pattern_releases() {
    SvcSim m = svcSeeded(SS::MAVLINK, S::APPLIED);
    for (int i = 0; i < 50; i++) m.step(SS::MAVLINK, 0.0f, 0.0f, S::APPLIED);   // autopilot zeroes ch2
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 25.0f, 0.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 25.0f, 12.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 25.0f, 23.9f, S::APPLIED));
    TEST_ASSERT_TRUE(m.step(SS::MAVLINK, 25.0f, 24.0f, S::APPLIED));
    for (int i = 0; i < 50; i++) m.step(SS::MAVLINK, 25.0f, 25.0f, S::RELEASING);
    for (int i = 0; i < 50; i++) m.step(SS::MAVLINK, 25.0f, 25.0f, S::RELEASED);
    m.step(SS::MAVLINK, 0.0f, 25.0f, S::RELEASED);   // brake_hold_s expires
    TEST_ASSERT_EQUAL_INT(1, m.fires);
}

// 2.11: a pending target change 30 → 80 waits for the new target
static void test_svc_pending_target_change() {
    SvcSim m = svcSeeded(SS::MAVLINK, S::APPLIED);
    m.step(SS::MAVLINK, 30.0f, 0.0f, S::APPLIED);
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 80.0f, 25.0f, S::APPLIED));
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 80.0f, 30.0f, S::APPLIED));   // the old target reached
    TEST_ASSERT_FALSE(m.step(SS::MAVLINK, 80.0f, 78.9f, S::APPLIED));
    TEST_ASSERT_TRUE(m.st.pending);
    TEST_ASSERT_TRUE(m.step(SS::MAVLINK, 80.0f, 79.0f, S::APPLIED));
    TEST_ASSERT_EQUAL_INT(1, m.fires);
}

// Boot edge (design "Risks / open-loop estimate"): the estimate starts at 100 % and runs down to 0
// over travel_ms until the released sensor syncs it. A press while the estimate is still at or above
// target − 1 % hands over at once (accepted; bench task 8.3). The law follows the estimate as is.
static void test_svc_boot_estimate_hands_over_at_once() {
    SvcSim m;
    m.step(SS::NONE, 0.0f, 100.0f, S::APPLIED);       // boot: FAILSAFE, estimate 100 %
    m.step(SS::MAVLINK, 0.0f, 90.0f, S::APPLIED);     // link up: re-seed
    m.step(SS::MAVLINK, 0.0f, 80.0f, S::APPLIED);
    TEST_ASSERT_TRUE(m.step(SS::MAVLINK, 50.0f, 70.0f, S::APPLIED));   // press at estimate 70 %
    TEST_ASSERT_EQUAL_INT(1, m.fires);
}

// Initial state
static void test_svc_initial() {
    ParkBrakeSvcRelease s = parkBrakeSvcInitial();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.prevTargetPct);
    TEST_ASSERT_FALSE(s.pending);
    TEST_ASSERT_TRUE(s.prevSource == SS::NONE);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, PB_LAW_SVC_TOL_PCT);
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
    RUN_TEST(test_svc_initial);
    RUN_TEST(test_svc_rising_edge_applied);
    RUN_TEST(test_svc_from_unknown);
    RUN_TEST(test_svc_held_at_apply_time);
    RUN_TEST(test_svc_failsafe_frozen_target);
    RUN_TEST(test_svc_takeover_inherited);
    RUN_TEST(test_svc_dropped_then_second_press);
    RUN_TEST(test_svc_not_applied_states);
    RUN_TEST(test_svc_never_applies);
    RUN_TEST(test_svc_copilot_pattern_releases);
    RUN_TEST(test_svc_pending_target_change);
    RUN_TEST(test_svc_boot_estimate_hands_over_at_once);
    return UNITY_END();
}
