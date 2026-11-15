// Native unit tests for the pre-crank dwell / armed-crank decision / validated crank RPM
// law (include/CrankLaw.h, change add-precrank-dwell-setting).
// Run: pio test -e native
#include <unity.h>
#include "CrankLaw.h"

void setUp() {}
void tearDown() {}

static const uint16_t THRESHOLD_QUAD  = 1500;   // ENGINE_RUNNING_RPM_THRESHOLD on master
static const uint16_t THRESHOLD_AZTEC = 800;    // ENGINE_RUNNING_RPM_THRESHOLD on aztec

static void test_dwell_valid_bounds() {
    TEST_ASSERT_FALSE(crankDwellMsValid(999));
    TEST_ASSERT_FALSE(crankDwellMsValid(8001));
    TEST_ASSERT_FALSE(crankDwellMsValid(0));
    TEST_ASSERT_TRUE(crankDwellMsValid(1000));
    TEST_ASSERT_TRUE(crankDwellMsValid(3000));
    TEST_ASSERT_TRUE(crankDwellMsValid(8000));
}

static void test_dwell_or_default() {
    TEST_ASSERT_EQUAL_UINT16(3000, crankDwellMsOrDefault(0));
    TEST_ASSERT_EQUAL_UINT16(3000, crankDwellMsOrDefault(999));
    TEST_ASSERT_EQUAL_UINT16(3000, crankDwellMsOrDefault(8001));
    TEST_ASSERT_EQUAL_UINT16(3000, crankDwellMsOrDefault(65535));
    TEST_ASSERT_EQUAL_UINT16(1500, crankDwellMsOrDefault(1500));
}

static void test_dwell_elapsed_sentinel() {
    // ignitionHighSince 0 = ignition line OFF → never elapsed, whatever the time.
    TEST_ASSERT_FALSE(crankDwellElapsed(0, 0, 3000));
    TEST_ASSERT_FALSE(crankDwellElapsed(0, 10000, 3000));
    TEST_ASSERT_FALSE(crankDwellElapsed(0, 0xFFFFFFFFu, 1000));
}

static void test_dwell_elapsed_boundary() {
    TEST_ASSERT_FALSE(crankDwellElapsed(1000, 1000 + 2999, 3000));
    TEST_ASSERT_TRUE(crankDwellElapsed(1000, 1000 + 3000, 3000));
}

static void test_dwell_elapsed_wraparound() {
    // since = 0xFFFFF000, now = 0x00000400: 0x1000 + 0x400 = 5120 ms later across the wrap.
    TEST_ASSERT_TRUE(crankDwellElapsed(0xFFFFF000u, 0x00000400u, 3000));
    TEST_ASSERT_FALSE(crankDwellElapsed(0xFFFFF000u, 0x00000400u, 6000));
}

static void test_armed_wait_engage() {
    const uint32_t since = 10000;
    TEST_ASSERT_EQUAL(CrankArmedAction::WAIT,
                      crankArmedDecide(0, THRESHOLD_QUAD, since, since + 2999, 3000));
    TEST_ASSERT_EQUAL(CrankArmedAction::ENGAGE,
                      crankArmedDecide(0, THRESHOLD_QUAD, since, since + 3000, 3000));
    // At 2 s: a 1 s dwell has elapsed, an 8 s dwell has not.
    TEST_ASSERT_EQUAL(CrankArmedAction::ENGAGE,
                      crankArmedDecide(0, THRESHOLD_QUAD, since, since + 2000, 1000));
    TEST_ASSERT_EQUAL(CrankArmedAction::WAIT,
                      crankArmedDecide(0, THRESHOLD_QUAD, since, since + 2000, 8000));
    // Line not up yet (sentinel) → WAIT.
    TEST_ASSERT_EQUAL(CrankArmedAction::WAIT,
                      crankArmedDecide(0, THRESHOLD_QUAD, 0, since + 20000, 1000));
}

static void test_armed_suppress_first() {
    const uint32_t since = 10000;
    TEST_ASSERT_EQUAL(CrankArmedAction::SUPPRESS,
                      crankArmedDecide(1500, THRESHOLD_QUAD, since, since + 100, 3000));
    TEST_ASSERT_EQUAL(CrankArmedAction::SUPPRESS,
                      crankArmedDecide(1500, THRESHOLD_QUAD, since, since + 5000, 3000));
    TEST_ASSERT_NOT_EQUAL(CrankArmedAction::SUPPRESS,
                          crankArmedDecide(1499, THRESHOLD_QUAD, since, since + 100, 3000));
    TEST_ASSERT_NOT_EQUAL(CrankArmedAction::SUPPRESS,
                          crankArmedDecide(1499, THRESHOLD_QUAD, since, since + 5000, 3000));
    TEST_ASSERT_EQUAL(CrankArmedAction::SUPPRESS,
                      crankArmedDecide(800, THRESHOLD_AZTEC, since, since + 100, 3000));
}

static void test_crank_rpm_validated() {
    TEST_ASSERT_EQUAL_UINT16(0, crankRpm(false, 1600));
    TEST_ASSERT_EQUAL_UINT16(1600, crankRpm(true, 1600));
    TEST_ASSERT_EQUAL_UINT16(0, crankRpm(true, 0));
}

static void test_stale_rpm_after_off() {
    // The bug: engine at 1600 RPM, ignition OFF, straight back to ON within the 5 s
    // CAN stale timeout — the stored value is still "valid".
    uint16_t storedRpm = 1600;
    const bool dataValid = true;
    // Before the fix the stale value suppressed the crank:
    TEST_ASSERT_EQUAL(CrankArmedAction::SUPPRESS,
                      crankArmedDecide(crankRpm(dataValid, storedRpm), THRESHOLD_QUAD, 20000, 20010, 3000));
    // Ignition goes OFF → VehicleController clears the stored RPM (dataValid untouched).
    if (ignitionWentOff(true, false)) storedRpm = 0;
    TEST_ASSERT_EQUAL_UINT16(0, crankRpm(dataValid, storedRpm));
    // Back to ON 1 s later: the line comes up at 21000, the crank waits, then engages.
    const uint32_t since = 21000;
    TEST_ASSERT_EQUAL(CrankArmedAction::WAIT,
                      crankArmedDecide(crankRpm(dataValid, storedRpm), THRESHOLD_QUAD, since, since + 1000, 3000));
    TEST_ASSERT_EQUAL(CrankArmedAction::ENGAGE,
                      crankArmedDecide(crankRpm(dataValid, storedRpm), THRESHOLD_QUAD, since, since + 3000, 3000));
}

static void test_ignition_went_off_edge() {
    TEST_ASSERT_TRUE(ignitionWentOff(true, false));
    TEST_ASSERT_FALSE(ignitionWentOff(false, false));
    TEST_ASSERT_FALSE(ignitionWentOff(false, true));
    TEST_ASSERT_FALSE(ignitionWentOff(true, true));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_dwell_valid_bounds);
    RUN_TEST(test_dwell_or_default);
    RUN_TEST(test_dwell_elapsed_sentinel);
    RUN_TEST(test_dwell_elapsed_boundary);
    RUN_TEST(test_dwell_elapsed_wraparound);
    RUN_TEST(test_armed_wait_engage);
    RUN_TEST(test_armed_suppress_first);
    RUN_TEST(test_crank_rpm_validated);
    RUN_TEST(test_stale_rpm_after_off);
    RUN_TEST(test_ignition_went_off_edge);
    return UNITY_END();
}
