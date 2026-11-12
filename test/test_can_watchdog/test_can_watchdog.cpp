// Native unit tests for the OBD-II no-data watchdog gating (include/CanWatchdogLaw.h).
// Run: pio test -e native
#include <unity.h>
#include "CanWatchdogLaw.h"

using CanWatchdogLaw::obdNoDataDue;

void setUp() {}
void tearDown() {}

static const uint32_t NO_DATA = 15000;
static const uint32_t MIN_INT = 15000;

// Ignition OFF: never due, however long the bus has been silent.
static void test_ignition_off_never_due() {
    uint32_t s = 123;
    TEST_ASSERT_FALSE(obdNoDataDue(false, 600000, 0, 1000, 0, NO_DATA, MIN_INT, &s));
    TEST_ASSERT_EQUAL_UINT32(0, s);
    TEST_ASSERT_FALSE(obdNoDataDue(false, 600000, 5000, 1000, 0, NO_DATA, MIN_INT, &s));
}

// Ignition on for a long time, ECU silent since boot: fires after 15 s, as before the gating.
static void test_on_since_boot_fires_at_15s() {
    // begin() at 1000, ignition on at 1000
    TEST_ASSERT_FALSE(obdNoDataDue(true, 15999, 0, 1000, 1000, NO_DATA, MIN_INT));
    uint32_t s = 0;
    TEST_ASSERT_TRUE(obdNoDataDue(true, 16000, 0, 1000, 1000, NO_DATA, MIN_INT, &s));
    TEST_ASSERT_EQUAL_UINT32(15000, s);
}

// Booted with the ignition OFF for 10 min, then ACC: no instant trip, fresh 15 s window.
static void test_fresh_window_after_switch_on() {
    const uint32_t begin = 1000, on = 600000;
    TEST_ASSERT_FALSE(obdNoDataDue(true, on, 0, begin, on, NO_DATA, MIN_INT));
    TEST_ASSERT_FALSE(obdNoDataDue(true, on + 14999, 0, begin, on, NO_DATA, MIN_INT));
    TEST_ASSERT_TRUE(obdNoDataDue(true, on + 15000, 0, begin, on, NO_DATA, MIN_INT));
}

// The ECU answered, ignition went OFF for a while and back ON: the stale reply age is ignored.
static void test_stale_reply_ignored_after_switch_on() {
    const uint32_t reply = 50000, on = 200000;
    TEST_ASSERT_FALSE(obdNoDataDue(true, on + 1, reply, 1000, on, NO_DATA, MIN_INT));
    TEST_ASSERT_TRUE(obdNoDataDue(true, on + 15000, reply, 1000, on, NO_DATA, MIN_INT));
}

// With the ignition on, a reply resets the window (reply later than ignition-on).
static void test_reply_restarts_window() {
    const uint32_t on = 10000, reply = 40000;
    TEST_ASSERT_FALSE(obdNoDataDue(true, reply + 14999, reply, 1000, on, NO_DATA, MIN_INT));
    TEST_ASSERT_TRUE(obdNoDataDue(true, reply + 15000, reply, 1000, on, NO_DATA, MIN_INT));
}

// The minimum spacing from the last re-init of any kind (e.g. an LWS watchdog re-init) holds.
static void test_min_interval_from_any_reinit() {
    const uint32_t on = 1000, lwsReinit = 20000;
    // silent 20 s since on, but an LWS re-init 5 s ago (before any reply: window from it)
    TEST_ASSERT_FALSE(obdNoDataDue(true, 25000, 0, lwsReinit, on, NO_DATA, MIN_INT));
    TEST_ASSERT_TRUE(obdNoDataDue(true, lwsReinit + 15000, 0, lwsReinit, on, NO_DATA, MIN_INT));
    // after a reply: silence from the reply, spacing from the re-init
    TEST_ASSERT_FALSE(obdNoDataDue(true, 30000, 5000, 20000, on, NO_DATA, MIN_INT));
    TEST_ASSERT_TRUE(obdNoDataDue(true, 35000, 5000, 20000, on, NO_DATA, MIN_INT));
}

// millis() wrap: ignition on just before the wrap, checks just after it.
static void test_millis_wrap() {
    const uint32_t on = 0xFFFFF000u;          // 4096 ms before the wrap
    const uint32_t begin = 0xFFF00000u;
    TEST_ASSERT_FALSE(obdNoDataDue(true, 0x00001000u, 0, begin, on, NO_DATA, MIN_INT));   // 8 s
    TEST_ASSERT_TRUE(obdNoDataDue(true, on + 15000u, 0, begin, on, NO_DATA, MIN_INT));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_ignition_off_never_due);
    RUN_TEST(test_on_since_boot_fires_at_15s);
    RUN_TEST(test_fresh_window_after_switch_on);
    RUN_TEST(test_stale_reply_ignored_after_switch_on);
    RUN_TEST(test_reply_restarts_window);
    RUN_TEST(test_min_interval_from_any_reinit);
    RUN_TEST(test_millis_wrap);
    return UNITY_END();
}
