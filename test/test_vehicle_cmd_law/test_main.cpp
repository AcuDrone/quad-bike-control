// Native unit tests for the GCS vehicle-command law (include/VehicleCommandLaw.h,
// change add-gcs-vehicle-commands). Run: pio test -e native
#include <unity.h>
#include <math.h>
#include "VehicleCommandLaw.h"

void setUp() {}
void tearDown() {}

static const uint16_t CAPS_QUAD = 0x1F;

static VehicleCmdDecodeKind kindOf(float p1, float p2, uint16_t caps = CAPS_QUAD) {
    return vehicleCmdDecode(p1, p2, caps).kind;
}

// ---- Decode / whitelist ---------------------------------------------------------------------

static void test_decode_functions_accepted() {
    for (int fn = 1; fn <= 4; fn++) {
        VehicleCmdDecoded d = vehicleCmdDecode((float)fn, 0.0f, CAPS_QUAD);
        TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::FUNCTION, (int)d.kind);
        TEST_ASSERT_EQUAL(fn, (int)d.fn);
        TEST_ASSERT_EQUAL(0, d.value);
    }
    VehicleCmdDecoded q = vehicleCmdDecode(0.0f, 2.0f, CAPS_QUAD);
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::QUERY, (int)q.kind);
    TEST_ASSERT_EQUAL(2, q.value);
    VehicleCmdDecoded hb = vehicleCmdDecode(5.0f, 1.0f, 0x3F);
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::FUNCTION, (int)hb.kind);
    TEST_ASSERT_EQUAL((int)VehicleCmdFunction::HANDBRAKE, (int)hb.fn);
    TEST_ASSERT_EQUAL(1, hb.value);
}

static void test_decode_unknown_function() {
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(6.0f, 0.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(-1.0f, 0.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(2.5f, 0.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(NAN, 0.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(INFINITY, 0.0f));
}

static void test_decode_capability_bits() {
    // Handbrake with bit5 clear (quad, or buggy without the X6 driver)
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(5.0f, 1.0f, 0x1F));
    // ...even with an invalid value: unsupported comes first
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(5.0f, 7.0f, 0x1F));
    // Gear H with bit1 clear
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(1.0f, 1.0f, 0x1D));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::FUNCTION, (int)kindOf(1.0f, 2.0f, 0x1D));
    // Lock with bit4 clear
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(4.0f, 1.0f, 0x0F));
    // No gears at all
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)kindOf(1.0f, 0.0f, 0x1C));
}

static void test_decode_values() {
    // Tolerance: 0.005 off is the integer, 0.5 off is not.
    VehicleCmdDecoded d = vehicleCmdDecode(3.0f, 1.005f, CAPS_QUAD);
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::FUNCTION, (int)d.kind);
    TEST_ASSERT_EQUAL(1, d.value);
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::FUNCTION, (int)kindOf(2.995f, 0.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(3.0f, 0.5f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(2.0f, 1.5f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(2.0f, NAN));
    // Out of the table
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(2.0f, 3.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(1.0f, -2.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(1.0f, 3.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(3.0f, 2.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(4.0f, -1.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(5.0f, 2.0f, 0x3F));
    // Every gear in the table
    for (int g = -1; g <= 2; g++) {
        TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::FUNCTION, (int)kindOf(1.0f, (float)g));
    }
    // Query: 0/1/2 only
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(0.0f, 3.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::DENIED, (int)kindOf(0.0f, -1.0f));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::QUERY, (int)kindOf(0.0f, 0.0f, 0));
}

// ---- ACK per interlock ----------------------------------------------------------------------

static VcmdGearInputs gearIn(bool mav, bool eng, bool init, bool at, bool speedOk) {
    VcmdGearInputs in;
    in.underMavlink = mav;
    in.engineRunning = eng;
    in.transInitialised = init;
    in.alreadyAtTarget = at;
    in.speedOk = speedOk;
    return in;
}

static void test_gear_interlocks() {
    bool set = true;
    VehicleCmdOutcome o = vcmdGearDecide(gearIn(false, true, true, false, true), set);   // fail-safe / web
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::TEMPORARILY_REJECTED, (int)o.result);
    TEST_ASSERT_EQUAL(VCMD_REASON_FAILSAFE, o.reason);
    TEST_ASSERT_FALSE(set);
    // Fail-safe first, before the engine check
    o = vcmdGearDecide(gearIn(false, false, false, false, false), set);
    TEST_ASSERT_EQUAL(VCMD_REASON_FAILSAFE, o.reason);
    o = vcmdGearDecide(gearIn(true, false, true, false, true), set);                     // engine off
    TEST_ASSERT_EQUAL(VCMD_REASON_ENGINE_OFF, o.reason);
    TEST_ASSERT_FALSE(set);
    o = vcmdGearDecide(gearIn(true, true, false, false, true), set);                     // not initialised
    TEST_ASSERT_EQUAL(VCMD_REASON_ENGINE_OFF, o.reason);
    o = vcmdGearDecide(gearIn(true, true, true, false, false), set);                     // speed (R/H/L)
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::TEMPORARILY_REJECTED, (int)o.result);
    TEST_ASSERT_EQUAL(VCMD_REASON_SPEED, o.reason);
    TEST_ASSERT_FALSE(set);
    // N at speed: the transmission interlock allows N (speedOk = true) → accepted (D-F)
    o = vcmdGearDecide(gearIn(true, true, true, false, true), set);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)o.result);
    TEST_ASSERT_TRUE(set);
    // Already at target: accepted, nothing moves (even at speed)
    o = vcmdGearDecide(gearIn(true, true, true, true, false), set);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)o.result);
    TEST_ASSERT_EQUAL(0, o.reason);
    TEST_ASSERT_FALSE(set);
}

static void test_ignition_interlocks() {
    VcmdIgnAction a = VcmdIgnAction::NONE;
    // OFF: always accepted (fail-safe / web / cranking), unless the relays are unavailable
    VehicleCmdOutcome o = vcmdIgnitionDecide(VCMD_IGN_OFF, false, true, true, a);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)o.result);
    TEST_ASSERT_EQUAL((int)VcmdIgnAction::OFF, (int)a);
    o = vcmdIgnitionDecide(VCMD_IGN_OFF, false, false, false, a);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::FAILED, (int)o.result);
    TEST_ASSERT_EQUAL((int)VcmdIgnAction::NONE, (int)a);
    // ACC / ON not under MAVLink control → reason 3
    o = vcmdIgnitionDecide(VCMD_IGN_ACC, false, true, false, a);
    TEST_ASSERT_EQUAL(VCMD_REASON_FAILSAFE, o.reason);
    TEST_ASSERT_EQUAL((int)VcmdIgnAction::NONE, (int)a);
    o = vcmdIgnitionDecide(VCMD_IGN_ON, false, true, false, a);
    TEST_ASSERT_EQUAL(VCMD_REASON_FAILSAFE, o.reason);
    // ACC
    o = vcmdIgnitionDecide(VCMD_IGN_ACC, true, true, false, a);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)o.result);
    TEST_ASSERT_EQUAL((int)VcmdIgnAction::ACC, (int)a);
    // ON with the engine running → no crank; stopped → crank
    o = vcmdIgnitionDecide(VCMD_IGN_ON, true, true, true, a);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)o.result);
    TEST_ASSERT_EQUAL((int)VcmdIgnAction::IGNITION_NO_CRANK, (int)a);
    o = vcmdIgnitionDecide(VCMD_IGN_ON, true, true, false, a);
    TEST_ASSERT_EQUAL((int)VcmdIgnAction::CRANK, (int)a);
    // Relays unavailable → FAILED
    o = vcmdIgnitionDecide(VCMD_IGN_ON, true, false, false, a);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::FAILED, (int)o.result);
}

static void test_light_and_lock() {
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)vcmdLightDecide(true).result);   // any source
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::FAILED, (int)vcmdLightDecide(false).result);
    VehicleCmdOutcome o = vcmdLockDecide(false, true);   // web / fail-safe
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::TEMPORARILY_REJECTED, (int)o.result);
    TEST_ASSERT_EQUAL(VCMD_REASON_FAILSAFE, o.reason);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)vcmdLockDecide(true, true).result);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::FAILED, (int)vcmdLockDecide(true, false).result);
}

// ---- RC decode and last change wins ---------------------------------------------------------

static void test_rc_decode() {
    TEST_ASSERT_EQUAL(VCMD_RC_ABSENT, vcmdRcGear(0));
    TEST_ASSERT_EQUAL(VCMD_RC_ABSENT, vcmdRcGear(879));
    TEST_ASSERT_EQUAL(VCMD_RC_ABSENT, vcmdRcGear(2161));
    TEST_ASSERT_EQUAL(VCMD_GEAR_L, vcmdRcGear(880));
    TEST_ASSERT_EQUAL(VCMD_GEAR_L, vcmdRcGear(1306));
    TEST_ASSERT_EQUAL(VCMD_GEAR_N, vcmdRcGear(1307));
    TEST_ASSERT_EQUAL(VCMD_GEAR_N, vcmdRcGear(1733));
    TEST_ASSERT_EQUAL(VCMD_GEAR_R, vcmdRcGear(1734));
    TEST_ASSERT_EQUAL(VCMD_GEAR_R, vcmdRcGear(2160));
    TEST_ASSERT_EQUAL(VCMD_IGN_OFF, vcmdRcIgnition(1200));
    TEST_ASSERT_EQUAL(VCMD_IGN_ACC, vcmdRcIgnition(1201));
    TEST_ASSERT_EQUAL(VCMD_IGN_ACC, vcmdRcIgnition(1520));
    TEST_ASSERT_EQUAL(VCMD_IGN_ON, vcmdRcIgnition(1521));
    TEST_ASSERT_EQUAL(VCMD_RC_ABSENT, vcmdRcIgnition(870));
    TEST_ASSERT_EQUAL(0, vcmdRcSwitch(1520));
    TEST_ASSERT_EQUAL(1, vcmdRcSwitch(1521));
    TEST_ASSERT_EQUAL(VCMD_RC_ABSENT, vcmdRcSwitch(2170));
}

static void test_command_holds_against_constant_rc() {
    RcEdge light;
    bool state = false;   // vehicle headlight
    if (light.step(vcmdRcSwitch(1100))) state = false;   // first frame: OFF applied
    state = true;                                        // GCS headlight ON accepted
    for (int i = 0; i < 50; i++) {
        if (light.step(vcmdRcSwitch(1100))) state = false;
    }
    TEST_ASSERT_TRUE(state);
}

static void test_rc_edge_wins() {
    RcEdge lock;
    bool state = false;
    TEST_ASSERT_TRUE(lock.step(vcmdRcSwitch(1100)));
    state = true;   // GCS lock ON
    if (lock.step(vcmdRcSwitch(1900))) state = true;
    TEST_ASSERT_TRUE(state);
    if (lock.step(vcmdRcSwitch(1100))) state = false;
    TEST_ASSERT_FALSE(state);
}

static void test_rc_absent_is_not_a_change() {
    RcEdge ign;
    TEST_ASSERT_TRUE(ign.step(vcmdRcIgnition(1000)));    // OFF
    // GCS ON accepted meanwhile; the channel then disappears and returns to the same value
    TEST_ASSERT_FALSE(ign.step(vcmdRcIgnition(0)));
    TEST_ASSERT_FALSE(ign.step(vcmdRcIgnition(870)));
    TEST_ASSERT_FALSE(ign.step(vcmdRcIgnition(2170)));
    TEST_ASSERT_FALSE(ign.step(vcmdRcIgnition(1000)));
    // Absent from the start: nothing applied until the first in-band value
    RcEdge fresh;
    TEST_ASSERT_FALSE(fresh.step(vcmdRcIgnition(0)));
    TEST_ASSERT_TRUE(fresh.step(vcmdRcIgnition(1600)));
}

// Boot, fail-safe exit and WEB → MAVLINK (web control released, or RC back with a browser
// connected) are the same re-sync: every RC value set before is applied again, overwriting a
// GCS / web command given meanwhile (owner decisions D-A, D-H).
static void test_resync_every_cause() {
    const char* causes[] = {"boot", "failsafe exit", "web released", "rc back with browser"};
    for (int c = 0; c < 4; c++) {
        VcmdRcState rc;
        if (c > 0) {
            // RC was in control before: values stored, nothing pending
            rc.stepGear(vcmdRcGear(1000));
            rc.gearDone();
            rc.ignition.step(vcmdRcIgnition(1700));
            rc.light.step(vcmdRcSwitch(1100));
            rc.lock.step(vcmdRcSwitch(1100));
            // Same values again → no change while in control
            TEST_ASSERT_FALSE_MESSAGE(rc.ignition.step(vcmdRcIgnition(1700)), causes[c]);
            // Not in control for a while (fail-safe or web portal): resync every such loop
            rc.resync();
            rc.resync();
        }
        // GCS / web set the light on and the ignition off meanwhile; RC is back, unchanged
        TEST_ASSERT_TRUE_MESSAGE(rc.stepGear(vcmdRcGear(1000)), causes[c]);
        TEST_ASSERT_TRUE_MESSAGE(rc.gearPending, causes[c]);
        TEST_ASSERT_EQUAL_MESSAGE(VCMD_GEAR_L, rc.pendingGear, causes[c]);
        TEST_ASSERT_TRUE_MESSAGE(rc.ignition.step(vcmdRcIgnition(1700)), causes[c]);   // → requestCrank()
        TEST_ASSERT_TRUE_MESSAGE(rc.light.step(vcmdRcSwitch(1100)), causes[c]);        // light off again
        TEST_ASSERT_TRUE_MESSAGE(rc.lock.step(vcmdRcSwitch(1100)), causes[c]);
        // ...and then edge-only again
        TEST_ASSERT_FALSE_MESSAGE(rc.light.step(vcmdRcSwitch(1100)), causes[c]);
    }
}

static void test_resync_with_absent_channel() {
    VcmdRcState rc;
    rc.light.step(vcmdRcSwitch(1900));
    rc.resync();
    TEST_ASSERT_FALSE(rc.light.step(vcmdRcSwitch(0)));   // absent after the re-sync: nothing applied
    TEST_ASSERT_TRUE(rc.light.step(vcmdRcSwitch(1900))); // first in-band value applies
}

static void test_pending_rc_gear() {
    VcmdRcState rc;
    rc.stepGear(vcmdRcGear(1500));   // N, first frame
    rc.gearDone();
    // RC moves to L with the engine stopped → pending
    TEST_ASSERT_TRUE(rc.stepGear(vcmdRcGear(1000)));
    TEST_ASSERT_TRUE(rc.gearPending);
    // A rejected GCS gear does not touch it (the caller only calls onGcsGearAccepted on accept)
    TEST_ASSERT_TRUE(rc.gearPending);
    // Constant RC: still pending, no new edge
    TEST_ASSERT_FALSE(rc.stepGear(vcmdRcGear(1000)));
    TEST_ASSERT_TRUE(rc.gearPending);
    // An accepted GCS gear cancels it
    rc.onGcsGearAccepted();
    TEST_ASSERT_FALSE(rc.gearPending);
    TEST_ASSERT_FALSE(rc.stepGear(vcmdRcGear(1000)));
    TEST_ASSERT_FALSE(rc.gearPending);
    // A later RC edge replaces / re-creates it
    TEST_ASSERT_TRUE(rc.stepGear(vcmdRcGear(1900)));
    TEST_ASSERT_TRUE(rc.gearPending);
    TEST_ASSERT_EQUAL(VCMD_GEAR_R, rc.pendingGear);
    TEST_ASSERT_TRUE(rc.stepGear(vcmdRcGear(1500)));
    TEST_ASSERT_EQUAL(VCMD_GEAR_N, rc.pendingGear);
    // Absent channel keeps the pending gear
    TEST_ASSERT_FALSE(rc.stepGear(vcmdRcGear(0)));
    TEST_ASSERT_TRUE(rc.gearPending);
    // Re-sync drops it
    rc.resync();
    TEST_ASSERT_FALSE(rc.gearPending);
}

// ---- Caps, versions, IGN_STATE --------------------------------------------------------------

static void test_caps_and_qb_caps() {
    TEST_ASSERT_EQUAL_HEX16(0x1F, vehicleCapsMask(true, true, true, true, true, false));
    TEST_ASSERT_EQUAL_HEX16(0x3F, vehicleCapsMask(true, true, true, true, true, true));
    TEST_ASSERT_EQUAL_HEX16(0x0F, vehicleCapsMask(true, true, true, true, false, false));
    TEST_ASSERT_EQUAL_INT32(16777247, qbCapsValue(0x1F, 1));
    TEST_ASSERT_EQUAL_INT32(16777279, qbCapsValue(0x3F, 1));
    TEST_ASSERT_EQUAL_HEX32(0, (uint32_t)qbCapsValue(0xFFFF, 1) & 0x00FF0000u);
    TEST_ASSERT_EQUAL_HEX32(0x0100FFFFu, (uint32_t)qbCapsValue(0xFFFF, 1));
}

static void test_version_codes() {
    TEST_ASSERT_EQUAL_INT32(10528, firmwareVersionCode("1.5.28"));
    TEST_ASSERT_EQUAL_INT32(10530, firmwareVersionCode("1.5.30"));
    TEST_ASSERT_EQUAL_INT32(21003, firmwareVersionCode("2.10.3"));
    TEST_ASSERT_EQUAL_INT32(0, firmwareVersionCode("1.5"));
    TEST_ASSERT_EQUAL_INT32(0, firmwareVersionCode("1.5.x"));
    TEST_ASSERT_EQUAL_INT32(0, firmwareVersionCode(""));
    static_assert(firmwareVersionCode("1.5.30") == 10530, "constexpr");
    TEST_ASSERT_EQUAL_HEX32(0x00020100, (uint32_t)hardwareVersionCode(0, 1, 2));
    TEST_ASSERT_EQUAL_HEX32(0x00020200, (uint32_t)hardwareVersionCode(0, 2, 2));
    TEST_ASSERT_EQUAL_HEX32(0x00000100, (uint32_t)hardwareVersionCode(0, 1, 0));
}

static void test_ign_state_code() {
    TEST_ASSERT_EQUAL(0, ignStateCode(0, false));
    TEST_ASSERT_EQUAL(1, ignStateCode(1, false));
    TEST_ASSERT_EQUAL(3, ignStateCode(1, true));    // pre-crank dwell
    TEST_ASSERT_EQUAL(2, ignStateCode(2, false));
    TEST_ASSERT_EQUAL(3, ignStateCode(3, false));
    TEST_ASSERT_EQUAL(0, ignStateCode(0, true));    // OFF never reads as cranking
}

// ---- Handbrake (aztec) -----------------------------------------------------------------------

static void test_park_brake_ack() {
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)vehicleCmdParkBrakeAck(ParkBrakeResult::STARTED).result);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)vehicleCmdParkBrakeAck(ParkBrakeResult::STARTED_SPEED_UNKNOWN).result);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)vehicleCmdParkBrakeAck(ParkBrakeResult::REVERSING).result);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::ACCEPTED, (int)vehicleCmdParkBrakeAck(ParkBrakeResult::ALREADY_RUNNING).result);
    VehicleCmdOutcome o = vehicleCmdParkBrakeAck(ParkBrakeResult::REFUSED_MOVING);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::TEMPORARILY_REJECTED, (int)o.result);
    TEST_ASSERT_EQUAL(VCMD_REASON_PBRAKE_SPEED, o.reason);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::FAILED, (int)vehicleCmdParkBrakeAck(ParkBrakeResult::REFUSED_DRIVER).result);
    TEST_ASSERT_EQUAL((int)VehicleCmdResult::FAILED, (int)vehicleCmdParkBrakeAck(ParkBrakeResult::REFUSED_NVS).result);
    // Never reason 3
    const ParkBrakeResult all[] = {ParkBrakeResult::STARTED, ParkBrakeResult::STARTED_SPEED_UNKNOWN,
                                   ParkBrakeResult::ALREADY_RUNNING, ParkBrakeResult::REVERSING,
                                   ParkBrakeResult::REFUSED_MOVING, ParkBrakeResult::REFUSED_DRIVER,
                                   ParkBrakeResult::REFUSED_NVS};
    for (ParkBrakeResult r : all) {
        TEST_ASSERT_NOT_EQUAL(VCMD_REASON_FAILSAFE, vehicleCmdParkBrakeAck(r).reason);
    }
}

static void test_park_brake_caps() {
    // Buggy with the X6 driver: 0x3F, QB_CAPS 16777279; without it: 0x1F and function 5 unsupported.
    const uint16_t withDriver = vehicleCapsMask(true, true, true, true, true, true);
    const uint16_t noDriver   = vehicleCapsMask(true, true, true, true, true, false);
    TEST_ASSERT_EQUAL_HEX16(0x3F, withDriver);
    TEST_ASSERT_EQUAL_INT32(16777279, qbCapsValue(withDriver, 1));
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::FUNCTION, (int)vehicleCmdDecode(5.0f, 0.0f, withDriver).kind);
    TEST_ASSERT_EQUAL((int)VehicleCmdDecodeKind::UNSUPPORTED, (int)vehicleCmdDecode(5.0f, 0.0f, noDriver).kind);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_decode_functions_accepted);
    RUN_TEST(test_decode_unknown_function);
    RUN_TEST(test_decode_capability_bits);
    RUN_TEST(test_decode_values);
    RUN_TEST(test_gear_interlocks);
    RUN_TEST(test_ignition_interlocks);
    RUN_TEST(test_light_and_lock);
    RUN_TEST(test_rc_decode);
    RUN_TEST(test_command_holds_against_constant_rc);
    RUN_TEST(test_rc_edge_wins);
    RUN_TEST(test_rc_absent_is_not_a_change);
    RUN_TEST(test_resync_every_cause);
    RUN_TEST(test_resync_with_absent_channel);
    RUN_TEST(test_pending_rc_gear);
    RUN_TEST(test_caps_and_qb_caps);
    RUN_TEST(test_version_codes);
    RUN_TEST(test_ign_state_code);
    RUN_TEST(test_park_brake_ack);
    RUN_TEST(test_park_brake_caps);
    return UNITY_END();
}
