// Native tests of include/IboosterLaw.h (change replace-aztec-brake-with-ibooster, tasks 2.2 and
// 7a.2): the unit state machine (design D4), percent -> rod -> flow (D5), settings (D6), the stale
// command (D7), reached (D8) and the brake STATUSTEXT decision (D10).
#include <unity.h>
#include <math.h>
#include "IboosterLaw.h"

void setUp() {}
void tearDown() {}

// ---- Simulation helpers ------------------------------------------------------------------------------
struct Sim {
    IbLaw law;
    IbCommand cmd;
    IbUnitView u;
    uint32_t now;
    IbStepOut last;
    uint32_t stateEvents;

    Sim() : now(1000), last(), stateEvents(0) {
        law = ibLawInitial(now);
        cmd = {};
        u = {};
        cmd.applyUnits = ibRateUnits(IB_LAW_RATE_DEFAULT_MLPS);
        cmd.releaseUnits = ibRateUnits(IB_LAW_RATE_DEFAULT_MLPS);
    }
    // The unit sends a fresh 0x39D / 0x38F now (all three frames count as "heard")
    void unit(uint8_t st, uint8_t is, uint8_t drv, int16_t rodCmm, int8_t partner = 1, uint8_t runout = 51) {
        u.have39D = true; u.t39D = now; u.st = st; u.is = is; u.drv = drv; u.rodCmm = rodCmm;
        u.have38F = true; u.t38F = now; u.partner = partner; u.runoutBar = runout;
        u.heard = true; u.tAny = now;
    }
    void ready(int16_t rod = 0) { unit(5, 0, 1, rod); }
    IbStepOut step(uint32_t dtMs = 10) {
        now += dtMs;
        cmd.stampMs = now;   // a live loop
        last = ibStep(law, cmd, u, now);
        if (last.ev & IB_EV_STATE) stateEvents++;
        return last;
    }
    // Step without refreshing the command stamp (loop stalled)
    IbStepOut stepStale(uint32_t dtMs = 10) {
        now += dtMs;
        last = ibStep(law, cmd, u, now);
        return last;
    }
    // Drive to READY: ignition on, unit ready
    void toReady() {
        cmd.ignOn = true;
        ready();
        step();   // ASLEEP -> STARTING
        ready();
        step();   // STARTING -> READY
        TEST_ASSERT_TRUE(law.state == IbState::READY);
    }
};

// ---- Settings / conversions ----------------------------------------------------------------------------
static void test_defaults_and_units() {
    TEST_ASSERT_EQUAL(160, IB_LAW_MAX_MM10_DEFAULT);
    TEST_ASSERT_EQUAL(8, IB_LAW_RATE_DEFAULT_MLPS);
    TEST_ASSERT_EQUAL(1024, ibRateUnits(8));          // 8 ml/s = 1024 units
    TEST_ASSERT_EQUAL(128, ibRateUnits(1));
    TEST_ASSERT_EQUAL(5120, ibRateUnits(40));
    TEST_ASSERT_EQUAL(128, ibRateUnits(0));           // clamped to 1 ml/s
    TEST_ASSERT_EQUAL(5120, ibRateUnits(1000));       // clamped to 40 ml/s
    TEST_ASSERT_TRUE(ibMaxMm10Valid(20) && ibMaxMm10Valid(160));
    TEST_ASSERT_FALSE(ibMaxMm10Valid(19) || ibMaxMm10Valid(161) || ibMaxMm10Valid(0) || ibMaxMm10Valid(255));
    TEST_ASSERT_TRUE(ibMaxMm10OnGrid(85));
    TEST_ASSERT_FALSE(ibMaxMm10OnGrid(87));
    TEST_ASSERT_EQUAL(160, ibMaxMm10OrDefault(255));  // corrupted NVS value -> default
    TEST_ASSERT_EQUAL(8, ibRateOrDefault(0));
    TEST_ASSERT_EQUAL(8, ibRateOrDefault(41));
    TEST_ASSERT_EQUAL(40, ibRateOrDefault(40));
}

static void test_percent_to_rod() {
    TEST_ASSERT_EQUAL(80, ibTargetCmm(5.0f, 160));      // 5 % of 16 mm = 0.80 mm
    TEST_ASSERT_EQUAL(1600, ibTargetCmm(100.0f, 160));  // 100 % = 16.00 mm
    TEST_ASSERT_EQUAL(400, ibTargetCmm(50.0f, 80));     // 50 % of 8 mm = 4.00 mm
    TEST_ASSERT_EQUAL(0, ibTargetCmm(0.0f, 160));
    // Ceiling: whatever the percent or the stored stroke, never above 16.00 mm
    TEST_ASSERT_EQUAL(1600, ibTargetCmm(150.0f, 160));
    TEST_ASSERT_EQUAL(1600, ibTargetCmm(1e30f, 255));
    TEST_ASSERT_EQUAL(1600, ibTargetCmm(100.0f, 100000));
    TEST_ASSERT_EQUAL(1600, ibTargetCmm(INFINITY, 160));
    TEST_ASSERT_EQUAL(0, ibTargetCmm(NAN, 160));        // garbage -> released
    TEST_ASSERT_EQUAL(0, ibTargetCmm(-20.0f, 160));
    TEST_ASSERT_EQUAL(200, ibTargetCmm(100.0f, 0));     // a zero stroke is clamped to 2.0 mm
    for (int p = 0; p <= 1000; p++) {
        for (int m = -10; m <= 400; m += 7) {
            TEST_ASSERT_TRUE(ibTargetCmm((float)p * 0.5f, m) <= IB_LAW_ROD_CEIL_CMM);
        }
    }
}

static void test_flow_law() {
    const int32_t up = ibRateUnits(8), down = ibRateUnits(8);
    TEST_ASSERT_EQUAL(0, ibFlowUnits(200, 195, up, down));     // deadband 0.05 mm
    TEST_ASSERT_EQUAL(0, ibFlowUnits(200, 205, up, down));
    TEST_ASSERT_EQUAL(15, ibFlowUnits(200, 194, up, down));    // 0.06 mm x 250 /mm
    TEST_ASSERT_EQUAL(250, ibFlowUnits(300, 200, up, down));   // 1 mm -> 250 units
    TEST_ASSERT_EQUAL(-250, ibFlowUnits(100, 200, up, down));
    TEST_ASSERT_EQUAL(1024, ibFlowUnits(1600, 0, up, down));   // apply cap 8 ml/s
    TEST_ASSERT_EQUAL(-1024, ibFlowUnits(0, 1600, up, down));  // release cap 8 ml/s
    TEST_ASSERT_EQUAL(256, ibFlowUnits(1600, 0, ibRateUnits(2), down));
    TEST_ASSERT_EQUAL(-384, ibFlowUnits(0, 1600, up, ibRateUnits(3)));
    // Caps are re-clamped: 0 -> 1 ml/s, huge -> 40 ml/s
    TEST_ASSERT_EQUAL(128, ibFlowUnits(1600, 0, 0, 0));
    TEST_ASSERT_EQUAL(-128, ibFlowUnits(0, 1600, 0, 0));
    TEST_ASSERT_EQUAL(4000, ibFlowUnits(1600, 0, 60000, 60000));
    // A target above the ceiling is re-clamped in the law too
    TEST_ASSERT_EQUAL(0, ibFlowUnits(5000, 1600, 60000, 60000));
    // Flow word always within 0x6A00..0x9200
    TEST_ASSERT_EQUAL_HEX16(0x7E00, ibFlowWord(0));
    TEST_ASSERT_EQUAL_HEX16(0x9200, ibFlowWord(100000));
    TEST_ASSERT_EQUAL_HEX16(0x6A00, ibFlowWord(-100000));
    for (int32_t t = 0; t <= 2000; t += 37) {
        for (int32_t r = -600; r <= 2500; r += 41) {
            const int32_t f = ibFlowUnits(t, r, 60000, 60000);
            TEST_ASSERT_TRUE(f <= 5120 && f >= -5120);
            const uint16_t w = ibFlowWord(f);
            TEST_ASSERT_TRUE(w >= 0x6A00 && w <= 0x9200);
        }
    }
}

static void test_speed_byte() {
    TEST_ASSERT_EQUAL(0, ibSpeedByte(5.0f, false));          // invalid -> 0
    TEST_ASSERT_EQUAL(0, ibSpeedByte(NAN, true));
    TEST_ASSERT_EQUAL(0, ibSpeedByte(-3.0f, true));
    TEST_ASSERT_EQUAL(3, ibSpeedByte(1.2f / 3.6f, true));    // 1.2 km/h -> 3
    TEST_ASSERT_EQUAL(25, ibSpeedByte(10.0f / 3.6f, true));  // 10 km/h -> 25
    TEST_ASSERT_EQUAL(255, ibSpeedByte(200.0f, true));       // clamp
    TEST_ASSERT_EQUAL(255, ibSpeedByte(INFINITY, true));
}

static void test_tx_enabled() {
    TEST_ASSERT_TRUE(ibTxEnabled(true, false, 0xFFFFFFFFUL));
    TEST_ASSERT_FALSE(ibTxEnabled(false, false, 0));
    TEST_ASSERT_TRUE(ibTxEnabled(false, true, 499));          // heard with ignition reported OFF
    TEST_ASSERT_FALSE(ibTxEnabled(false, true, 500));         // 500 ms of silence
}

// ---- State machine -------------------------------------------------------------------------------------
static void test_asleep_and_start() {
    Sim m;
    IbStepOut o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::ASLEEP);
    TEST_ASSERT_FALSE(o.txEnabled);
    TEST_ASSERT_FALSE(o.req);
    // Ignition on -> STARTING, idle frames
    m.cmd.ignOn = true;
    o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::STARTING);
    TEST_ASSERT_TRUE(o.ev & IB_EV_STATE);
    TEST_ASSERT_TRUE(o.evFrom == IbState::ASLEEP && o.evTo == IbState::STARTING);
    TEST_ASSERT_TRUE(o.txEnabled);
    TEST_ASSERT_FALSE(o.req);
    TEST_ASSERT_EQUAL_HEX16(0x7E00, o.flowWord);
    // Status ready but the unit does not see us yet (0x38F 0x80): stays STARTING
    m.unit(5, 0, 1, 0, 0);
    TEST_ASSERT_TRUE(m.step().state == IbState::STARTING);
    // Sees us -> READY
    m.unit(5, 0, 1, 0, 1);
    o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::READY);
    TEST_ASSERT_FALSE(o.req);
}

static void test_heard_with_ignition_off_starts() {
    Sim m;
    m.cmd.ignOn = false;
    m.ready();   // the unit is heard (relay expander faulted, reports OFF, Relay_1 physically on)
    IbStepOut o = m.step();
    TEST_ASSERT_TRUE(o.txEnabled);
    TEST_ASSERT_TRUE(o.state == IbState::STARTING);
    m.ready();
    TEST_ASSERT_TRUE(m.step().state == IbState::READY);
    // A brake is honoured while the unit is heard
    m.cmd.targetCmm = 400;
    m.ready();
    o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::BRAKING);
    TEST_ASSERT_TRUE(o.req);
}

static void test_asleep_only_ignition_off_and_500ms() {
    Sim m;
    m.toReady();
    // Ignition OFF, unit still talking: frames continue
    m.cmd.ignOn = false;
    m.ready();
    TEST_ASSERT_TRUE(m.step().txEnabled);
    // Unit goes quiet: 490 ms after its last frame still enabled and no LOST (ignition OFF = going
    // to sleep)
    IbStepOut o = m.step(480);
    TEST_ASSERT_TRUE(o.txEnabled);
    TEST_ASSERT_TRUE(o.state == IbState::READY);
    o = m.step(10);   // 500 ms of silence
    TEST_ASSERT_FALSE(o.txEnabled);
    TEST_ASSERT_TRUE(o.state == IbState::ASLEEP);
    TEST_ASSERT_TRUE(o.evFrom == IbState::READY);
    // Ignition on but silent: frames go out (STARTING), never ASLEEP
    m.cmd.ignOn = true;
    for (int i = 0; i < 100; i++) {
        o = m.step(10);
        TEST_ASSERT_TRUE(o.txEnabled);
        TEST_ASSERT_TRUE(o.state == IbState::STARTING);
    }
}

static void test_status6_self_cycle_stays_ready() {
    Sim m;
    m.toReady();
    for (int i = 0; i < 200; i++) {   // ~2 s of st=6 is=5 (self cycle / to_idle)
        m.unit(6, 5, 1, 3);
        TEST_ASSERT_TRUE(m.step().state == IbState::READY);
    }
    TEST_ASSERT_EQUAL(0, (int)(m.last.ev & IB_EV_STATE));
}

static void test_lost_at_251_not_250() {
    Sim m;
    m.toReady();
    m.ready();
    TEST_ASSERT_TRUE(m.step(0).state == IbState::READY);
    TEST_ASSERT_TRUE(m.step(250).state == IbState::READY);   // 0x39D exactly 250 ms old
    IbStepOut o = m.step(1);                                  // 251 ms
    TEST_ASSERT_TRUE(o.state == IbState::LOST);
    TEST_ASSERT_EQUAL(251, o.evMs);
    TEST_ASSERT_TRUE(o.txEnabled);                            // idle frames continue
    TEST_ASSERT_FALSE(o.req);
    // Frames return -> STARTING -> READY
    m.ready();
    TEST_ASSERT_TRUE(m.step().state == IbState::STARTING);
    m.ready();
    TEST_ASSERT_TRUE(m.step().state == IbState::READY);
}

static void test_lost_clears_request_and_no_unit() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 800;
    m.ready();
    TEST_ASSERT_TRUE(m.step().state == IbState::BRAKING);
    IbStepOut o = m.step(260);
    TEST_ASSERT_TRUE(o.state == IbState::LOST);
    TEST_ASSERT_FALSE(o.req);
    TEST_ASSERT_EQUAL_HEX16(0x7E00, o.flowWord);
    // 20 s of LOST -> no_unit once
    int noUnit = 0;
    for (int i = 0; i < 2100; i++) {
        o = m.step(10);
        TEST_ASSERT_TRUE(o.state == IbState::LOST);
        TEST_ASSERT_FALSE(o.req);
        if (o.ev & IB_EV_NO_UNIT) noUnit++;
    }
    TEST_ASSERT_EQUAL(1, noUnit);
}

// Owner Q2 (a): the target is kept while not available; READY again -> BRAKING by itself
static void test_target_held_while_lost_then_resumes() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 1000;
    m.ready();
    m.step();
    m.step(300);
    TEST_ASSERT_TRUE(m.law.state == IbState::LOST);
    for (int i = 0; i < 50; i++) TEST_ASSERT_FALSE(m.step().req);
    m.ready(200);
    TEST_ASSERT_TRUE(m.step().state == IbState::STARTING);
    TEST_ASSERT_FALSE(m.last.req);
    m.ready(200);
    TEST_ASSERT_TRUE(m.step().state == IbState::READY);
    TEST_ASSERT_FALSE(m.last.req);
    m.ready(200);
    IbStepOut o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::BRAKING);
    TEST_ASSERT_TRUE(o.req);
    TEST_ASSERT_EQUAL(1024, o.flow);   // 8 mm error, apply cap
}

static void test_fault_and_self_recovery() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 500;
    m.ready();
    TEST_ASSERT_TRUE(m.step().req);
    // Reboot-type st=2: FAULT at once, request cleared
    m.unit(2, 0, 1, 100);
    IbStepOut o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::FAULT);
    TEST_ASSERT_FALSE(o.req);
    TEST_ASSERT_TRUE(o.txEnabled);
    // drv=3 also a fault, and frames gone: FAULT stays (leaves only on the unit's evidence)
    m.unit(1, 0, 3, 0);
    TEST_ASSERT_TRUE(m.step().state == IbState::FAULT);
    for (int i = 0; i < 100; i++) TEST_ASSERT_TRUE(m.step(10).state == IbState::FAULT);
    // ~12 s later the unit reports ready by itself -> READY, then BRAKING with the held target
    m.ready();
    o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::READY);
    TEST_ASSERT_TRUE(o.evFrom == IbState::FAULT);
    m.ready();
    TEST_ASSERT_TRUE(m.step().state == IbState::BRAKING);
}

static void test_fault_latched_once_after_20s() {
    Sim m;
    m.toReady();
    m.unit(2, 0, 3, 0);
    m.step();
    TEST_ASSERT_TRUE(m.law.state == IbState::FAULT);
    const uint32_t t0 = m.now;
    int latched = 0;
    for (int i = 0; i < 2500; i++) {   // 25 s
        m.unit(2, 0, 3, 0);
        const IbStepOut o = m.step(10);
        if (o.ev & IB_EV_FAULT_LATCHED) {
            latched++;
            TEST_ASSERT_TRUE(m.now - t0 >= IB_LAW_FAULT_PERSIST_MS);
            TEST_ASSERT_TRUE(m.now - t0 < IB_LAW_FAULT_PERSIST_MS + 10);
        }
    }
    TEST_ASSERT_EQUAL(1, latched);
}

static void test_start_timeout_once() {
    Sim m;
    m.cmd.ignOn = true;
    int timeouts = 0;
    for (int i = 0; i < 2500; i++) {
        m.unit(4, 0, 0, 0);   // active_good_check, never ready
        if (m.step(10).ev & IB_EV_START_TIMEOUT) timeouts++;
        TEST_ASSERT_TRUE(m.last.state == IbState::STARTING);
    }
    TEST_ASSERT_EQUAL(1, timeouts);
}

static void test_ready_lost_condition_goes_starting() {
    Sim m;
    m.toReady();
    m.unit(5, 0, 1, 0, 0);   // the unit no longer sees our frames
    TEST_ASSERT_TRUE(m.step().state == IbState::STARTING);
    m.ready();
    TEST_ASSERT_TRUE(m.step().state == IbState::READY);
    m.unit(4, 0, 1, 0);      // not-ready status
    TEST_ASSERT_TRUE(m.step().state == IbState::STARTING);
}

// ---- Braking / releasing -------------------------------------------------------------------------------
static void test_braking_release_handoff() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 1600;
    m.ready();
    IbStepOut o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::BRAKING);
    TEST_ASSERT_TRUE(o.req);
    TEST_ASSERT_EQUAL_HEX16(0x7E00 + 1024, o.flowWord);
    // Accepted, rod near target
    m.unit(6, 3, 1, 1590);
    o = m.step();
    TEST_ASSERT_TRUE(o.reached);     // 0.10 mm from the target
    TEST_ASSERT_EQUAL(25, o.flow);   // 0.10 mm x 250 /mm
    // Release: target 0 -> RELEASING, request kept, flow at the release cap
    m.cmd.targetCmm = 0;
    m.unit(6, 3, 1, 1600);
    o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::RELEASING);
    TEST_ASSERT_TRUE(o.req);
    TEST_ASSERT_FALSE(o.reached);
    TEST_ASSERT_EQUAL(-1024, o.flow);
    m.unit(6, 3, 1, 300);
    o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::RELEASING);
    TEST_ASSERT_EQUAL(-750, o.flow);
    m.unit(6, 3, 1, 101);
    TEST_ASSERT_TRUE(m.step().state == IbState::RELEASING);
    m.unit(6, 3, 1, 100);   // 1.00 mm: hand-off
    o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::READY);
    TEST_ASSERT_FALSE(o.req);
    TEST_ASSERT_EQUAL_HEX16(0x7E00, o.flowWord);
    TEST_ASSERT_FALSE(o.ev & IB_EV_RELEASE_SLOW);
}

static void test_release_timeout() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 800;
    m.unit(6, 3, 1, 800);
    m.step();
    m.cmd.targetCmm = 0;
    m.unit(6, 3, 1, 800);
    m.step();
    TEST_ASSERT_TRUE(m.law.state == IbState::RELEASING);
    int slow = 0;
    for (int i = 0; i < 400; i++) {
        m.unit(6, 3, 1, 500);   // stuck at 5 mm
        const IbStepOut o = m.step(10);
        if (o.ev & IB_EV_RELEASE_SLOW) {
            slow++;
            TEST_ASSERT_TRUE(o.state == IbState::READY);
            TEST_ASSERT_FALSE(o.req);
        }
    }
    TEST_ASSERT_EQUAL(1, slow);
}

static void test_repress_during_release() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 800;
    m.unit(6, 3, 1, 800);
    m.step();
    m.cmd.targetCmm = 0;
    m.step();
    TEST_ASSERT_TRUE(m.law.state == IbState::RELEASING);
    m.cmd.targetCmm = 1200;
    m.unit(6, 3, 1, 600);
    const IbStepOut o = m.step();
    TEST_ASSERT_TRUE(o.state == IbState::BRAKING);
    TEST_ASSERT_TRUE(o.req);
    TEST_ASSERT_EQUAL(1024, o.flow);
}

static void test_small_brake_below_handoff() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = ibTargetCmm(5.0f, 160);   // 0.80 mm
    m.unit(6, 3, 1, 80);
    TEST_ASSERT_TRUE(m.step().state == IbState::BRAKING);
    m.cmd.targetCmm = 0;
    m.unit(6, 3, 1, 80);
    TEST_ASSERT_TRUE(m.step().state == IbState::RELEASING);
    TEST_ASSERT_TRUE(m.step().state == IbState::READY);   // rod <= 1 mm: request cleared at once
}

// The P loop holds (flow 0) on a rod sample older than 100 ms; it never uses a stale one
static void test_old_rod_sample_holds() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 1600;
    m.unit(6, 3, 1, 200);
    IbStepOut o = m.step(0);
    TEST_ASSERT_EQUAL(1024, o.flow);
    o = m.step(100);           // 100 ms old: still used
    TEST_ASSERT_EQUAL(1024, o.flow);
    o = m.step(1);             // 101 ms old: hold
    TEST_ASSERT_EQUAL(0, o.flow);
    TEST_ASSERT_TRUE(o.req);
    TEST_ASSERT_EQUAL_HEX16(0x7E00, o.flowWord);
    TEST_ASSERT_FALSE(o.reached);
    m.unit(6, 3, 1, 1000);     // a fresh frame: P resumes from the new rod, no jump beyond the cap
    o = m.step();
    TEST_ASSERT_EQUAL(1024, o.flow);
    m.unit(6, 3, 1, 1590);
    o = m.step();
    TEST_ASSERT_EQUAL(25, o.flow);
}

// Rates in every path: a changed command caps the very next slot; garbage caps are clamped
static void test_rates_every_path() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 1600;
    m.unit(6, 3, 1, 0);
    TEST_ASSERT_EQUAL(1024, m.step().flow);
    m.cmd.applyUnits = ibRateUnits(2);
    TEST_ASSERT_EQUAL(256, m.step().flow);
    m.cmd.applyUnits = 0;          // never published / zeroed record
    TEST_ASSERT_EQUAL(128, m.step().flow);
    m.cmd.applyUnits = 65535;      // corrupted
    TEST_ASSERT_EQUAL(4000, m.step().flow);   // P (16 mm x 250) below the 40 ml/s cap
    m.cmd.targetCmm = 65535;       // corrupted target: clamped to 16.00 mm
    m.unit(6, 3, 1, 1600);
    IbStepOut o = m.step();
    TEST_ASSERT_EQUAL(1600, o.targetCmm);
    TEST_ASSERT_EQUAL(0, o.flow);
    TEST_ASSERT_TRUE(o.reached);
}

static void test_reached_rules() {
    // never outside BRAKING
    TEST_ASSERT_FALSE(ibReached(IbState::READY, true, 3, 800, 800, true, true, 0));
    TEST_ASSERT_FALSE(ibReached(IbState::RELEASING, true, 3, 800, 800, true, true, 0));
    TEST_ASSERT_FALSE(ibReached(IbState::LOST, true, 3, 800, 800, true, true, 0));
    // not accepted / stale -> false
    TEST_ASSERT_FALSE(ibReached(IbState::BRAKING, true, 0, 800, 800, false, true, 51));
    TEST_ASSERT_FALSE(ibReached(IbState::BRAKING, false, 3, 800, 800, false, true, 51));
    // tolerance 0.2 mm
    TEST_ASSERT_TRUE(ibReached(IbState::BRAKING, true, 3, 780, 800, false, true, 51));
    TEST_ASSERT_TRUE(ibReached(IbState::BRAKING, true, 3, 820, 800, false, true, 51));
    TEST_ASSERT_FALSE(ibReached(IbState::BRAKING, true, 3, 779, 800, false, true, 51));
    // stall, pRunout 0 (fresh 0x38F only)
    TEST_ASSERT_TRUE(ibReached(IbState::BRAKING, true, 3, 300, 800, true, true, 51));
    TEST_ASSERT_TRUE(ibReached(IbState::BRAKING, true, 3, 300, 800, false, true, 0));
    TEST_ASSERT_FALSE(ibReached(IbState::BRAKING, true, 3, 300, 800, false, false, 0));
}

static void test_reached_by_stall_in_step() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 1600;
    m.unit(6, 3, 1, 0);
    m.step();               // accepted at rest (arming reference 0 mm)
    m.unit(6, 3, 1, 900);   // moved, then pressure stops the rod at 9 mm
    IbStepOut o = m.step();
    TEST_ASSERT_FALSE(o.reached);
    for (int i = 0; i < 29; i++) {
        m.unit(6, 3, 1, (int16_t)(900 + (i % 2)));   // < 0.05 mm progress
        o = m.step(10);
        TEST_ASSERT_FALSE(o.reached);
    }
    m.unit(6, 3, 1, 901);
    o = m.step(10);   // 300 ms without progress
    TEST_ASSERT_TRUE(o.reached);
    // Progress resets the stall
    m.unit(6, 3, 1, 950);
    TEST_ASSERT_FALSE(m.step(10).reached);
}

static void test_reached_by_runout_zero() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 1600;
    m.unit(6, 3, 1, 0, 1, 51);
    TEST_ASSERT_FALSE(m.step().reached);   // accepted at rest
    m.unit(6, 3, 1, 1400, 1, 51);
    TEST_ASSERT_FALSE(m.step().reached);
    m.unit(6, 3, 1, 1400, 1, 0);   // stroke stop
    TEST_ASSERT_TRUE(m.step().reached);
}

static void test_unit_drop_after_accept_timeout() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 800;
    int drops = 0;
    for (int i = 0; i < 150; i++) {
        m.unit(6, 5, 1, 0);   // never accepts (is 5)
        const IbStepOut o = m.step(10);
        TEST_ASSERT_TRUE(o.req);   // the request is kept
        if (o.ev & IB_EV_UNIT_DROP) {
            drops++;
            TEST_ASSERT_TRUE(o.evMs >= IB_LAW_ACCEPT_TIMEOUT_MS);
        }
    }
    TEST_ASSERT_EQUAL(1, drops);
    // Accepts: a later non-accept episode is reported again
    m.unit(6, 3, 1, 700);
    m.step();
    for (int i = 0; i < 120; i++) {
        m.unit(6, 5, 1, 0);
        if (m.step(10).ev & IB_EV_UNIT_DROP) drops++;
    }
    TEST_ASSERT_EQUAL(2, drops);
}

// Normal acceptance at 260-290 ms is not a drop
static void test_normal_accept_not_a_drop() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 800;
    for (int i = 0; i < 28; i++) {
        m.unit(5, 0, 1, 0);
        TEST_ASSERT_FALSE(m.step(10).ev & IB_EV_UNIT_DROP);
    }
    for (int i = 0; i < 200; i++) {
        m.unit(6, 3, 1, 700);
        TEST_ASSERT_FALSE(m.step(10).ev & IB_EV_UNIT_DROP);
    }
}

// ---- Stale command (D7) --------------------------------------------------------------------------------
static void test_stale_command_holds_target() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 1000;
    m.cmd.speedByte = 7;
    m.unit(6, 3, 1, 500);
    m.step();
    const uint32_t stamp = m.cmd.stampMs;
    IbStepOut o;
    for (int i = 0; i < 50; i++) {
        m.unit(6, 3, 1, 500);
        o = m.stepStale(10);
    }
    TEST_ASSERT_EQUAL(500, m.now - stamp);
    TEST_ASSERT_FALSE(o.cmdStale);             // exactly 500 ms: not yet
    m.unit(6, 3, 1, 500);
    o = m.stepStale(10);
    TEST_ASSERT_TRUE(o.cmdStale);
    TEST_ASSERT_TRUE(o.state == IbState::BRAKING);
    TEST_ASSERT_TRUE(o.req);                   // the last target is held
    TEST_ASSERT_EQUAL(1000, o.targetCmm);
    TEST_ASSERT_EQUAL(1024, o.flow);
    for (int i = 0; i < 100; i++) {
        m.unit(6, 3, 1, 1000);
        o = m.stepStale(10);
        TEST_ASSERT_TRUE(o.req);
    }
    // The loop returns: one cmd_stale event with the longest age
    m.unit(6, 3, 1, 1000);
    o = m.step();
    TEST_ASSERT_TRUE(o.ev & IB_EV_CMD_STALE);
    TEST_ASSERT_EQUAL(1510, o.evStaleMs);
    TEST_ASSERT_FALSE(m.step().ev & IB_EV_CMD_STALE);
}

static void test_no_command_yet_not_stale() {
    Sim m;
    m.cmd.stampMs = 0;
    m.now += 5000;
    const IbStepOut o = ibStep(m.law, m.cmd, m.u, m.now);
    TEST_ASSERT_FALSE(o.cmdStale);
    TEST_ASSERT_FALSE(o.txEnabled);   // all-zero command = ignition OFF, idle
}

// ---- Cranking -------------------------------------------------------------------------------------------
static void test_crank_window() {
    Sim m;
    m.toReady();
    TEST_ASSERT_FALSE(m.step().crank);
    m.cmd.cranking = true;
    m.ready();
    TEST_ASSERT_TRUE(m.step().crank);
    m.cmd.cranking = false;
    IbStepOut o;
    for (int i = 0; i < 299; i++) {   // 2990 ms after the crank, unit healthy
        m.ready();
        o = m.step(10);
    }
    TEST_ASSERT_TRUE(o.crank);
    TEST_ASSERT_TRUE(o.state == IbState::READY);
    m.unit(2, 0, 1, 0);   // a cranking dip reboots the unit inside the window
    o = m.step(5);
    TEST_ASSERT_TRUE(o.crank);
    TEST_ASSERT_TRUE(o.state == IbState::FAULT);
    o = m.step(10);       // 3005 ms after the crank
    TEST_ASSERT_FALSE(o.crank);
}

// ---- STATUSTEXT (7a.2) ----------------------------------------------------------------------------------
static void test_statustext_flapping() {
    IbTextState s = ibTextInitial();
    uint32_t now = 100000;
    int texts = 0;
    IbText lastSent = IbText::NONE;
    IbState cur = IbState::READY;
    TEST_ASSERT_TRUE(ibStatustextStep(s, cur, now, true) == IbText::NONE);   // baseline READY: silent
    for (int sec = 0; sec <= 20; sec++) {
        cur = (sec % 2 == 0) ? IbState::LOST : IbState::READY;
        for (int k = 0; k < 40; k++) {   // reports every 25 ms within this second
            const IbText t = ibStatustextStep(s, cur, now, true);
            if (t != IbText::NONE) {
                texts++;
                lastSent = t;
            }
            now += 25;
        }
    }
    TEST_ASSERT_TRUE(texts <= 5);
    // After the flapping stops the last text sent describes the current state
    for (int k = 0; k < 400; k++) {
        const IbText t = ibStatustextStep(s, cur, now, true);
        if (t != IbText::NONE) lastSent = t;
        now += 25;
    }
    TEST_ASSERT_TRUE(lastSent == ibTextClass(cur));
}

static void test_statustext_cancel_and_silent() {
    // READY without an announced warning: nothing
    IbTextState s = ibTextInitial();
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::STARTING, 0, true) == IbText::NONE);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::READY, 10, true) == IbText::NONE);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::BRAKING, 20, true) == IbText::NONE);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::ASLEEP, 30, true) == IbText::NONE);
    // LOST -> WARNING text at once
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::LOST, 1000, true) == IbText::LOST);
    // READY -> OK held by the 5 s hold-off, then LOST again inside it: nothing pending
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::READY, 2000, true) == IbText::NONE);
    TEST_ASSERT_TRUE(s.pending == IbText::OK);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::LOST, 3000, true) == IbText::NONE);
    TEST_ASSERT_TRUE(s.pending == IbText::NONE);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::LOST, 7000, true) == IbText::NONE);
    // FAULT replaces; held back by the shared limiter it stays pending
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::FAULT, 7100, false) == IbText::NONE);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::FAULT, 7200, true) == IbText::FAULT);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::READY, 13000, true) == IbText::OK);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::READY, 19000, true) == IbText::NONE);   // once
    // A new LOST within the hold-off after OK (sent at 13 s), then READY again before it goes out:
    // nothing at all
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::LOST, 14000, true) == IbText::NONE);
    TEST_ASSERT_TRUE(s.pending == IbText::LOST);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::READY, 15000, true) == IbText::NONE);
    TEST_ASSERT_TRUE(s.pending == IbText::NONE);
    TEST_ASSERT_TRUE(ibStatustextStep(s, IbState::READY, 30000, true) == IbText::NONE);
}


// ---- Review fixes (2026-11-23) -------------------------------------------------------------------------
// Rod model: moves with the flow at ~1.9 mm/s per ml/s (bench), 10 ms per step.
static int16_t rodModel(int16_t rod, int32_t flowUnits, uint32_t dtMs) {
    const float mmPerS = (float)flowUnits / 128.0f * 1.9f;
    float r = (float)rod + mmPerS * 100.0f * (float)dtMs / 1000.0f;
    if (r < 0.0f) r = 0.0f;
    if (r > 1662.0f) r = 1662.0f;
    return (int16_t)(r + (r >= 0 ? 0.5f : -0.5f));
}

// Finding 1: the output rod does not move during the ~2 mm free-travel take-up after the unit
// accepts; no "reached" (by stall or by pRunout) may come from that.
static void test_takeup_is_not_reached() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 400;
    m.cmd.applyUnits = ibRateUnits(1);   // 1 ml/s: take-up lasts ~1 s
    for (int i = 0; i < 100; i++) {      // 1 s accepted, rod still at 0
        m.unit(6, 3, 1, 0);
        TEST_ASSERT_FALSE(m.step(10).reached);
    }
    m.unit(6, 3, 1, 0, 1, 0);            // pRunout 0 while the rod has not moved
    TEST_ASSERT_FALSE(m.step(10).reached);
}

// Finding 1: after the rod has moved, a pressure stall is still "reached"
static void test_stall_after_movement_is_reached() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 1600;
    m.unit(6, 3, 1, 0);
    m.step();
    for (int i = 0; i < 30; i++) {       // take-up
        m.unit(6, 3, 1, 0);
        m.step(10);
    }
    m.unit(6, 3, 1, 500);                // moved 5 mm, then pressure holds it
    m.step(10);
    IbStepOut o;
    for (int i = 0; i < 31; i++) {
        m.unit(6, 3, 1, 501);
        o = m.step(10);
    }
    TEST_ASSERT_TRUE(o.reached);
}

// Finding 2: release from 16 mm at 1 and 2 ml/s keeps the request until the hand-off
static void test_release_slow_rate_keeps_request() {
    for (int rate = 1; rate <= 2; rate++) {
        Sim m;
        m.toReady();
        m.cmd.releaseUnits = ibRateUnits(rate);
        m.cmd.targetCmm = 1600;
        m.unit(6, 3, 1, 1600);
        m.step();
        m.cmd.targetCmm = 0;
        int16_t rod = 1600;
        IbStepOut o;
        bool handedOff = false;
        for (int i = 0; i < 3000 && !handedOff; i++) {   // up to 30 s
            m.unit(6, 3, 1, rod);
            o = m.step(10);
            if (!o.req) {
                TEST_ASSERT_TRUE_MESSAGE(rod <= 200, "request cleared above 2 mm");
                TEST_ASSERT_FALSE(o.ev & IB_EV_RELEASE_SLOW);
                handedOff = true;
            }
            rod = rodModel(rod, o.flow, 10);
        }
        TEST_ASSERT_TRUE(handedOff);
    }
}

// Finding 4: ignition OFF -> ON within 500 ms with the unit silent (lost its Wake) is no LOST
static void test_ignition_blip_no_lost() {
    Sim m;
    m.toReady();
    m.cmd.ignOn = false;
    m.step(10);
    m.step(200);                // unit silent (age 210 ms)
    m.cmd.ignOn = true;         // back on before 500 ms
    for (int i = 0; i < 100; i++) {
        const IbStepOut o = m.step(10);
        TEST_ASSERT_FALSE(o.state == IbState::LOST);
        TEST_ASSERT_FALSE((o.ev & IB_EV_STATE) && o.evTo == IbState::LOST);
        TEST_ASSERT_FALSE(o.req);
    }
    TEST_ASSERT_TRUE(m.law.state == IbState::STARTING);
    m.ready();
    TEST_ASSERT_TRUE(m.step().state == IbState::READY);
}

// Finding 8: ignition OFF during BRAKING: no LOST, flow 0 on the old rod, ASLEEP at 500 ms
static void test_ignition_off_while_braking() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 800;
    m.unit(6, 3, 1, 400);
    IbStepOut o = m.step(0);
    TEST_ASSERT_TRUE(o.state == IbState::BRAKING);
    m.cmd.ignOn = false;
    for (int t = 10; t < 500; t += 10) {
        o = m.step(10);
        TEST_ASSERT_TRUE(o.state == IbState::BRAKING);
        TEST_ASSERT_FALSE(o.ev & IB_EV_STATE);
        if (t > IB_LAW_ROD_HOLD_MS) TEST_ASSERT_EQUAL(0, o.flow);
    }
    o = m.step(10);
    TEST_ASSERT_TRUE(o.state == IbState::ASLEEP);
    TEST_ASSERT_FALSE(o.req);
    TEST_ASSERT_FALSE(o.txEnabled);
}

// Finding 7: in RELEASING with the ignition reported OFF and a stale 0x39D a re-press does not
// start a new request (BRAKING)
static void test_repress_on_stale_status_stays_releasing() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 800;
    m.unit(6, 3, 1, 800);
    m.step(0);
    m.cmd.targetCmm = 0;
    m.step(10);
    TEST_ASSERT_TRUE(m.law.state == IbState::RELEASING);
    m.cmd.ignOn = false;
    m.step(290);                // 0x39D 300 ms old, unit still "heard" within 500 ms
    m.cmd.targetCmm = 1000;
    const IbStepOut o = m.step(10);
    TEST_ASSERT_FALSE(o.state == IbState::BRAKING);
}


// Finding 1: a rod that never moves after acceptance is reported once as "no move" after
// IB_LAW_NO_MOVE_MS, never as reached; the flag stays while it lasts
static void test_blocked_rod_no_move() {
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 800;
    m.unit(6, 3, 1, 0);
    m.step(0);
    int events = 0;
    IbStepOut o;
    for (int t = 10; t <= 5000; t += 10) {
        m.unit(6, 3, 1, 0);
        o = m.step(10);
        TEST_ASSERT_FALSE(o.reached);
        if (o.ev & IB_EV_NO_MOVE) {
            events++;
            TEST_ASSERT_TRUE(t >= IB_LAW_NO_MOVE_MS);
        }
        if (t < IB_LAW_NO_MOVE_MS) TEST_ASSERT_FALSE(o.noMove);
    }
    TEST_ASSERT_EQUAL(1, events);
    TEST_ASSERT_TRUE(o.noMove);
    TEST_ASSERT_TRUE(o.req);              // the request stays; only reported
    m.unit(6, 3, 1, 50);                  // it moves after all
    TEST_ASSERT_FALSE(m.step(10).noMove);
}

// Finding 2: a release that stops responding ends, bounded, with release_slow; one that keeps
// progressing past the nominal timeout is not cut
static void test_release_timeout_scales_and_bounds() {
    // 16 mm at 8 ml/s: nominal ~1 s -> minimum 3 s; stuck at 12 mm -> dropped at 3 s
    Sim m;
    m.toReady();
    m.cmd.targetCmm = 1600;
    m.unit(6, 3, 1, 1600);
    m.step(0);
    m.cmd.targetCmm = 0;
    m.unit(6, 3, 1, 1600);
    m.step(10);
    TEST_ASSERT_TRUE(m.law.state == IbState::RELEASING);
    const uint32_t t0 = m.now;
    int slow = 0;
    uint32_t tSlow = 0;
    for (int i = 0; i < 4000 && !slow; i++) {
        m.unit(6, 3, 1, 1200);
        const IbStepOut o = m.step(10);
        if (o.ev & IB_EV_RELEASE_SLOW) { slow++; tSlow = m.now - t0; }
    }
    TEST_ASSERT_EQUAL(1, slow);
    TEST_ASSERT_TRUE(tSlow >= IB_LAW_RELEASE_MIN_MS && tSlow <= IB_LAW_RELEASE_MIN_MS + 20);
    // Creeping but progressing (0.1 mm per 0.5 s) past its nominal time: kept until the hard cap
    Sim c;
    c.toReady();
    c.cmd.targetCmm = 400;
    c.unit(6, 3, 1, 400);
    c.step(0);
    c.cmd.targetCmm = 0;
    c.unit(6, 3, 1, 400);
    c.step(10);
    const uint32_t c0 = c.now;
    int16_t rod = 400;
    uint32_t tEnd = 0;
    for (int i = 0; i < 4000 && !tEnd; i++) {
        if (i % 50 == 0 && rod > 150) rod -= 10;
        c.unit(6, 3, 1, rod);
        const IbStepOut o = c.step(10);
        if (!o.req) tEnd = c.now - c0;
    }
    TEST_ASSERT_TRUE(tEnd > IB_LAW_RELEASE_MIN_MS);   // not cut at the nominal timeout
    TEST_ASSERT_TRUE(tEnd <= IB_LAW_RELEASE_MAX_MS + 20);
}

static void test_release_timeout_value() {
    TEST_ASSERT_EQUAL(IB_LAW_RELEASE_MIN_MS, ibReleaseTimeoutMs(1600, ibRateUnits(8)));
    const uint32_t t1 = ibReleaseTimeoutMs(1600, ibRateUnits(1));   // ~7.9 s nominal
    TEST_ASSERT_TRUE(t1 > 15000 && t1 <= IB_LAW_RELEASE_MAX_MS);
    TEST_ASSERT_TRUE(ibReleaseTimeoutMs(1600, ibRateUnits(2)) > 8000);
    TEST_ASSERT_EQUAL(IB_LAW_RELEASE_MIN_MS, ibReleaseTimeoutMs(50, ibRateUnits(1)));
    TEST_ASSERT_EQUAL(IB_LAW_RELEASE_MAX_MS, ibReleaseTimeoutMs(-1, ibRateUnits(8)));   // unknown rod
}

// Finding 7: plausibility of a checksum-valid rod value
static void test_rod_plausible() {
    TEST_ASSERT_TRUE(ibRodPlausible(-2));
    TEST_ASSERT_TRUE(ibRodPlausible(-100));
    TEST_ASSERT_FALSE(ibRodPlausible(-101));
    TEST_ASSERT_TRUE(ibRodPlausible(1664));
    TEST_ASSERT_TRUE(ibRodPlausible(4000));
    TEST_ASSERT_FALSE(ibRodPlausible(4001));
    TEST_ASSERT_FALSE(ibRodPlausible(-500));   // raw 0
    TEST_ASSERT_FALSE(ibRodPlausible(5898));   // raw 4095
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_defaults_and_units);
    RUN_TEST(test_percent_to_rod);
    RUN_TEST(test_flow_law);
    RUN_TEST(test_speed_byte);
    RUN_TEST(test_tx_enabled);
    RUN_TEST(test_asleep_and_start);
    RUN_TEST(test_heard_with_ignition_off_starts);
    RUN_TEST(test_asleep_only_ignition_off_and_500ms);
    RUN_TEST(test_status6_self_cycle_stays_ready);
    RUN_TEST(test_lost_at_251_not_250);
    RUN_TEST(test_lost_clears_request_and_no_unit);
    RUN_TEST(test_target_held_while_lost_then_resumes);
    RUN_TEST(test_fault_and_self_recovery);
    RUN_TEST(test_fault_latched_once_after_20s);
    RUN_TEST(test_start_timeout_once);
    RUN_TEST(test_ready_lost_condition_goes_starting);
    RUN_TEST(test_braking_release_handoff);
    RUN_TEST(test_release_timeout);
    RUN_TEST(test_repress_during_release);
    RUN_TEST(test_small_brake_below_handoff);
    RUN_TEST(test_old_rod_sample_holds);
    RUN_TEST(test_rates_every_path);
    RUN_TEST(test_reached_rules);
    RUN_TEST(test_reached_by_stall_in_step);
    RUN_TEST(test_reached_by_runout_zero);
    RUN_TEST(test_unit_drop_after_accept_timeout);
    RUN_TEST(test_normal_accept_not_a_drop);
    RUN_TEST(test_stale_command_holds_target);
    RUN_TEST(test_no_command_yet_not_stale);
    RUN_TEST(test_crank_window);
    RUN_TEST(test_statustext_flapping);
    RUN_TEST(test_statustext_cancel_and_silent);
    RUN_TEST(test_takeup_is_not_reached);
    RUN_TEST(test_stall_after_movement_is_reached);
    RUN_TEST(test_release_slow_rate_keeps_request);
    RUN_TEST(test_ignition_blip_no_lost);
    RUN_TEST(test_ignition_off_while_braking);
    RUN_TEST(test_repress_on_stale_status_stays_releasing);
    RUN_TEST(test_blocked_rod_no_move);
    RUN_TEST(test_release_timeout_scales_and_bounds);
    RUN_TEST(test_release_timeout_value);
    RUN_TEST(test_rod_plausible);
    return UNITY_END();
}
