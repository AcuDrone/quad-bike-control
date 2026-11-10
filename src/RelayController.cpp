#include "RelayController.h"
#include <Preferences.h>
#include "Debug.h"

// Relay-number sets (Constants.h) and both Relay_N -> bit tables (RelayBoardLaw.h) are
// checked at compile time: the sets are disjoint, non-empty and cover all eight relays;
// each table is a permutation of the eight port bits; and the masks mapped through EITHER
// table are disjoint and OR to 0xFF. begin() repeats the mask check at runtime for the
// detected board. The wheel-lock set exists only where the lock is fitted.
#if RELAY_WHEEL_LOCK_FITTED
#define RELAY_SET_WHEEL_LOCK_OR_0_ RELAY_SET_WHEEL_LOCK
#else
#define RELAY_SET_WHEEL_LOCK_OR_0_ 0
#endif
static_assert(relaySetsValid(RELAY_SET_IGNITION, RELAY_SET_STARTER, RELAY_SET_FRONT_LIGHT,
                             RELAY_SET_WHEEL_LOCK_OR_0_, RELAY_SET_RESERVED),
              "relay function sets overlap, are empty or do not cover all eight relays");
static_assert(relayTableIsPermutation(RELAY_TABLE_PCA9557) && relayTableIsPermutation(RELAY_TABLE_XL9535),
              "relay board tables must map relays 1..8 onto all eight port bits");
static_assert(relayMasksValid(relayBuildMasks(RELAY_TABLE_PCA9557, RELAY_SET_IGNITION, RELAY_SET_STARTER,
                                              RELAY_SET_FRONT_LIGHT, RELAY_SET_WHEEL_LOCK_OR_0_,
                                              RELAY_SET_RESERVED)),
              "relay masks on the PCA9557 board overlap or leave a gap");
static_assert(relayMasksValid(relayBuildMasks(RELAY_TABLE_XL9535, RELAY_SET_IGNITION, RELAY_SET_STARTER,
                                              RELAY_SET_FRONT_LIGHT, RELAY_SET_WHEEL_LOCK_OR_0_,
                                              RELAY_SET_RESERVED)),
              "relay masks on the XL9535 board overlap or leave a gap");
#if !RELAY_WHEEL_LOCK_FITTED
// Aztec: the PCA9557 masks must stay exactly what the bench-verified build used.
static_assert(relayPortMask(RELAY_TABLE_PCA9557, RELAY_SET_IGNITION) == 0x10 &&
              relayPortMask(RELAY_TABLE_PCA9557, RELAY_SET_STARTER) == 0x20 &&
              relayPortMask(RELAY_TABLE_PCA9557, RELAY_SET_FRONT_LIGHT) == 0x40 &&
              relayPortMask(RELAY_TABLE_PCA9557, RELAY_SET_RESERVED) == 0x8F,
              "aztec PCA9557 relay masks changed");
#endif
static_assert(PCA9557_ADDR_RELAYS == RELAY_LAW_PCA9557_ADDR &&
              XL9535_ADDR_MIN == RELAY_LAW_XL9535_ADDR_MIN && XL9535_ADDR_MAX == RELAY_LAW_XL9535_ADDR_MAX &&
              XL9535_ADDR_MIN == XL9535Expander::ADDR_MIN && XL9535_ADDR_MAX == XL9535Expander::ADDR_MAX,
              "relay board addresses disagree between Constants.h, RelayBoardLaw.h and the driver");
static_assert(XL9535_ADDR_MAX - XL9535_ADDR_MIN + 1 == 8, "one XL9535 instance per address 0x20..0x27");

RelayController::RelayController(TwoWire& bus)
    : pca_(bus, PCA9557_ADDR_RELAYS),
      xl_{ {bus, 0x20}, {bus, 0x21}, {bus, 0x22}, {bus, 0x23},
           {bus, 0x24}, {bus, 0x25}, {bus, 0x26}, {bus, 0x27} },
      port_(nullptr),
      table_(nullptr),
      boardKind_(RelayBoardKind::NONE),
      boardAddr_(0),
      bootSetting_(RelayBoardSetting::AUTO),
      storedSetting_(RelayBoardSetting::AUTO),
      boardError_(RelayBoardError::NONE),
      maskIgnition_(0),
      maskStarter_(0),
      maskFrontLight_(0),
#if RELAY_WHEEL_LOCK_FITTED
      maskWheelLock_(0),
#endif
      maskReserved_(0),
      outputShadow_(0x00),
      initialized_(false),
      faulted_(false),
      starterInhibit_(false),
      allOffEscalation_(false),
      initPending_(false),
      lastRecoverMs_(0),
      recoverFailLogged_(false),
      currentIgnitionState_(IgnitionState::OFF),
      frontLightOn_(false),
#if RELAY_WHEEL_LOCK_FITTED
      wheelLockOn_(false),
#else
      wheelLockRejectLogged_(false),
#endif
      crankingStartTime_(0),
      isCranking_(false),
      testMask_(0),
      testOnSince_{},
      ignitionHighSince_(0),
      crankArmed_(false) {
}

RelayBoardSetting RelayController::loadBoardSetting() {
    // Read-only open: fails (→ auto) while the namespace has never been written.
    Preferences prefs;
    uint8_t raw = (uint8_t)RelayBoardSetting::AUTO;
    if (prefs.begin(RELAY_BOARD_NVS_NS, true)) {
        raw = prefs.getUChar(RELAY_BOARD_NVS_KEY, (uint8_t)RelayBoardSetting::AUTO);
        prefs.end();
    }
    bool valid = true;
    RelayBoardSetting setting = relayBoardSettingFromRaw(raw, valid);
    if (!valid) {
        // Logged and treated as auto; deliberately NOT rewritten (a newer firmware may own it).
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] WARNING: stored relay board setting %u is out of range — using auto\n",
            (unsigned)raw);
    }
    return setting;
}

bool RelayController::saveBoardSetting(RelayBoardSetting setting) {
    // Persist first; the RAM copy (telemetry relay_board_cfg) changes only once NVS holds it.
    // One key in the controller-owned "relay" namespace — never clear() it.
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(RELAY_BOARD_NVS_NS, false)) {
        ok = prefs.putUChar(RELAY_BOARD_NVS_KEY, (uint8_t)setting) == sizeof(uint8_t);
        prefs.end();
    }
    if (!ok) {
        Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] ERROR: relay board setting not saved");
        return false;
    }
    storedSetting_ = setting;
    Debug::printfFeature(DebugFeature::RELAY,
        "[RELAY] Relay board setting stored: %s (active: %s @0x%02X, applies after reboot)\n",
        relayBoardSettingToken(setting), relayBoardKindName(boardKind_), (unsigned)getBoardAddress());
    return true;
}

void RelayController::logDetectionFault(uint16_t ackMask) {
    static const char* const reasons[] = {
        "", "both relay boards answer (0x1F and an XL9535 address) — select one in the web UI",
        "no relay board answers at 0x1F or 0x20..0x27",
        "more than one XL9535 address answers — fix the A0..A2 pads",
        "the selected relay board does not answer (setting does not match the bus)",
        "initialization failed"
    };
    const uint8_t idx = (uint8_t)boardError_;
    Debug::printfFeature(DebugFeature::RELAY,
        "[RELAY] ERROR: relay board detection failed (%s, setting %s, probe mask 0x%03X): %s — "
        "every relay stays OFF, nothing written; reboot after fixing\n",
        relayBoardErrorToken(boardError_), relayBoardSettingToken(bootSetting_), (unsigned)ackMask,
        idx < sizeof(reasons) / sizeof(reasons[0]) ? reasons[idx] : "?");

    // Report what the PCA9557 block (0x18-0x1F) and the XL9535 block (0x20-0x27) look like
    // on THIS bus, so a mis-strapped board or a board on the wrong connector is obvious from
    // one line. 0x1A / 0x1C are the on-board expanders on I2C2 — listed, never taken for a
    // relay board. Bounded (16 zero-length probes) and non-blocking.
    char found[16 * 6 + 1];
    size_t used = 0;
    const uint8_t lo = pca_.probeRangeMin() < RELAY_LAW_SCAN_ADDR_MIN ? pca_.probeRangeMin() : RELAY_LAW_SCAN_ADDR_MIN;
    const uint8_t hi = xl_[0].probeRangeMax() > RELAY_LAW_SCAN_ADDR_MAX ? xl_[0].probeRangeMax() : RELAY_LAW_SCAN_ADDR_MAX;
    for (uint8_t addr = lo; addr <= hi && used < sizeof(found); addr++) {
        PCA9557Expander probe(pca_.bus(), addr);   // zero-length write only
        if (probe.isPresent()) {
            used += snprintf(found + used, sizeof(found) - used, " 0x%02X", addr);
        }
    }
    Debug::printfFeature(DebugFeature::RELAY,
        "[RELAY] I2C scan 0x%02X-0x%02X (0x1A/0x1C = on-board expanders, not relay boards):%s\n",
        lo, hi, used ? found : " none");
}

bool RelayController::begin() {
    initialized_ = false;
    faulted_ = false;
    starterInhibit_ = false;
    allOffEscalation_ = false;
    initPending_ = false;
    recoverFailLogged_ = false;
    outputShadow_ = 0x00;
    testMask_ = 0;
    port_ = nullptr;
    table_ = nullptr;
    boardKind_ = RelayBoardKind::NONE;
    boardAddr_ = 0;
    boardError_ = RelayBoardError::NONE;

    // Tracked state starts safe whatever happens below.
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;
#if RELAY_WHEEL_LOCK_FITTED
    wheelLockOn_ = false;    // the port is written all-off below, lock included
#endif
    ignitionHighSince_ = 0;
    crankArmed_ = false;
    isCranking_ = false;

    // 1. Setting (absent / out-of-range → auto)
    bootSetting_ = loadBoardSetting();
    storedSetting_ = bootSetting_;

    // 2. Detection: nine zero-length probes (0x1F, 0x20..0x27) — nothing is written.
    uint16_t ack = 0;
    if (pca_.isPresent()) {
        ack |= RELAY_ACK_PCA9557;
    }
    for (uint8_t i = 0; i < 8; i++) {
        if (xl_[i].isPresent()) {
            ack |= relayAckXl9535(xl_[i].address());
        }
    }
    const RelayBoardDecision d = relayBoardDecide(ack, bootSetting_);
    if (d.error != RelayBoardError::NONE) {
        // Never a guess: no write to any device, every setter a logging no-op, no retry.
        boardError_ = d.error;
        logDetectionFault(ack);
        return false;
    }

    // 3. Latch the board and build its masks.
    IRelayPort* port = nullptr;
    if (d.kind == RelayBoardKind::PCA9557) {
        port = &pca_;
        table_ = &RELAY_TABLE_PCA9557;
    } else {
        port = &xl_[d.address - XL9535_ADDR_MIN];
        table_ = &RELAY_TABLE_XL9535;
    }
    const RelayPortMasks m = relayBuildMasks(*table_, RELAY_SET_IGNITION, RELAY_SET_STARTER,
                                             RELAY_SET_FRONT_LIGHT, RELAY_SET_WHEEL_LOCK_OR_0_,
                                             RELAY_SET_RESERVED);
    if (!relayMasksValid(m)) {
        // Unreachable while the static_asserts above hold; refuse to drive the port anyway.
        Debug::printlnFeature(DebugFeature::RELAY,
            "[RELAY] ERROR: relay masks overlap or leave a gap — relay port NOT initialized");
        table_ = nullptr;
        boardError_ = RelayBoardError::INIT_FAILED;
        return false;
    }
    maskIgnition_ = m.ignition;
    maskStarter_ = m.starter;
    maskFrontLight_ = m.frontLight;
#if RELAY_WHEEL_LOCK_FITTED
    maskWheelLock_ = m.wheelLock;
#endif
    maskReserved_ = m.reserved;
    port_ = port;
    boardKind_ = d.kind;
    boardAddr_ = d.address;

    Debug::printfFeature(DebugFeature::RELAY,
        "[RELAY] Relay board: %s @0x%02X (setting %s); masks ign 0x%02X start 0x%02X light 0x%02X "
        "reserved 0x%02X\n",
        port_->name(), boardAddr_, relayBoardSettingToken(bootSetting_),
        maskIgnition_, maskStarter_, maskFrontLight_, maskReserved_);

    // 4. Hardware-safe all-off init (output → polarity → config → pin verify).
    initialized_ = true;   // the board is latched from here on, even if the init fails
    const char* step = nullptr;
    if (!port_->initAllOff(&step)) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] ERROR: %s @0x%02X all-off init failed at step '%s' — relays unavailable, "
            "retrying every %u ms\n",
            port_->name(), boardAddr_, step ? step : "?", (unsigned)RELAY_RECOVER_MS);
        faulted_ = true;
        initPending_ = true;
        recoverFailLogged_ = true;     // this episode is already logged
        lastRecoverMs_ = millis();
        boardError_ = RelayBoardError::INIT_FAILED;
        return false;
    }

    Debug::printfFeature(DebugFeature::RELAY,
        "[RELAY] Initialized on %s @0x%02X: all 8 relays OFF (pins verified)\n",
        port_->name(), boardAddr_);
    return true;
}

void RelayController::setIgnitionState(IgnitionState state) {
    if (!initialized_ || faulted_) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Ignition command %s ignored — relay expander unavailable\n",
            getRelayIgnitionStateName(state));
        return;
    }

    if (currentIgnitionState_ != state) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Ignition: %s -> %s\n",
                     getRelayIgnitionStateName(currentIgnitionState_),
                     getRelayIgnitionStateName(state));

        if (state == IgnitionState::CRANKING && starterInhibit_) {
            Debug::printlnFeature(DebugFeature::RELAY,
                "[RELAY] Crank refused — starter fault latched until a relay write verifies");
            return;
        }

        // A real ignition command takes the port over from the web relay test: every
        // test bit is dropped before the function relays are written.
        if (state != IgnitionState::OFF) {
            clearRelayTests("ignition command takes over");
        }

        // The ignition/ECU line (ignition mask) is HIGH in
        // ACC/IGNITION/CRANKING, LOW only in OFF. Start the pre-crank dwell clock on the
        // LOW->HIGH edge; reset it only on OFF (jitter between ACC and IGNITION keeps the
        // ignition line powered, so it must NOT reset).
        bool ignitionWasHigh = (currentIgnitionState_ != IgnitionState::OFF);
        bool ignitionWillBeHigh = (state != IgnitionState::OFF);
        if (!ignitionWasHigh && ignitionWillBeHigh) {
            ignitionHighSince_ = millis();
        } else if (!ignitionWillBeHigh) {
            ignitionHighSince_ = 0;
        }

        // If entering CRANKING state, start timer
        if (state == IgnitionState::CRANKING) {
            crankingStartTime_ = millis();
            isCranking_ = true;
            Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Cranking started - monitoring RPM and timeout");
        } else if (currentIgnitionState_ == IgnitionState::CRANKING) {
            // Exiting CRANKING state
            isCranking_ = false;
        }

        // Moving to OFF or ACC cancels any pending or active crank, and re-arms the
        // retry gesture (a subsequent fresh IGNITION selection will crank again).
        if (state == IgnitionState::OFF || state == IgnitionState::ACC) {
            crankArmed_ = false;
        }

        currentIgnitionState_ = state;
        updateRelays();
    }
}

void RelayController::requestCrank() {
    if (!initialized_ || faulted_ || starterInhibit_) {
        Debug::printlnFeature(DebugFeature::RELAY,
            "[RELAY] Crank request ignored — relay expander unavailable or starter fault latched");
        return;
    }
    // Bring up the ignition/ECU line first (starts the dwell clock) if it was off.
    if (currentIgnitionState_ == IgnitionState::OFF) {
        setIgnitionState(IgnitionState::ACC);
    }
    // Don't disturb a crank already in progress.
    if (isCranking_) {
        return;
    }
    crankArmed_ = true;
    Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Crank armed - waiting for ignition-line pre-crank dwell");
}

void RelayController::setFrontLight(bool on) {
    if (!initialized_ || faulted_) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Front light command (%s) ignored — relay expander unavailable\n",
            on ? "ON" : "OFF");
        return;
    }
    if (frontLightOn_ != on) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Front Light: %s\n", on ? "ON" : "OFF");
        if (on) {
            clearRelayTests("front light command takes over");
        }
        frontLightOn_ = on;
        updateRelays();
    }
}

bool RelayController::setWheelLock(bool locked) {
#if RELAY_WHEEL_LOCK_FITTED
    // Quad bike: the lock drives the wheel-lock mask (RELAY_SET_WHEEL_LOCK: Relay_5 + Relay_6).
    if (!initialized_ || faulted_) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Wheel lock command (%s) ignored — relay expander unavailable\n",
            locked ? "LOCKED" : "UNLOCKED");
        return false;
    }
    if (wheelLockOn_ != locked) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Front Wheel Lock: %s\n", locked ? "LOCKED" : "UNLOCKED");
        if (locked) {
            clearRelayTests("wheel lock command takes over");
        }
        wheelLockOn_ = locked;
        updateRelays();
    }
    return true;
#else
    // Not fitted (aztec): no relay behind the wheel-lock function.
    // Reject rather than "succeed" silently; the port is never touched, so the
    // reserved lock relays (Relay_4..Relay_6) can't be energized through this path.
    // Release requests arrive every control loop from the MAVLink servo channel and
    // are trivially satisfied (the lock is never engaged).
    if (!locked) {
        return true;
    }
    if (!wheelLockRejectLogged_) {
        wheelLockRejectLogged_ = true;
        Debug::printlnFeature(DebugFeature::RELAY,
            "[RELAY] Wheel lock request rejected — front-wheel lock not fitted on this vehicle "
            "(further requests ignored silently)");
    }
    return false;
#endif
}

void RelayController::allOff() {
    Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Fail-safe: All relays OFF");

    // Update state (a fitted wheel lock deliberately holds its last state)
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;
    isCranking_ = false;
    ignitionHighSince_ = 0;      // OFF resets the pre-crank dwell
    crankArmed_ = false;   // abort any pending crank
    clearRelayTests("fail-safe all-off");

    if (!initialized_) {
        Debug::printlnFeature(DebugFeature::RELAY,
            "[RELAY] Fail-safe write skipped — relay expander never initialized");
        return;
    }

    // The fail-safe direction is always attempted, even while faulted.
    updateRelays();
}

void RelayController::update(uint16_t engineRpm) {
    if (!initialized_) {
        return;  // degraded: the rest of the vehicle keeps running
    }

    if (faulted_) {
        // writePort() already dropped the test bits when the fault was raised; this
        // keeps it so if anything set one since.
        if (clearRelayTests("relay expander faulted")) {
            outputShadow_ = computeShadow();
        }
        // While the starter escalation is latched the shadow is already all-off.
        if (allOffEscalation_) {
            outputShadow_ = 0x00;
        }
        // Every RELAY_RECOVER_MS the single attempt is replaced by a re-init of the port
        // (a reset expander has lost its configuration — XL9535 VIN brown-out).
        uint32_t now = millis();
        if ((now - lastRecoverMs_) >= RELAY_RECOVER_MS) {
            lastRecoverMs_ = now;
            attemptRecovery();
            return;
        }
        // Otherwise keep pushing the safe state, one attempt per call, until it verifies.
        // Not while the boot init is pending: an unconfigured port is touched only by the
        // ordered re-init above.
        if (!initPending_) {
            writePort(outputShadow_, 1);
        }
        return;  // no crank logic while the port cannot be trusted
    }

    // Web relay test auto-off (RELAY_TEST_TIMEOUT_MS, starter RELAY_TEST_STARTER_MAX_MS).
    expireRelayTests();

    // Fire a pending (armed) crank once the ignition-line pre-crank dwell is satisfied.
    if (crankArmed_ && !isCranking_) {
        if (engineRpm >= ENGINE_RUNNING_RPM_THRESHOLD) {
            // Engine already running — suppress the crank, settle into IGNITION.
            crankArmed_ = false;
            Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Crank suppressed - engine already running (RPM: %d)\n", engineRpm);
            setIgnitionState(IgnitionState::IGNITION);
            return;
        }
        if (ignitionHighSince_ != 0 && (millis() - ignitionHighSince_) >= ACC_PRECRANK_DWELL_MS) {
            crankArmed_ = false;
            Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Pre-crank dwell satisfied - engaging starter");
            setIgnitionState(IgnitionState::CRANKING);  // sets isCranking_ / crankingStartTime_
        }
        // else: keep holding ACC (ignition mask high, starter mask low) until the dwell elapses
    }

    // Only monitor if currently cranking
    if (!isCranking_ || currentIgnitionState_ != IgnitionState::CRANKING) {
        return;
    }

    uint32_t crankingDuration = millis() - crankingStartTime_;

    // Check if engine has started (RPM above threshold)
    if (engineRpm >= ENGINE_RUNNING_RPM_THRESHOLD) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Engine started! RPM: %d - stopping cranking\n", engineRpm);
        setIgnitionState(IgnitionState::IGNITION);
        return;
    }

    // Check if cranking timeout exceeded
    if (crankingDuration >= CRANKING_TIMEOUT) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Cranking timeout (%lu ms) - stopping cranking\n", crankingDuration);
        Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] WARNING: Engine did not start");
        setIgnitionState(IgnitionState::IGNITION);
        return;
    }
}

uint8_t RelayController::computeShadow() const {
    bool ignitionLine = (currentIgnitionState_ != IgnitionState::OFF);
    // The starter FUNCTION bit is ONLY ever set in CRANKING (a web test of the starter
    // relay is the only other way, ignition OFF only and capped at RELAY_TEST_STARTER_MAX_MS).
    bool starter = (currentIgnitionState_ == IgnitionState::CRANKING);

    // Start from all-off so the reserved relays (RELAY_SET_RESERVED: the spares, and
    // on aztec the three pending locks) are written off unless the web relay test
    // energizes one. Relay numbers per vehicle: see RelayController.h.
    uint8_t shadow = 0x00;
    shadow = withMask(shadow, maskIgnition_,   ignitionLine);
    shadow = withMask(shadow, maskStarter_,    starter);
    shadow = withMask(shadow, maskFrontLight_, frontLightOn_);
#if RELAY_WHEEL_LOCK_FITTED
    shadow = withMask(shadow, maskWheelLock_,  wheelLockOn_);
#endif
    shadow &= (uint8_t)~maskReserved_;                                 // belt and braces
    shadow |= testMask_;                                               // web relay test (ignition OFF only)
    return shadow;
}

void RelayController::updateRelays() {
    uint8_t shadow = computeShadow();
    outputShadow_ = shadow;
    writePort(shadow);
}

bool RelayController::clearRelayTests(const char* why) {
    if (testMask_ == 0) {
        return false;
    }
    Debug::printfFeature(DebugFeature::RELAY,
        "[RELAY] Test relays forced OFF (relays 0x%02X) — %s\n",
        (unsigned)getRelayTestBitsFromMask(testMask_), why);
    testMask_ = 0;
    return true;
}

void RelayController::expireRelayTests() {
    if (testMask_ == 0) {
        return;
    }
    uint32_t now = millis();
    bool changed = false;
    for (uint8_t n = 1; n <= 8; n++) {
        uint8_t mask = relayIoMask(n);
        if ((testMask_ & mask) == 0) {
            continue;
        }
        uint32_t limit = (mask & maskStarter_) ? RELAY_TEST_STARTER_MAX_MS : RELAY_TEST_TIMEOUT_MS;
        if ((now - testOnSince_[n - 1]) >= limit) {
            testMask_ &= (uint8_t)~mask;
            changed = true;
            Debug::printfFeature(DebugFeature::RELAY,
                "[RELAY] Test Relay_%u auto-OFF after %lu ms\n", n, (unsigned long)limit);
        }
    }
    if (changed) {
        updateRelays();
    }
}

uint8_t RelayController::getRelayTestBitsFromMask(uint8_t portBits) const {
    // Through the active board's table, so the bitfield means the same relays on either board.
    return table_ ? relaySetFromPort(*table_, portBits) : (uint8_t)0;
}

uint8_t RelayController::getRelayTestBits() const {
    if (!isAvailable()) {
        return 0;
    }
    // From the shadow (what was last written and verified), limited to the test bits
    // so a function relay is never reported as a test relay.
    return getRelayTestBitsFromMask(outputShadow_ & testMask_);
}

bool RelayController::setRelayTest(uint8_t relayNumber, bool on, const char*& reason) {
    reason = nullptr;
    if (relayNumber < 1 || relayNumber > 8) {
        reason = "Relay number must be 1-8";
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Test request for relay %u rejected — %s\n", relayNumber, reason);
        return false;
    }
    // 0 while no board was detected — refused just below as "unavailable".
    uint8_t mask = relayIoMask(relayNumber);
    if (!initialized_ || faulted_ || mask == 0) {
        reason = "Relay expander unavailable";
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Test Relay_%u %s ignored — relay expander unavailable\n",
            relayNumber, on ? "ON" : "OFF");
        return false;
    }

    if (!on) {
        if ((testMask_ & mask) == 0) {
            return true;   // already off
        }
        testMask_ &= (uint8_t)~mask;
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Test Relay_%u OFF\n", relayNumber);
        updateRelays();
        return true;       // a failed write faults the driver, which drops every test bit
    }

    // ON: only on a quiet port — ignition OFF, no crank pending, no function relay
    // energized (light off, wheel lock released where fitted).
    if (currentIgnitionState_ != IgnitionState::OFF || crankArmed_ || isCranking_) {
        reason = "Relay test only allowed with ignition OFF";
    } else if (frontLightOn_) {
        reason = "Relay test refused - front light is on (all relay functions must be off)";
    }
#if RELAY_WHEEL_LOCK_FITTED
    else if (wheelLockOn_) {
        reason = "Relay test refused - wheel lock is engaged (all relay functions must be off)";
    }
#endif
    if (reason) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Test Relay_%u ON rejected — %s\n", relayNumber, reason);
        return false;
    }
    if (testMask_ & mask) {
        return true;       // already on; the auto-off timer keeps running from the first ON
    }

    testMask_ |= mask;
    testOnSince_[relayNumber - 1] = millis();
    Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Test Relay_%u ON (auto-off in %lu ms)\n",
        relayNumber,
        (unsigned long)((mask & maskStarter_) ? RELAY_TEST_STARTER_MAX_MS : RELAY_TEST_TIMEOUT_MS));
    updateRelays();
    if (!isAvailable()) {
        reason = "Relay write did not verify - all test relays forced off";
        return false;
    }
    return true;
}

bool RelayController::writePort(uint8_t value, uint8_t attempts) {
    if (!initialized_ || port_ == nullptr || initPending_) {
        // No board, or the boot init has not verified yet: only attemptRecovery() may
        // touch an unconfigured port (its config may still be all-inputs).
        return false;
    }

    uint8_t pins = 0;
    for (uint8_t attempt = 0; attempt < attempts; attempt++) {
        // Verify on the PINS (input register), not the latch: a reset expander (config
        // back to inputs, relays dropped) keeps a latch that could still read back fine.
        if (port_->writeOutputs(value) &&
            port_->readPins(pins) &&
            pins == value) {

            if (faulted_) {
                faulted_ = false;
                allOffEscalation_ = false;
                starterInhibit_ = false;
                recoverFailLogged_ = false;
                Debug::printlnFeature(DebugFeature::RELAY,
                    "[RELAY] Relay expander verified clean again — fault cleared");
            }
            return true;
        }
    }

    // Any starter bit set in the failed write escalates (the mask is one relay today,
    // but treat it as a mask so a paralleled starter would still escalate).
    bool starterWasSet = (value & maskStarter_) != 0;
    if (!faulted_) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] FAULT: port write 0x%02X did not verify on the pins after %d attempts "
            "(last pins 0x%02X)%s\n",
            value, (int)attempts, pins,
            starterWasSet ? " — STARTER WAS COMMANDED" : "");
        lastRecoverMs_ = millis();    // first re-init RELAY_RECOVER_MS after the raise
        recoverFailLogged_ = false;   // new fault episode
    }
    faulted_ = true;

    // Web test relays go off immediately on a fault: drop them from the shadow and,
    // unless the starter escalation below already forces 0x00, push the reduced byte
    // once (update() keeps retrying it while faulted).
    if (clearRelayTests("relay port write did not verify")) {
        outputShadow_ = computeShadow();
        if (!starterWasSet) {
            port_->writeOutputs(outputShadow_);
        }
    }

    if (starterWasSet) {
        escalateStarter();
    }

    return false;
}

void RelayController::escalateStarter() {
    // Starter escalation: never leave a starter latched. Force the port off
    // immediately, latch ignition OFF, and refuse crank requests until a
    // write verifies cleanly again. update() repeats the all-off write.
    Debug::printlnFeature(DebugFeature::RELAY,
        "[RELAY] Starter escalation: forcing all relays OFF, latching ignition OFF");
    allOffEscalation_ = true;
    starterInhibit_ = true;
    clearRelayTests("starter escalation");
    outputShadow_ = 0x00;
    forceAllOffWrite();
    currentIgnitionState_ = IgnitionState::OFF;
    isCranking_ = false;
    crankArmed_ = false;
    ignitionHighSince_ = 0;
}

void RelayController::attemptRecovery() {
    if (port_ == nullptr) {
        return;   // detection faults are never retried
    }
    // The starter bit is NEVER seeded into the latch by a recovery: if the shadow holds it,
    // the starter escalation applies first (all-off, ignition OFF, crank inhibit) and the
    // recovery seeds that all-off state.
    if (outputShadow_ & maskStarter_) {
        escalateStarter();
    }
    // Seed = the current shadow (amended design): a fault from a transient bus error that
    // did not reset the expander must not blip an energized ignition relay. The latch is
    // still written before the config register, so the XL9535 all-on glitch cannot occur.
    const uint8_t seed = outputShadow_;
    const char* step = nullptr;
    if (!port_->initPort(seed, &step)) {
        if (!recoverFailLogged_) {
            recoverFailLogged_ = true;
            Debug::printfFeature(DebugFeature::RELAY,
                "[RELAY] Recovery: %s @0x%02X re-init (seed 0x%02X) failed at step '%s' — "
                "retrying every %u ms (logged once per fault)\n",
                port_->name(), boardAddr_, seed, step ? step : "?", (unsigned)RELAY_RECOVER_MS);
        }
        return;
    }
    // The port is configured again: one verified write of the shadow clears the fault.
    initPending_ = false;
    if (writePort(outputShadow_, 1)) {
        if (boardError_ == RelayBoardError::INIT_FAILED) {
            boardError_ = RelayBoardError::NONE;
        }
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Recovery: %s @0x%02X re-initialized, shadow 0x%02X verified on the pins\n",
            port_->name(), boardAddr_, outputShadow_);
    }
}

void RelayController::forceAllOffWrite() {
    // Best effort, single transaction — the retry loop lives in update().
    if (port_ != nullptr && !initPending_) {
        port_->writeOutputs(0x00);
    }
}
