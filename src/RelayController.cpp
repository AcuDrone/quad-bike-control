#include "RelayController.h"
#include "Debug.h"

// The aztec function masks must be disjoint and, together with the reserved
// relays, cover the whole port exactly once.
static_assert((RELAY_MASK_IGNITION & RELAY_MASK_STARTER) == 0 &&
              (RELAY_MASK_IGNITION & RELAY_MASK_FRONT_LIGHT) == 0 &&
              (RELAY_MASK_STARTER & RELAY_MASK_FRONT_LIGHT) == 0 &&
              ((RELAY_MASK_IGNITION | RELAY_MASK_STARTER | RELAY_MASK_FRONT_LIGHT) & RELAY_MASK_RESERVED) == 0,
              "relay function masks overlap");
static_assert((RELAY_MASK_IGNITION | RELAY_MASK_STARTER | RELAY_MASK_FRONT_LIGHT | RELAY_MASK_RESERVED) == 0xFF,
              "relay masks must cover all eight relays");
// The relay-test lookup must agree with the function masks and the reserved set,
// and map the eight relays onto eight distinct port bits.
static_assert(RelayController::relayIoMask(1) == RELAY_MASK_IGNITION &&
              RelayController::relayIoMask(2) == RELAY_MASK_STARTER &&
              RelayController::relayIoMask(3) == RELAY_MASK_FRONT_LIGHT,
              "relay test lookup disagrees with the function masks");
static_assert((RelayController::relayIoMask(4) | RelayController::relayIoMask(5) |
               RelayController::relayIoMask(6) | RelayController::relayIoMask(7) |
               RelayController::relayIoMask(8)) == RELAY_MASK_RESERVED,
              "relay test lookup disagrees with RELAY_MASK_RESERVED");
static_assert((RelayController::relayIoMask(1) | RelayController::relayIoMask(2) |
               RelayController::relayIoMask(3) | RelayController::relayIoMask(4) |
               RelayController::relayIoMask(5) | RelayController::relayIoMask(6) |
               RelayController::relayIoMask(7) | RelayController::relayIoMask(8)) == 0xFF &&
              RelayController::relayIoMask(0) == 0 && RelayController::relayIoMask(9) == 0,
              "relay test lookup must map relays 1..8 onto all eight port bits");

RelayController::RelayController(TwoWire& bus)
    : expander_(bus, PCA9557_ADDR_RELAYS),
      outputShadow_(0x00),
      initialized_(false),
      faulted_(false),
      starterInhibit_(false),
      allOffEscalation_(false),
      currentIgnitionState_(IgnitionState::OFF),
      frontLightOn_(false),
      wheelLockRejectLogged_(false),
      crankingStartTime_(0),
      isCranking_(false),
      testMask_(0),
      testOnSince_{},
      ignitionHighSince_(0),
      crankArmed_(false) {
}

bool RelayController::begin() {
    initialized_ = false;
    faulted_ = false;
    starterInhibit_ = false;
    allOffEscalation_ = false;
    outputShadow_ = 0x00;
    testMask_ = 0;

    if (!expander_.isPresent()) {
        // Report what the whole PCA9557 address block (0x18-0x1F) looks like on
        // THIS bus, so a mis-strapped board or a board on the wrong connector is
        // obvious from one line. Naming a single "expected" address would
        // misattribute: on I2C2 the on-board mux U2 @0x1C always answers.
        // Bounded (8 zero-length probes) and non-blocking.
        char found[8 * 6 + 1];
        size_t used = 0;
        for (uint8_t addr = 0x18; addr <= 0x1F; addr++) {
            PCA9557Expander probe(expander_.bus(), addr);
            if (probe.isPresent()) {
                used += snprintf(found + used, sizeof(found) - used, " 0x%02X", addr);
            }
        }
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] ERROR: no expander at 0x%02X on this bus; "
            "PCA9557-range devices found:%s\n",
            PCA9557_ADDR_RELAYS, used ? found : " none");
        return false;
    }

    // Drive every output LOW *before* enabling the outputs, so configuring the
    // direction register can never latch a stale high onto a relay coil.
    if (!expander_.writeOutputs(0x00) || !expander_.setDirection(0x00)) {
        Debug::printlnFeature(DebugFeature::RELAY,
            "[RELAY] ERROR: relay expander configuration write failed");
        return false;
    }

    uint8_t readback = 0xFF;
    if (!expander_.readOutputRegister(readback) || readback != 0x00) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] ERROR: all-off write did not verify (read back 0x%02X)\n", readback);
        return false;
    }

    initialized_ = true;

    // Set initial state
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;
    ignitionHighSince_ = 0;
    crankArmed_ = false;
    isCranking_ = false;

    Debug::printfFeature(DebugFeature::RELAY,
        "[RELAY] Initialized on PCA9557 @0x%02X: all 8 relays OFF\n",
        PCA9557_ADDR_RELAYS);

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

        // The ignition/ECU line (Relay_1, RELAY_MASK_IGNITION) is HIGH in
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
    // Aztec: no relay behind the wheel-lock function (RELAY_WHEEL_LOCK_FITTED = 0).
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
}

void RelayController::allOff() {
    Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Fail-safe: All relays OFF");

    // Update state
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
        // Keep pushing the safe state until the expander verifies clean again.
        // While the starter escalation is latched the shadow is already all-off.
        if (allOffEscalation_) {
            outputShadow_ = 0x00;
        }
        writePort(outputShadow_, 1);
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
    // The starter FUNCTION bit is ONLY ever set in CRANKING (a Relay_2 web test is
    // the only other way, ignition OFF only and capped at RELAY_TEST_STARTER_MAX_MS).
    bool starter = (currentIgnitionState_ == IgnitionState::CRANKING);

    // Start from all-off so the reserved relays (RELAY_MASK_RESERVED: the three
    // pending locks Relay_4/5/6 and the spares Relay_7/8) are written off unless the
    // web relay test energizes one.
    uint8_t shadow = 0x00;
    shadow = withMask(shadow, RELAY_MASK_IGNITION,    ignitionLine);   // Relay_1 (IO4)
    shadow = withMask(shadow, RELAY_MASK_STARTER,     starter);        // Relay_2 (IO5)
    shadow = withMask(shadow, RELAY_MASK_FRONT_LIGHT, frontLightOn_);  // Relay_3 (IO6)
    shadow &= (uint8_t)~RELAY_MASK_RESERVED;                           // belt and braces
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
        uint32_t limit = (mask & RELAY_MASK_STARTER) ? RELAY_TEST_STARTER_MAX_MS : RELAY_TEST_TIMEOUT_MS;
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

uint8_t RelayController::getRelayTestBitsFromMask(uint8_t portBits) {
    uint8_t bits = 0;
    for (uint8_t n = 1; n <= 8; n++) {
        if (portBits & relayIoMask(n)) {
            bits |= (uint8_t)(1u << (n - 1));
        }
    }
    return bits;
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
    uint8_t mask = relayIoMask(relayNumber);
    if (mask == 0) {
        reason = "Relay number must be 1-8";
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Test request for relay %u rejected — %s\n", relayNumber, reason);
        return false;
    }
    if (!initialized_ || faulted_) {
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

    // ON: only on a quiet port — ignition OFF, no crank pending, no function relay energized.
    if (currentIgnitionState_ != IgnitionState::OFF || crankArmed_ || isCranking_) {
        reason = "Relay test only allowed with ignition OFF";
    } else if (frontLightOn_) {
        reason = "Relay test refused - front light is on (all relay functions must be off)";
    }
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
        (unsigned long)((mask & RELAY_MASK_STARTER) ? RELAY_TEST_STARTER_MAX_MS : RELAY_TEST_TIMEOUT_MS));
    updateRelays();
    if (!isAvailable()) {
        reason = "Relay write did not verify - all test relays forced off";
        return false;
    }
    return true;
}

bool RelayController::writePort(uint8_t value, uint8_t attempts) {
    if (!initialized_) {
        return false;
    }

    for (uint8_t attempt = 0; attempt < attempts; attempt++) {
        uint8_t readback = 0;
        if (expander_.writeOutputs(value) &&
            expander_.readOutputRegister(readback) &&
            readback == value) {

            if (faulted_) {
                faulted_ = false;
                allOffEscalation_ = false;
                starterInhibit_ = false;
                Debug::printlnFeature(DebugFeature::RELAY,
                    "[RELAY] Relay expander verified clean again — fault cleared");
            }
            return true;
        }
    }

    // Any starter bit set in the failed write escalates (the mask is one relay today,
    // but treat it as a mask so a paralleled starter would still escalate).
    bool starterWasSet = (value & RELAY_MASK_STARTER) != 0;
    if (!faulted_) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] FAULT: port write 0x%02X did not verify after %d attempts%s\n",
            value, RELAY_WRITE_RETRIES,
            starterWasSet ? " — STARTER WAS COMMANDED" : "");
    }
    faulted_ = true;

    // Web test relays go off immediately on a fault: drop them from the shadow and,
    // unless the starter escalation below already forces 0x00, push the reduced byte
    // once (update() keeps retrying it while faulted).
    if (clearRelayTests("relay port write did not verify")) {
        outputShadow_ = computeShadow();
        if (!starterWasSet) {
            expander_.writeOutputs(outputShadow_);
        }
    }

    if (starterWasSet) {
        // Starter escalation: never leave a starter latched. Force the port off
        // immediately, latch ignition OFF, and refuse crank requests until a
        // write verifies cleanly again. update() repeats the all-off write.
        Debug::printlnFeature(DebugFeature::RELAY,
            "[RELAY] Starter escalation: forcing all relays OFF, latching ignition OFF");
        allOffEscalation_ = true;
        starterInhibit_ = true;
        outputShadow_ = 0x00;
        forceAllOffWrite();
        currentIgnitionState_ = IgnitionState::OFF;
        isCranking_ = false;
        crankArmed_ = false;
        ignitionHighSince_ = 0;
    }

    return false;
}

void RelayController::forceAllOffWrite() {
    // Best effort, single transaction — the retry loop lives in update().
    expander_.writeOutputs(0x00);
}
