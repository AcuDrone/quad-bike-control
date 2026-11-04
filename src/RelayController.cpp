#include "RelayController.h"
#include "Debug.h"

// The function masks must be pairwise disjoint, disjoint from the reserved relays,
// and together with them cover the four relays of this board exactly once.
#define RELAY_FUNCTION_MASKS_ (RELAY_MASK_IGNITION | RELAY_MASK_STARTER | RELAY_MASK_FRONT_LIGHT | RELAY_MASK_WHEEL_LOCK)
static_assert(RELAY_WHEEL_LOCK_FITTED == 1,
              "the DevKitC-1 board always drives the front-wheel lock on Relay 4 (GPIO39)");
static_assert((RELAY_MASK_IGNITION & RELAY_MASK_STARTER) == 0 &&
              (RELAY_MASK_IGNITION & RELAY_MASK_FRONT_LIGHT) == 0 &&
              (RELAY_MASK_IGNITION & RELAY_MASK_WHEEL_LOCK) == 0 &&
              (RELAY_MASK_STARTER & RELAY_MASK_FRONT_LIGHT) == 0 &&
              (RELAY_MASK_STARTER & RELAY_MASK_WHEEL_LOCK) == 0 &&
              (RELAY_MASK_FRONT_LIGHT & RELAY_MASK_WHEEL_LOCK) == 0 &&
              (RELAY_FUNCTION_MASKS_ & RELAY_MASK_RESERVED) == 0,
              "relay function masks overlap");
static_assert((RELAY_FUNCTION_MASKS_ | RELAY_MASK_RESERVED) == ((1u << RELAY_COUNT) - 1),
              "relay masks must cover all RELAY_COUNT relays");
// The relay-test lookup must agree with the function masks: Relay 1 ignition,
// Relay 2 starter, Relay 3 light, Relay 4 wheel lock.
static_assert(RelayController::relayIoMask(1) == RELAY_MASK_IGNITION &&
              RelayController::relayIoMask(2) == RELAY_MASK_STARTER &&
              RelayController::relayIoMask(3) == RELAY_MASK_FRONT_LIGHT &&
              RelayController::relayIoMask(4) == RELAY_MASK_WHEEL_LOCK &&
              RelayController::relayIoMask(0) == 0 &&
              RelayController::relayIoMask(RELAY_COUNT + 1) == 0,
              "relay test lookup disagrees with the function masks");
#undef RELAY_FUNCTION_MASKS_

RelayController::RelayController()
    : outputShadow_(0x00),
      initialized_(false),
      currentIgnitionState_(IgnitionState::OFF),
      frontLightOn_(false),
      wheelLockOn_(false),
      crankingStartTime_(0),
      isCranking_(false),
      testMask_(0),
      testOnSince_{},
      ignitionHighSince_(0),
      crankArmed_(false) {
}

bool RelayController::begin() {
    // Drive every output LOW before enabling it, so configuring the pin as an
    // output can never latch a stale high onto a relay coil.
    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
    digitalWrite(PIN_RELAY3, LOW);
    digitalWrite(PIN_WHEEL_LOCK, LOW);
    pinMode(PIN_RELAY1, OUTPUT);
    pinMode(PIN_RELAY2, OUTPUT);
    pinMode(PIN_RELAY3, OUTPUT);
    pinMode(PIN_WHEEL_LOCK, OUTPUT);

    outputShadow_ = 0x00;
    testMask_ = 0;
    writeOutputs(outputShadow_);

    // Set initial state
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;
    wheelLockOn_ = false;
    ignitionHighSince_ = 0;
    crankArmed_ = false;
    isCranking_ = false;
    initialized_ = true;

    Debug::printfFeature(DebugFeature::RELAY,
        "[RELAY] Initialized (direct GPIO): RELAY1=%d, RELAY2=%d, RELAY3=%d, WHEEL_LOCK=%d, all OFF\n",
        (int)PIN_RELAY1, (int)PIN_RELAY2, (int)PIN_RELAY3, (int)PIN_WHEEL_LOCK);

    return true;
}

void RelayController::setIgnitionState(IgnitionState state) {
    if (currentIgnitionState_ != state) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Ignition: %s -> %s\n",
                     getRelayIgnitionStateName(currentIgnitionState_),
                     getRelayIgnitionStateName(state));

        // A real ignition command takes the outputs over from the web relay test: every
        // test bit is dropped before the function relays are written.
        if (state != IgnitionState::OFF) {
            clearRelayTests("ignition command takes over");
        }

        // The ignition/ECU line (RELAY_MASK_IGNITION) is HIGH in
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
    if (!initialized_) {
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
}

void RelayController::allOff() {
    Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Fail-safe: All relays OFF");

    // Update state (the wheel lock deliberately holds its last state)
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;
    isCranking_ = false;
    ignitionHighSince_ = 0;      // OFF resets the pre-crank dwell
    crankArmed_ = false;   // abort any pending crank
    clearRelayTests("fail-safe all-off");

    updateRelays();
}

void RelayController::update(uint16_t engineRpm) {
    if (!initialized_) {
        return;
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
        // else: keep holding ACC (ignition relay high, starter relay low) until the dwell elapses
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

    uint8_t shadow = 0x00;
    shadow = withMask(shadow, RELAY_MASK_IGNITION,    ignitionLine);
    shadow = withMask(shadow, RELAY_MASK_STARTER,     starter);
    shadow = withMask(shadow, RELAY_MASK_FRONT_LIGHT, frontLightOn_);
    shadow = withMask(shadow, RELAY_MASK_WHEEL_LOCK,  wheelLockOn_);
    shadow |= testMask_;                                               // web relay test (ignition OFF only)
    return shadow;
}

void RelayController::updateRelays() {
    if (!initialized_) {
        return;
    }
    outputShadow_ = computeShadow();
    writeOutputs(outputShadow_);
}

void RelayController::writeOutputs(uint8_t value) {
    digitalWrite(PIN_RELAY1,     (value & relayIoMask(1)) ? HIGH : LOW);
    digitalWrite(PIN_RELAY2,     (value & relayIoMask(2)) ? HIGH : LOW);
    digitalWrite(PIN_RELAY3,     (value & relayIoMask(3)) ? HIGH : LOW);
    digitalWrite(PIN_WHEEL_LOCK, (value & relayIoMask(4)) ? HIGH : LOW);
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
    for (uint8_t n = 1; n <= RELAY_COUNT; n++) {
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

uint8_t RelayController::getRelayTestBitsFromMask(uint8_t bits) {
    uint8_t out = 0;
    for (uint8_t n = 1; n <= RELAY_COUNT; n++) {
        if (bits & relayIoMask(n)) {
            out |= (uint8_t)(1u << (n - 1));
        }
    }
    return out;
}

uint8_t RelayController::getRelayTestBits() const {
    if (!initialized_) {
        return 0;
    }
    // From the shadow (what was last written), limited to the test bits so a
    // function relay is never reported as a test relay.
    return getRelayTestBitsFromMask(outputShadow_ & testMask_);
}

bool RelayController::setRelayTest(uint8_t relayNumber, bool on, const char*& reason) {
    reason = nullptr;
    uint8_t mask = relayIoMask(relayNumber);
    if (mask == 0) {
        reason = "Relay number must be 1-4";
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Test request for relay %u rejected — %s\n", relayNumber, reason);
        return false;
    }
    if (!initialized_) {
        reason = "Relay outputs not initialized";
        return false;
    }

    if (!on) {
        if ((testMask_ & mask) == 0) {
            return true;   // already off
        }
        testMask_ &= (uint8_t)~mask;
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Test Relay_%u OFF\n", relayNumber);
        updateRelays();
        return true;
    }

    // ON: only on a quiet board — ignition OFF, no crank pending, no function relay
    // energized (light off, wheel lock released).
    if (currentIgnitionState_ != IgnitionState::OFF || crankArmed_ || isCranking_) {
        reason = "Relay test only allowed with ignition OFF";
    } else if (frontLightOn_) {
        reason = "Relay test refused - front light is on (all relay functions must be off)";
    } else if (wheelLockOn_) {
        reason = "Relay test refused - wheel lock is engaged (all relay functions must be off)";
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
    return true;
}
