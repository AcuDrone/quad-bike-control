#include "RelayController.h"
#include "Debug.h"

RelayController::RelayController(TwoWire& bus)
    : expander_(bus, PCA9557_ADDR_RELAYS),
      outputShadow_(0x00),
      initialized_(false),
      faulted_(false),
      starterInhibit_(false),
      allOffEscalation_(false),
      currentIgnitionState_(IgnitionState::OFF),
      frontLightOn_(false),
      wheelLockOn_(false),
      crankingStartTime_(0),
      isCranking_(false),
      r1HighSince_(0),
      crankArmed_(false) {
}

bool RelayController::begin() {
    initialized_ = false;
    faulted_ = false;
    starterInhibit_ = false;
    allOffEscalation_ = false;
    outputShadow_ = 0x00;

    if (!expander_.isPresent()) {
        // The single most likely assembly mistake is the relay board strapped to
        // the on-board mux expander's address, so report that case distinctly.
        PCA9557Expander muxProbe(expander_.bus(), PCA9557_ADDR_MUX_RESERVED);
        if (muxProbe.isPresent()) {
            Debug::printfFeature(DebugFeature::RELAY,
                "[RELAY] ERROR: nothing at 0x%02X but a PCA9557 answers at 0x%02X — "
                "the relay board is strapped to the reserved mux address; "
                "strap A2A1A0 = 000 (0x%02X)\n",
                PCA9557_ADDR_RELAYS, PCA9557_ADDR_MUX_RESERVED, PCA9557_ADDR_RELAYS);
        } else {
            Debug::printfFeature(DebugFeature::RELAY,
                "[RELAY] ERROR: relay expander not responding at 0x%02X on I2C2\n",
                PCA9557_ADDR_RELAYS);
        }
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
    wheelLockOn_ = false;
    r1HighSince_ = 0;
    crankArmed_ = false;
    isCranking_ = false;

    Debug::printfFeature(DebugFeature::RELAY,
        "[RELAY] Initialized on PCA9557 @0x%02X (I2C2): all 8 relays OFF\n",
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

        // R1 (ignition/ECU line) is HIGH in ACC/IGNITION/CRANKING, LOW only in OFF.
        // Start the pre-crank dwell clock on the LOW->HIGH edge; reset it only on OFF
        // (jitter between ACC and IGNITION keeps R1 powered, so it must NOT reset).
        bool r1WasHigh = (currentIgnitionState_ != IgnitionState::OFF);
        bool r1WillBeHigh = (state != IgnitionState::OFF);
        if (!r1WasHigh && r1WillBeHigh) {
            r1HighSince_ = millis();
        } else if (!r1WillBeHigh) {
            r1HighSince_ = 0;
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
    Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Crank armed - waiting for R1 pre-crank dwell");
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
        frontLightOn_ = on;
        updateRelays();
    }
}

void RelayController::setWheelLock(bool locked) {
    if (!initialized_ || faulted_) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] Wheel lock command (%s) ignored — relay expander unavailable\n",
            locked ? "LOCKED" : "UNLOCKED");
        return;
    }
    if (wheelLockOn_ != locked) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Front Wheel Lock: %s\n", locked ? "LOCKED" : "UNLOCKED");
        wheelLockOn_ = locked;
        updateRelays();
    }
}

void RelayController::allOff() {
    Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Fail-safe: All relays OFF");

    // Update state (the wheel lock deliberately holds its last state)
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;
    isCranking_ = false;
    r1HighSince_ = 0;      // OFF resets the pre-crank dwell
    crankArmed_ = false;   // abort any pending crank

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
        // Keep pushing the safe state until the expander verifies clean again.
        // While the starter escalation is latched the shadow is already all-off.
        if (allOffEscalation_) {
            outputShadow_ = 0x00;
        }
        writePort(outputShadow_, 1);
        return;  // no crank logic while the port cannot be trusted
    }

    // Fire a pending (armed) crank once the R1 pre-crank dwell is satisfied.
    if (crankArmed_ && !isCranking_) {
        if (engineRpm >= ENGINE_RUNNING_RPM_THRESHOLD) {
            // Engine already running — suppress the crank, settle into IGNITION.
            crankArmed_ = false;
            Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Crank suppressed - engine already running (RPM: %d)\n", engineRpm);
            setIgnitionState(IgnitionState::IGNITION);
            return;
        }
        if (r1HighSince_ != 0 && (millis() - r1HighSince_) >= ACC_PRECRANK_DWELL_MS) {
            crankArmed_ = false;
            Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Pre-crank dwell satisfied - engaging starter");
            setIgnitionState(IgnitionState::CRANKING);  // sets isCranking_ / crankingStartTime_
        }
        // else: keep holding ACC (R1 high, R2 low) until the dwell elapses
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

void RelayController::updateRelays() {
    bool ignitionLine = (currentIgnitionState_ != IgnitionState::OFF);
    // The starter bit is ONLY ever set in CRANKING.
    bool starter = (currentIgnitionState_ == IgnitionState::CRANKING);

    uint8_t shadow = 0x00;   // Relay_5-8 are always written off
    shadow = withBit(shadow, RELAY_BIT_IGNITION,    ignitionLine);
    shadow = withBit(shadow, RELAY_BIT_STARTER,     starter);
    shadow = withBit(shadow, RELAY_BIT_FRONT_LIGHT, frontLightOn_);
    shadow = withBit(shadow, RELAY_BIT_WHEEL_LOCK,  wheelLockOn_);

    outputShadow_ = shadow;
    writePort(shadow);
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

    bool starterWasSet = (value & (1u << RELAY_BIT_STARTER)) != 0;
    if (!faulted_) {
        Debug::printfFeature(DebugFeature::RELAY,
            "[RELAY] FAULT: port write 0x%02X did not verify after %d attempts%s\n",
            value, RELAY_WRITE_RETRIES,
            starterWasSet ? " — STARTER WAS COMMANDED" : "");
    }
    faulted_ = true;

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
        r1HighSince_ = 0;
    }

    return false;
}

void RelayController::forceAllOffWrite() {
    // Best effort, single transaction — the retry loop lives in update().
    expander_.writeOutputs(0x00);
}
