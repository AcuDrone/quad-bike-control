#include "RelayController.h"
#include "Debug.h"

RelayController::RelayController()
    : currentIgnitionState_(IgnitionState::OFF),
      frontLightOn_(false),
      wheelLockOn_(false),
      crankingStartTime_(0),
      isCranking_(false),
      r1HighSince_(0),
      crankArmed_(false) {
}

bool RelayController::begin() {
    pinMode(PIN_RELAY1, OUTPUT);
    pinMode(PIN_RELAY2, OUTPUT);
    pinMode(PIN_RELAY3, OUTPUT);
    pinMode(PIN_WHEEL_LOCK, OUTPUT);

    // Initialize all relays to OFF (safe state)
    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
    digitalWrite(PIN_RELAY3, LOW);
    digitalWrite(PIN_WHEEL_LOCK, LOW);

    // Set initial state
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;
    wheelLockOn_ = false;
    r1HighSince_ = 0;
    crankArmed_ = false;
    isCranking_ = false;

    Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Initialized: RELAY1=%d, RELAY2=%d, RELAY3=%d\n",
                  PIN_RELAY1, PIN_RELAY2, PIN_RELAY3);

    return true;
}

void RelayController::setIgnitionState(IgnitionState state) {
    if (currentIgnitionState_ != state) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Ignition: %s -> %s\n",
                     getRelayIgnitionStateName(currentIgnitionState_),
                     getRelayIgnitionStateName(state));

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
    if (frontLightOn_ != on) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Front Light: %s\n", on ? "ON" : "OFF");
        frontLightOn_ = on;
        digitalWrite(PIN_RELAY3, on ? HIGH : LOW);
    }
}

void RelayController::setWheelLock(bool locked) {
    if (wheelLockOn_ != locked) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Front Wheel Lock: %s\n", locked ? "LOCKED" : "UNLOCKED");
        wheelLockOn_ = locked;
        digitalWrite(PIN_WHEEL_LOCK, locked ? HIGH : LOW);
    }
}



void RelayController::allOff() {
    Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Fail-safe: All relays OFF");

    // Turn off all relays
    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
    digitalWrite(PIN_RELAY3, LOW);

    // Update state
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;
    isCranking_ = false;
    r1HighSince_ = 0;      // OFF resets the pre-crank dwell
    crankArmed_ = false;   // abort any pending crank
}

void RelayController::update(uint16_t engineRpm) {
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

    switch (currentIgnitionState_) {
        case IgnitionState::OFF:
            digitalWrite(PIN_RELAY1, LOW);
            digitalWrite(PIN_RELAY2, LOW);
            break;

        case IgnitionState::ACC:
            digitalWrite(PIN_RELAY1, HIGH);
            digitalWrite(PIN_RELAY2, LOW);
            break;

        case IgnitionState::IGNITION:
            digitalWrite(PIN_RELAY1, HIGH);
            digitalWrite(PIN_RELAY2, LOW);
            break;

        case IgnitionState::CRANKING:
            digitalWrite(PIN_RELAY1, HIGH);
            digitalWrite(PIN_RELAY2, HIGH);
            break;
    }
}
