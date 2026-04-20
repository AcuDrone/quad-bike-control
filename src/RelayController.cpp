#include "RelayController.h"
#include "Debug.h"

RelayController::RelayController()
    : currentIgnitionState_(IgnitionState::OFF),
      frontLightOn_(false),
      crankingStartTime_(0),
      isCranking_(false) {
}

bool RelayController::begin() {
    pinMode(PIN_RELAY1, OUTPUT);
    pinMode(PIN_RELAY2, OUTPUT);
    pinMode(PIN_RELAY3, OUTPUT);

    // Initialize all relays to OFF (safe state)
    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
    digitalWrite(PIN_RELAY3, LOW);

    // Set initial state
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;

    Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Initialized: RELAY1=%d, RELAY2=%d, RELAY3=%d\n",
                  PIN_RELAY1, PIN_RELAY2, PIN_RELAY3);

    return true;
}

void RelayController::setIgnitionState(IgnitionState state) {
    if (currentIgnitionState_ != state) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Ignition: %s -> %s\n",
                     getRelayIgnitionStateName(currentIgnitionState_),
                     getRelayIgnitionStateName(state));

        // If entering CRANKING state, start timer
        if (state == IgnitionState::CRANKING) {
            crankingStartTime_ = millis();
            isCranking_ = true;
            Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Cranking started - monitoring RPM and timeout");
        } else if (currentIgnitionState_ == IgnitionState::CRANKING) {
            // Exiting CRANKING state
            isCranking_ = false;
        }

        currentIgnitionState_ = state;
        updateRelays();
    }
}

void RelayController::setFrontLight(bool on) {
    if (frontLightOn_ != on) {
        Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Front Light: %s\n", on ? "ON" : "OFF");
        frontLightOn_ = on;
        digitalWrite(PIN_RELAY3, on ? HIGH : LOW);
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
}

void RelayController::update(uint16_t engineRpm) {
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
