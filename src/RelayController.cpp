#include "RelayController.h"
#include "Debug.h"

RelayController::RelayController()
    : relay1Pin_(0),
      relay2Pin_(0),
      relay3Pin_(0),
      currentIgnitionState_(IgnitionState::OFF),
      frontLightOn_(false),
      crankingStartTime_(0),
      isCranking_(false) {
}

bool RelayController::begin(uint8_t relay1Pin, uint8_t relay2Pin, uint8_t relay3Pin) {
    // Store pin assignments
    relay1Pin_ = relay1Pin;
    relay2Pin_ = relay2Pin;
    relay3Pin_ = relay3Pin;

    // Configure pins as outputs
    pinMode(relay1Pin_, OUTPUT);
    pinMode(relay2Pin_, OUTPUT);
    pinMode(relay3Pin_, OUTPUT);

    // Initialize all relays to OFF (safe state)
    digitalWrite(relay1Pin_, LOW);
    digitalWrite(relay2Pin_, LOW);
    digitalWrite(relay3Pin_, LOW);

    // Set initial state
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;

    Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Initialized: RELAY1=%d, RELAY2=%d, RELAY3=%d\n",
                  relay1Pin_, relay2Pin_, relay3Pin_);

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
        digitalWrite(relay3Pin_, on ? HIGH : LOW);
    }
}

void RelayController::allOff() {
    Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Fail-safe: All relays OFF");

    // Turn off all relays
    digitalWrite(relay1Pin_, LOW);
    digitalWrite(relay2Pin_, LOW);
    digitalWrite(relay3Pin_, LOW);

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
            // Both relays OFF
            digitalWrite(relay1Pin_, LOW);
            digitalWrite(relay2Pin_, LOW);
            break;

        case IgnitionState::ACC:
            // Accessory power only (RELAY2 ON, RELAY1 OFF)
            digitalWrite(relay1Pin_, LOW);
            digitalWrite(relay2Pin_, HIGH);
            break;

        case IgnitionState::IGNITION:
            // Full power (both relays ON)
            digitalWrite(relay1Pin_, HIGH);
            digitalWrite(relay2Pin_, HIGH);
            break;

        case IgnitionState::CRANKING:
            // Cranking: same as IGNITION (starter motor engaged via RELAY1)
            // Note: Actual cranking control is in update() method
            digitalWrite(relay1Pin_, HIGH);
            digitalWrite(relay2Pin_, HIGH);
            break;
    }
}
