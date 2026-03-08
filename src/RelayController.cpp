#include "RelayController.h"

RelayController::RelayController()
    : relay1Pin_(0),
      relay2Pin_(0),
      relay3Pin_(0),
      currentIgnitionState_(IgnitionState::OFF),
      frontLightOn_(false) {
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

    Serial.printf("[RELAY] Initialized: RELAY1=%d, RELAY2=%d, RELAY3=%d\n",
                  relay1Pin_, relay2Pin_, relay3Pin_);

    return true;
}

void RelayController::setIgnitionState(IgnitionState state) {
    if (currentIgnitionState_ != state) {
        Serial.printf("[RELAY] Ignition: %s -> %s\n",
                     getRelayIgnitionStateName(currentIgnitionState_),
                     getRelayIgnitionStateName(state));
        currentIgnitionState_ = state;
        updateRelays();
    }
}

void RelayController::setFrontLight(bool on) {
    if (frontLightOn_ != on) {
        Serial.printf("[RELAY] Front Light: %s\n", on ? "ON" : "OFF");
        frontLightOn_ = on;
        digitalWrite(relay3Pin_, on ? HIGH : LOW);
    }
}

void RelayController::allOff() {
    Serial.println("[RELAY] Fail-safe: All relays OFF");

    // Turn off all relays
    digitalWrite(relay1Pin_, LOW);
    digitalWrite(relay2Pin_, LOW);
    digitalWrite(relay3Pin_, LOW);

    // Update state
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;
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
    }
}
