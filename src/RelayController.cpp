#include "RelayController.h"
#include "Debug.h"

RelayController::RelayController()
    : mcp_(nullptr),
      currentIgnitionState_(IgnitionState::OFF),
      frontLightOn_(false),
      crankingStartTime_(0),
      isCranking_(false) {
}

bool RelayController::begin(MCP23017Controller& mcp) {
    mcp_ = &mcp;

    if (!mcp_->isInitialized()) {
        Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] ERROR: MCP23017 not initialized");
        return false;
    }

    // Configure relay pins as outputs on MCP23017
    mcp_->pinMode(MCP_PIN_RELAY1, OUTPUT);
    mcp_->pinMode(MCP_PIN_RELAY2, OUTPUT);
    mcp_->pinMode(MCP_PIN_RELAY3, OUTPUT);

    // Initialize all relays to OFF (safe state)
    mcp_->digitalWrite(MCP_PIN_RELAY1, LOW);
    mcp_->digitalWrite(MCP_PIN_RELAY2, LOW);
    mcp_->digitalWrite(MCP_PIN_RELAY3, LOW);

    // Set initial state
    currentIgnitionState_ = IgnitionState::OFF;
    frontLightOn_ = false;

    Debug::printfFeature(DebugFeature::RELAY, "[RELAY] Initialized on MCP23017: RELAY1=%d, RELAY2=%d, RELAY3=%d\n",
                  MCP_PIN_RELAY1, MCP_PIN_RELAY2, MCP_PIN_RELAY3);

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
        if (mcp_) mcp_->digitalWrite(MCP_PIN_RELAY3, on ? HIGH : LOW);
    }
}

void RelayController::allOff() {
    Debug::printlnFeature(DebugFeature::RELAY, "[RELAY] Fail-safe: All relays OFF");

    // Turn off all relays
    if (mcp_) {
        mcp_->digitalWrite(MCP_PIN_RELAY1, LOW);
        mcp_->digitalWrite(MCP_PIN_RELAY2, LOW);
        mcp_->digitalWrite(MCP_PIN_RELAY3, LOW);
    }

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
    if (!mcp_) return;

    switch (currentIgnitionState_) {
        case IgnitionState::OFF:
            mcp_->digitalWrite(MCP_PIN_RELAY1, LOW);
            mcp_->digitalWrite(MCP_PIN_RELAY2, LOW);
            break;

        case IgnitionState::ACC:
            mcp_->digitalWrite(MCP_PIN_RELAY1, HIGH);
            mcp_->digitalWrite(MCP_PIN_RELAY2, LOW);
            break;

        case IgnitionState::IGNITION:
            mcp_->digitalWrite(MCP_PIN_RELAY1, HIGH);
            mcp_->digitalWrite(MCP_PIN_RELAY2, LOW);
            break;

        case IgnitionState::CRANKING:
            mcp_->digitalWrite(MCP_PIN_RELAY1, HIGH);
            mcp_->digitalWrite(MCP_PIN_RELAY2, HIGH);
            break;
    }
}
