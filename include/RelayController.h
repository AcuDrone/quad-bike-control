#ifndef RELAY_CONTROLLER_H
#define RELAY_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"

/**
 * @brief Relay controller for ignition and lighting systems
 *
 * Manages relay outputs for vehicle ignition states (OFF/ACC/IGNITION)
 * and front light control. Provides fail-safe methods for safety.
 */
class RelayController {
public:
    /**
     * @brief Ignition state enum
     */
    enum class IgnitionState {
        OFF,        // All power off (both relays LOW)
        ACC,        // Accessory power only (RELAY2 HIGH, RELAY1 LOW)
        IGNITION,   // Full ignition (both relays HIGH)
        CRANKING    // Cranking starter motor (same as IGNITION, auto-stops after 5s or when engine starts)
    };

    RelayController();

    /**
     * @brief Initialize relay controller with MCP23017 GPIO expander
     * @param mcp Reference to initialized MCP23017Controller
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Set ignition state
     * @param state Desired ignition state (OFF/ACC/IGNITION)
     */
    void setIgnitionState(IgnitionState state);

    /**
     * @brief Set front light state
     * @param on true to turn light ON, false for OFF
     */
    void setFrontLight(bool on);

    /**
     * @brief Update cranking state based on engine RPM
     *
     * Call this method periodically (every loop iteration) to monitor cranking.
     * Automatically stops cranking if:
     * - Engine RPM exceeds threshold (engine started)
     * - 5 seconds elapsed without engine start
     *
     * @param engineRpm Current engine RPM from CAN bus
     */
    void update(uint16_t engineRpm);

    /**
     * @brief Get current ignition state
     * @return Current ignition state
     */
    IgnitionState getIgnitionState() const { return currentIgnitionState_; }

    /**
     * @brief Get current front light state
     * @return true if light is ON, false if OFF
     */
    bool getFrontLight() const { return frontLightOn_; }

    /**
     * @brief Fail-safe: turn all relays OFF
     * Sets ignition to OFF and turns off front light
     */
    void allOff();

private:

    // State tracking
    IgnitionState currentIgnitionState_;
    bool frontLightOn_;

    // Cranking state tracking
    uint32_t crankingStartTime_;   // Time when cranking started (milliseconds)
    bool isCranking_;              // True if currently in cranking state

    // Helper methods
    void updateRelays();
};

/**
 * @brief Get ignition state name as string
 * @param state Ignition state enum
 * @return State name string
 */
inline const char* getRelayIgnitionStateName(RelayController::IgnitionState state) {
    switch (state) {
        case RelayController::IgnitionState::OFF: return "OFF";
        case RelayController::IgnitionState::ACC: return "ACC";
        case RelayController::IgnitionState::IGNITION: return "IGNITION";
        case RelayController::IgnitionState::CRANKING: return "CRANKING";
        default: return "UNKNOWN";
    }
}

#endif // RELAY_CONTROLLER_H
