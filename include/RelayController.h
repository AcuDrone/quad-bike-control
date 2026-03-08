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
        IGNITION    // Full ignition (both relays HIGH)
    };

    RelayController();

    /**
     * @brief Initialize relay controller
     * @param relay1Pin GPIO pin for ignition main power (default: PIN_RELAY1)
     * @param relay2Pin GPIO pin for accessory power (default: PIN_RELAY2)
     * @param relay3Pin GPIO pin for front light (default: PIN_RELAY3)
     * @return true if initialization successful
     */
    bool begin(uint8_t relay1Pin = PIN_RELAY1,
               uint8_t relay2Pin = PIN_RELAY2,
               uint8_t relay3Pin = PIN_RELAY3);

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
    // Pin assignments
    uint8_t relay1Pin_;  // Ignition main power
    uint8_t relay2Pin_;  // Accessory power
    uint8_t relay3Pin_;  // Front light

    // State tracking
    IgnitionState currentIgnitionState_;
    bool frontLightOn_;

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
        default: return "UNKNOWN";
    }
}

#endif // RELAY_CONTROLLER_H
