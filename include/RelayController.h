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
        OFF,        // All power off (RELAY1 LOW, RELAY2 LOW)
        ACC,        // Ignition/ECU line powered (RELAY1 HIGH, RELAY2 LOW)
        IGNITION,   // Ignition/ECU line powered, starter off (RELAY1 HIGH, RELAY2 LOW)
        CRANKING    // Starter engaged (RELAY1 HIGH, RELAY2 HIGH); auto-stops on RPM or CRANKING_TIMEOUT
    };

    RelayController();

    /**
     * @brief Initialize relay controller (direct GPIO outputs)
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Set ignition state
     * @param state Desired ignition state (OFF/ACC/IGNITION)
     *
     * Moving to OFF or ACC cancels any pending or active crank. Only OFF resets the
     * R1 pre-crank dwell timer (ACC<->IGNITION jitter keeps R1 powered).
     */
    void setIgnitionState(IgnitionState state);

    /**
     * @brief Request engine start with the pre-crank dwell (RC/MAVLink path).
     *
     * Call once on a FRESH operator selection of IGNITION. Brings up the ignition/ECU
     * line (ACC) if it was off, then arms a dwell-gated crank: update() engages the
     * starter only once R1 has been powered for ACC_PRECRANK_DWELL_MS (immediately if
     * it already has), and suppresses the crank if the engine is already running.
     * While waiting, the reported state is ACC. Does not re-crank while held.
     */
    void requestCrank();

    /**
     * @brief Set front light state
     * @param on true to turn light ON, false for OFF
     */
    void setFrontLight(bool on);

    /**
     * @brief Set front-wheel lock relay state
     * @param locked true to engage the lock (relay HIGH), false to release
     *
     * Independent of the ignition relays and deliberately NOT cleared by allOff(),
     * so the lock holds its last state through a MAVLink failsafe / link drop.
     */
    void setWheelLock(bool locked);

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
     * @brief Get current front-wheel lock state
     * @return true if locked
     */
    bool getWheelLock() const { return wheelLockOn_; }

    /**
     * @brief Fail-safe: turn ignition and front light OFF.
     * The front-wheel lock is intentionally left untouched (holds last state).
     */
    void allOff();

private:

    // State tracking
    IgnitionState currentIgnitionState_;
    bool frontLightOn_;
    bool wheelLockOn_;

    // Cranking state tracking
    uint32_t crankingStartTime_;   // Time when cranking started (milliseconds)
    bool isCranking_;              // True if currently in cranking state

    // Pre-crank dwell tracking
    uint32_t r1HighSince_;         // millis() when R1 (ignition/ECU) went LOW->HIGH; 0 while OFF
    bool crankArmed_;              // True if a dwell-gated crank is pending

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
