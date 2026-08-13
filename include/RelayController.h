#ifndef RELAY_CONTROLLER_H
#define RELAY_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include "Constants.h"
#include "PCA9557Expander.h"

/**
 * @brief Relay controller for ignition, starter, lighting and wheel lock.
 *
 * Manages relay outputs for vehicle ignition states (OFF/ACC/IGNITION/CRANKING),
 * front light and front-wheel lock. The relays live on the external relay board's
 * PCA9557 @ `PCA9557_ADDR_RELAYS` (0x1F, as-shipped strap A2A1A0=111), not on
 * ESP32 GPIO. The bus is chosen by the owner at construction: the board sits on
 * its intended home X15 → I2C2 (`Wire1`), bench-verified. X14 → I2C1 (`Wire`)
 * works too and is the contingency if that header is ever damaged.
 *
 * The board's IO routing is NOT 1:1 (Relay_1=IO4 … Relay_8=IO1), and two functions
 * drive a paralleled PAIR of relays, so each function is a whole-port MASK from
 * `Constants.h` rather than a single bit:
 *   ignition/ECU line = Relay_1 + Relay_2 (RELAY_MASK_IGNITION, two paralleled relays),
 *   starter           = Relay_3           (RELAY_MASK_STARTER),
 *   front light       = Relay_4           (RELAY_MASK_FRONT_LIGHT),
 *   front-wheel lock  = Relay_5 + Relay_6 (RELAY_MASK_WHEEL_LOCK, two paralleled relays),
 *   Relay_7 / Relay_8 = spare (always written off).
 *
 * The whole 8-bit port is written from a single shadow byte and verified by
 * reading the output register back, because over I2C a write can fail silently
 * (a GPIO write cannot). The starter is escalated separately: a latched starter
 * destroys the starter motor and can move the vehicle, so an unverified starter
 * engagement aborts the crank and a verification failure while the starter bit
 * is set forces repeated all-off writes and latches ignition OFF.
 *
 * Note: a physically welded relay contact is invisible to the read-back, which
 * reflects the expander pin and not the contact. That risk stays with hardware.
 */
class RelayController {
public:
    /**
     * @brief Ignition state enum
     */
    enum class IgnitionState {
        OFF,        // All power off (ignition mask LOW, starter mask LOW)
        ACC,        // Ignition/ECU line powered (ignition mask HIGH, starter mask LOW)
        IGNITION,   // Ignition/ECU line powered, starter off (ignition mask HIGH, starter mask LOW)
        CRANKING    // Starter engaged (ignition mask HIGH, starter mask HIGH); auto-stops on RPM or CRANKING_TIMEOUT
    };

    /**
     * @param bus I2C bus carrying the relay board (bound at construction; the bus
     *            itself is opened by `main.cpp` before `begin()` is called)
     */
    explicit RelayController(TwoWire& bus = Wire1);

    /**
     * @brief Initialize the relay expander and drive every relay OFF.
     *
     * Probes `PCA9557_ADDR_RELAYS`, configures all eight pins as outputs and
     * writes 0x00 with read-back verification before returning. If nothing
     * answers, the whole PCA9557 address block (0x18-0x1F) is probed on the same
     * bus and the ACKing addresses are logged, so a mis-strapped board or a board
     * plugged into the wrong connector is visible from one line.
     *
     * @return true if initialization succeeded; false leaves the driver in a
     *         degraded state where every setter is a logging no-op
     */
    bool begin();

    /**
     * @brief Set ignition state
     * @param state Desired ignition state (OFF/ACC/IGNITION)
     *
     * Moving to OFF or ACC cancels any pending or active crank. Only OFF resets the
     * ignition-line pre-crank dwell timer (ACC<->IGNITION jitter keeps it powered).
     */
    void setIgnitionState(IgnitionState state);

    /**
     * @brief Request engine start with the pre-crank dwell (RC/MAVLink path).
     *
     * Call once on a FRESH operator selection of IGNITION. Brings up the ignition/ECU
     * line (ACC) if it was off, then arms a dwell-gated crank: update() engages the
     * starter only once the ignition line has been powered for ACC_PRECRANK_DWELL_MS (immediately if
     * it already has), and suppresses the crank if the engine is already running.
     * While waiting, the reported state is ACC. Does not re-crank while held.
     * Refused while the expander is unavailable or a starter fault is latched.
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
     * - CRANKING_TIMEOUT elapsed without engine start
     *
     * While the driver is faulted this instead re-attempts the safe-state port
     * write on every call until it verifies.
     *
     * @param engineRpm Current engine RPM from CAN bus
     */
    void update(uint16_t engineRpm);

    /**
     * @brief Get current ignition state
     * @return Current ignition state; OFF whenever the expander is unavailable
     */
    IgnitionState getIgnitionState() const {
        return isAvailable() ? currentIgnitionState_ : IgnitionState::OFF;
    }

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
     * @brief True when the relay expander failed to initialize or a port write
     * could not be verified. Published in telemetry as `io_relay_fault`.
     */
    bool isFaulted() const { return !initialized_ || faulted_; }

    /** @brief True when relay commands are actually reaching the hardware */
    bool isAvailable() const { return initialized_ && !faulted_; }

    /**
     * @brief Fail-safe: turn ignition and front light OFF.
     * The front-wheel lock is intentionally left untouched (holds last state).
     */
    void allOff();

private:
    PCA9557Expander expander_;
    uint8_t outputShadow_;   // whole-port shadow; bits per RELAY_MASK_* in Constants.h

    bool initialized_;       // begin() succeeded
    bool faulted_;           // a port write could not be verified
    bool starterInhibit_;    // crank requests refused until a write verifies clean again
    bool allOffEscalation_;  // repeat the all-off write on every update() while faulted

    // State tracking
    IgnitionState currentIgnitionState_;
    bool frontLightOn_;
    bool wheelLockOn_;

    // Cranking state tracking
    uint32_t crankingStartTime_;   // Time when cranking started (milliseconds)
    bool isCranking_;              // True if currently in cranking state

    // Pre-crank dwell tracking
    uint32_t ignitionHighSince_;   // millis() when the ignition/ECU line (Relay_1+Relay_2) went LOW->HIGH; 0 while OFF
    bool crankArmed_;              // True if a dwell-gated crank is pending

    /** @brief Recompute the shadow byte from the tracked state and write the port */
    void updateRelays();

    /**
     * @brief Write the whole port and verify it by reading the output register back.
     * Retries up to `attempts` times. On persistent failure raises the fault and,
     * if any `RELAY_MASK_STARTER` bit was set, runs the starter escalation (all-off,
     * ignition OFF, crank inhibit).
     * @param value    port byte to write
     * @param attempts verification attempts (the faulted-path retry in update() uses
     *                 1, so a wedged bus cannot cost the control loop several I2C
     *                 timeouts per iteration)
     * @return true if the write verified
     */
    bool writePort(uint8_t value, uint8_t attempts = RELAY_WRITE_RETRIES);

    /** @brief Immediate best-effort all-relays-off write (no retry loop) */
    void forceAllOffWrite();

    /**
     * @brief Set or clear a whole function mask in the shadow byte.
     *
     * A mask can cover more than one relay (ignition and wheel lock each drive two
     * paralleled relays), so every bit of the mask moves together.
     */
    static uint8_t withMask(uint8_t value, uint8_t mask, bool on) {
        return on ? (uint8_t)(value | mask) : (uint8_t)(value & (uint8_t)~mask);
    }
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
