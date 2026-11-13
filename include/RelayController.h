#ifndef RELAY_CONTROLLER_H
#define RELAY_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "RelayBoardLaw.h"   // RelayBoardKind only (getBoardKind() is always NONE here)

/**
 * @brief Relay controller for ignition, starter, lighting and the front-wheel lock.
 *
 * Manages relay outputs for vehicle ignition states (OFF/ACC/IGNITION/CRANKING),
 * the front light and the front-wheel lock. On the hand-wired DevKitC-1 the four
 * relays are driven straight from ESP32 GPIO (active HIGH), no port expander:
 *     Relay 1 (PIN_RELAY1,     GPIO36) = ignition/ECU line (RELAY_MASK_IGNITION)
 *     Relay 2 (PIN_RELAY2,     GPIO37) = starter           (RELAY_MASK_STARTER)
 *     Relay 3 (PIN_RELAY3,     GPIO38) = front light       (RELAY_MASK_FRONT_LIGHT)
 *     Relay 4 (PIN_WHEEL_LOCK, GPIO39) = front-wheel lock  (RELAY_MASK_WHEEL_LOCK)
 *
 * The four outputs are written from a single shadow byte (bit0 = Relay 1 … bit3 =
 * Relay 4). A GPIO write cannot fail silently, so there is no read-back
 * verification and no starter escalation on this board.
 *
 * Web relay test: any single relay (1..RELAY_COUNT) can be toggled for a wiring
 * check, but only while the ignition is OFF and no function relay is energized
 * (light off, wheel lock released). Test bits go through the same shadow byte as
 * the functions, auto-release after RELAY_TEST_TIMEOUT_MS (the starter relay:
 * RELAY_TEST_STARTER_MAX_MS), and are all cleared immediately by any real
 * ignition/crank/light-on/wheel-lock command and allOff().
 *
 * Front-wheel lock: fitted on the quad bike (RELAY_WHEEL_LOCK_FITTED = 1, Relay 4);
 * allOff() deliberately leaves it in its last state.
 *
 * master's I2C relay-board API (PCA9557 / XL9535 detection, board selector, verified
 * writes, fault state) does not exist on this board. The parts the shared callers use
 * are kept with their direct-GPIO meaning: the relays are available as soon as begin()
 * ran (isAvailable()), never faulted afterwards, and getBoardKind() is always
 * RelayBoardKind::NONE (GCS hardware-version bits 16-23 = 0, "no I2C relay board").
 */
class RelayController {
public:
    /**
     * @brief Ignition state enum
     */
    enum class IgnitionState {
        OFF,        // All power off (ignition relay LOW, starter relay LOW)
        ACC,        // Ignition/ECU line powered (ignition relay HIGH, starter relay LOW)
        IGNITION,   // Ignition/ECU line powered, starter off (ignition relay HIGH, starter relay LOW)
        CRANKING    // Starter engaged (ignition relay HIGH, starter relay HIGH); auto-stops on RPM or CRANKING_TIMEOUT
    };

    RelayController();

    /**
     * @brief Configure the four relay GPIOs as outputs and drive every relay OFF.
     * @return true (a direct GPIO setup cannot fail)
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
     */
    void requestCrank();

    /**
     * @brief Set front light state
     * @param on true to turn light ON, false for OFF
     */
    void setFrontLight(bool on);

    /**
     * @brief Front-wheel lock request (drives RELAY_MASK_WHEEL_LOCK, Relay 4).
     * @param locked requested lock state
     *
     * A LOCK request first releases every web test relay. Independent of the
     * ignition relays and deliberately NOT cleared by allOff(), so the lock holds
     * its last state through a MAVLink failsafe / link drop.
     * @return true when applied or already in the requested state; false only
     *         before begin()
     */
    bool setWheelLock(bool locked);

    /**
     * @brief Web relay test: energize or release one relay by its number.
     * @param relayNumber 1..RELAY_COUNT (mapped to its shadow bit through relayIoMask())
     * @param on          true to energize, false to release
     * @param reason      set to a human-readable error when the request is refused
     * @return true if applied (or already in the requested state)
     *
     * Turning a relay ON is refused unless the ignition is OFF, no crank is armed,
     * the front light is off and the wheel lock is released. Turning OFF is always
     * accepted.
     */
    bool setRelayTest(uint8_t relayNumber, bool on, const char*& reason);

    /**
     * @brief Relays currently energized by the web relay test, in relay-number order
     * (bit0 = Relay 1 … bit3 = Relay 4). 0 before begin().
     */
    uint8_t getRelayTestBits() const;

    /** @brief Shadow bit of relay `relayNumber` (1..RELAY_COUNT); 0 for any other number */
    static constexpr uint8_t relayIoMask(uint8_t relayNumber) {
        return (relayNumber >= 1 && relayNumber <= RELAY_COUNT)
            ? (uint8_t)(1u << (relayNumber - 1)) : (uint8_t)0;
    }

    /** @brief True if a relay is wired behind the wheel-lock function (always on this board) */
    static constexpr bool isWheelLockFitted() { return RELAY_WHEEL_LOCK_FITTED != 0; }

    /**
     * @brief Update cranking state based on engine RPM
     *
     * Call this method periodically (every loop iteration) to monitor cranking.
     * Automatically stops cranking if:
     * - Engine RPM exceeds threshold (engine started)
     * - CRANKING_TIMEOUT elapsed without engine start
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
     * @brief True while a dwell-gated crank is armed (requestCrank() → pre-crank dwell).
     * The reported state is ACC meanwhile; IGN_STATE shows it as CRANKING.
     */
    bool isCrankPending() const { return initialized_ && crankArmed_; }

    /** @brief Relays usable: begin() ran (a direct GPIO write cannot fail afterwards) */
    bool isAvailable() const { return initialized_; }

    /** @brief Inverse of isAvailable(): true only before begin() */
    bool isFaulted() const { return !initialized_; }

    /** @brief No I2C relay board on this board: always RelayBoardKind::NONE */
    RelayBoardKind getBoardKind() const { return RelayBoardKind::NONE; }

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
     * @brief Fail-safe: turn ignition, starter and front light OFF and release every
     * web test relay. The front-wheel lock deliberately holds its last state.
     */
    void allOff();

private:
    uint8_t outputShadow_;   // relay shadow; bits per RELAY_MASK_* in Constants.h
    bool initialized_;       // begin() ran

    // State tracking
    IgnitionState currentIgnitionState_;
    bool frontLightOn_;
    bool wheelLockOn_;             // commanded front-wheel lock state (held through allOff())

    // Cranking state tracking
    uint32_t crankingStartTime_;   // Time when cranking started (milliseconds)
    bool isCranking_;              // True if currently in cranking state

    // Web relay test
    uint8_t testMask_;                     // shadow bits energized by the relay test
    uint32_t testOnSince_[RELAY_COUNT];    // millis() each test relay (index = relay number - 1) went on

    // Pre-crank dwell tracking
    uint32_t ignitionHighSince_;   // millis() when the ignition/ECU line went LOW->HIGH; 0 while OFF
    bool crankArmed_;              // True if a dwell-gated crank is pending

    /** @brief Recompute the shadow byte from the tracked state and write the GPIOs */
    void updateRelays();

    /** @brief Shadow byte for the tracked functions plus the active test bits */
    uint8_t computeShadow() const;

    /**
     * @brief Drop every web test bit (no GPIO write — the caller writes next).
     * @param why logged when any test bit was actually set
     * @return true if any test bit was set
     */
    bool clearRelayTests(const char* why);

    /** @brief Release test relays whose auto-off time has elapsed */
    void expireRelayTests();

    /** @brief Shadow bits -> relay-number bitfield (bit0 = Relay 1 … bit3 = Relay 4) */
    static uint8_t getRelayTestBitsFromMask(uint8_t bits);

    /** @brief Drive the four relay GPIOs from a shadow byte */
    void writeOutputs(uint8_t value);

    /** @brief Set or clear a whole function mask in the shadow byte. */
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
