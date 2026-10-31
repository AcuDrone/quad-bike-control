#ifndef RELAY_CONTROLLER_H
#define RELAY_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include "Constants.h"
#include "PCA9557Expander.h"

/**
 * @brief Relay controller for ignition, starter, lighting and (where fitted) the
 * front-wheel lock.
 *
 * Manages relay outputs for vehicle ignition states (OFF/ACC/IGNITION/CRANKING),
 * the front light and, when RELAY_WHEEL_LOCK_FITTED is 1, the front-wheel lock. The relays live on the external relay board's
 * PCA9557 @ `PCA9557_ADDR_RELAYS` (0x1F, as-shipped strap A2A1A0=111), not on
 * ESP32 GPIO. The bus is chosen by the owner at construction: the board sits on
 * its intended home X15 → I2C2 (`Wire1`), bench-verified. X14 → I2C1 (`Wire`)
 * works too and is the contingency if that header is ever damaged.
 *
 * The board's IO routing is NOT 1:1 (Relay_1=IO4 … Relay_8=IO1), so each function
 * is a whole-port MASK from `Constants.h` rather than a relay number. The masks are
 * per vehicle (the vehicle's Constants.h):
 *   quad bike (master, RELAY_WHEEL_LOCK_FITTED = 1):
 *     ignition/ECU line = Relay_1 + Relay_2 (RELAY_MASK_IGNITION, two paralleled relays),
 *     starter           = Relay_3 (RELAY_MASK_STARTER),
 *     front light       = Relay_4 (RELAY_MASK_FRONT_LIGHT),
 *     front-wheel lock  = Relay_5 + Relay_6 (RELAY_MASK_WHEEL_LOCK, two paralleled relays),
 *     Relay_7 / Relay_8 = spare (always off)        — RELAY_MASK_RESERVED.
 *   aztec buggy (aztec, RELAY_WHEEL_LOCK_FITTED = 0):
 *     ignition/ECU line = Relay_1 (RELAY_MASK_IGNITION, single relay),
 *     starter           = Relay_2 (RELAY_MASK_STARTER),
 *     front light       = Relay_3 (RELAY_MASK_FRONT_LIGHT),
 *     Relay_4/5/6       = three separate locks, reserved until a later change (always off),
 *     Relay_7 / Relay_8 = spare (always off)        — together RELAY_MASK_RESERVED.
 *
 * Web relay test: any single relay (1..8) can be toggled for a wiring check, but only
 * while the ignition is OFF and no function relay is energized (the wheel lock, where
 * fitted, must be released). Test bits go through the same shadow byte and verified
 * writePort() as the functions, auto-release after RELAY_TEST_TIMEOUT_MS (the starter
 * relay: RELAY_TEST_STARTER_MAX_MS), and are all cleared immediately by any real
 * ignition/crank/light-on/wheel-lock command, allOff(), a relay-write fault and the
 * starter escalation.
 *
 * Front-wheel lock: fitted on the quad bike (RELAY_WHEEL_LOCK_FITTED = 1, Relay_5 +
 * Relay_6); allOff() deliberately leaves it in its last state. NOT fitted on aztec
 * (RELAY_WHEEL_LOCK_FITTED = 0): setWheelLock() is kept so the MAVLink/web plumbing
 * compiles, but it rejects a LOCK request (logged once), and getWheelLock() always
 * reports released.
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
     * @brief Front-wheel lock request.
     * @param locked requested lock state
     *
     * Fitted (RELAY_WHEEL_LOCK_FITTED = 1, quad bike): drives RELAY_MASK_WHEEL_LOCK.
     * A LOCK request first releases every web test relay. Refused while the expander
     * is unavailable.
     *
     * Not fitted (RELAY_WHEEL_LOCK_FITTED = 0, aztec buggy): there is no relay behind
     * this function, so a LOCK request is rejected: nothing is written to the port, the
     * reported state stays "released", and the first LOCK request after boot is logged
     * once.
     * @return true when applied or already in the requested state; false when the
     *         request cannot be honoured (expander unavailable, or LOCK while not fitted)
     */
    bool setWheelLock(bool locked);

    /**
     * @brief Web relay test: energize or release one relay by its board number.
     * @param relayNumber 1..8 (Relay_N, mapped to its IO bit through relayIoMask())
     * @param on          true to energize, false to release
     * @param reason      set to a human-readable error when the request is refused
     * @return true if applied (or already in the requested state)
     *
     * Turning a relay ON is refused unless the expander is available, the ignition is
     * OFF, no crank is armed, the front light is off and the wheel lock (where fitted)
     * is released. Turning OFF is always accepted
     * while the expander is available.
     */
    bool setRelayTest(uint8_t relayNumber, bool on, const char*& reason);

    /**
     * @brief Relays currently energized by the web relay test, in relay-number order
     * (bit0 = Relay_1 … bit7 = Relay_8), derived from the output shadow byte through
     * the IO routing. 0 while the expander is unavailable.
     */
    uint8_t getRelayTestBits() const;

    /** @brief Port mask (IO bit) of relay `relayNumber` (1..8); 0 for any other number */
    static constexpr uint8_t relayIoMask(uint8_t relayNumber) {
        // Relay.PrjPcb routing: Relay_1=IO4 Relay_2=IO5 Relay_3=IO6 Relay_4=IO7
        //                       Relay_5=IO3 Relay_6=IO2 Relay_7=IO0 Relay_8=IO1
        return relayNumber == 1 ? (uint8_t)(1u << 4) :
               relayNumber == 2 ? (uint8_t)(1u << 5) :
               relayNumber == 3 ? (uint8_t)(1u << 6) :
               relayNumber == 4 ? (uint8_t)(1u << 7) :
               relayNumber == 5 ? (uint8_t)(1u << 3) :
               relayNumber == 6 ? (uint8_t)(1u << 2) :
               relayNumber == 7 ? (uint8_t)(1u << 0) :
               relayNumber == 8 ? (uint8_t)(1u << 1) : (uint8_t)0;
    }

    /** @brief True if a relay is wired behind the wheel-lock function (quad: true, aztec: false) */
    static constexpr bool isWheelLockFitted() { return RELAY_WHEEL_LOCK_FITTED != 0; }

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
     * @return the commanded lock state when fitted; always false when not fitted
     *         (aztec — see isWheelLockFitted())
     */
#if RELAY_WHEEL_LOCK_FITTED
    bool getWheelLock() const { return wheelLockOn_; }
#else
    bool getWheelLock() const { return false; }
#endif

    /**
     * @brief True when the relay expander failed to initialize or a port write
     * could not be verified. Published in telemetry as `io_relay_fault`.
     */
    bool isFaulted() const { return !initialized_ || faulted_; }

    /** @brief True when relay commands are actually reaching the hardware */
    bool isAvailable() const { return initialized_ && !faulted_; }

    /**
     * @brief Fail-safe: turn ignition, starter and front light OFF and release every
     * web test relay. The front-wheel lock (where fitted) deliberately holds its last
     * state; every other relay ends up off.
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
#if RELAY_WHEEL_LOCK_FITTED
    bool wheelLockOn_;             // commanded front-wheel lock state (held through allOff())
#else
    bool wheelLockRejectLogged_;   // the "wheel lock not fitted" message is logged once per boot
#endif

    // Cranking state tracking
    uint32_t crankingStartTime_;   // Time when cranking started (milliseconds)
    bool isCranking_;              // True if currently in cranking state

    // Web relay test
    uint8_t testMask_;                 // port bits (IO) energized by the relay test
    uint32_t testOnSince_[8];          // millis() each test relay (index = relay number - 1) went on

    // Pre-crank dwell tracking
    uint32_t ignitionHighSince_;   // millis() when the ignition/ECU line went LOW->HIGH; 0 while OFF
    bool crankArmed_;              // True if a dwell-gated crank is pending

    /** @brief Recompute the shadow byte from the tracked state and write the port */
    void updateRelays();

    /** @brief Shadow byte for the tracked functions plus the active test bits */
    uint8_t computeShadow() const;

    /**
     * @brief Drop every web test bit (no port write — the caller writes next).
     * @param why logged when any test bit was actually set
     * @return true if any test bit was set
     */
    bool clearRelayTests(const char* why);

    /** @brief Release test relays whose auto-off time has elapsed */
    void expireRelayTests();

    /** @brief Port bits -> relay-number bitfield (bit0 = Relay_1 … bit7 = Relay_8) */
    static uint8_t getRelayTestBitsFromMask(uint8_t portBits);

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
     * A mask may cover more than one relay (none do on aztec today, but the quad
     * bike pairs relays), so every bit of the mask moves together.
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
