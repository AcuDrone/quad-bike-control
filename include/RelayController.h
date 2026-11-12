#ifndef RELAY_CONTROLLER_H
#define RELAY_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include "Constants.h"
#include "IRelayPort.h"
#include "PCA9557Expander.h"
#include "XL9535Expander.h"
#include "RelayBoardLaw.h"

/**
 * @brief Relay controller for ignition, starter, lighting and (where fitted) the
 * front-wheel lock.
 *
 * Manages relay outputs for vehicle ignition states (OFF/ACC/IGNITION/CRANKING),
 * the front light and, when RELAY_WHEEL_LOCK_FITTED is 1, the front-wheel lock. The
 * relays live on an external relay board on I2C, not on ESP32 GPIO. The bus is chosen
 * by the owner at construction: the board sits on its intended home X15 → I2C2
 * (`Wire1`), bench-verified. X14 → I2C1 (`Wire`) works too and is the contingency if
 * that header is ever damaged.
 *
 * TWO RELAY BOARDS, one image. Exactly one of them is used, selected ONCE in begin()
 * (before any relay write) and latched until reboot:
 *   - the custom PCA9557 relay board @0x1F (PCA9557_ADDR_RELAYS; Relay.PrjPcb routing,
 *     NOT 1:1: Relay_1=IO4 … Relay_8=IO1);
 *   - the XL9535-K8V5 relay board @0x20..0x27 (XL9535 = PCA9535-compatible, port P0
 *     only, identity routing Relay_N = P0.(N-1), bench-confirmed 2026-11-07).
 * The persisted setting (NVS RELAY_BOARD_NVS_NS / RELAY_BOARD_NVS_KEY: auto | pca9557 |
 * xl9535, default auto) and nine zero-length probes (0x1F, 0x20..0x27) go through the
 * pure relayBoardDecide() (RelayBoardLaw.h). Any ambiguity — both boards, none, two
 * XL9535 addresses, a manual setting contradicting the bus — is a relay fault: nothing
 * is written to any device, every setter is a logging no-op, the reason is published
 * (telemetry `relay_board_error`) and the 0x18..0x27 scan is logged. Never a guess; a
 * detection fault is not retried before reboot. A changed setting applies after reboot.
 * The chip is driven only through IRelayPort; both driver instances are members (no heap).
 *
 * Functions are RELAY-NUMBER sets in Constants.h (RELAY_SET_*, bit0 = Relay_1); begin()
 * maps them through the detected board's table into whole-port masks. Per vehicle:
 *   quad bike (master, RELAY_WHEEL_LOCK_FITTED = 1):
 *     ignition/ECU line = Relay_1 + Relay_2 (RELAY_SET_IGNITION, two paralleled relays),
 *     starter           = Relay_3 (RELAY_SET_STARTER),
 *     front light       = Relay_4 (RELAY_SET_FRONT_LIGHT),
 *     front-wheel lock  = Relay_5 + Relay_6 (RELAY_SET_WHEEL_LOCK, two paralleled relays),
 *     Relay_7 / Relay_8 = spare (always off)        — RELAY_SET_RESERVED.
 *   aztec buggy (aztec, RELAY_WHEEL_LOCK_FITTED = 1):
 *     ignition/ECU line = Relay_1 (RELAY_SET_IGNITION, single relay),
 *     starter           = Relay_2 (RELAY_SET_STARTER),
 *     front light       = Relay_3 (RELAY_SET_FRONT_LIGHT),
 *     lock              = Relay_4 + Relay_5 + Relay_6 switched together (RELAY_SET_WHEEL_LOCK;
 *                         ⚠ PCA9557 IO7|IO3|IO2 = 0x8C, X7/X8/X9; XL9535 P0.3..P0.5 = 0x38),
 *     Relay_7 / Relay_8 = spare (always off)        — RELAY_SET_RESERVED.
 *
 * Web relay test: any single relay (1..8) can be toggled for a wiring check, but only
 * while the ignition is OFF and no function relay is energized (the wheel lock, where
 * fitted, must be released). Test bits go through the same shadow byte and verified
 * writePort() as the functions, auto-release after RELAY_TEST_TIMEOUT_MS (the starter
 * relay: RELAY_TEST_STARTER_MAX_MS), and are all cleared immediately by any real
 * ignition/crank/light-on/wheel-lock command, allOff(), a relay-write fault and the
 * starter escalation.
 *
 * Lock ("wheel lock" in the API): fitted on both vehicles (RELAY_WHEEL_LOCK_FITTED = 1) —
 * aztec Relay_4 + Relay_5 + Relay_6, quad bike Relay_5 + Relay_6. allOff() deliberately
 * leaves it in its last state on both. With RELAY_WHEEL_LOCK_FITTED = 0 setWheelLock() is
 * kept so the MAVLink/web plumbing compiles, but it rejects a LOCK request (logged once),
 * and getWheelLock() always reports released.
 *
 * The whole 8-bit port is written from a single shadow byte and verified by reading
 * the PINS back (IRelayPort::readPins(), the input register), because over I2C a write
 * can fail silently (a GPIO write cannot) and a reset expander (XL9535 VIN brown-out:
 * configuration back to all-inputs, relays dropped) keeps a latch that would still read
 * back fine. The starter is escalated separately: a latched starter destroys the starter
 * motor and can move the vehicle, so an unverified starter engagement aborts the crank
 * and a verification failure while the starter bit is set forces repeated all-off
 * writes and latches ignition OFF.
 *
 * Recovery: while faulted, update() makes one single-attempt write per call and, every
 * RELAY_RECOVER_MS, replaces it by IRelayPort::initPort(shadow) (output = shadow →
 * polarity 0 → config 0 → pin verify) plus one verified shadow write. Seeding the latch
 * with the shadow (not 0x00) keeps a transient bus error from blipping an energized
 * ignition relay; writing the latch before the config register still rules out the
 * all-relays-on glitch. The starter bit is never seeded: if the shadow holds it, the
 * starter escalation runs first and recovery seeds all-off. At boot begin() always uses
 * initAllOff() (seed 0x00). A detected board whose boot init fails enters the same timed
 * recovery (`relay_board_error` = init_failed until it verifies).
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
     * @brief Select the relay board, build the masks and drive every relay OFF.
     *
     * Reads the setting (NVS `relay`/`board`), probes 0x1F and 0x20..0x27 and decides
     * through relayBoardDecide(); the result is latched until reboot. On a detection
     * fault nothing is written, the reason and the 0x18..0x27 scan are logged and the
     * fault is not retried. On success the function masks are built from the board's
     * table (checked at runtime) and initAllOff() runs (output 0x00 → polarity 0x00 →
     * config 0x00 → pins read 0x00). If that init fails, the board stays latched and
     * the update() recovery timer retries it.
     *
     * @return true only if a board was detected and its all-off state verifies; false
     *         leaves the driver degraded (every setter a logging no-op)
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
     * Fitted (RELAY_WHEEL_LOCK_FITTED = 1; aztec: Relay_4+5+6, quad: Relay_5+6): drives the
     * whole wheel-lock mask (RELAY_SET_WHEEL_LOCK) with a verified whole-port write.
     * A LOCK request first releases every web test relay. Refused while the expander
     * is unavailable.
     *
     * Not fitted (RELAY_WHEEL_LOCK_FITTED = 0): there is no relay behind
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

    /**
     * @brief Port mask of relay `relayNumber` (1..8) on the ACTIVE board (RelayBoardLaw.h
     * table: PCA9557 Relay.PrjPcb routing or XL9535 identity); 0 for any other number
     * and while no board was detected.
     */
    uint8_t relayIoMask(uint8_t relayNumber) const {
        return table_ ? relayTableMask(*table_, relayNumber) : (uint8_t)0;
    }

    /** @brief True if relays are wired behind the wheel-lock function (RELAY_WHEEL_LOCK_FITTED) */
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
     * write (one attempt per call) and, every RELAY_RECOVER_MS, re-initializes the
     * relay port seeded with the shadow (attemptRecovery()) until it verifies.
     *
     * @param engineRpm EFFECTIVE engine RPM (tacho when fitted, CAN otherwise; 0 when none is valid)
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
     * @brief True while the starter relay output bit is set in the port shadow — a crank
     * (IgnitionState::CRANKING) or a web relay test of the starter relay. Used to suppress
     * the 12 V battery low warning during the cranking sag.
     */
    bool isStarterEnergized() const { return (outputShadow_ & maskStarter_) != 0; }

    /**
     * @brief Get current front light state
     * @return true if light is ON, false if OFF
     */
    bool getFrontLight() const { return frontLightOn_; }

    /**
     * @brief True while a dwell-gated crank is armed (requestCrank() → pre-crank dwell).
     * The reported state is ACC meanwhile; IGN_STATE shows it as CRANKING.
     */
    bool isCrankPending() const { return isAvailable() && crankArmed_; }

    /**
     * @brief Get current front-wheel lock state
     * @return the commanded lock state when fitted; always false when not fitted
     *         (see isWheelLockFitted())
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

    // ---- Relay board selection (telemetry relay_board / _addr / _cfg / _reboot / _error) ----

    /** @brief Board in use, latched at boot (NONE when detection failed) */
    RelayBoardKind getBoardKind() const { return boardKind_; }

    /** @brief 7-bit address of the board in use; 0 when none */
    uint8_t getBoardAddress() const { return boardKind_ == RelayBoardKind::NONE ? 0 : boardAddr_; }

    /** @brief Setting used for detection at boot (out-of-range stored values read as AUTO) */
    RelayBoardSetting getBootSetting() const { return bootSetting_; }

    /** @brief Setting currently stored in NVS (changes apply after reboot) */
    RelayBoardSetting getStoredSetting() const { return storedSetting_; }

    /** @brief True while the stored setting differs from the one used at boot */
    bool isBoardRebootPending() const { return storedSetting_ != bootSetting_; }

    /** @brief Why no board is in use / the boot init failed; NONE otherwise */
    RelayBoardError getBoardError() const { return boardError_; }

    /**
     * @brief Persist a new relay board setting (NVS `relay`/`board`). The running board,
     * address and masks are untouched — the value is used from the next boot.
     * @return true when NVS holds the new value (RAM copy updated only then)
     */
    bool saveBoardSetting(RelayBoardSetting setting);

    /**
     * @brief Fail-safe: turn ignition, starter and front light OFF and release every
     * web test relay. The front-wheel lock (where fitted) deliberately holds its last
     * state; every other relay ends up off.
     */
    void allOff();

private:
    // Both drivers are members (no heap); port_ points at the detected one. The XL9535
    // can sit at any of 0x20..0x27, so there is one instance per address (each is bound
    // to its address at construction and also serves as that address's detection probe).
    PCA9557Expander pca_;
    XL9535Expander xl_[8];
    IRelayPort* port_;               // nullptr until a board is detected
    const uint8_t (*table_)[8];      // active Relay_N -> port bit table; nullptr while none

    // Board selection (latched in begin())
    RelayBoardKind boardKind_;
    uint8_t boardAddr_;
    RelayBoardSetting bootSetting_;
    RelayBoardSetting storedSetting_;
    RelayBoardError boardError_;

    // Function port masks, built in begin() from RELAY_SET_* and the active table
    uint8_t maskIgnition_;
    uint8_t maskStarter_;
    uint8_t maskFrontLight_;
#if RELAY_WHEEL_LOCK_FITTED
    uint8_t maskWheelLock_;
#endif
    uint8_t maskReserved_;

    uint8_t outputShadow_;   // whole-port shadow; bits per the runtime masks above

    bool initialized_;       // begin() succeeded
    bool faulted_;           // a port write could not be verified
    bool starterInhibit_;    // crank requests refused until a write verifies clean again
    bool allOffEscalation_;  // repeat the all-off write on every update() while faulted
    bool initPending_;       // boot initPort failed: only the timed recovery may touch the port
    uint32_t lastRecoverMs_; // millis() of the fault raise / last recovery attempt
    bool recoverFailLogged_; // a failed recovery attempt is logged once per fault episode

    // State tracking
    IgnitionState currentIgnitionState_;
    bool frontLightOn_;
#if RELAY_WHEEL_LOCK_FITTED
    bool wheelLockOn_;             // commanded lock state (held through allOff())
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

    /** @brief Port bits -> relay-number bitfield (bit0 = Relay_1 … bit7 = Relay_8), active table */
    uint8_t getRelayTestBitsFromMask(uint8_t portBits) const;

    /** @brief Read the stored setting from NVS (absent / out-of-range → AUTO, logged, not rewritten) */
    static RelayBoardSetting loadBoardSetting();

    /** @brief Log the detection fault and the 0x18..0x27 scan of this bus */
    void logDetectionFault(uint16_t ackMask);

    /** @brief Starter escalation: all-off shadow + write, ignition latched OFF, crank inhibit */
    void escalateStarter();

    /**
     * @brief Timed recovery while faulted: initPort(shadow) (starter never seeded) and one
     * verified write of the shadow. Bounded: four init transactions + one write + one read.
     */
    void attemptRecovery();

    /**
     * @brief Write the whole port and verify it by reading the PINS back (readPins()).
     * Retries up to `attempts` times. On persistent failure raises the fault and,
     * if any starter-mask bit was set, runs the starter escalation (all-off,
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
