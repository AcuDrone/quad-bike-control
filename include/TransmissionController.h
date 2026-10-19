#ifndef TRANSMISSION_CONTROLLER_H
#define TRANSMISSION_CONTROLLER_H

#include "ServoController.h"
#include "Constants.h"
#include "BoardInputs.h"
#include <Preferences.h>

/**
 * @brief Vehicle data structure for transmission safety checks
 */
struct TransmissionVehicleData {
    uint8_t vehicleSpeed;       // km/h (0-255) — CAN-sourced; dead (OBD-II PID 0x0D is disabled)
    uint32_t lastUpdateTime;    // millis() timestamp
    bool dataValid;             // true if CAN communication is healthy
    float sensorSpeedMs;        // hall speed sensor reading (m/s) — the live speed source
    bool sensorSpeedValid;      // hall sensor health; false means "speed unknown", NOT "stopped"
    bool sensorSpeedSuspicious; // latched implausible pulse loss; reading is a decaying upper bound
};

/**
 * @brief Transmission controller — PWM servo-based gear selector
 *
 * Controls a PWM servo (GXservo HS-1005SGT 150KG on aztec, 800–2200 µs) to select gears.
 * Gear positions are stored and exposed as float percentages (0.0–100.0).
 * The physical gear is sensed as a 3-bit code on three opto inputs (aztec; see
 * `decodeGearCode()`) and confirms gear arrival.
 * Overshoot-and-return motion profile ensures mechanical detent engagement.
 */
class TransmissionController {
public:
    /**
     * @brief Transmission gear selection enumeration
     */
    enum class Gear {
        GEAR_HIGH = 0,
        GEAR_LOW = 1,
        GEAR_NEUTRAL = 2,
        GEAR_REVERSE = 3,
        GEAR_UNKNOWN = 4
    };

    TransmissionController();
    ~TransmissionController();

    /**
     * @brief Initialize the transmission servo
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Set transmission to target gear
     * @param gear Target gear
     * @return true if gear change initiated
     */
    bool setGear(Gear gear);

    /**
     * @brief Get current gear from the physical gear code inputs
     */
    Gear getCurrentGear() const;

/**
     * @brief Get default servo position for a gear (0.0–100.0 %)
     */
    float getGearPosition(Gear gear) const;

    /**
     * @brief Get human-readable gear name
     */
    const char* getGearName(Gear gear) const;

    /**
     * @brief Set vehicle data for safety interlocks
     */
    void setVehicleData(const TransmissionVehicleData& data);

    /**
     * @brief Returns true while a gear change is in progress (boost trigger)
     */
    bool needsThrottleBoost() const;

    /**
     * @brief Returns true while any gear change phase is active (servo moving or dwelling)
     */
    bool isGearChangeActive() const { return gearChangePhase_ != GearChangePhase::NONE; }

    /**
     * @brief Attach the opto-input reader that supplies the gear code.
     *
     * The three gear code lines are on the board's PCA9557 input expander (port
     * bits 0/1/6), so there is nothing to configure here beyond binding the
     * snapshot source. Without it `getPhysicalGear()` reports UNKNOWN.
     *
     * @param inputs Board input reader (must outlive this controller)
     */
    void initGearSensors(BoardInputs& inputs);

    /**
     * @brief Read the physical gear from the opto-input snapshot.
     *
     * Cached for `TRANS_GEAR_READ_INTERVAL_MS`. A code that decodes to UNKNOWN
     * (000, 100, 110, 111) while the servo is idle defers to the next expander poll
     * (at most `TRANS_GEAR_READ_RETRY_COUNT` times) instead of issuing back-to-back
     * reads; an invalid (stale) snapshot reports `GEAR_UNKNOWN`, which is the
     * intended degraded mode.
     */
    Gear getPhysicalGear() const;

    /**
     * @brief Decode a 3-bit gear code [bit0 bit1 bit6] into a gear.
     *
     * The ONE place codes are mapped to gears, through the runtime table
     * `gearCodes_` (defaults `GEAR_CODE_LOW/HIGH/NEUTRAL/REVERSE` from Constants.h,
     * overridden by the web calibration stored in NVS). Assigned codes are always
     * distinct and never 000 or 111; an UNSET gear never matches. Every other code — including 000
     * (mid-shift, all lines open) and 111 (opto feed lost, all float HIGH) — is
     * `GEAR_UNKNOWN`.
     */
    Gear decodeGearCode(uint8_t code) const;

    /** Runtime-table sentinel: gear has no code (lost it to a take-over); it never decodes. */
    static constexpr uint8_t GEAR_CODE_UNSET = 0xFF;

    /** @brief Active gear code for a gear (0..7), or GEAR_CODE_UNSET (also for GEAR_UNKNOWN) */
    uint8_t getGearCodeFor(Gear gear) const;

    /** @brief true if the active code for `gear` equals its Constants.h default */
    bool isGearCodeDefault(Gear gear) const;

    /**
     * @brief Load the gear code table from NVS ("transmission" code_r/n/h/l).
     *
     * A missing key keeps that gear's default; a stored GEAR_CODE_UNSET keeps the gear
     * unset. If the resulting table is invalid (an assigned code of 000/111/out of range,
     * or two gears sharing a code) the whole table falls back to the Constants.h
     * defaults and an error is logged.
     */
    void loadGearCodes();

    /**
     * @brief Capture the CURRENT live gear code as the code for `gear` (web calibration).
     *
     * Rejected (returns false, `msg` says why) when: `gear` is UNKNOWN, a gear change is
     * in progress, the opto-input snapshot is missing/stale, or the code is 000 or 111
     * (reserved for UNKNOWN). If the code is assigned to a different gear, this gear
     * takes it over and the other gear becomes GEAR_CODE_UNSET (reported in `msg`).
     * On success the change is persisted to NVS and takes effect immediately.
     */
    bool captureGearCode(Gear gear, String& msg);

    /**
     * @brief Restore the Constants.h default gear codes (NVS keys removed), effective immediately.
     * @return false if NVS could not be opened (RAM table is still reset)
     */
    bool resetGearCodes();

    /**
     * @brief Raw gear code from the current input snapshot (telemetry `gear_code`).
     * @return 0..7, or -1 when there is no input source or the snapshot is stale
     */
    int8_t getGearCode() const;

    /** @brief One-letter gear name: "R", "N", "H", "L", or "?" for UNKNOWN */
    static const char* getGearLetter(Gear gear);

    /**
     * @brief Update gear change phase transitions — call every loop iteration
     */
    void update();

    /**
     * @brief Restore servo position from NVS; return true to skip legacy autohome
     */
    bool restoreStateIfValid();

    /**
     * @brief Load user-configured gear positions from NVS
     */
    void loadDefaultPositions();

    /**
     * @brief Save and apply a gear's default position
     * @param gear  Target gear (must not be GEAR_UNKNOWN)
     * @param positionPct  Servo position 0.0–100.0 %
     * @return true if saved successfully
     */
    bool setDefaultPosition(Gear gear, float positionPct);

    /**
     * @brief Get the gear the transmission is moving toward (current step)
     */
    Gear getTargetGear() const { return targetGear_; }

    /**
     * @brief Get the gear the current step is moving away from.
     * Valid while a step is active; equals the destination once settled.
     */
    Gear getFromGear() const { return fromGear_; }

    /**
     * @brief No-op — servo holds position; retained for API compatibility
     */
    void stop() {}

    /**
     * @brief Save transmission state to NVS
     */
    void saveState(Gear gear, float positionPct, bool valid);

    /**
     * @brief Load transmission state from NVS
     * @return true if a state record was found
     */
    bool loadState(Gear& gear, float& positionPct, bool& valid);

    /**
     * @brief Get current commanded servo position (0.0–100.0 %)
     */
    float getCurrentServoPct() const {
        return (servo_.getMicroseconds() - TRANS_SERVO_MIN_US) / float(TRANS_SERVO_MAX_US - TRANS_SERVO_MIN_US) * 100.0f;
    }

    /**
     * @brief Move servo directly to a percentage (bypasses gear state machine — for calibration)
     */
    void moveToPercent(float pct);

    /**
     * @brief Load per-gear overshoot values from NVS
     */
    void loadGearOvershoots();

private:
    enum class GearChangePhase {
        NONE,
        OVERSHOOT,        // servo moving to overshoot position
        OVERSHOOT_DWELL,  // dwelling at overshoot for TRANS_OVERSHOOT_DWELL_MS
        RETURN            // servo moving back to final position (gear assumed engaged)
    };

    ServoController servo_;

    Gear targetGear_;
    Gear fromGear_;              // gear the current step is moving away from
    GearChangePhase gearChangePhase_;
    float finalGearPositionPct_;
    float overshootDirection_;   // +1.0 or -1.0, direction of overshoot from target
    uint32_t gearPhaseStartTime_;
    uint32_t settleMs_;

    TransmissionVehicleData vehicleData_;
    uint32_t lastGearCheckTime_;
    uint32_t lastStatusLogTime_;
    bool lastGearMismatch_;
    Gear lastLoggedGear_;
    bool lastLoggedMoving_;

    Gear queuedGear_;               // GEAR_UNKNOWN = no sequence pending, else = final destination
    uint32_t sequenceStepDwellStart_;  // millis() when step-dwell armed, 0 = not armed

    BoardInputs*     boardInputs_;         // gear code inputs, port bits 0/1/6 (not owned)

    mutable Gear     cachedPhysicalGear_;
    mutable uint32_t lastGearReadTime_;
    mutable uint32_t lastGearRetryTime_;   // last time an ambiguous read deferred to a newer snapshot
    mutable uint8_t  gearReadRetries_;     // deferrals used in the current read window
    mutable uint8_t  lastLoggedGearCode_;  // 0xFF until the first code is logged (never a valid code)

    uint8_t gearCodes_[4];                 // runtime code table, indexed by (int)Gear; GEAR_CODE_UNSET = none

    float gearPositions_[4];
    float overshootPcts_[4];

    bool canChangeGear(Gear targetGear) const;
    void commandServo(float positionPct);
    uint32_t computeSettleMs(float fromPct, float toPct) const;
    void saveDefaultPosition(Gear gear, float positionPct);
    void startGearChange(Gear gear);
    static Gear nextGearToward(Gear from, Gear to);
    static uint8_t defaultGearCode(Gear gear);
    static bool isGearCodeTableValid(const uint8_t codes[4]);
    void invalidateGearCache();
};

#endif // TRANSMISSION_CONTROLLER_H
