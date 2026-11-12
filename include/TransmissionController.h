#ifndef TRANSMISSION_CONTROLLER_H
#define TRANSMISSION_CONTROLLER_H

#include "ServoController.h"
#include "Constants.h"
#include "BoardInputs.h"
#include "TransmissionLaw.h"
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
 * Controls a PWM servo (HappyModel Super400 Plus, 800–2200 µs) to select gears.
 * Gear positions are stored and exposed as float percentages (0.0–100.0).
 * Physical hall-effect switches confirm gear arrival.
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
     * @return true if gear change initiated (or queued); false while not initialised
     */
    bool setGear(Gear gear);

    /**
     * @brief Get current gear from physical switches
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
     * @brief Attach the opto-input reader that supplies the gear switches.
     *
     * The four gear switches are no longer on ESP32 GPIO: they are In1-In4 of the
     * board's PCA9557 input expander, so there is nothing to configure here beyond
     * binding the snapshot source. Without it `getPhysicalGear()` reports UNKNOWN.
     *
     * @param inputs Board input reader (must outlive this controller)
     */
    void initGearSensors(BoardInputs& inputs);

    /**
     * @brief Read the physical gear from the opto-input snapshot.
     *
     * Cached for `TRANS_GEAR_READ_INTERVAL_MS`. An ambiguous reading (none or more
     * than one switch asserted) while the servo is idle and no sequence step is queued defers to the next expander
     * poll instead of issuing back-to-back reads; an invalid (stale) snapshot
     * reports `GEAR_UNKNOWN`, which is the intended degraded mode.
     */
    Gear getPhysicalGear() const;

    /**
     * @brief Update gear change phase transitions — call every loop iteration
     */
    void update();

    /**
     * @brief True once the expected gear is known (targetGear_ != GEAR_UNKNOWN).
     *
     * GEAR_UNKNOWN as the expected gear only ever means "never initialised":
     * startGearChange() never writes it.
     */
    bool isInitialised() const { return targetGear_ != Gear::GEAR_UNKNOWN; }

    /**
     * @brief Initialise the expected gear once (no-op when already initialised).
     *
     * Source, in order (decided by chooseTransmissionInit() in TransmissionLaw.h):
     * the physical sensor reading exactly one gear (direct move to that gear's position,
     * state saved valid); else a valid, plausible NVS state (direct move to state_pct);
     * else a real shift step into NEUTRAL (overshoot with TRANS_INIT_SETTLE_MS, dwell,
     * return) whose overshoot direction comes from the invalid NVS record.
     * Always leaves the controller initialised.
     */
    void initialise();

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
     * @brief Final destination: the queued gear of a multi-step sequence, else the target.
     */
    Gear getDestinationGear() const {
        return (queuedGear_ != Gear::GEAR_UNKNOWN) ? queuedGear_ : targetGear_;
    }

    /**
     * @brief Speed interlock for a gear change (N always allowed). Read-only query, the same
     * check setGear() runs; public for the GCS gear command's reason code.
     */
    bool canChangeGear(Gear targetGear) const;

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
        OVERSHOOT_DWELL,  // dwelling at overshoot until sensor-confirmed (TRANS_DWELL_CONFIRM_MS) or TRANS_OVERSHOOT_DWELL_MS
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
    TransmissionResync resync_;     // idle consecutive-mismatch counter
    bool initStepActive_;           // the initialisation step into NEUTRAL is running
    Gear lastLoggedGear_;
    bool lastLoggedMoving_;

    Gear queuedGear_;               // GEAR_UNKNOWN = no sequence pending, else = final destination
    uint32_t sequenceStepDwellStart_;  // millis() when step-dwell armed, 0 = not armed
    ShiftConfirm dwellConfirm_;     // sensor confirmation of the overshoot dwell
    ShiftConfirm pauseConfirm_;     // sensor confirmation of the inter-step pause
    uint32_t shiftStartMs_;         // millis() when the current single step / sequence started
    Gear shiftFromGear_;            // gear the current single step / sequence started from

    BoardInputs*     boardInputs_;         // gear switches In1-In4 (not owned)

    mutable Gear     cachedPhysicalGear_;
    mutable uint32_t lastGearReadTime_;
    mutable uint32_t lastGearRetryTime_;   // last time an ambiguous read deferred to a newer snapshot
    mutable uint8_t  gearReadRetries_;     // deferrals used in the current read window

    float gearPositions_[4];
    float overshootPcts_[4];

    void commandServo(float positionPct);
    uint32_t computeSettleMs(float fromPct, float toPct) const;
    void saveDefaultPosition(Gear gear, float positionPct);
    void startGearChange(Gear gear);
    void startInitStep(float direction);
    void markStateInvalid();
    static Gear nextGearToward(Gear from, Gear to);
};

#endif // TRANSMISSION_CONTROLLER_H
