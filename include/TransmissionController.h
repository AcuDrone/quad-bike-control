#ifndef TRANSMISSION_CONTROLLER_H
#define TRANSMISSION_CONTROLLER_H

#include "ServoController.h"
#include "Constants.h"
#include <Preferences.h>

/**
 * @brief Vehicle data structure for transmission safety checks
 */
struct TransmissionVehicleData {
    uint8_t vehicleSpeed;       // km/h (0-255)
    uint32_t lastUpdateTime;    // millis() timestamp
    bool dataValid;             // true if CAN communication is healthy
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
     * @return true if gear change initiated
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
     * @brief Configure gear selector switch GPIO pins
     */
    void initGearSensors();

    /**
     * @brief Read physical gear from hall-effect switches
     */
    Gear getPhysicalGear() const;

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
     * @brief Get the gear the transmission is moving toward
     */
    Gear getTargetGear() const { return targetGear_; }

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
     * @brief Get overshoot percent for a specific gear
     */
    float getGearOvershoot(Gear gear) const;

    /**
     * @brief Save and apply per-gear overshoot percent (0.0–20.0 %)
     * @return true if saved successfully
     */
    bool setGearOvershoot(Gear gear, float overshootPct);

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

    mutable Gear     cachedPhysicalGear_;
    mutable uint32_t lastGearReadTime_;

    float gearPositions_[4];
    float overshootPcts_[4];

    bool canChangeGear(Gear targetGear) const;
    void commandServo(float positionPct);
    uint32_t computeSettleMs(float fromPct, float toPct) const;
    void saveDefaultPosition(Gear gear, float positionPct);
    void startGearChange(Gear gear);
    static Gear nextGearToward(Gear from, Gear to);
};

#endif // TRANSMISSION_CONTROLLER_H
