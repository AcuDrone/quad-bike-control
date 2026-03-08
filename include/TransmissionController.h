#ifndef TRANSMISSION_CONTROLLER_H
#define TRANSMISSION_CONTROLLER_H

#include "BTS7960Controller.h"
#include "Constants.h"

/**
 * @brief Vehicle data structure for transmission safety checks
 */
struct TransmissionVehicleData {
    uint8_t vehicleSpeed;       // km/h (0-255)
    uint32_t lastUpdateTime;    // millis() timestamp
    bool dataValid;             // true if CAN communication is healthy
};

/**
 * @brief High-level transmission controller with gear selection
 *
 * Extends BTS7960Controller with gear-based interface.
 * Translates gear enum (R, N, L, H) to calibrated encoder positions.
 *
 * Features:
 * - Enum-based gear selection (HIGH, LOW, NEUTRAL, REVERSE)
 * - Automatic position control to target gear
 * - Calibrated encoder positions for each gear
 * - Status reporting (current gear, moving state)
 */
class TransmissionController : public BTS7960Controller {
public:
    /**
     * @brief Transmission gear selection enumeration
     *
     * Four gears with calibrated encoder positions:
     * - GEAR_HIGH (H): 0 counts (home position)
     * - GEAR_LOW (L): -300 counts
     * - GEAR_NEUTRAL (N): -5000 counts
     * - GEAR_REVERSE (R): -10000 counts
     */
    enum class Gear {
        GEAR_HIGH = 0,      // H gear at encoder position 0
        GEAR_LOW = 1,       // L gear at encoder position -300
        GEAR_NEUTRAL = 2,   // N gear at encoder position -5000
        GEAR_REVERSE = 3    // R gear at encoder position -10000
    };

    TransmissionController();
    ~TransmissionController();

    /**
     * @brief Set transmission to target gear
     *
     * Automatically moves actuator to calibrated position for selected gear.
     * Uses encoder feedback for closed-loop position control.
     *
     * @param gear Target gear (HIGH, LOW, NEUTRAL, REVERSE)
     * @param speed Movement speed (0-255), default 150
     * @return true if gear change initiated, false if already at target or error
     */
    bool setGear(Gear gear, uint8_t speed = 150);

    /**
     * @brief Get current gear based on encoder position
     *
     * Determines current gear by comparing encoder position to calibrated positions.
     * Uses TRANS_POSITION_TOLERANCE for position matching.
     *
     * @return Current gear, or NEUTRAL if position doesn't match any gear
     */
    Gear getCurrentGear() const;

    /**
     * @brief Check if transmission is at target gear
     *
     * @param gear Target gear to check
     * @return true if at target gear position (within tolerance)
     */
    bool isAtGear(Gear gear) const;

    /**
     * @brief Get encoder position for a given gear
     *
     * @param gear Gear to query
     * @return Encoder position for the gear
     */
    int32_t getGearPosition(Gear gear) const;

    /**
     * @brief Get human-readable gear name
     *
     * @param gear Gear to convert to string
     * @return Gear name (e.g., "HIGH", "LOW", "NEUTRAL", "REVERSE")
     */
    const char* getGearName(Gear gear) const;

    /**
     * @brief Set vehicle data from CAN bus for safety checks
     *
     * @param data Vehicle data (speed, validity, etc.)
     */
    void setVehicleData(const TransmissionVehicleData& data);

    /**
     * @brief Check if transmission needs throttle boost
     *
     * @return true if gear change is in progress and boost is needed
     */
    bool needsThrottleBoost() const;

private:
    Gear targetGear_;  // Target gear for current move
    TransmissionVehicleData vehicleData_;  // Vehicle data for safety checks

    /**
     * @brief Check if gear change is safe based on vehicle speed
     *
     * @param targetGear Target gear for change
     * @return true if gear change is allowed, false if blocked
     */
    bool canChangeGear(Gear targetGear) const;
};

#endif // TRANSMISSION_CONTROLLER_H
