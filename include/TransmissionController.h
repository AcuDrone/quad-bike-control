#ifndef TRANSMISSION_CONTROLLER_H
#define TRANSMISSION_CONTROLLER_H

#include "BTS7960Controller.h"
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
     * Four gears with calibrated encoder positions (relative to physical stop at 0):
     * - GEAR_REVERSE (R): ~50 counts
     * - GEAR_NEUTRAL (N): ~2000 counts
     * - GEAR_LOW (L): ~4000 counts
     * - GEAR_HIGH (H): ~6000 counts
     * - GEAR_UNKNOWN: Invalid/unclear gear state
     *
     * All positions are auto-calibrated during startup by detecting physical sensors.
     */
    enum class Gear {
        GEAR_HIGH = 0,      // H gear
        GEAR_LOW = 1,       // L gear
        GEAR_NEUTRAL = 2,   // N gear
        GEAR_REVERSE = 3,   // R gear
        GEAR_UNKNOWN = 4    // Invalid or unclear gear state
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
     * @param speed Movement speed (0-255), default 255 (full speed)
     * @return true if gear change initiated, false if already at target or error
     */
    bool setGear(Gear gear, uint8_t speed = 255);

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

    /**
     * @brief Initialize gear position sensors on MCP23017
     *
     * Configures gear selector input pins on MCP23017 Port B with pull-ups.
     * Active-low configuration: pin reads LOW when gear is selected.
     *
     */
    void initGearSensors();

    /**
     * @brief Read physical gear position from GPIO sensors
     *
     * Reads the gear selector position switches to determine which gear is physically selected.
     * Uses active-low logic: the pin that reads LOW indicates the selected gear.
     *
     * @return Physical gear position based on switch state, or GEAR_NEUTRAL if no switch active or multiple active
     */
    Gear getPhysicalGear() const;

    /**
     * @brief Check if encoder position matches physical gear sensors
     *
     * Compares the encoder-based gear position with the physical switch position
     * for safety verification and diagnostics.
     *
     * @return true if encoder gear matches physical switch gear, false if mismatch
     */
    bool isGearPositionValid() const;

    /**
     * @brief Update transmission position control with physical gear verification
     *
     * Extends BTS7960Controller::update() to add physical gear sensor verification.
     * Checks that encoder position matches physical gear switches and logs warnings
     * if mismatches are detected.
     *
     * Call this in main loop to handle position control and safety monitoring.
     */
    void update();

    /**
     * @brief Restore encoder position from NVS and skip autohome if state is valid
     *
     * Loads saved transmission state. If valid=true and the physical gear switch
     * matches the saved gear, restores the encoder to the saved position and
     * returns true (caller skips autohome). Otherwise returns false.
     *
     * @return true if state restored (skip autohome), false if autohome is needed
     */
    bool restoreStateIfValid();

    /**
     * @brief Load user-configured default positions from NVS into gearPositions_
     *
     * Reads def_reverse/neutral/low/high keys. Sets 0 for any absent key.
     * Call before loadCalibration() so calibration overwrites if present.
     */
    void loadDefaultPositions();

    /**
     * @brief Set and persist a single gear's default position
     *
     * Validates range (0–10000), writes to NVS, and updates gearPositions_ immediately.
     *
     * @param gear Target gear (must not be GEAR_UNKNOWN)
     * @param position Encoder count (0–10000)
     * @return true if saved, false if gear invalid or position out of range
     */
    bool setDefaultPosition(Gear gear, int32_t position);

private:
    enum class GearChangePhase {
        NONE,      // No active gear change, or single-phase direct move
        OVERSHOOT, // Phase 1: moving past the target by TRANS_GEAR_OVERSHOOT counts
        RETURN     // Phase 2: returning from overshoot to final gear position
    };

    Gear targetGear_;  // Target gear for current move
    GearChangePhase gearChangePhase_;  // Current overshoot phase
    int32_t finalGearPosition_;        // True target position, saved during Phase 1
    TransmissionVehicleData vehicleData_;  // Vehicle data for safety checks
    uint32_t lastGearCheckTime_;  // Timestamp of last physical gear check (ms)
    uint32_t lastMovementLogTime_;  // Timestamp of last movement log (ms)
    uint32_t lastStatusLogTime_;  // Timestamp of last status log (ms)
    bool lastGearMismatch_;  // Track if last check had a mismatch (avoid spam)
    Gear lastLoggedGear_;  // Last logged gear (to detect changes)
    bool lastLoggedMoving_;  // Last logged movement state (to detect changes)

    int32_t gearPositions_[4];  // Active gear encoder positions (user defaults)

    void saveState(Gear gear, int32_t position, bool valid);
    bool loadState(Gear& gear, int32_t& position, bool& valid);
    void saveDefaultPosition(Gear gear, int32_t position);

    /**
     * @brief Check if gear change is safe based on vehicle speed
     *
     * @param targetGear Target gear for change
     * @return true if gear change is allowed, false if blocked
     */
    bool canChangeGear(Gear targetGear) const;
};

#endif // TRANSMISSION_CONTROLLER_H
