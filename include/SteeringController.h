#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "AS5600Sensor.h"
#include "BTS7960Controller.h"

/**
 * @brief Steering actuator controller: BTS7960 H-bridge with proportional PWM,
 * absolute position feedback from an AS5600 magnetic angle sensor.
 *
 * All positions are AS5600 raw counts (0-4095, ~0.088°/LSB). Position control
 * works in wrap-safe counts relative to the calibrated center; center and
 * left/right travel limits are captured at the current wheel position via the
 * web UI and persisted in NVS ("steering"/"c_ang","l_ang","r_ang").
 *
 * No homing: position is absolute. On sensor fault (I2C failure or magnet
 * lost) the motor stops immediately and position commands are ignored until
 * valid readings return.
 */
class SteeringController {
public:
    SteeringController();

    /**
     * @brief Initialize the BTS7960 (LEDC PWM) and the AS5600 sensor
     * @return true if both the motor driver and the sensor initialize
     */
    bool begin(gpio_num_t rpwmPin, gpio_num_t lpwmPin,
               uint8_t rpwmChannel, uint8_t lpwmChannel,
               gpio_num_t sdaPin, gpio_num_t sclPin);

    /**
     * @brief Update sensor reading and position control — call every loop iteration
     */
    void update();

    /**
     * @brief Stop the motor and cancel any active move/jog
     */
    void stop();

    // === Steering interface ===

    /**
     * @brief Set steering by percentage: -100 (left limit) .. 0 (center) .. +100 (right limit).
     * Left and right ranges may be asymmetric.
     * @return false if rejected (not calibrated or sensor invalid)
     */
    bool setSteeringPercent(float percent);
    float getSteeringPercent() const;

    // === Calibration ===

    /**
     * @brief Momentary open-loop jog for calibration: -1 = left, +1 = right, 0 = stop.
     * Auto-stops if not refreshed within STEER_JOG_TIMEOUT_MS.
     */
    void jog(int8_t direction);

    /**
     * @brief Capture the current AS5600 angle as center / left limit / right limit
     * and persist to NVS. Limits are validated against center (opposite sides,
     * minimum STEER_CAL_MIN_SPAN counts).
     * @return true if captured and saved
     */
    bool captureCenter();
    bool captureLeftLimit();
    bool captureRightLimit();

    /**
     * @brief Load calibration from NVS. Call at init.
     */
    void loadCalibration();

    // === State ===

    bool isCalibrated() const { return calibrated_; }
    bool isSensorOk() const { return sensorOk_; }

    /** @brief Last AS5600 raw angle reading (0-4095) */
    int32_t getRawAngle() const { return rawAngle_; }

    /** @brief Calibrated raw angles (-1 if not set) */
    int32_t getCenter() const { return center_; }
    int32_t getLeftLimit() const { return leftLimit_; }
    int32_t getRightLimit() const { return rightLimit_; }

private:
    AS5600Sensor sensor_;
    BTS7960Controller motor_;

    // Sensor state
    int32_t rawAngle_;          // last valid raw reading (0-4095)
    bool sensorOk_;

    // Calibration (raw angles, -1 = not set)
    int32_t center_;
    int32_t leftLimit_;
    int32_t rightLimit_;
    bool calibrated_;
    bool invert_;               // sensor counts decrease when the wheel moves right
    int32_t relLeft_;           // normalized left limit relative to center (< 0)
    int32_t relRight_;          // normalized right limit relative to center (> 0)

    // Position control state (normalized counts relative to center)
    int32_t targetRel_;
    bool isMoving_;
    uint32_t moveStartTime_;

    // Jog state
    int8_t jogDir_;
    uint32_t lastJogRefresh_;

    // Stall detection
    int32_t lastStallPosition_;
    uint32_t lastStallCheckTime_;

    /** @brief Signed shortest delta from center, sensor-direction normalized (positive = right) */
    int32_t relPosition() const;

    /** @brief Signed shortest delta between two raw angles: ((a - b + 2048) & 4095) - 2048 */
    static int32_t wrapDelta(int32_t a, int32_t b);

    /** @brief Refresh sensor reading + magnet status (single I2C burst); stop the motor on fault */
    bool refreshSensor();

    /** @brief Recompute calibrated_/invert_/relLeft_/relRight_ from stored angles */
    void revalidateCalibration();

    /** @brief Persist calibration angles to NVS */
    void saveCalibration();
};

#endif // STEERING_CONTROLLER_H
