#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "EncoderCounter.h"

/**
 * @brief Steering actuator controller using BTS7960 H-bridge at full speed
 *
 * Uses plain GPIO digital write for direction (no PWM/LEDC needed).
 * Position feedback via hall sensor encoder (PCNT).
 * Auto-homes to left physical limit on startup, then moves to center.
 * Left limit = 0 (home), center = runtime-calibrated center_ (NVS), right limit = 2*center_.
 */
class SteeringController {
public:
    SteeringController();

    /**
     * @brief Initialize GPIO pins for BTS7960 direction control
     * @param rpwmPin GPIO pin for right movement
     * @param lpwmPin GPIO pin for left movement
     * @return true if initialization successful
     */
    bool begin(gpio_num_t rpwmPin, gpio_num_t lpwmPin);

    /**
     * @brief Attach encoder for position feedback
     */
    void attachEncoder(EncoderCounter* encoder);

    // === Movement (full speed) ===

    void moveRight();
    void moveLeft();
    void stop();

    // === Position control ===

    /**
     * @brief Set target position and start moving
     */
    void setPosition(int32_t targetPosition);

    /**
     * @brief Update position control — call every loop iteration
     */
    void update();

    bool isAtPosition(int32_t target, int32_t tolerance = STEER_POSITION_TOLERANCE) const;
    int32_t getPosition() const;
    bool isMoving() const { return isMoving_; }

    // === Steering interface ===

    /**
     * @brief Set steering by percentage
     * @param percent -100 (full left) to +100 (full right), 0 = center
     */
    void setSteeringPercent(float percent);
    float getSteeringPercent() const;

    // === Calibration ===

    /**
     * @brief Auto-home to physical left limit (stall detection)
     * Resets encoder to 0 at the limit.
     * @return true if home found, false on timeout
     */
    bool autoHome(uint32_t timeout = STEER_HOMING_TIMEOUT);

    /**
     * @brief Get the calibrated center (straight-ahead) position in encoder counts.
     */
    int32_t getCenter() const { return center_; }

    /**
     * @brief Set and persist the center position (encoder counts). Updates the live
     * steering mapping immediately (right limit becomes 2*center). Rejects count <= 0.
     * @return true if accepted and saved
     */
    bool setCenter(int32_t count);

    /**
     * @brief Drive the actuator to a raw encoder position for calibration preview.
     * Bypasses the percent mapping; clamps to >= 0. Honors move/stall timeouts.
     */
    void moveToRawCount(int32_t count);

    /**
     * @brief Load the center position from NVS (default STEER_DEFAULT_CENTER). Call at init.
     */
    void loadCenter();

private:
    gpio_num_t rpwmPin_;
    gpio_num_t lpwmPin_;
    EncoderCounter* encoder_;

    int32_t center_;   // calibrated straight-ahead position (encoder counts); right limit = 2*center_

    // Position control state
    int32_t targetPosition_;
    bool isMoving_;
    uint32_t moveStartTime_;

    // Stall detection
    int32_t lastStallPosition_;
    uint32_t lastStallCheckTime_;
};

#endif // STEERING_CONTROLLER_H
