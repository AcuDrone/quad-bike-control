#ifndef BTS7960_CONTROLLER_H
#define BTS7960_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "EncoderCounter.h"

/**
 * @brief Controller for BTS7960 dual H-bridge driver for linear actuators
 *
 * Controls linear actuators (brake and transmission gear selector) via BTS7960
 * H-bridge driver with hardwired enable pins.
 * Uses RPWM (extend) and LPWM (retract) for position control.
 *
 * CRITICAL SAFETY NOTES:
 * - R_EN and L_EN pins are hardwired to 5V (always enabled)
 * - NEVER set both RPWM and LPWM high simultaneously - this will SHORT
 *   the H-bridge and DESTROY the BTS7960 controller!
 * - This class enforces this safety constraint automatically
 * - Only one direction can be active at a time (extend OR retract, never both)
 * - BTS7960 does NOT support electrical braking via simultaneous PWM
 */
class BTS7960Controller {
public:
    BTS7960Controller();
    ~BTS7960Controller();

    /**
     * @brief Initialize the actuator controller
     *
     * @param rpwmPin GPIO pin for RPWM (extend/forward PWM)
     * @param lpwmPin GPIO pin for LPWM (retract/reverse PWM)
     * @param rpwmChannel LEDC channel for RPWM
     * @param lpwmChannel LEDC channel for LPWM
     * @return true if initialization successful
     */
    bool begin(gpio_num_t rpwmPin, gpio_num_t lpwmPin, uint8_t rpwmChannel, uint8_t lpwmChannel);

    /**
     * @brief Set actuator movement speed and direction
     *
     * @param speed Speed value: -255 (full retract) to +255 (full extend), 0 = stop
     */
    void setSpeed(int16_t speed);

    /**
     * @brief Stop actuator (coast)
     *
     * Sets both PWM outputs to 0, actuator coasts to stop
     */
    void stop();

    /**
     * @brief Brake actuator (same as stop for BTS7960)
     *
     * BTS7960 does NOT support electrical braking via simultaneous RPWM+LPWM
     * (this would short the H-bridge and destroy the controller!)
     * This method is an alias for stop() for API compatibility.
     */
    void brake();

    /**
     * @brief Get current actuator speed
     *
     * @return Current speed (-255 to +255)
     */
    int16_t getSpeed() const { return currentSpeed_; }

    /**
     * @brief Check if actuator is stopped
     *
     * @return true if speed is 0
     */
    bool isStopped() const { return currentSpeed_ == 0; }

    /**
     * @brief Check if controller is initialized
     *
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Attach encoder for position feedback
     *
     * @param encoder Pointer to EncoderCounter instance
     */
    void attachEncoder(EncoderCounter* encoder);

    /**
     * @brief Check if encoder is attached
     *
     * @return true if encoder is attached
     */
    bool hasEncoder() const { return encoder_ != nullptr; }

    /**
     * @brief Get current encoder position (if encoder attached)
     *
     * @return Current position in encoder counts, or 0 if no encoder
     */
    int32_t getPosition() const;

    /**
     * @brief Move to target position (requires encoder)
     *
     * @param targetPosition Target position in encoder counts
     * @param speed Speed to move (0-255)
     * @return true if moving, false if no encoder or already at position
     */
    bool moveToPosition(int32_t targetPosition, uint8_t speed = 150);

    /**
     * @brief Check if at target position (within tolerance)
     *
     * @param targetPosition Target position to check
     * @param tolerance Position tolerance (+/- counts)
     * @return true if at target position
     */
    bool isAtPosition(int32_t targetPosition, int32_t tolerance = TRANS_POSITION_TOLERANCE) const;

    /**
     * @brief Auto-home: drive to limit until stall, then set zero
     *
     * @param direction Direction to home: positive = extend, negative = retract
     * @param homingSpeed Speed during homing (0-255)
     * @param timeout Maximum time for homing (ms)
     * @return true if homing successful
     */
    bool autoHome(int8_t direction, uint8_t homingSpeed = TRANS_HOMING_SPEED, uint32_t timeout = TRANS_HOMING_TIMEOUT);

    /**
     * @brief Update position control (call in main loop if using position control)
     */
    void update();

    /**
     * @brief Check if position control is currently active
     *
     * @return true if actively moving to target position
     */
    bool isPositionControlActive() const { return positionControlActive_; }

    /**
     * @brief Stop position control and clear target
     *
     * Stops the actuator and deactivates position control mode.
     */
    void stopPositionControl();

    /**
     * @brief Recalibrate encoder to expected position
     *
     * Adjusts encoder position to match expected value (used when physical
     * sensors confirm position but encoder has drifted).
     *
     * @param expectedPosition The correct position in encoder counts
     */
    void recalibrateEncoder(int32_t expectedPosition);

private:
    gpio_num_t rpwmPin_;
    gpio_num_t lpwmPin_;
    uint8_t rpwmChannel_;
    uint8_t lpwmChannel_;
    int16_t currentSpeed_;
    uint8_t currentRPWM_;  // Track current RPWM value
    uint8_t currentLPWM_;  // Track current LPWM value
    bool initialized_;

    // Encoder and position control
    EncoderCounter* encoder_;      // Optional encoder for position feedback
    int32_t targetPosition_;       // Target position for position control
    bool positionControlActive_;   // Flag for active position control
    uint32_t lastEncoderChange_;   // Timestamp of last encoder change (for stall detection)
    int32_t lastEncoderPosition_;  // Last encoder position (for stall detection)

    /**
     * @brief Set RPWM duty cycle (extend/forward)
     *
     * SAFETY: Automatically clears LPWM to prevent simultaneous extend/retract
     *
     * @param duty PWM duty cycle (0-255)
     */
    void setRPWM(uint8_t duty);

    /**
     * @brief Set LPWM duty cycle (retract/reverse)
     *
     * SAFETY: Automatically clears RPWM to prevent simultaneous extend/retract
     *
     * @param duty PWM duty cycle (0-255)
     */
    void setLPWM(uint8_t duty);

    /**
     * @brief Clamp value to valid range
     *
     * @param value Value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped value
     */
    template<typename T>
    T clamp(T value, T min, T max) const {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
};

#endif // BTS7960_CONTROLLER_H
