#ifndef BTS7960_CONTROLLER_H
#define BTS7960_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"

/**
 * @brief Controller for BTS7960 dual H-bridge driver for linear actuators
 *
 * Controls the brake linear actuator via BTS7960 H-bridge driver with
 * hardwired enable pins.
 * Uses RPWM (extend) and LPWM (retract) for speed/direction control.
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

private:
    gpio_num_t rpwmPin_;
    gpio_num_t lpwmPin_;
    uint8_t rpwmChannel_;
    uint8_t lpwmChannel_;
    int16_t currentSpeed_;
    uint8_t currentRPWM_;  // Track current RPWM value
    uint8_t currentLPWM_;  // Track current LPWM value
    bool initialized_;

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
