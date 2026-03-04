#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief Controller for PWM servo motors using ESP32 LEDC peripheral
 *
 * Provides hardware PWM control for standard hobby servos with configurable
 * pulse width ranges and angle mapping.
 */
class ServoController {
public:
    ServoController();
    ~ServoController();

    /**
     * @brief Initialize the servo controller
     *
     * @param pin GPIO pin number for PWM output
     * @param channel LEDC channel number (0-5 on ESP32-C6)
     * @param minUs Minimum pulse width in microseconds (default 1000)
     * @param maxUs Maximum pulse width in microseconds (default 2000)
     * @return true if initialization successful
     */
    bool begin(gpio_num_t pin, uint8_t channel, uint16_t minUs = 1000, uint16_t maxUs = 2000);

    /**
     * @brief Set servo position by angle
     *
     * @param degrees Angle in degrees (0-180)
     */
    void setAngle(float degrees);

    /**
     * @brief Set servo position by pulse width
     *
     * @param us Pulse width in microseconds
     */
    void setMicroseconds(uint16_t us);

    /**
     * @brief Get current servo angle
     *
     * @return Current angle in degrees
     */
    float getAngle() const { return currentAngle_; }

    /**
     * @brief Get current pulse width
     *
     * @return Current pulse width in microseconds
     */
    uint16_t getMicroseconds() const { return currentUs_; }

    /**
     * @brief Disable PWM output
     */
    void disable();

    /**
     * @brief Check if servo is initialized
     *
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

private:
    gpio_num_t pin_;
    uint8_t channel_;
    uint16_t minUs_;
    uint16_t maxUs_;
    float currentAngle_;
    uint16_t currentUs_;
    bool initialized_;

    /**
     * @brief Convert angle to pulse width
     *
     * @param degrees Angle in degrees
     * @return Pulse width in microseconds
     */
    uint16_t angleToPulseWidth(float degrees) const;

    /**
     * @brief Convert pulse width to PWM duty cycle
     *
     * @param us Pulse width in microseconds
     * @return PWM duty cycle value
     */
    uint32_t usToDutyCycle(uint16_t us) const;

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

#endif // SERVO_CONTROLLER_H
