#ifndef ENCODER_COUNTER_H
#define ENCODER_COUNTER_H

#include <Arduino.h>
#include "driver/pulse_cnt.h"

/**
 * @brief Incremental encoder counter using ESP32 PCNT peripheral
 *
 * Tracks position using hardware quadrature decoding via PCNT.
 *
 * Features:
 * - Hardware quadrature decoding (no CPU overhead)
 * - Direction detection (up/down counting)
 * - Zero/reset position
 * - Get absolute position
 */
class EncoderCounter {
public:
    EncoderCounter();
    ~EncoderCounter();

    /**
     * @brief Initialize encoder counter with PCNT
     *
     * @param channelA GPIO pin for encoder channel A
     * @param channelB GPIO pin for encoder channel B
     * @param unitId PCNT unit ID (0-3 on ESP32-C6)
     * @return true if initialization successful
     */
    bool begin(gpio_num_t channelA, gpio_num_t channelB, int unitId = 0);

    /**
     * @brief Get current encoder position
     *
     * @return Current position in encoder counts
     */
    int32_t getPosition() const;

    /**
     * @brief Set encoder position (for calibration/homing)
     *
     * @param position New position value
     */
    void setPosition(int32_t position);

    /**
     * @brief Reset encoder to zero
     */
    void reset();

    /**
     * @brief Check if encoder is initialized
     *
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get position change since last call
     *
     * @return Delta position (change since last getDelta() call)
     */
    int32_t getDelta();

    /**
     * @brief Get raw hardware counter value (for diagnostics)
     *
     * @return Raw PCNT counter value
     */
    int getRawCount() const { return readHardwareCounter(); }

    /**
     * @brief Read current GPIO pin states (for diagnostics)
     *
     * @param stateA Output: state of channel A (0 or 1)
     * @param stateB Output: state of channel B (0 or 1)
     */
    void readGPIOStates(int& stateA, int& stateB) const;

private:
    pcnt_unit_handle_t pcntUnit_;
    pcnt_channel_handle_t pcntChannelA_;
    pcnt_channel_handle_t pcntChannelB_;
    gpio_num_t channelA_;
    gpio_num_t channelB_;
    int32_t offsetPosition_;  // Offset for software position adjustment
    int32_t lastPosition_;    // Last position for delta calculation
    bool initialized_;

    /**
     * @brief Read raw hardware counter value
     *
     * @return Raw PCNT counter value
     */
    int readHardwareCounter() const;

    /**
     * @brief Update internal position tracking
     */
    void updatePosition();
};

#endif // ENCODER_COUNTER_H
