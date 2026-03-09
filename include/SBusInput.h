#ifndef SBUSINPUT_H
#define SBUSINPUT_H

#include <Arduino.h>
#include "sbus.h"
#include "Constants.h"
#include "TransmissionController.h"

/**
 * @brief SBUS input handler for ArduPilot Rover control
 *
 * Wraps Bolder Flight SBUS library and provides vehicle-specific command mapping
 * for steering, throttle, transmission, brake, ignition, and lights.
 */
class SBusInput {
public:
    /**
     * @brief Ignition state enum for relay control
     */
    enum class IgnitionState {
        OFF,        // All power off
        ACC,        // Accessory power only
        IGNITION    // Full ignition (accessory + main power, triggers auto-cranking)
    };

    /**
     * @brief Signal quality metrics structure
     */
    struct SignalQuality {
        uint32_t totalFrames;        // Total frames received since init
        uint32_t errorFrames;        // Frames with validation errors
        float frameRate;             // Current frame rate (Hz)
        float errorRate;             // Error percentage (0-100)
        uint32_t signalAge;          // Time since last valid frame (ms)
        bool isValid;                // Signal within timeout threshold
    };

    SBusInput();

    /**
     * @brief Initialize SBUS receiver
     * @param rxPin UART RX pin (default: PIN_SBUS_RX)
     * @param uartNum UART number (default: SBUS_UART_NUM)
     * @return true if initialization successful
     */
    bool begin(uint8_t rxPin = PIN_SBUS_RX, uint8_t uartNum = SBUS_UART_NUM);

    /**
     * @brief Update SBUS receiver - call every loop iteration
     * Reads available frames, decodes channels, updates statistics
     */
    void update();

    // ============================================================================
    // RAW CHANNEL ACCESS
    // ============================================================================

    /**
     * @brief Get raw channel value in microseconds
     * @param channel Channel number (1-16)
     * @return Channel value in microseconds (880-2160), 0 if invalid
     */
    uint16_t getChannel(uint8_t channel) const;

    /**
     * @brief Get all 16 raw channel values
     * @param channels Array to fill with channel values (must be size 16)
     */
    void getRawChannels(uint16_t* channels) const;

    // ============================================================================
    // MAPPED VEHICLE COMMANDS
    // ============================================================================

    /**
     * @brief Get steering command
     * @return Steering percentage (-100 to +100), 0 = center
     */
    float getSteering() const;

    /**
     * @brief Get throttle command
     * @return Throttle percentage (0 to 100), 0 = idle
     */
    float getThrottle() const;

    /**
     * @brief Get gear selection
     * @return Gear enum (REVERSE/NEUTRAL/LOW)
     */
    TransmissionController::Gear getGear() const;

    /**
     * @brief Get brake command
     * @return Brake percentage (0 to 100), 0 = released
     */
    float getBrake() const;

    /**
     * @brief Get ignition state
     * @return Ignition state enum (OFF/ACC/IGNITION)
     */
    IgnitionState getIgnitionState() const;

    /**
     * @brief Get front light state
     * @return true if light should be ON, false if OFF
     */
    bool getFrontLight() const;

    // ============================================================================
    // SIGNAL MONITORING
    // ============================================================================

    /**
     * @brief Check if SBUS signal is valid (within timeout)
     * @return true if signal received within SBUS_SIGNAL_TIMEOUT
     */
    bool isSignalValid() const;

    /**
     * @brief Get time since last valid frame
     * @return Age in milliseconds
     */
    uint32_t getSignalAge() const;

    /**
     * @brief Get comprehensive signal quality metrics
     * @return SignalQuality struct with all metrics
     */
    SignalQuality getSignalQuality() const;

private:
    // SBUS library object (pointer because it requires HardwareSerial in constructor)
    bfs::SbusRx* sbus_;
    bfs::SbusData sbusData_;

    // Signal tracking
    uint32_t lastFrameTime_;
    uint32_t totalFrames_;
    uint32_t errorFrames_;
    uint32_t lastFrameRate_;
    uint32_t frameRateUpdateTime_;

    // Helper methods
    float applyDeadband(float value, float center, float deadband) const;
    uint16_t rawToMicroseconds(uint16_t rawValue) const;
    float mapToPercentage(uint16_t valueUs, uint16_t minUs, uint16_t maxUs) const;
    float mapToPercentageBidirectional(uint16_t valueUs, uint16_t minUs, uint16_t centerUs, uint16_t maxUs) const;
};

/**
 * @brief Get ignition state name as string
 * @param state Ignition state enum
 * @return State name string
 */
inline const char* getIgnitionStateName(SBusInput::IgnitionState state) {
    switch (state) {
        case SBusInput::IgnitionState::OFF: return "OFF";
        case SBusInput::IgnitionState::ACC: return "ACC";
        case SBusInput::IgnitionState::IGNITION: return "IGNITION";
        default: return "UNKNOWN";
    }
}

#endif // SBUSINPUT_H
