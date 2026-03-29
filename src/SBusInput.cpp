#include "SBusInput.h"
#include "Debug.h"

SBusInput::SBusInput()
    : sbus_(nullptr),
      lastFrameTime_(0),
      totalFrames_(0),
      errorFrames_(0),
      lastFrameRate_(0),
      frameRateUpdateTime_(0) {
}

bool SBusInput::begin(uint8_t rxPin, uint8_t uartNum) {
    // Get HardwareSerial based on UART number
    // ESP32-C6 has UART0 (Serial) and UART1 (Serial1)
    HardwareSerial* serial;
    if (uartNum == UART_NUM_1) {
        serial = &Serial1;
    } else if (uartNum == UART_NUM_0) {
        serial = &Serial;
    } else {
        Debug::printlnFeature(DebugFeature::SBUS,"[SBUS] ERROR: Invalid UART number (ESP32-C6 has UART0 and UART1 only)");
        return false;
    }
    
    sbus_ = new bfs::SbusRx(serial, rxPin, -1, true);
    sbus_->Begin();

    // Reset statistics
    lastFrameTime_ = millis();
    totalFrames_ = 0;
    errorFrames_ = 0;
    frameRateUpdateTime_ = millis();

    Debug::printfFeature(DebugFeature::SBUS,"[SBUS] Initialized on UART%d, RX pin %d\n", uartNum, rxPin);
    return true;
}

void SBusInput::update() {
    if (!sbus_) {
        return;  // Not initialized
    }

    // Read SBUS data if available
    if (sbus_->Read()) {
        // Get decoded data
        sbusData_ = sbus_->data();

        // Update frame timestamp
        lastFrameTime_ = millis();
        totalFrames_++;

        // Check for errors (lost frames or failsafe)
        if (sbusData_.lost_frame || sbusData_.failsafe) {
            errorFrames_++;
        }
    }

    // Log all channels every 1 second
    static uint32_t lastLogTime = 0;
    static uint32_t byteCount = 0;
    uint32_t now = millis();

    // Count raw bytes available on the UART (bypass SBUS library)
    HardwareSerial* serial = (SBUS_UART_NUM == UART_NUM_1) ? &Serial1 : &Serial;
    byteCount += serial->available();

    if (now - lastLogTime >= 1000) {
        lastLogTime = now;
        Debug::printfFeature(DebugFeature::SBUS,
            "[SBUS] frames:%lu errors:%lu age:%lums valid:%s uartBytes:%lu\n",
            totalFrames_, errorFrames_, getSignalAge(), isSignalValid() ? "Y" : "N", byteCount);
        Debug::printfFeature(DebugFeature::SBUS,
            "[SBUS] raw: %u %u %u %u %u %u %u %u\n",
            sbusData_.ch[0], sbusData_.ch[1], sbusData_.ch[2], sbusData_.ch[3],
            sbusData_.ch[4], sbusData_.ch[5], sbusData_.ch[6], sbusData_.ch[7]);
        Debug::printfFeature(DebugFeature::SBUS,
            "[SBUS] CH1:%4u CH2:%4u CH3:%4u CH4:%4u CH5:%4u CH6:%4u CH7:%4u CH8:%4u\n",
            getChannel(1), getChannel(2), getChannel(3), getChannel(4),
            getChannel(5), getChannel(6), getChannel(7), getChannel(8));
        byteCount = 0;
    }
}

// ============================================================================
// RAW CHANNEL ACCESS
// ============================================================================

uint16_t SBusInput::getChannel(uint8_t channel) const {
    if (channel < 1 || channel > 16) {
        return 0;  // Invalid channel
    }

    // SBUS library uses 0-based indexing
    uint16_t rawValue = sbusData_.ch[channel - 1];

    // Convert raw value to microseconds
    return rawToMicroseconds(rawValue);
}

void SBusInput::getRawChannels(uint16_t* channels) const {
    for (uint8_t i = 0; i < 16; i++) {
        channels[i] = rawToMicroseconds(sbusData_.ch[i]);
    }
}

// ============================================================================
// MAPPED VEHICLE COMMANDS
// ============================================================================

float SBusInput::getSteering() const {
    uint16_t valueUs = getChannel(SBusChannelConfig::STEERING);

    // Map to -100 to +100 with center at 1520μs
    float percentage = mapToPercentageBidirectional(valueUs, SBUS_US_MIN, SBUS_US_CENTER, SBUS_US_MAX);

    // Apply center deadband
    percentage = applyDeadband(percentage, 0.0f, SBUS_STEERING_DEADBAND);

    return percentage;
}

float SBusInput::getThrottle() const {
    uint16_t valueUs = getChannel(SBusChannelConfig::THROTTLE);

    // Combined channel: above center = throttle (0-100%)
    if (valueUs <= SBUS_US_CENTER) {
        return 0.0f;
    }

    float percentage = mapToPercentage(valueUs, SBUS_US_CENTER, SBUS_US_MAX);

    // Apply idle deadband near center
    if (percentage < SBUS_THROTTLE_DEADBAND) {
        percentage = 0.0f;
    }

    return percentage;
}

TransmissionController::Gear SBusInput::getGear() const {
    uint16_t valueUs = getChannel(SBusChannelConfig::TRANSMISSION);

    // Map to gear based on ranges
    if (valueUs >= SBUS_GEAR_REVERSE_MIN && valueUs <= SBUS_GEAR_REVERSE_MAX) {
        return TransmissionController::Gear::GEAR_REVERSE;
    } else if (valueUs >= SBUS_GEAR_NEUTRAL_MIN && valueUs <= SBUS_GEAR_NEUTRAL_MAX) {
        return TransmissionController::Gear::GEAR_NEUTRAL;
    } else if (valueUs >= SBUS_GEAR_LOW_MIN && valueUs <= SBUS_GEAR_LOW_MAX) {
        return TransmissionController::Gear::GEAR_LOW;
    } else {
        // Default to NEUTRAL if out of range
        return TransmissionController::Gear::GEAR_NEUTRAL;
    }
}

float SBusInput::getBrake() const {
    uint16_t valueUs = getChannel(SBusChannelConfig::THROTTLE);

    // Combined channel: below center = brake (0-100%), 1500μs=0%, 1000μs=100%
    if (valueUs >= SBUS_US_CENTER) {
        return 0.0f;
    }

    // Invert: 1500μs→0%, 1000μs→100%
    uint16_t inverted = SBUS_US_CENTER - (valueUs - SBUS_US_MIN);
    return mapToPercentage(inverted, SBUS_US_MIN, SBUS_US_CENTER);
}

SBusInput::IgnitionState SBusInput::getIgnitionState() const {
    uint16_t valueUs = getChannel(SBusChannelConfig::IGNITION);

    // Map to ignition state based on ranges
    if (valueUs >= SBUS_IGNITION_OFF_MIN && valueUs <= SBUS_IGNITION_OFF_MAX) {
        return IgnitionState::OFF;
    } else if (valueUs >= SBUS_IGNITION_ACC_MIN && valueUs <= SBUS_IGNITION_ACC_MAX) {
        return IgnitionState::ACC;
    } else if (valueUs >= SBUS_IGNITION_ON_MIN && valueUs <= SBUS_IGNITION_ON_MAX) {
        return IgnitionState::IGNITION;
    } else {
        // Default to OFF if out of range (safe default)
        return IgnitionState::OFF;
    }
}

bool SBusInput::getFrontLight() const {
    uint16_t valueUs = getChannel(SBusChannelConfig::FRONT_LIGHT);

    // Simple threshold: >1520μs = ON
    return (valueUs > SBUS_FRONT_LIGHT_THRESHOLD);
}

// ============================================================================
// SIGNAL MONITORING
// ============================================================================

bool SBusInput::isSignalValid() const {
    return (millis() - lastFrameTime_) < SBUS_SIGNAL_TIMEOUT;
}

uint32_t SBusInput::getSignalAge() const {
    return millis() - lastFrameTime_;
}

SBusInput::SignalQuality SBusInput::getSignalQuality() const {
    SignalQuality quality;

    quality.totalFrames = totalFrames_;
    quality.errorFrames = errorFrames_;
    quality.signalAge = getSignalAge();
    quality.isValid = isSignalValid();

    // Calculate error rate
    if (totalFrames_ > 0) {
        quality.errorRate = (float)errorFrames_ / (float)totalFrames_ * 100.0f;
    } else {
        quality.errorRate = 0.0f;
    }

    // Calculate frame rate (update every second)
    uint32_t now = millis();
    if (now - frameRateUpdateTime_ >= 1000) {
        // Estimate based on typical SBUS frame rate (70-140Hz)
        // For now, just report a nominal value if signal is valid
        quality.frameRate = isSignalValid() ? 100.0f : 0.0f;
    } else {
        quality.frameRate = lastFrameRate_;
    }

    return quality;
}

// ============================================================================
// HELPER METHODS
// ============================================================================

float SBusInput::applyDeadband(float value, float center, float deadband) const {
    float offset = value - center;

    if (abs(offset) < deadband) {
        return center;
    }

    return value;
}

uint16_t SBusInput::rawToMicroseconds(uint16_t rawValue) const {
    constexpr float slope =
        (float)(SBUS_US_MAX - SBUS_US_MIN) /
        (float)(SBUS_RAW_MAX - SBUS_RAW_MIN);

    constexpr float intercept =
        (float)SBUS_US_MIN - (float)SBUS_RAW_MIN * slope;

    uint16_t valueUs = static_cast<uint16_t>(rawValue * slope + intercept + 0.5f);

    // Clamp to valid range
    if (valueUs < SBUS_US_MIN) valueUs = SBUS_US_MIN;
    if (valueUs > SBUS_US_MAX) valueUs = SBUS_US_MAX;

    return valueUs;
}

float SBusInput::mapToPercentage(uint16_t valueUs, uint16_t minUs, uint16_t maxUs) const {
    // Clamp input
    if (valueUs < minUs) valueUs = minUs;
    if (valueUs > maxUs) valueUs = maxUs;

    // Map to 0-100%
    float percentage = (float)(valueUs - minUs) / (float)(maxUs - minUs) * 100.0f;

    // Clamp output
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;

    return percentage;
}

float SBusInput::mapToPercentageBidirectional(uint16_t valueUs, uint16_t minUs, uint16_t centerUs, uint16_t maxUs) const {
    // Clamp input
    if (valueUs < minUs) valueUs = minUs;
    if (valueUs > maxUs) valueUs = maxUs;

    float percentage;

    if (valueUs < centerUs) {
        // Map minUs..centerUs to -100..0
        percentage = (float)(valueUs - centerUs) / (float)(centerUs - minUs) * 100.0f;
    } else {
        // Map centerUs..maxUs to 0..100
        percentage = (float)(valueUs - centerUs) / (float)(maxUs - centerUs) * 100.0f;
    }

    // Clamp output
    if (percentage < -100.0f) percentage = -100.0f;
    if (percentage > 100.0f) percentage = 100.0f;

    return percentage;
}
