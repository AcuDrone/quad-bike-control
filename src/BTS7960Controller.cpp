#include "BTS7960Controller.h"
#include "Constants.h"
#include "driver/ledc.h"

BTS7960Controller::BTS7960Controller()
    : rpwmPin_(GPIO_NUM_NC)
    , lpwmPin_(GPIO_NUM_NC)
    , rpwmChannel_(0)
    , lpwmChannel_(0)
    , currentSpeed_(0)
    , currentRPWM_(0)
    , currentLPWM_(0)
    , initialized_(false)
    , encoder_(nullptr)
    , targetPosition_(0)
    , positionControlActive_(false)
    , lastEncoderChange_(0)
    , lastEncoderPosition_(0)
{
}

BTS7960Controller::~BTS7960Controller() {
    if (initialized_) {
        stop();
    }
}

bool BTS7960Controller::begin(gpio_num_t rpwmPin, gpio_num_t lpwmPin,
                               uint8_t rpwmChannel, uint8_t lpwmChannel) {
    rpwmPin_ = rpwmPin;
    lpwmPin_ = lpwmPin;
    rpwmChannel_ = rpwmChannel;
    lpwmChannel_ = lpwmChannel;

    // Configure LEDC timer for motor PWM (10kHz)
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        Serial.printf("BTS7960: Failed to configure timer: %d\n", err);
        return false;
    }

    // Configure RPWM channel
    ledc_channel_config_t rpwm_conf = {
        .gpio_num = rpwmPin_,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = static_cast<ledc_channel_t>(rpwmChannel_),
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = 0
        }
    };

    err = ledc_channel_config(&rpwm_conf);
    if (err != ESP_OK) {
        Serial.printf("BTS7960: Failed to configure RPWM channel: %d\n", err);
        return false;
    }

    // Configure LPWM channel
    ledc_channel_config_t lpwm_conf = {
        .gpio_num = lpwmPin_,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = static_cast<ledc_channel_t>(lpwmChannel_),
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = 0
        }
    };

    err = ledc_channel_config(&lpwm_conf);
    if (err != ESP_OK) {
        Serial.printf("BTS7960: Failed to configure LPWM channel: %d\n", err);
        return false;
    }

    initialized_ = true;

    // Ensure motor is stopped
    stop();

    Serial.printf("BTS7960: Initialized RPWM pin %d ch %d, LPWM pin %d ch %d\n",
                  rpwmPin_, rpwmChannel_, lpwmPin_, lpwmChannel_);
    Serial.println("BTS7960: Note - Enable pins hardwired to 5V (always active)");
    return true;
}

void BTS7960Controller::setSpeed(int16_t speed) {
    if (!initialized_) {
        Serial.println("BTS7960: Not initialized");
        return;
    }

    // Clamp speed to valid range
    speed = clamp<int16_t>(speed, (int16_t)MOTOR_MAX_REVERSE, (int16_t)MOTOR_MAX_FORWARD);
    currentSpeed_ = speed;

    if (speed > 0) {
        // Extend actuator
        setRPWM((uint8_t)speed);
        setLPWM(0);
    } else if (speed < 0) {
        // Retract actuator
        setRPWM(0);
        setLPWM((uint8_t)(-speed));
    } else {
        // Stop (coast)
        stop();
    }
}

void BTS7960Controller::stop() {
    if (!initialized_) {
        return;
    }

    // Set both PWM to 0 (coast)
    setRPWM(0);
    setLPWM(0);
    currentSpeed_ = 0;
}

void BTS7960Controller::brake() {
    // CRITICAL SAFETY: BTS7960 does NOT support electrical braking!
    // Setting both RPWM and LPWM high simultaneously will SHORT the H-bridge
    // and DESTROY the controller. Always use coast (both low) instead.

    // Brake is an alias for stop() for API compatibility
    stop();

    Serial.println("BTS7960: Brake called (coasting to stop - no electrical braking)");
}

void BTS7960Controller::setRPWM(uint8_t duty) {
    // SAFETY: Prevent simultaneous extend and retract
    // If setting RPWM (extend), automatically clear LPWM (retract)
    if (duty > 0 && currentLPWM_ > 0) {
        Serial.println("BTS7960 SAFETY: Clearing LPWM (retract) before setting RPWM (extend)");
        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(lpwmChannel_), 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(lpwmChannel_));
        currentLPWM_ = 0;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(rpwmChannel_), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(rpwmChannel_));
    currentRPWM_ = duty;
}

void BTS7960Controller::setLPWM(uint8_t duty) {
    // SAFETY: Prevent simultaneous extend and retract
    // If setting LPWM (retract), automatically clear RPWM (extend)
    if (duty > 0 && currentRPWM_ > 0) {
        Serial.println("BTS7960 SAFETY: Clearing RPWM (extend) before setting LPWM (retract)");
        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(rpwmChannel_), 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(rpwmChannel_));
        currentRPWM_ = 0;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(lpwmChannel_), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(lpwmChannel_));
    currentLPWM_ = duty;
}

// ============================================================================
// ENCODER AND POSITION CONTROL METHODS
// ============================================================================

void BTS7960Controller::attachEncoder(EncoderCounter* encoder) {
    encoder_ = encoder;
    if (encoder_ != nullptr) {
        lastEncoderPosition_ = encoder_->getPosition();
        lastEncoderChange_ = millis();
        Serial.println("BTS7960: Encoder attached");
    }
}

int32_t BTS7960Controller::getPosition() const {
    if (encoder_ == nullptr) {
        return 0;
    }
    return encoder_->getPosition();
}

bool BTS7960Controller::moveToPosition(int32_t targetPosition, uint8_t speed) {
    if (encoder_ == nullptr) {
        Serial.println("BTS7960: No encoder attached for position control");
        return false;
    }

    int32_t currentPos = encoder_->getPosition();
    int32_t error = targetPosition - currentPos;

    // Check if already at target position
    if (abs(error) <= TRANS_POSITION_TOLERANCE) {
        stop();
        positionControlActive_ = false;
        return false;
    }

    // Set movement direction and speed based on error
    targetPosition_ = targetPosition;
    positionControlActive_ = true;

    // Initialize stall detection
    lastEncoderPosition_ = currentPos;
    lastEncoderChange_ = millis();

    if (error > 0) {
        // Need to extend (positive direction)
        setSpeed(speed);
    } else {
        // Need to retract (negative direction)
        setSpeed(-speed);
    }

    return true;
}

bool BTS7960Controller::isAtPosition(int32_t targetPosition, int32_t tolerance) const {
    if (encoder_ == nullptr) {
        return false;
    }

    int32_t currentPos = encoder_->getPosition();
    int32_t error = abs(targetPosition - currentPos);
    return error <= tolerance;
}

bool BTS7960Controller::autoHome(int8_t direction, uint8_t homingSpeed, uint32_t timeout) {
    if (encoder_ == nullptr) {
        Serial.println("BTS7960: No encoder attached for auto-homing");
        return false;
    }

    Serial.println("BTS7960: Starting auto-homing sequence...");

    // Start moving in specified direction
    int16_t speed = (direction > 0) ? homingSpeed : -homingSpeed;
    setSpeed(speed);

    uint32_t startTime = millis();
    int32_t lastPosition = encoder_->getPosition();
    uint32_t lastChangeTime = millis();

    // Monitor for stall (no encoder change for TRANS_STALL_TIMEOUT)
    while (true) {
        int32_t currentPos = encoder_->getPosition();

        // Check if encoder changed
        if (abs(currentPos - lastPosition) >= TRANS_STALL_THRESHOLD) {
            lastPosition = currentPos;
            lastChangeTime = millis();
        }

        // Check for stall (no movement for timeout period)
        if (millis() - lastChangeTime >= TRANS_STALL_TIMEOUT) {
            Serial.println("BTS7960: Stall detected - setting zero position");
            stop();
            delay(100);  // Let actuator settle
            encoder_->reset();
            Serial.println("BTS7960: Auto-homing complete!");
            return true;
        }

        // Check for overall timeout
        if (millis() - startTime >= timeout) {
            Serial.println("BTS7960: Auto-homing timeout!");
            stop();
            return false;
        }

        delay(10);  // Small delay for loop
    }
}

void BTS7960Controller::stopPositionControl() {
    if (positionControlActive_) {
        stop();
        positionControlActive_ = false;
    }
}

void BTS7960Controller::recalibrateEncoder(int32_t expectedPosition) {
    if (encoder_ == nullptr) {
        Serial.println("BTS7960: Cannot recalibrate - no encoder attached");
        return;
    }

    int32_t currentPosition = encoder_->getPosition();
    int32_t correction = expectedPosition - currentPosition;

    Serial.printf("BTS7960: Recalibrating encoder: %ld -> %ld (correction: %+ld)\n",
                  currentPosition, expectedPosition, correction);

    // Reset encoder counter and set to expected position
    encoder_->reset();
    encoder_->setPosition(expectedPosition);

    // Update stall detection tracking
    lastEncoderPosition_ = expectedPosition;
    lastEncoderChange_ = millis();
}

void BTS7960Controller::update() {
    if (!positionControlActive_ || encoder_ == nullptr) {
        return;
    }

    int32_t currentPos = encoder_->getPosition();
    int32_t error = targetPosition_ - currentPos;

    // Check if we've reached target position
    if (abs(error) <= TRANS_POSITION_TOLERANCE) {
        stop();
        delay(50);  // Brief settling delay for mechanical stability

        // Re-check position after settling to confirm we're still at target
        int32_t settledPos = encoder_->getPosition();
        int32_t settledError = abs(targetPosition_ - settledPos);

        if (settledError <= TRANS_POSITION_TOLERANCE) {
            positionControlActive_ = false;
            Serial.printf("BTS7960: Reached target position %ld (actual: %ld, error: %ld)\n",
                          targetPosition_, settledPos, settledError);
            return;
        } else {
            // Position drifted during settling, continue control
            Serial.printf("BTS7960: Position drift detected, continuing...\n");
        }
    }

    // Proportional speed control: reduce speed as we approach target
    int32_t absError = abs(error);
    int16_t baseSpeed = 255;  // Base movement speed (increased for faster travel)
    int16_t adjustedSpeed;

    // Multi-stage speed ramping for smooth and precise positioning
    if (absError > 200) {
        // Very far from target: maximum speed
        adjustedSpeed = baseSpeed;
    } else if (absError > 100) {
        // Far from target: 85% speed
        adjustedSpeed = (baseSpeed * 85) / 100;
    } else if (absError > 50) {
        // Medium distance: 65% speed
        adjustedSpeed = (baseSpeed * 65) / 100;
    } else if (absError > 30) {
        // Approaching target: 45% speed
        adjustedSpeed = (baseSpeed * 45) / 100;
    } else if (absError > 15) {
        // Close to target: 30% speed
        adjustedSpeed = (baseSpeed * 30) / 100;
    } else {
        // Very close: minimum reliable speed for precision
        adjustedSpeed = (baseSpeed * 20) / 100;  // 20% for final approach
    }

    // Ensure minimum speed for reliable motor movement
    if (adjustedSpeed < 30) {
        adjustedSpeed = 30;  // Minimum PWM to overcome static friction
    }

    // Apply speed in correct direction
    if (error > 0) {
        setSpeed(adjustedSpeed);  // Extend
    } else {
        setSpeed(-adjustedSpeed);  // Retract
    }

    // Check for stall during position control
    // Only check for stall if we're not very close to target (> 15 counts away)
    if (absError > 15) {
        if (abs(currentPos - lastEncoderPosition_) >= TRANS_STALL_THRESHOLD) {
            lastEncoderPosition_ = currentPos;
            lastEncoderChange_ = millis();
        } else if (millis() - lastEncoderChange_ >= TRANS_STALL_TIMEOUT) {
            // Stall detected during position control
            stop();
            positionControlActive_ = false;
            Serial.printf("BTS7960: Stall detected at position %ld (target was %ld)\n",
                          currentPos, targetPosition_);
        }
    } else {
        // When very close to target, update stall timer to prevent false stall detection
        if (abs(currentPos - lastEncoderPosition_) >= 1) {
            lastEncoderPosition_ = currentPos;
            lastEncoderChange_ = millis();
        }
    }
}
