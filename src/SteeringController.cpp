#include "SteeringController.h"
#include "Debug.h"

SteeringController::SteeringController()
    : rpwmPin_(GPIO_NUM_NC),
      lpwmPin_(GPIO_NUM_NC),
      encoder_(nullptr),
      targetPosition_(0),
      isMoving_(false),
      moveStartTime_(0),
      lastStallPosition_(0),
      lastStallCheckTime_(0) {
}

bool SteeringController::begin(gpio_num_t rpwmPin, gpio_num_t lpwmPin) {
    rpwmPin_ = rpwmPin;
    lpwmPin_ = lpwmPin;

    pinMode(rpwmPin_, OUTPUT);
    pinMode(lpwmPin_, OUTPUT);

    // Start in stopped state
    digitalWrite(rpwmPin_, LOW);
    digitalWrite(lpwmPin_, LOW);

    Debug::printfFeature(DebugFeature::SERVO, "[STEER] Initialized: RPWM=%d, LPWM=%d\n",
                  rpwmPin_, lpwmPin_);
    return true;
}

void SteeringController::attachEncoder(EncoderCounter* encoder) {
    encoder_ = encoder;
}

// === Movement ===

void SteeringController::moveRight() {
    digitalWrite(lpwmPin_, LOW);
    digitalWrite(rpwmPin_, HIGH);
}

void SteeringController::moveLeft() {
    digitalWrite(rpwmPin_, LOW);
    digitalWrite(lpwmPin_, HIGH);
}

void SteeringController::stop() {
    digitalWrite(rpwmPin_, LOW);
    digitalWrite(lpwmPin_, LOW);
    isMoving_ = false;
}

// === Position control ===

void SteeringController::setPosition(int32_t targetPosition) {
    // Clamp to limits
    if (targetPosition < 0) targetPosition = 0;
    if (targetPosition > STEER_RIGHT_LIMIT) targetPosition = STEER_RIGHT_LIMIT;

    targetPosition_ = targetPosition;
    isMoving_ = true;
    moveStartTime_ = millis();

    // Reset stall detection
    if (encoder_) {
        lastStallPosition_ = encoder_->getPosition();
    }
    lastStallCheckTime_ = millis();
}

void SteeringController::update() {
    if (!isMoving_ || !encoder_) return;

    int32_t currentPos = encoder_->getPosition();
    int32_t error = targetPosition_ - currentPos;

    // Check if at target
    if (abs(error) <= STEER_POSITION_TOLERANCE) {
        stop();
        return;
    }

    // Check movement timeout
    if (millis() - moveStartTime_ >= STEER_MOVE_TIMEOUT) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Movement timeout — stopping");
        stop();
        return;
    }

    // Stall detection
    uint32_t now = millis();
    if (now - lastStallCheckTime_ >= STEER_STALL_TIMEOUT) {
        if (abs(currentPos - lastStallPosition_) < STEER_STALL_THRESHOLD) {
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] Stall detected at position %ld\n", currentPos);
            stop();
            return;
        }
        lastStallPosition_ = currentPos;
        lastStallCheckTime_ = now;
    }

    // Move toward target (full speed)
    if (error > 0) {
        moveRight();
    } else {
        moveLeft();
    }
}

bool SteeringController::isAtPosition(int32_t target, int32_t tolerance) const {
    if (!encoder_) return false;
    return abs(encoder_->getPosition() - target) <= tolerance;
}

int32_t SteeringController::getPosition() const {
    if (!encoder_) return 0;
    return encoder_->getPosition();
}

// === Steering interface ===

void SteeringController::setSteeringPercent(float percent) {
    // Clamp to -100..+100
    if (percent < -100.0f) percent = -100.0f;
    if (percent > 100.0f) percent = 100.0f;

    // Symmetric range based on smaller side from center
    int32_t halfRange = (int32_t)STEER_CENTER_POSITION;

    int32_t target = STEER_CENTER_POSITION + (int32_t)((percent / 100.0f) * halfRange);
    setPosition(target);
}

float SteeringController::getSteeringPercent() const {
    if (!encoder_) return 0.0f;

    int32_t halfRange = (int32_t)STEER_CENTER_POSITION;
    if (halfRange == 0) return 0.0f;

    float percent = ((float)(encoder_->getPosition() - STEER_CENTER_POSITION) / halfRange) * 100.0f;
    if (percent < -100.0f) percent = -100.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

// === Calibration ===

bool SteeringController::autoHome(uint32_t timeout) {
    if (!encoder_) return false;

    Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Auto-homing to left limit...");

    uint32_t startTime = millis();
    int32_t lastPos = encoder_->getPosition();
    uint32_t lastChangeTime = millis();

    // Move left until stall
    moveLeft();

    while (millis() - startTime < timeout) {
        int32_t currentPos = encoder_->getPosition();

        if (abs(currentPos - lastPos) >= STEER_STALL_THRESHOLD) {
            lastPos = currentPos;
            lastChangeTime = millis();
        }

        if (millis() - lastChangeTime >= STEER_STALL_TIMEOUT) {
            // Stall detected — at physical limit
            stop();
            encoder_->reset();
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Home found, encoder reset to 0");
            return true;
        }

        delay(10);
    }

    stop();
    Debug::printlnFeature(DebugFeature::SERVO, "[STEER] ERROR: Auto-home timeout");
    return false;
}
