#include "SteeringController.h"
#include "Debug.h"
#include <Preferences.h>

SteeringController::SteeringController()
    : rawAngle_(0),
      sensorOk_(false),
      center_(-1),
      leftLimit_(-1),
      rightLimit_(-1),
      calibrated_(false),
      invert_(false),
      relLeft_(0),
      relRight_(0),
      targetRel_(0),
      isMoving_(false),
      moveStartTime_(0),
      jogDir_(0),
      lastJogRefresh_(0),
      lastStallPosition_(0),
      lastStallCheckTime_(0) {
}

bool SteeringController::begin(gpio_num_t rpwmPin, gpio_num_t lpwmPin,
                               uint8_t rpwmChannel, uint8_t lpwmChannel,
                               gpio_num_t sdaPin, gpio_num_t sclPin) {
    if (!motor_.begin(rpwmPin, lpwmPin, rpwmChannel, lpwmChannel)) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] ERROR: BTS7960 init failed");
        return false;
    }
    motor_.stop();

    if (!sensor_.begin(sdaPin, sclPin)) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] ERROR: AS5600 init failed");
        return false;
    }

    // Prime sensor state
    uint16_t raw;
    bool magnetOk = false;
    if (sensor_.read(raw, magnetOk)) {
        rawAngle_ = raw;
        sensorOk_ = magnetOk;
    }

    Debug::printfFeature(DebugFeature::SERVO, "[STEER] Initialized: angle=%ld, sensor %s\n",
                         (long)rawAngle_, sensorOk_ ? "OK" : "FAULT");
    return true;
}

// === Sensor ===

int32_t SteeringController::wrapDelta(int32_t a, int32_t b) {
    return ((a - b + 2048) & 4095) - 2048;
}

int32_t SteeringController::relPosition() const {
    int32_t rel = wrapDelta(rawAngle_, center_);
    return invert_ ? -rel : rel;
}

bool SteeringController::refreshSensor() {
    uint16_t raw;
    bool magnetOk = false;
    bool readOk = sensor_.read(raw, magnetOk);
    if (readOk) {
        rawAngle_ = raw;
    }

    bool ok = readOk && magnetOk;
    if (ok != sensorOk_) {
        sensorOk_ = ok;
        if (!ok) {
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] SENSOR FAULT (%s) — motor stopped\n",
                                 readOk ? "magnet lost" : "I2C read failed");
        } else {
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Sensor recovered");
        }
    }

    if (!sensorOk_) {
        motor_.stop();
        isMoving_ = false;
        jogDir_ = 0;
    }
    return sensorOk_;
}

// === Movement ===

void SteeringController::stop() {
    motor_.stop();
    isMoving_ = false;
    jogDir_ = 0;
}

void SteeringController::jog(int8_t direction) {
    if (direction != 0 && !sensorOk_) return;   // no open-loop drive without stall backstop data

    if (direction == 0) {
        if (jogDir_ != 0) stop();
        return;
    }

    isMoving_ = false;   // jog overrides position control
    if (jogDir_ != direction) {
        lastStallPosition_ = relPosition();
        lastStallCheckTime_ = millis();
    }
    jogDir_ = (direction > 0) ? 1 : -1;
    lastJogRefresh_ = millis();
    motor_.setSpeed(jogDir_ * STEER_JOG_DUTY);
}

void SteeringController::update() {
    if (!refreshSensor()) return;

    uint32_t now = millis();

    // Stall detection — applies to any active drive (position move or jog)
    if (isMoving_ || jogDir_ != 0) {
        int32_t currentPos = relPosition();
        if (now - lastStallCheckTime_ >= STEER_STALL_TIMEOUT) {
            if (abs(currentPos - lastStallPosition_) < STEER_STALL_THRESHOLD) {
                Debug::printfFeature(DebugFeature::SERVO, "[STEER] Stall detected at %ld — stopping\n",
                                     (long)currentPos);
                stop();
                return;
            }
            lastStallPosition_ = currentPos;
            lastStallCheckTime_ = now;
        }
    }

    // Jog mode (open loop, momentary)
    if (jogDir_ != 0) {
        if (now - lastJogRefresh_ >= STEER_JOG_TIMEOUT_MS) {
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Jog timeout — stopping");
            stop();
        }
        return;
    }

    if (!isMoving_) return;

    // Closed-loop proportional position control
    int32_t error = targetRel_ - relPosition();

    if (abs(error) <= STEER_POSITION_TOLERANCE) {
        stop();
        return;
    }

    if (now - moveStartTime_ >= STEER_MOVE_TIMEOUT) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Movement timeout — stopping");
        stop();
        return;
    }

    float duty = fabsf((float)error) * STEER_KP;
    if (duty < STEER_PWM_MIN_DUTY) duty = STEER_PWM_MIN_DUTY;
    if (duty > STEER_PWM_MAX_DUTY) duty = STEER_PWM_MAX_DUTY;

    // Motor positive = wheel right = normalized position increasing
    int16_t speed = (int16_t)duty;
    motor_.setSpeed(error > 0 ? speed : (int16_t)-speed);
}

// === Steering interface ===

bool SteeringController::setSteeringPercent(float percent) {
    if (!calibrated_ || !sensorOk_) {
        return false;
    }

    if (percent < -100.0f) percent = -100.0f;
    if (percent > 100.0f) percent = 100.0f;

    // Asymmetric mapping: -100 -> relLeft_ (<0), 0 -> 0, +100 -> relRight_ (>0)
    int32_t target;
    if (percent >= 0.0f) {
        target = (int32_t)((percent / 100.0f) * relRight_);
    } else {
        target = (int32_t)((-percent / 100.0f) * relLeft_);
    }

    // Clamp to calibrated limits
    if (target < relLeft_) target = relLeft_;
    if (target > relRight_) target = relRight_;

    targetRel_ = target;
    jogDir_ = 0;
    isMoving_ = true;
    moveStartTime_ = millis();
    lastStallPosition_ = relPosition();
    lastStallCheckTime_ = millis();
    return true;
}

float SteeringController::getSteeringPercent() const {
    if (!calibrated_) return 0.0f;

    int32_t rel = relPosition();
    float percent;
    if (rel >= 0) {
        percent = (relRight_ != 0) ? ((float)rel / relRight_) * 100.0f : 0.0f;
    } else {
        percent = (relLeft_ != 0) ? -((float)rel / relLeft_) * 100.0f : 0.0f;
    }
    if (percent < -100.0f) percent = -100.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

// === Calibration ===

bool SteeringController::captureCenter() {
    if (!sensorOk_) return false;
    center_ = rawAngle_;
    revalidateCalibration();
    saveCalibration();
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] Center captured: %ld (calibrated=%d)\n",
                         (long)center_, calibrated_);
    return true;
}

bool SteeringController::captureLeftLimit() {
    if (!sensorOk_ || center_ < 0) return false;
    int32_t rel = wrapDelta(rawAngle_, center_);
    if (abs(rel) < STEER_CAL_MIN_SPAN) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Left limit too close to center — rejected");
        return false;
    }
    leftLimit_ = rawAngle_;
    revalidateCalibration();
    saveCalibration();
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] Left limit captured: %ld (calibrated=%d)\n",
                         (long)leftLimit_, calibrated_);
    return true;
}

bool SteeringController::captureRightLimit() {
    if (!sensorOk_ || center_ < 0) return false;
    int32_t rel = wrapDelta(rawAngle_, center_);
    if (abs(rel) < STEER_CAL_MIN_SPAN) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Right limit too close to center — rejected");
        return false;
    }
    rightLimit_ = rawAngle_;
    revalidateCalibration();
    saveCalibration();
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] Right limit captured: %ld (calibrated=%d)\n",
                         (long)rightLimit_, calibrated_);
    return true;
}

void SteeringController::revalidateCalibration() {
    calibrated_ = false;
    invert_ = false;
    relLeft_ = 0;
    relRight_ = 0;

    if (center_ < 0 || leftLimit_ < 0 || rightLimit_ < 0) return;

    int32_t relL = wrapDelta(leftLimit_, center_);
    int32_t relR = wrapDelta(rightLimit_, center_);

    // Limits must be on opposite sides of center, each at least the minimum span away
    if (relL == 0 || relR == 0) return;
    if ((relL < 0) == (relR < 0)) return;
    if (abs(relL) < STEER_CAL_MIN_SPAN || abs(relR) < STEER_CAL_MIN_SPAN) return;

    // Normalize sensor direction so that positive = right
    invert_ = (relR < 0);
    relLeft_ = invert_ ? -relL : relL;
    relRight_ = invert_ ? -relR : relR;
    calibrated_ = true;
}

void SteeringController::loadCalibration() {
    Preferences prefs;
    if (prefs.begin("steering", true)) {
        center_ = prefs.getInt("c_ang", -1);
        leftLimit_ = prefs.getInt("l_ang", -1);
        rightLimit_ = prefs.getInt("r_ang", -1);
        prefs.end();
    }
    revalidateCalibration();
    Debug::printfFeature(DebugFeature::SERVO,
                         "[STEER] Calibration loaded: center=%ld left=%ld right=%ld -> %s\n",
                         (long)center_, (long)leftLimit_, (long)rightLimit_,
                         calibrated_ ? "CALIBRATED" : "NOT CALIBRATED");
}

void SteeringController::saveCalibration() {
    Preferences prefs;
    if (prefs.begin("steering", false)) {
        prefs.putInt("c_ang", center_);
        prefs.putInt("l_ang", leftLimit_);
        prefs.putInt("r_ang", rightLimit_);
        prefs.end();
    }
}
