#include "ThrottleController.h"
#include "Debug.h"
#include <Preferences.h>

ThrottleController::ThrottleController()
    : idleUs_(THROTTLE_DEFAULT_IDLE_US),
      fullUs_(THROTTLE_DEFAULT_FULL_US),
      pendingIdleUs_(THROTTLE_DEFAULT_IDLE_US),
      pendingFullUs_(THROTTLE_DEFAULT_FULL_US),
      calibrating_(false) {
}

bool ThrottleController::begin(gpio_num_t pin, uint8_t channel) {
    loadCalibration();
    // Servo spans the full mechanical range; calibration lives within [idleUs_, fullUs_].
    // Start at the calibrated idle position.
    if (!servo_.begin(pin, channel, THROTTLE_SERVO_MIN_US, THROTTLE_SERVO_MAX_US, idleUs_)) {
        return false;
    }
    Debug::printfFeature(DebugFeature::VEHICLE,
        "[THROTTLE] Calibration idle=%uus full=%uus\n", idleUs_, fullUs_);
    return true;
}

uint16_t ThrottleController::clampToServoRange(int32_t us) {
    if (us < THROTTLE_SERVO_MIN_US) us = THROTTLE_SERVO_MIN_US;
    if (us > THROTTLE_SERVO_MAX_US) us = THROTTLE_SERVO_MAX_US;
    return (uint16_t)us;
}

uint16_t ThrottleController::percentToUs(float pct) const {
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint16_t)((int32_t)idleUs_ + (int32_t)(((int32_t)fullUs_ - (int32_t)idleUs_) * (pct / 100.0f)));
}

uint8_t ThrottleController::usToPercent(uint16_t us) const {
    // Degenerate window (never produced by saveCalibration()/loadCalibration(), both of
    // which enforce THROTTLE_MIN_SPAN_US) — report 0 rather than divide by zero.
    if (fullUs_ == idleUs_) return 0;
    int32_t pct = ((int32_t)us - (int32_t)idleUs_) * 100 /
                  ((int32_t)fullUs_ - (int32_t)idleUs_);
    if (pct < 0) pct = 0;       // servo below the calibrated idle endpoint
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

void ThrottleController::setThrottlePercent(float pct) {
    if (calibrating_) return;
    servo_.setMicroseconds(percentToUs(pct));
}

void ThrottleController::setThrottleUs(uint16_t us) {
    if (calibrating_) return;
    servo_.setMicroseconds(clampToServoRange(us));
}

void ThrottleController::idle() {
    if (calibrating_) return;
    servo_.setMicroseconds(idleUs_);
}

bool ThrottleController::beginCalibration(bool engineRunning) {
    if (engineRunning) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[THROTTLE] Calibration refused - engine running");
        return false;
    }
    pendingIdleUs_ = idleUs_;
    pendingFullUs_ = fullUs_;
    calibrating_ = true;
    Debug::printlnFeature(DebugFeature::VEHICLE, "[THROTTLE] Calibration started");
    return true;
}

void ThrottleController::jogIdle(uint16_t us) {
    if (!calibrating_) return;
    pendingIdleUs_ = clampToServoRange(us);
    servo_.setMicroseconds(pendingIdleUs_);   // live preview; servo holds this value
}

void ThrottleController::jogFull(uint16_t us) {
    if (!calibrating_) return;
    pendingFullUs_ = clampToServoRange(us);
    servo_.setMicroseconds(pendingFullUs_);   // live preview; servo holds this value
}

bool ThrottleController::saveCalibration(String& err) {
    if (!calibrating_) { err = "Not calibrating"; return false; }

    uint16_t idle = pendingIdleUs_;
    uint16_t full = pendingFullUs_;
    if (idle < THROTTLE_SERVO_MIN_US || full > THROTTLE_SERVO_MAX_US) {
        err = "Out of servo range";
        return false;
    }
    if ((int32_t)full - (int32_t)idle < THROTTLE_MIN_SPAN_US) {
        err = "Full must exceed idle by >= " + String(THROTTLE_MIN_SPAN_US) + "us";
        return false;
    }

    idleUs_ = idle;
    fullUs_ = full;

    Preferences prefs;
    if (prefs.begin("throttle", false)) {
        prefs.putUShort("idle_us", idleUs_);
        prefs.putUShort("full_us", fullUs_);
        prefs.end();
    }

    calibrating_ = false;
    servo_.setMicroseconds(idleUs_);
    Debug::printfFeature(DebugFeature::VEHICLE,
        "[THROTTLE] Calibration saved idle=%uus full=%uus\n", idleUs_, fullUs_);
    return true;
}

void ThrottleController::cancelCalibration() {
    if (!calibrating_) return;
    calibrating_ = false;
    servo_.setMicroseconds(idleUs_);   // discard pending, return to active idle
    Debug::printlnFeature(DebugFeature::VEHICLE, "[THROTTLE] Calibration cancelled");
}

void ThrottleController::loadCalibration() {
    Preferences prefs;
    if (prefs.begin("throttle", true)) {
        idleUs_ = prefs.getUShort("idle_us", THROTTLE_DEFAULT_IDLE_US);
        fullUs_ = prefs.getUShort("full_us", THROTTLE_DEFAULT_FULL_US);
        prefs.end();
    }
    // Guard against corrupt/incompatible stored values.
    if (idleUs_ < THROTTLE_SERVO_MIN_US || fullUs_ > THROTTLE_SERVO_MAX_US ||
        (int32_t)fullUs_ - (int32_t)idleUs_ < THROTTLE_MIN_SPAN_US) {
        idleUs_ = THROTTLE_DEFAULT_IDLE_US;
        fullUs_ = THROTTLE_DEFAULT_FULL_US;
    }
    pendingIdleUs_ = idleUs_;
    pendingFullUs_ = fullUs_;
}
