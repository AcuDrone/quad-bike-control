#include "LwsSensor.h"

const char* lwsFaultToken(LwsFault f) {
    switch (f) {
        case LwsFault::NONE:           return "none";
        case LwsFault::ZEROING:        return "zeroing";
        case LwsFault::CAN_DOWN:       return "can_down";
        case LwsFault::TIMEOUT:        return "timeout";
        case LwsFault::RANGE_LATCHED:  return "range_latched";
        case LwsFault::SENSOR_FAULT:   return "sensor_fault";
        case LwsFault::NOT_CALIBRATED: return "not_calibrated";
        case LwsFault::ANGLE_INVALID:  return "angle_invalid";
    }
    return "sensor_fault";
}

LwsSensor::LwsSensor(uint32_t timeoutMs, uint8_t recoverFrames)
    : timeoutMs_(timeoutMs),
      recoverFrames_(recoverFrames),
      last_{},
      haveFrame_(false),
      lastMs_(0),
      crcErr_(0),
      cntErr_(0),
      miss_(0),
      accepted_(0),
      windowStartMs_(0),
      windowCount_(0),
      fps_(0),
      windowStarted_(false),
      goodStreak_(0),
      fault_(LwsFault::TIMEOUT),
      healthy_(false) {
}

void LwsSensor::rollWindow(uint32_t nowMs) {
    if (!windowStarted_) {
        windowStarted_ = true;
        windowStartMs_ = nowMs;
        windowCount_ = 0;
        return;
    }
    uint32_t elapsed = nowMs - windowStartMs_;
    if (elapsed < 1000) return;
    // A window with no evaluate()/onFrame() call for > 2 s carries no frames into the next one.
    fps_ = (elapsed < 2000) ? windowCount_ : 0;
    windowCount_ = 0;
    windowStartMs_ = nowMs;
}

LwsFault LwsSensor::frameFault() const {
    uint8_t code = lwsFault(last_);
    if (code == LWS_FAULT_RANGE_LATCHED) return LwsFault::RANGE_LATCHED;
    if (!lwsOk(last_) || code != 0)      return LwsFault::SENSOR_FAULT;
    if (!lwsCal(last_))                  return LwsFault::NOT_CALIBRATED;
    if (!lwsAngleValid(last_))           return LwsFault::ANGLE_INVALID;
    return LwsFault::NONE;
}

bool LwsSensor::onFrame(const uint8_t* buf, uint8_t len, uint32_t nowMs) {
    rollWindow(nowMs);
    if (buf == nullptr || len < LWS_DLC_MIN) return false;   // ignored, not an error
    if (!lwsCrcOk(buf)) {
        crcErr_++;
        return false;
    }
    uint8_t counter = lwsCounter(buf);
    if (haveFrame_) {
        uint8_t prev = lwsCounter(last_);
        if (counter == prev) {
            cntErr_++;     // duplicate or frozen sender
            return false;
        }
        // Lost frames are harmless (every frame is absolute): only counted, and only
        // across a short gap — after a timeout the 4-bit counter delta means nothing.
        if ((nowMs - lastMs_) <= timeoutMs_) {
            uint8_t delta = (uint8_t)((counter - prev) & 0x0F);
            if (delta > 1) miss_ += (uint32_t)(delta - 1);
        }
    }

    uint8_t n = (len > sizeof(last_)) ? (uint8_t)sizeof(last_) : len;
    for (uint8_t i = 0; i < sizeof(last_); i++) last_[i] = (i < n) ? buf[i] : 0;
    haveFrame_ = true;
    lastMs_ = nowMs;
    accepted_++;
    windowCount_++;

    if (frameFault() == LwsFault::NONE) {
        if (goodStreak_ < 255) goodStreak_++;
    } else {
        goodStreak_ = 0;   // faults take effect on the first frame that shows them
    }
    return true;
}

LwsFault LwsSensor::evaluate(uint32_t nowMs, bool zeroing, bool canUp) {
    rollWindow(nowMs);

    LwsFault f;
    if (zeroing) {
        f = LwsFault::ZEROING;
    } else if (!canUp) {
        f = LwsFault::CAN_DOWN;
    } else if (!haveFrame_ || (nowMs - lastMs_) > timeoutMs_) {
        f = LwsFault::TIMEOUT;
    } else {
        f = frameFault();
    }

    // Recovery from any non-frame fault needs a fresh run of good frames.
    if (f == LwsFault::ZEROING || f == LwsFault::CAN_DOWN || f == LwsFault::TIMEOUT) {
        goodStreak_ = 0;
    }

    healthy_ = (f == LwsFault::NONE) && goodStreak_ >= recoverFrames_;
    // The token is never NONE while unhealthy: during the recovery streak the last
    // fault stays reported until STEER_LWS_RECOVER_FRAMES good frames have arrived.
    if (f != LwsFault::NONE || healthy_ || fault_ == LwsFault::NONE) {
        fault_ = f;
    }
    if (!healthy_ && fault_ == LwsFault::NONE) fault_ = LwsFault::TIMEOUT;   // defensive, unreachable
    return fault_;
}
