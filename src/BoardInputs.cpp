#include "BoardInputs.h"
#include "Debug.h"

// Rate limit for the "read failed" chatter while a fault persists.
static constexpr uint32_t BOARD_INPUT_FAIL_LOG_MS = 1000;

BoardInputs::BoardInputs(TwoWire& bus)
    : expander_(bus, PCA9557_ADDR_INPUTS),
      snapshot_{false, false, false, false, false, false, 0},
      rawInputs_(0),
      initialized_(false),
      faulted_(true),          // faulted until the first good read proves otherwise
      failureCount_(0),
      lastPollMs_(0),
      lastGoodReadMs_(0),
      hasGoodRead_(false),
      lastRecoverAttemptMs_(0),
      lastFailLogMs_(0) {
}

bool BoardInputs::configureDevice() {
    if (!expander_.isPresent()) {
        return false;
    }
    // All eight pins are inputs; no polarity inversion (the sense is applied in
    // firmware via BOARD_INPUT_ACTIVE_LOW so there is exactly one place to flip).
    if (!expander_.setPolarityInversion(0x00)) {
        return false;
    }
    return expander_.setDirection(0xFF);
}

bool BoardInputs::begin() {
    lastPollMs_ = millis();
    lastRecoverAttemptMs_ = lastPollMs_;

    if (!configureDevice()) {
        Debug::printfFeature(DebugFeature::VEHICLE,
            "[IO] ERROR: opto-input expander not responding at 0x%02X on I2C2\n",
            PCA9557_ADDR_INPUTS);
        initialized_ = false;
        faulted_ = true;
        return false;
    }

    uint8_t raw = 0;
    if (!expander_.readInputs(raw)) {
        Debug::printlnFeature(DebugFeature::VEHICLE,
            "[IO] ERROR: opto-input expander seed read failed");
        initialized_ = false;
        faulted_ = true;
        return false;
    }

    initialized_ = true;
    faulted_ = false;
    failureCount_ = 0;
    hasGoodRead_ = true;
    lastGoodReadMs_ = millis();
    applyRaw(raw);

    Debug::printfFeature(DebugFeature::VEHICLE,
        "[IO] Opto inputs initialized @0x%02X raw=0x%02X (R:%d N:%d L:%d H:%d brake:%d)\n",
        PCA9557_ADDR_INPUTS, raw,
        snapshot_.gearReverse, snapshot_.gearNeutral, snapshot_.gearLow,
        snapshot_.gearHigh, snapshot_.brakeReleased);
    return true;
}

void BoardInputs::update() {
    uint32_t now = millis();
    if ((now - lastPollMs_) < BOARD_INPUT_POLL_MS) {
        return;
    }
    lastPollMs_ = now;

    // While faulted (including "never initialized"), re-configure the device at a
    // slow cadence before attempting the read again.
    if (faulted_ && (now - lastRecoverAttemptMs_) >= BOARD_INPUT_RECOVER_MS) {
        lastRecoverAttemptMs_ = now;
        if (configureDevice()) {
            initialized_ = true;
        }
    }

    uint8_t raw = 0;
    if (initialized_ && expander_.readInputs(raw)) {
        bool wasFaulted = faulted_;
        applyRaw(raw);
        snapshot_.valid = true;
        failureCount_ = 0;
        hasGoodRead_ = true;
        lastGoodReadMs_ = now;
        if (wasFaulted) {
            faulted_ = false;
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[IO] Opto inputs recovered (raw=0x%02X)\n", raw);
        }
        return;
    }

    // Failed poll: hold the last-known snapshot, count the failure, retry next poll.
    failureCount_++;

    uint32_t age = hasGoodRead_ ? (now - lastGoodReadMs_) : UINT32_MAX;
    if (age > BOARD_INPUT_STALE_MS) {
        snapshot_.valid = false;
        if (!faulted_) {
            faulted_ = true;
            lastFailLogMs_ = now;
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[IO] FAULT: opto inputs stale (%lu ms, %lu consecutive failures) — "
                "gear reports UNKNOWN, brake endstop unconfirmed\n",
                (unsigned long)age, (unsigned long)failureCount_);
        } else if ((now - lastFailLogMs_) >= BOARD_INPUT_FAIL_LOG_MS) {
            lastFailLogMs_ = now;
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[IO] opto inputs still unreachable (%lu failures)\n",
                (unsigned long)failureCount_);
        }
    }
}

BoardInputs::Snapshot BoardInputs::getSnapshot() const {
    Snapshot s = snapshot_;
    s.ageMs = getAgeMs();
    return s;
}

uint32_t BoardInputs::getAgeMs() const {
    if (!hasGoodRead_) {
        return UINT32_MAX;
    }
    return millis() - lastGoodReadMs_;
}

bool BoardInputs::bitAsserted(uint8_t raw, uint8_t bit) {
    bool level = (raw >> bit) & 0x01;
#if BOARD_INPUT_ACTIVE_LOW
    return !level;
#else
    return level;
#endif
}

void BoardInputs::applyRaw(uint8_t raw) {
    rawInputs_ = raw;
    snapshot_.gearReverse   = bitAsserted(raw, BOARD_IN_BIT_GEAR_REVERSE);
    snapshot_.gearNeutral   = bitAsserted(raw, BOARD_IN_BIT_GEAR_NEUTRAL);
    snapshot_.gearLow       = bitAsserted(raw, BOARD_IN_BIT_GEAR_LOW);
    snapshot_.gearHigh      = bitAsserted(raw, BOARD_IN_BIT_GEAR_HIGH);
    snapshot_.brakeReleased = bitAsserted(raw, BOARD_IN_BIT_BRAKE_LIMIT);
    snapshot_.valid = true;
}
