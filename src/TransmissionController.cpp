#include "TransmissionController.h"
#include "Debug.h"

TransmissionController::TransmissionController()
    : targetGear_(Gear::GEAR_NEUTRAL)
    , gearChangePhase_(GearChangePhase::NONE)
    , finalGearPositionPct_(TRANS_GEAR_DEFAULT_NEUTRAL_PCT)
    , overshootDirection_(1.0f)
    , overshootRetryCount_(0)
    , gearPhaseStartTime_(0)
    , settleMs_(TRANS_SERVO_SETTLE_MIN_MS)
    , lastGearCheckTime_(0)
    , lastStatusLogTime_(0)
    , lastGearMismatch_(false)
    , lastLoggedGear_(Gear::GEAR_UNKNOWN)
    , lastLoggedMoving_(false)
{
    vehicleData_.vehicleSpeed = 0;
    vehicleData_.lastUpdateTime = 0;
    vehicleData_.dataValid = false;

    gearPositions_[(int)Gear::GEAR_HIGH]    = TRANS_GEAR_DEFAULT_HIGH_PCT;
    gearPositions_[(int)Gear::GEAR_LOW]     = TRANS_GEAR_DEFAULT_LOW_PCT;
    gearPositions_[(int)Gear::GEAR_NEUTRAL] = TRANS_GEAR_DEFAULT_NEUTRAL_PCT;
    gearPositions_[(int)Gear::GEAR_REVERSE] = TRANS_GEAR_DEFAULT_REVERSE_PCT;

    overshootPcts_[(int)Gear::GEAR_HIGH]    = TRANS_GEAR_OVERSHOOT_H_PCT;
    overshootPcts_[(int)Gear::GEAR_LOW]     = TRANS_GEAR_OVERSHOOT_L_PCT;
    overshootPcts_[(int)Gear::GEAR_NEUTRAL] = TRANS_GEAR_OVERSHOOT_N_PCT;
    overshootPcts_[(int)Gear::GEAR_REVERSE] = TRANS_GEAR_OVERSHOOT_R_PCT;

    pullbackPcts_[(int)Gear::GEAR_HIGH]    = TRANS_GEAR_PULLBACK_H_PCT;
    pullbackPcts_[(int)Gear::GEAR_LOW]     = TRANS_GEAR_PULLBACK_L_PCT;
    pullbackPcts_[(int)Gear::GEAR_NEUTRAL] = TRANS_GEAR_PULLBACK_N_PCT;
    pullbackPcts_[(int)Gear::GEAR_REVERSE] = TRANS_GEAR_PULLBACK_R_PCT;
}

TransmissionController::~TransmissionController() {
}

bool TransmissionController::begin() {
    bool ok = servo_.begin(PIN_TRANS_SERVO, LEDC_CH_TRANS_SERVO,
                           TRANS_SERVO_MIN_US, TRANS_SERVO_MAX_US, 0);
    return ok;
}

// ── helpers ──────────────────────────────────────────────────────────────────

void TransmissionController::commandServo(float positionPct) {
    positionPct = constrain(positionPct, 0.0f, 100.0f);
    uint16_t us = (uint16_t)(TRANS_SERVO_MIN_US
                  + positionPct / 100.0f * (TRANS_SERVO_MAX_US - TRANS_SERVO_MIN_US));
    servo_.setMicroseconds(us);
}

uint32_t TransmissionController::computeSettleMs(float fromPct, float toPct) const {
    float dist = fabsf(toPct - fromPct);
    uint32_t ms = (uint32_t)(dist * TRANS_SERVO_MS_PER_PCT);
    return max(ms, (uint32_t)TRANS_SERVO_SETTLE_MIN_MS);
}

// ── gear control ─────────────────────────────────────────────────────────────

bool TransmissionController::setGear(Gear gear) {
    if (gear == Gear::GEAR_UNKNOWN) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION, "[TRANS] ERROR: Cannot set UNKNOWN gear");
        return false;
    }
    if (!canChangeGear(gear)) {
        return false;
    }

    if (getPhysicalGear() == gear) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "[TRANS] Already at %s\n", getGearName(gear));
        return false;
    }

    float targetPct = getGearPosition(gear);
    saveState(gear, getCurrentServoPct(), false);

    float diff = targetPct - getCurrentServoPct();
    overshootDirection_ = (diff >= 0.0f) ? 1.0f : -1.0f;
    overshootRetryCount_ = 0;
    float overshootPct = constrain(targetPct + overshootDirection_ * overshootPcts_[(int)gear], 0.0f, 100.0f);

    targetGear_ = gear;
    finalGearPositionPct_ = targetPct;
    gearChangePhase_ = GearChangePhase::OVERSHOOT;
    settleMs_ = computeSettleMs(getCurrentServoPct(), overshootPct);
    gearPhaseStartTime_ = millis();

    commandServo(overshootPct);

    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Gear %s: overshoot %.1f%% -> final %.1f%% (settle %lums)\n",
        getGearName(gear), overshootPct, targetPct, settleMs_);
    return true;
}

TransmissionController::Gear TransmissionController::getCurrentGear() const {
    return getPhysicalGear();
}

bool TransmissionController::isAtGear(Gear gear) const {
    return getPhysicalGear() == gear;
}

float TransmissionController::getGearPosition(Gear gear) const {
    if (gear == Gear::GEAR_UNKNOWN) return 0.0f;
    return gearPositions_[(int)gear];
}

const char* TransmissionController::getGearName(Gear gear) const {
    switch (gear) {
        case Gear::GEAR_HIGH:    return "HIGH";
        case Gear::GEAR_LOW:     return "LOW";
        case Gear::GEAR_NEUTRAL: return "NEUTRAL";
        case Gear::GEAR_REVERSE: return "REVERSE";
        default:                 return "UNKNOWN";
    }
}

void TransmissionController::setVehicleData(const TransmissionVehicleData& data) {
    vehicleData_ = data;
}

bool TransmissionController::needsThrottleBoost() const {
    return gearChangePhase_ != GearChangePhase::NONE;
}

bool TransmissionController::canChangeGear(Gear targetGear) const {
    if (targetGear == Gear::GEAR_NEUTRAL) return true;

    if (vehicleData_.dataValid) {
        if (vehicleData_.vehicleSpeed > TRANS_SPEED_INTERLOCK_THRESHOLD) {
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] Gear change blocked: vehicle moving at %d km/h\n",
                vehicleData_.vehicleSpeed);
            return false;
        }
    } else {
        uint32_t dataAge = millis() - vehicleData_.lastUpdateTime;
        if (vehicleData_.lastUpdateTime > 0 && dataAge < TRANS_CAN_TIMEOUT) {
            Debug::printlnFeature(DebugFeature::TRANSMISSION,
                "[TRANS] WARNING: CAN data invalid, blocking gear change");
            return false;
        } else {
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] WARNING: CAN timeout (%lu ms), allowing gear change\n", dataAge);
        }
    }
    return true;
}

// ── gear sensors ─────────────────────────────────────────────────────────────

void TransmissionController::initGearSensors() {
    gpio_set_direction(PIN_GEAR_REVERSE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_GEAR_REVERSE, GPIO_FLOATING);
    gpio_set_direction(PIN_GEAR_NEUTRAL, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_GEAR_NEUTRAL, GPIO_FLOATING);
    gpio_set_direction(PIN_GEAR_LOW, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_GEAR_LOW, GPIO_FLOATING);
    gpio_set_direction(PIN_GEAR_HIGH, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_GEAR_HIGH, GPIO_FLOATING);
    Debug::printlnFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Gear sensors initialized (active-low, external pull-ups)");
}

TransmissionController::Gear TransmissionController::getPhysicalGear() const {
    bool reverseActive = !digitalRead(PIN_GEAR_REVERSE);
    bool neutralActive = !digitalRead(PIN_GEAR_NEUTRAL);
    bool lowActive     = !digitalRead(PIN_GEAR_LOW);
    bool highActive    = !digitalRead(PIN_GEAR_HIGH);

    uint8_t activeCount = reverseActive + neutralActive + lowActive + highActive;
    if (activeCount == 0) return Gear::GEAR_UNKNOWN;
    if (activeCount > 1) {
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] ERROR: Multiple gear sensors active (R:%d N:%d L:%d H:%d)\n",
            reverseActive, neutralActive, lowActive, highActive);
        return Gear::GEAR_UNKNOWN;
    }
    if (reverseActive) return Gear::GEAR_REVERSE;
    if (neutralActive) return Gear::GEAR_NEUTRAL;
    if (lowActive)     return Gear::GEAR_LOW;
    return Gear::GEAR_HIGH;
}

bool TransmissionController::isGearPositionValid() const {
    return getCurrentGear() == getPhysicalGear();
}

// ── update loop ───────────────────────────────────────────────────────────────

void TransmissionController::update() {
    uint32_t now = millis();

    Gear physicalGear = getPhysicalGear();
    bool currentlyMoving = (gearChangePhase_ != GearChangePhase::NONE);

    // Periodic status log
    if (now - lastStatusLogTime_ >= 2000 ||
        physicalGear != lastLoggedGear_ ||
        currentlyMoving != lastLoggedMoving_) {

        lastStatusLogTime_ = now;
        lastLoggedGear_ = physicalGear;
        lastLoggedMoving_ = currentlyMoving;

        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Status: Physical=%s, ServoPct=%.1f%%, Phase=%s\n",
            getGearName(physicalGear), getCurrentServoPct(),
            gearChangePhase_ == GearChangePhase::NONE     ? "IDLE" :
            gearChangePhase_ == GearChangePhase::OVERSHOOT ? "OVERSHOOT" :
            gearChangePhase_ == GearChangePhase::PULLBACK  ? "PULLBACK" : "RETURN");
    }

    if (gearChangePhase_ == GearChangePhase::NONE) {
        // Check for mismatch when idle
        if (now - lastGearCheckTime_ >= TRANS_GEAR_CHECK_INTERVAL) {
            lastGearCheckTime_ = now;
            if (physicalGear != Gear::GEAR_UNKNOWN && physicalGear != targetGear_) {
                if (!lastGearMismatch_) {
                    Debug::printfFeature(DebugFeature::TRANSMISSION,
                        "[TRANS] MISMATCH: target=%s, physical=%s\n",
                        getGearName(targetGear_), getGearName(physicalGear));
                    lastGearMismatch_ = true;
                }
            } else {
                if (lastGearMismatch_) {
                    Debug::printfFeature(DebugFeature::TRANSMISSION,
                        "[TRANS] Mismatch resolved: now at %s\n", getGearName(physicalGear));
                    lastGearMismatch_ = false;
                }
            }
        }
        return;
    }

    // ── OVERSHOOT phase ──
    if (gearChangePhase_ == GearChangePhase::OVERSHOOT) {
        if (now - gearPhaseStartTime_ >= settleMs_) {
            settleMs_ = computeSettleMs(getCurrentServoPct(), finalGearPositionPct_);
            gearPhaseStartTime_ = now;
            gearChangePhase_ = GearChangePhase::RETURN;
            commandServo(finalGearPositionPct_);
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] RETURN phase: %.1f%% (settle %lums)\n",
                finalGearPositionPct_, settleMs_);
        }
        return;
    }

    // ── RETURN phase ──
    if (gearChangePhase_ == GearChangePhase::RETURN) {
        if (now - gearPhaseStartTime_ < settleMs_) return;

        if (physicalGear == targetGear_) {
            saveState(targetGear_, getCurrentServoPct(), true);
            gearChangePhase_ = GearChangePhase::NONE;
            overshootRetryCount_ = 0;
            lastGearMismatch_ = false;
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] Gear %s confirmed by switch at %.1f%%\n",
                getGearName(targetGear_), getCurrentServoPct());
            return;
        }

        // Switch not confirmed — pull back then retry with larger overshoot
        if (overshootRetryCount_ < 100) {
            float pullbackTarget = constrain(
                finalGearPositionPct_ - overshootDirection_ * pullbackPcts_[(int)targetGear_],
                0.0f, 100.0f);
            settleMs_ = computeSettleMs(getCurrentServoPct(), pullbackTarget);
            gearPhaseStartTime_ = now;
            gearChangePhase_ = GearChangePhase::PULLBACK;
            commandServo(pullbackTarget);
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] PULLBACK: %.1f%% (settle %lums)\n", pullbackTarget, settleMs_);
        } else {
            gearChangePhase_ = GearChangePhase::NONE;
            overshootRetryCount_ = 0;
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] WARNING: Gear %s not engaged after all attempts (physical %s)\n",
                getGearName(targetGear_), getGearName(physicalGear));
        }
    }

    // ── PULLBACK phase ──
    if (gearChangePhase_ == GearChangePhase::PULLBACK) {
        if (now - gearPhaseStartTime_ < settleMs_) return;
        overshootRetryCount_++;
        float multiplier = (overshootRetryCount_ == 1) ? 1.5f : 2.0f;
        float newOvershoot = constrain(
            finalGearPositionPct_ + overshootDirection_ * overshootPcts_[(int)targetGear_] * multiplier,
            0.0f, 100.0f);
        settleMs_ = computeSettleMs(getCurrentServoPct(), newOvershoot);
        gearPhaseStartTime_ = now;
        gearChangePhase_ = GearChangePhase::OVERSHOOT;
        commandServo(newOvershoot);
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Retry %d: overshoot %.1f%% (%.1fx) -> %.1f%% (settle %lums)\n",
            overshootRetryCount_, newOvershoot, multiplier, finalGearPositionPct_, settleMs_);
    }
}

// ── NVS persistence ───────────────────────────────────────────────────────────

void TransmissionController::saveState(Gear gear, float positionPct, bool valid) {
    Preferences prefs;
    if (!prefs.begin("transmission", false)) return;
    prefs.putBool("state_valid", valid);
    prefs.putInt("state_gear", (int)gear);
    prefs.putFloat("state_pct", positionPct);
    prefs.end();
}

bool TransmissionController::loadState(Gear& gear, float& positionPct, bool& valid) {
    Preferences prefs;
    if (!prefs.begin("transmission", true)) return false;
    if (!prefs.isKey("state_valid")) { prefs.end(); return false; }
    valid       = prefs.getBool("state_valid", false);
    gear        = (Gear)prefs.getInt("state_gear", (int)Gear::GEAR_UNKNOWN);
    positionPct = prefs.getFloat("state_pct", TRANS_GEAR_DEFAULT_NEUTRAL_PCT);
    prefs.end();
    return true;
}

void TransmissionController::loadDefaultPositions() {
    Preferences prefs;
    if (!prefs.begin("transmission", true)) return;
    gearPositions_[(int)Gear::GEAR_HIGH]    = prefs.getFloat("pct_h", TRANS_GEAR_DEFAULT_HIGH_PCT);
    gearPositions_[(int)Gear::GEAR_LOW]     = prefs.getFloat("pct_l", TRANS_GEAR_DEFAULT_LOW_PCT);
    gearPositions_[(int)Gear::GEAR_NEUTRAL] = prefs.getFloat("pct_n", TRANS_GEAR_DEFAULT_NEUTRAL_PCT);
    gearPositions_[(int)Gear::GEAR_REVERSE] = prefs.getFloat("pct_r", TRANS_GEAR_DEFAULT_REVERSE_PCT);
    prefs.end();
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Positions loaded: H=%.1f%% N=%.1f%% L=%.1f%% R=%.1f%%\n",
        gearPositions_[(int)Gear::GEAR_HIGH],    gearPositions_[(int)Gear::GEAR_NEUTRAL],
        gearPositions_[(int)Gear::GEAR_LOW],     gearPositions_[(int)Gear::GEAR_REVERSE]);
}

void TransmissionController::saveDefaultPosition(Gear gear, float positionPct) {
    static const char* keys[] = {"pct_h", "pct_l", "pct_n", "pct_r"};
    Preferences prefs;
    if (!prefs.begin("transmission", false)) return;
    prefs.putFloat(keys[(int)gear], positionPct);
    prefs.end();
    gearPositions_[(int)gear] = positionPct;
}

bool TransmissionController::setDefaultPosition(Gear gear, float positionPct) {
    if (gear == Gear::GEAR_UNKNOWN || (int)gear >= 4) return false;
    if (positionPct < 0.0f || positionPct > 100.0f) return false;
    saveDefaultPosition(gear, positionPct);
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Default for %s set to %.1f%%\n", getGearName(gear), positionPct);
    return true;
}

void TransmissionController::moveToPercent(float pct) {
    commandServo(pct);
}

float TransmissionController::getGearOvershoot(Gear gear) const {
    if (gear == Gear::GEAR_UNKNOWN) return 0.0f;
    return overshootPcts_[(int)gear];
}

bool TransmissionController::setGearOvershoot(Gear gear, float overshootPct) {
    if (gear == Gear::GEAR_UNKNOWN || (int)gear >= 4) return false;
    if (overshootPct < 0.0f || overshootPct > 20.0f) return false;
    overshootPcts_[(int)gear] = overshootPct;
    static const char* keys[] = {"ovr_h", "ovr_l", "ovr_n", "ovr_r"};
    Preferences prefs;
    if (prefs.begin("transmission", false)) {
        prefs.putFloat(keys[(int)gear], overshootPct);
        prefs.end();
    }
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Overshoot for %s set to %.1f%%\n", getGearName(gear), overshootPct);
    return true;
}

void TransmissionController::loadGearOvershoots() {
    Preferences prefs;
    if (!prefs.begin("transmission", true)) return;
    overshootPcts_[(int)Gear::GEAR_HIGH]    = prefs.getFloat("ovr_h", TRANS_GEAR_OVERSHOOT_H_PCT);
    overshootPcts_[(int)Gear::GEAR_LOW]     = prefs.getFloat("ovr_l", TRANS_GEAR_OVERSHOOT_L_PCT);
    overshootPcts_[(int)Gear::GEAR_NEUTRAL] = prefs.getFloat("ovr_n", TRANS_GEAR_OVERSHOOT_N_PCT);
    overshootPcts_[(int)Gear::GEAR_REVERSE] = prefs.getFloat("ovr_r", TRANS_GEAR_OVERSHOOT_R_PCT);
    prefs.end();
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Overshoots loaded: H=%.1f%% L=%.1f%% N=%.1f%% R=%.1f%%\n",
        overshootPcts_[(int)Gear::GEAR_HIGH], overshootPcts_[(int)Gear::GEAR_LOW],
        overshootPcts_[(int)Gear::GEAR_NEUTRAL], overshootPcts_[(int)Gear::GEAR_REVERSE]);
}

float TransmissionController::getGearPullback(Gear gear) const {
    if (gear == Gear::GEAR_UNKNOWN) return 0.0f;
    return pullbackPcts_[(int)gear];
}

bool TransmissionController::setGearPullback(Gear gear, float pullbackPct) {
    if (gear == Gear::GEAR_UNKNOWN || (int)gear >= 4) return false;
    if (pullbackPct < 0.0f || pullbackPct > 20.0f) return false;
    pullbackPcts_[(int)gear] = pullbackPct;
    static const char* keys[] = {"plb_h", "plb_l", "plb_n", "plb_r"};
    Preferences prefs;
    if (prefs.begin("transmission", false)) {
        prefs.putFloat(keys[(int)gear], pullbackPct);
        prefs.end();
    }
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Pullback for %s set to %.1f%%\n", getGearName(gear), pullbackPct);
    return true;
}

void TransmissionController::loadGearPullbacks() {
    Preferences prefs;
    if (!prefs.begin("transmission", true)) return;
    pullbackPcts_[(int)Gear::GEAR_HIGH]    = prefs.getFloat("plb_h", TRANS_GEAR_PULLBACK_H_PCT);
    pullbackPcts_[(int)Gear::GEAR_LOW]     = prefs.getFloat("plb_l", TRANS_GEAR_PULLBACK_L_PCT);
    pullbackPcts_[(int)Gear::GEAR_NEUTRAL] = prefs.getFloat("plb_n", TRANS_GEAR_PULLBACK_N_PCT);
    pullbackPcts_[(int)Gear::GEAR_REVERSE] = prefs.getFloat("plb_r", TRANS_GEAR_PULLBACK_R_PCT);
    prefs.end();
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Pullbacks loaded: H=%.1f%% L=%.1f%% N=%.1f%% R=%.1f%%\n",
        pullbackPcts_[(int)Gear::GEAR_HIGH], pullbackPcts_[(int)Gear::GEAR_LOW],
        pullbackPcts_[(int)Gear::GEAR_NEUTRAL], pullbackPcts_[(int)Gear::GEAR_REVERSE]);
}

bool TransmissionController::restoreStateIfValid() {
    Gear savedGear;
    float savedPct;
    bool valid;

    if (!loadState(savedGear, savedPct, valid)) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION,
            "[TRANS] No saved state, moving to NEUTRAL default");
        commandServo(gearPositions_[(int)Gear::GEAR_NEUTRAL]);
        return false;
    }
    if (!valid) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Saved state invalid (mid-change power loss), moving to NEUTRAL default");
        commandServo(gearPositions_[(int)Gear::GEAR_NEUTRAL]);
        return false;
    }

    commandServo(savedPct);
    targetGear_ = savedGear;
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Restored state: %s at %.1f%%\n", getGearName(savedGear), savedPct);
    return true;
}
