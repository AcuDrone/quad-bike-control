#include "TransmissionController.h"
#include "Debug.h"

TransmissionController::TransmissionController()
    : targetGear_(Gear::GEAR_NEUTRAL)
    , gearChangePhase_(GearChangePhase::NONE)
    , finalGearPositionPct_(TRANS_GEAR_DEFAULT_NEUTRAL_PCT)
    , overshootDirection_(1.0f)
    , gearPhaseStartTime_(0)
    , settleMs_(TRANS_SERVO_SETTLE_MIN_MS)
    , queuedGear_(Gear::GEAR_UNKNOWN)
    , sequenceStepDwellStart_(0)
    , cachedPhysicalGear_(Gear::GEAR_UNKNOWN)
    , lastGearReadTime_(0)
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

// Physical gear sequence: R(0) - N(1) - H(2) - L(3)
static const TransmissionController::Gear GEAR_SEQ[] = {
    TransmissionController::Gear::GEAR_REVERSE,
    TransmissionController::Gear::GEAR_NEUTRAL,
    TransmissionController::Gear::GEAR_HIGH,
    TransmissionController::Gear::GEAR_LOW
};

TransmissionController::Gear TransmissionController::nextGearToward(Gear from, Gear to) {
    if (from == to) return from;
    int fromIdx = -1, toIdx = -1;
    for (int i = 0; i < 4; i++) {
        if (GEAR_SEQ[i] == from) fromIdx = i;
        if (GEAR_SEQ[i] == to)   toIdx   = i;
    }
    if (fromIdx < 0 || toIdx < 0) return Gear::GEAR_NEUTRAL;
    return GEAR_SEQ[fromIdx + (fromIdx < toIdx ? 1 : -1)];
}

void TransmissionController::startGearChange(Gear gear) {
    float targetPct = getGearPosition(gear);
    saveState(gear, getCurrentServoPct(), false);

    float diff = targetPct - getCurrentServoPct();
    overshootDirection_ = (diff >= 0.0f) ? 1.0f : -1.0f;
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
}

bool TransmissionController::setGear(Gear finalGear) {
    if (finalGear == Gear::GEAR_UNKNOWN) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION, "[TRANS] ERROR: Cannot set UNKNOWN gear");
        return false;
    }

    // Idempotent: already queued for this exact destination, or active single-step change
    if (queuedGear_ == finalGear) return true;
    if (targetGear_ == finalGear && gearChangePhase_ != GearChangePhase::NONE) return true;

    if (!canChangeGear(finalGear)) {
        return false;
    }

    // Gear change active: queue the destination and let the current step finish.
    // Calling startGearChange() here would cancel the in-progress step and skip it.
    if (gearChangePhase_ != GearChangePhase::NONE) {
        queuedGear_ = finalGear;
        sequenceStepDwellStart_ = 0;
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Queue updated to %s (step %s in progress)\n",
            getGearName(finalGear), getGearName(targetGear_));
        return true;
    }

    // Idle: already at target
    if (targetGear_ == finalGear) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "[TRANS] Already at %s\n", getGearName(finalGear));
        queuedGear_ = Gear::GEAR_UNKNOWN;
        return false;
    }

    // Idle: start the first step toward finalGear
    Gear nextStep = nextGearToward(targetGear_, finalGear);

    queuedGear_ = (nextStep == finalGear) ? Gear::GEAR_UNKNOWN : finalGear;
    sequenceStepDwellStart_ = 0;

    if (queuedGear_ != Gear::GEAR_UNKNOWN) {
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Sequence %s -> ... -> %s, next step: %s\n",
            getGearName(targetGear_), getGearName(finalGear), getGearName(nextStep));
    }

    startGearChange(nextStep);
    return true;
}

TransmissionController::Gear TransmissionController::getCurrentGear() const {
    return getPhysicalGear();
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
    return gearChangePhase_ == GearChangePhase::OVERSHOOT_DWELL;
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
    uint32_t now = millis();
    if (now - lastGearReadTime_ < TRANS_GEAR_READ_INTERVAL_MS) {
        return cachedPhysicalGear_;
    }
    lastGearReadTime_ = now;

    bool reverseActive = !digitalRead(PIN_GEAR_REVERSE);
    bool neutralActive = !digitalRead(PIN_GEAR_NEUTRAL);
    bool lowActive     = !digitalRead(PIN_GEAR_LOW);
    bool highActive    = !digitalRead(PIN_GEAR_HIGH);
    uint8_t activeCount = reverseActive + neutralActive + lowActive + highActive;

    // Retry when result is not clean and servo is idle (filter transient bounces)
    uint8_t retries = 0;
    while (activeCount != 1 && gearChangePhase_ == GearChangePhase::NONE
           && retries < TRANS_GEAR_READ_RETRY_COUNT) {
        reverseActive = !digitalRead(PIN_GEAR_REVERSE);
        neutralActive = !digitalRead(PIN_GEAR_NEUTRAL);
        lowActive     = !digitalRead(PIN_GEAR_LOW);
        highActive    = !digitalRead(PIN_GEAR_HIGH);
        activeCount   = reverseActive + neutralActive + lowActive + highActive;
        retries++;
    }

    if (activeCount == 0) {
        cachedPhysicalGear_ = Gear::GEAR_UNKNOWN;
    } else if (activeCount > 1) {
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] ERROR: Multiple gear sensors active (R:%d N:%d L:%d H:%d)\n",
            reverseActive, neutralActive, lowActive, highActive);
        cachedPhysicalGear_ = Gear::GEAR_UNKNOWN;
    } else if (reverseActive) {
        cachedPhysicalGear_ = Gear::GEAR_REVERSE;
    } else if (neutralActive) {
        cachedPhysicalGear_ = Gear::GEAR_NEUTRAL;
    } else if (lowActive) {
        cachedPhysicalGear_ = Gear::GEAR_LOW;
    } else {
        cachedPhysicalGear_ = Gear::GEAR_HIGH;
    }
    return cachedPhysicalGear_;
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

        const char* phaseStr =
            gearChangePhase_ == GearChangePhase::NONE          ? "IDLE" :
            gearChangePhase_ == GearChangePhase::OVERSHOOT      ? "OVERSHOOT" :
            gearChangePhase_ == GearChangePhase::OVERSHOOT_DWELL ? "OVERSHOOT_DWELL" :
                                                                   "RETURN";
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Status: Physical=%s, ServoPct=%.1f%%, Phase=%s%s\n",
            getGearName(physicalGear), getCurrentServoPct(), phaseStr,
            queuedGear_ != Gear::GEAR_UNKNOWN ? " (seq)" : "");
    }

    // ── NONE phase ──
    if (gearChangePhase_ == GearChangePhase::NONE) {
        // Advance gear sequence if a final destination is queued
        if (queuedGear_ != Gear::GEAR_UNKNOWN) {
            // Use targetGear_ (last assumed-engaged gear) for sequence advancement.
            // The physical sensor is unreliable and is not used to gate progression.
            if (targetGear_ == queuedGear_) {
                Debug::printfFeature(DebugFeature::TRANSMISSION,
                    "[TRANS] Sequence complete at %s\n", getGearName(queuedGear_));
                queuedGear_ = Gear::GEAR_UNKNOWN;
                sequenceStepDwellStart_ = 0;
            } else if (canChangeGear(queuedGear_)) {
                // Arm dwell on first entry after a step completes
                if (sequenceStepDwellStart_ == 0) {
                    sequenceStepDwellStart_ = now;
                    Debug::printfFeature(DebugFeature::TRANSMISSION,
                        "[TRANS] Step assumed at %s, dwell %ums before next step\n",
                        getGearName(targetGear_), TRANS_SEQUENCE_STEP_DWELL_MS);
                } else if (now - sequenceStepDwellStart_ >= TRANS_SEQUENCE_STEP_DWELL_MS) {
                    sequenceStepDwellStart_ = 0;
                    Gear nextStep = nextGearToward(targetGear_, queuedGear_);
                    startGearChange(nextStep);
                }
            }
            return;
        }

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

    // ── OVERSHOOT phase: servo moving to overshoot position ──
    if (gearChangePhase_ == GearChangePhase::OVERSHOOT) {
        if (now - gearPhaseStartTime_ >= settleMs_) {
            gearPhaseStartTime_ = now;
            gearChangePhase_ = GearChangePhase::OVERSHOOT_DWELL;
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] OVERSHOOT_DWELL: waiting up to %ums for %s\n",
                TRANS_OVERSHOOT_DWELL_MS, getGearName(targetGear_));
        }
        return;
    }

    // ── OVERSHOOT_DWELL phase: hold at overshoot for TRANS_OVERSHOOT_DWELL_MS then assume engaged ──
    if (gearChangePhase_ == GearChangePhase::OVERSHOOT_DWELL) {
        if (now - gearPhaseStartTime_ < TRANS_OVERSHOOT_DWELL_MS) return;

        settleMs_ = computeSettleMs(getCurrentServoPct(), finalGearPositionPct_);
        gearPhaseStartTime_ = now;
        gearChangePhase_ = GearChangePhase::RETURN;
        commandServo(finalGearPositionPct_);
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Dwell complete, RETURN: %.1f%% (settle %lums)\n",
            finalGearPositionPct_, settleMs_);
        return;
    }

    // ── RETURN phase: servo moving back to final position ──
    if (gearChangePhase_ == GearChangePhase::RETURN) {
        if (now - gearPhaseStartTime_ < settleMs_) return;

        saveState(targetGear_, getCurrentServoPct(), true);
        gearChangePhase_ = GearChangePhase::NONE;
        lastGearMismatch_ = false;
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Gear %s engaged at %.1f%%\n",
            getGearName(targetGear_), getCurrentServoPct());
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
