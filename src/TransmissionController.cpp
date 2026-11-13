#include "TransmissionController.h"
#include "Debug.h"

// TransmissionLaw.h mirrors the Gear values so the pure logic can be tested natively.
static_assert((int)TransmissionController::Gear::GEAR_HIGH    == (int)TransGear::GEAR_HIGH,    "gear enum mirror");
static_assert((int)TransmissionController::Gear::GEAR_LOW     == (int)TransGear::GEAR_LOW,     "gear enum mirror");
static_assert((int)TransmissionController::Gear::GEAR_NEUTRAL == (int)TransGear::GEAR_NEUTRAL, "gear enum mirror");
static_assert((int)TransmissionController::Gear::GEAR_REVERSE == (int)TransGear::GEAR_REVERSE, "gear enum mirror");
static_assert((int)TransmissionController::Gear::GEAR_UNKNOWN == (int)TransGear::GEAR_UNKNOWN, "gear enum mirror");

TransmissionController::TransmissionController()
    : targetGear_(Gear::GEAR_UNKNOWN)   // not initialised until initialise()
    , fromGear_(Gear::GEAR_UNKNOWN)
    , gearChangePhase_(GearChangePhase::NONE)
    , finalGearPositionPct_(TRANS_GEAR_DEFAULT_NEUTRAL_PCT)
    , overshootDirection_(1.0f)
    , gearPhaseStartTime_(0)
    , settleMs_(TRANS_SERVO_SETTLE_MIN_MS)
    , queuedGear_(Gear::GEAR_UNKNOWN)
    , sequenceStepDwellStart_(0)
    , shiftStartMs_(0)
    , shiftFromGear_(Gear::GEAR_UNKNOWN)
    , cachedPhysicalGear_(Gear::GEAR_UNKNOWN)
    , lastGearReadTime_(0)
    , lastGearCheckTime_(0)
    , lastStatusLogTime_(0)
    , lastGearMismatch_(false)
    , initStepActive_(false)
    , lastLoggedGear_(Gear::GEAR_UNKNOWN)
    , lastLoggedMoving_(false)
{
    vehicleData_.vehicleSpeed = 0;
    vehicleData_.lastUpdateTime = 0;
    vehicleData_.dataValid = false;
    vehicleData_.sensorSpeedMs = 0.0f;
    vehicleData_.sensorSpeedValid = false;
    vehicleData_.sensorSpeedSuspicious = false;

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

    fromGear_ = targetGear_;   // capture outgoing gear before advancing the step
    targetGear_ = gear;
    finalGearPositionPct_ = targetPct;
    gearChangePhase_ = GearChangePhase::OVERSHOOT;
    settleMs_ = computeSettleMs(getCurrentServoPct(), overshootPct);
    gearPhaseStartTime_ = millis();
    resync_.reset();

    commandServo(overshootPct);

    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Gear %s: overshoot %.1f%% -> final %.1f%% (settle %lums)\n",
        getGearName(gear), overshootPct, targetPct, settleMs_);
}

// Initialisation step into NEUTRAL from an unknown lever / servo position. Same state machine
// as any step (so the boost PID / shift idle hold apply), but the overshoot direction comes
// from the NVS hint and the overshoot move uses a fixed settle: the servo has had no pulse, so
// getCurrentServoPct() is meaningless.
void TransmissionController::startInitStep(float direction) {
    const Gear gear = Gear::GEAR_NEUTRAL;
    float targetPct = getGearPosition(gear);
    markStateInvalid();   // keep state_gear / state_pct: the hint for the next boot

    overshootDirection_ = (direction >= 0.0f) ? 1.0f : -1.0f;
    float overshootPct = constrain(targetPct + overshootDirection_ * overshootPcts_[(int)gear], 0.0f, 100.0f);

    fromGear_ = Gear::GEAR_UNKNOWN;
    targetGear_ = gear;   // initialised from here on; a shift is active
    finalGearPositionPct_ = targetPct;
    gearChangePhase_ = GearChangePhase::OVERSHOOT;
    settleMs_ = TRANS_INIT_SETTLE_MS;
    gearPhaseStartTime_ = millis();
    initStepActive_ = true;
    resync_.reset();
    shiftFromGear_ = Gear::GEAR_UNKNOWN;
    shiftStartMs_ = gearPhaseStartTime_;

    commandServo(overshootPct);

    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Init step NEUTRAL: overshoot %.1f%% (dir %+.0f) -> final %.1f%% (settle %lums)\n",
        overshootPct, overshootDirection_, targetPct, settleMs_);
}

void TransmissionController::initialise() {
    if (isInitialised()) return;

    Gear nvsGear = Gear::GEAR_UNKNOWN;
    float nvsPct = NAN;
    bool nvsValid = false;
    bool nvsFound = loadState(nvsGear, nvsPct, nvsValid);

    TransInitDecision d = chooseTransmissionInit((TransGear)(int)getPhysicalGear(), gearPositions_,
                                                 nvsFound, nvsValid, (TransGear)(int)nvsGear, nvsPct);
    Gear gear = (Gear)(int)d.gear;

    switch (d.source) {
        case TransInitSource::SENSOR:
            commandServo(d.pct);
            targetGear_ = gear;
            fromGear_ = gear;
            saveState(gear, getCurrentServoPct(), true);
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] Initialised from sensor: %s\n", getGearName(gear));
            break;
        case TransInitSource::NVS:
            commandServo(d.pct);
            targetGear_ = gear;
            fromGear_ = gear;
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] Initialised from NVS: %s at %.1f%%\n", getGearName(gear), d.pct);
            break;
        case TransInitSource::STEP_TO_NEUTRAL:
        default:
            Debug::printlnFeature(DebugFeature::TRANSMISSION,
                "[TRANS] Init: sensor unknown, stepping to NEUTRAL");
            startInitStep(d.direction);
            break;
    }
}

bool TransmissionController::setGear(Gear finalGear) {
    if (finalGear == Gear::GEAR_UNKNOWN) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION, "[TRANS] ERROR: Cannot set UNKNOWN gear");
        return false;
    }

    // Not initialised: the current gear is unknown, so there is no step to compute. The
    // caller initialises first (web) or retries (MAVLink).
    if (!isInitialised()) {
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Gear %s refused: transmission not initialised\n", getGearName(finalGear));
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

    shiftFromGear_ = targetGear_;
    shiftStartMs_ = millis();
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

    // The hall speed sensor is the live speed source and is authoritative while healthy
    // (CAN speed is dead — PID 0x0D is disabled). While latched suspicious its reading is a
    // decaying physical upper bound on speed, so it is still trusted for this interlock.
    if (vehicleData_.sensorSpeedValid || vehicleData_.sensorSpeedSuspicious) {
        if (vehicleData_.sensorSpeedMs > TRANS_SPEED_INTERLOCK_THRESHOLD_MS) {
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] Gear change blocked: vehicle moving at %.2f m/s (%.1f km/h)\n",
                vehicleData_.sensorSpeedMs, vehicleData_.sensorSpeedMs * MS_TO_KMH);
            return false;
        }
        return true;
    }

    // Sensor uninitialized: fall back to the reviewed CAN-timeout policy below rather than
    // inventing a new one. A stuck-blocked gearbox is itself a hazard, and no firmware can
    // prove motion when a passive pulse sensor is silent.
    Debug::printlnFeature(DebugFeature::TRANSMISSION,
        "[TRANS] WARNING: speed sensor reading unavailable — falling back to CAN policy");

    if (vehicleData_.dataValid) {
        // The CAN field is natively km/h (uint8), so the m/s threshold is converted here —
        // a presentation-edge conversion for a foreign unit, not an internal one.
        if ((float)vehicleData_.vehicleSpeed > TRANS_SPEED_INTERLOCK_THRESHOLD_MS * MS_TO_KMH) {
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
                // Arm the pause on first entry after a step completes (or after setGear()
                // re-armed it). It ends once the sensor confirms the completed step's gear
                // for TRANS_DWELL_CONFIRM_MS, else after TRANS_SEQUENCE_STEP_DWELL_MS.
                if (sequenceStepDwellStart_ == 0) {
                    sequenceStepDwellStart_ = now;
                    pauseConfirm_.reset();
                    Debug::printfFeature(DebugFeature::TRANSMISSION,
                        "[TRANS] Step assumed at %s, pause up to %ums before next step\n",
                        getGearName(targetGear_), TRANS_SEQUENCE_STEP_DWELL_MS);
                }
                ShiftConfirmResult r = pauseConfirm_.step(now, sequenceStepDwellStart_,
                                                          physicalGear == targetGear_,
                                                          TRANS_DWELL_CONFIRM_MS,
                                                          TRANS_SEQUENCE_STEP_DWELL_MS);
                if (r != ShiftConfirmResult::WAIT) {
                    uint32_t waited = now - sequenceStepDwellStart_;
                    if (r == ShiftConfirmResult::CONFIRMED) {
                        Debug::printfFeature(DebugFeature::TRANSMISSION,
                            "[TRANS] Step %s confirmed after %lu ms\n",
                            getGearName(targetGear_), (unsigned long)waited);
                    } else {
                        Debug::printfFeature(DebugFeature::TRANSMISSION,
                            "[TRANS] Step %s pause timeout %lu ms\n",
                            getGearName(targetGear_), (unsigned long)waited);
                    }
                    sequenceStepDwellStart_ = 0;
                    Gear nextStep = nextGearToward(targetGear_, queuedGear_);
                    startGearChange(nextStep);
                }
            }
            return;
        }

        // Check for mismatch when idle (only once the expected gear is known)
        if (isInitialised() && now - lastGearCheckTime_ >= TRANS_GEAR_CHECK_INTERVAL) {
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

            // Resync: the same single sensor gear != target on TRANS_RESYNC_CONFIRM_CHECKS
            // consecutive checks → adopt it. The servo is not moved.
            if (resync_.step((TransGear)(int)physicalGear, (TransGear)(int)targetGear_,
                             TRANS_RESYNC_CONFIRM_CHECKS)) {
                Debug::printfFeature(DebugFeature::TRANSMISSION,
                    "[TRANS] Resynced from sensor: target=%s -> %s\n",
                    getGearName(targetGear_), getGearName(physicalGear));
                targetGear_ = physicalGear;
                fromGear_ = physicalGear;
                saveState(physicalGear, getCurrentServoPct(), true);
                lastGearMismatch_ = false;
            }
        }
        return;
    }

    // ── OVERSHOOT phase: servo moving to overshoot position ──
    if (gearChangePhase_ == GearChangePhase::OVERSHOOT) {
        if (now - gearPhaseStartTime_ >= settleMs_) {
            gearPhaseStartTime_ = now;
            gearChangePhase_ = GearChangePhase::OVERSHOOT_DWELL;
            dwellConfirm_.reset();
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] OVERSHOOT_DWELL: waiting up to %ums for %s\n",
                TRANS_OVERSHOOT_DWELL_MS, getGearName(targetGear_));
        }
        return;
    }

    // ── OVERSHOOT_DWELL phase: hold at overshoot until the sensor confirms the step's gear
    //    for TRANS_DWELL_CONFIRM_MS, else TRANS_OVERSHOOT_DWELL_MS (timeout) then assume engaged.
    //    needsThrottleBoost() is true exactly in this phase, so boost / idle hold follow it. ──
    if (gearChangePhase_ == GearChangePhase::OVERSHOOT_DWELL) {
        ShiftConfirmResult r = dwellConfirm_.step(now, gearPhaseStartTime_,
                                                  physicalGear == targetGear_,
                                                  TRANS_DWELL_CONFIRM_MS,
                                                  TRANS_OVERSHOOT_DWELL_MS);
        if (r == ShiftConfirmResult::WAIT) return;

        uint32_t waited = now - gearPhaseStartTime_;
        if (r == ShiftConfirmResult::CONFIRMED) {
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] Dwell confirmed after %lu ms\n", (unsigned long)waited);
        } else {
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] Dwell timeout %lu ms, gear assumed\n", (unsigned long)waited);
        }

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

        if (initStepActive_) {
            initStepActive_ = false;
            if (getPhysicalGear() == Gear::GEAR_UNKNOWN) {
                // No sensor confirmation: NEUTRAL is the working assumption (throttle stays
                // capped by the NEUTRAL target); the idle resync corrects it later.
                Debug::printlnFeature(DebugFeature::TRANSMISSION,
                    "[TRANS] Init: sensor silent, assuming NEUTRAL");
            }
        }
        saveState(targetGear_, getCurrentServoPct(), true);
        gearChangePhase_ = GearChangePhase::NONE;
        lastGearMismatch_ = false;
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Gear %s engaged at %.1f%%\n",
            getGearName(targetGear_), getCurrentServoPct());
        // Single step, or the last step of a sequence: log the whole shift's duration.
        if (queuedGear_ == Gear::GEAR_UNKNOWN || queuedGear_ == targetGear_) {
            Debug::printfFeature(DebugFeature::TRANSMISSION,
                "[TRANS] Shift %s -> %s took %lu ms\n",
                getGearName(shiftFromGear_), getGearName(targetGear_),
                (unsigned long)(now - shiftStartMs_));
        }
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

void TransmissionController::markStateInvalid() {
    Preferences prefs;
    if (!prefs.begin("transmission", false)) return;
    prefs.putBool("state_valid", false);
    prefs.end();
}

bool TransmissionController::loadState(Gear& gear, float& positionPct, bool& valid) {
    Preferences prefs;
    if (!prefs.begin("transmission", true)) return false;
    if (!prefs.isKey("state_valid")) { prefs.end(); return false; }
    valid       = prefs.getBool("state_valid", false);
    gear        = (Gear)prefs.getInt("state_gear", (int)Gear::GEAR_UNKNOWN);
    positionPct = prefs.getFloat("state_pct", NAN);   // missing → implausible (no hint)
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
