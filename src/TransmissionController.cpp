#include "TransmissionController.h"
#include "Debug.h"

TransmissionController::TransmissionController()
    : targetGear_(Gear::GEAR_NEUTRAL)
    , fromGear_(Gear::GEAR_NEUTRAL)
    , gearChangePhase_(GearChangePhase::NONE)
    , finalGearPositionPct_(TRANS_GEAR_DEFAULT_NEUTRAL_PCT)
    , overshootDirection_(1.0f)
    , gearPhaseStartTime_(0)
    , settleMs_(TRANS_SERVO_SETTLE_MIN_MS)
    , queuedGear_(Gear::GEAR_UNKNOWN)
    , sequenceStepDwellStart_(0)
    , boardInputs_(nullptr)
    , cachedPhysicalGear_(Gear::GEAR_UNKNOWN)
    , lastGearReadTime_(0)
    , lastGearRetryTime_(0)
    , gearReadRetries_(0)
    , lastLoggedGearCode_(0xFF)
    , lastGearCheckTime_(0)
    , lastStatusLogTime_(0)
    , lastGearMismatch_(false)
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

    for (int i = 0; i < 4; i++) gearCodes_[i] = defaultGearCode((Gear)i);
}

TransmissionController::~TransmissionController() {
}

bool TransmissionController::begin() {
    // The gear code table does not depend on the servo: load it first.
    loadGearCodes();
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

void TransmissionController::initGearSensors(BoardInputs& inputs) {
    boardInputs_ = &inputs;
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Gear code bound to opto input bits [%d %d %d] @0x%02X (engaged = HIGH)\n",
        BOARD_IN_BIT_GEAR_CODE_A, BOARD_IN_BIT_GEAR_CODE_B, BOARD_IN_BIT_GEAR_CODE_C,
        PCA9557_ADDR_INPUTS);
}

// ── gear code table ─────────────────────────────────────────────────────────
// Code bits [bit0 bit1 bit6], see Constants.h. The runtime table gearCodes_ starts
// from the GEAR_CODE_* defaults and may be overridden by the web calibration (NVS
// "transmission" code_r/n/h/l). Codes not in the table are GEAR_UNKNOWN.
namespace {
const char* const GEAR_CODE_KEYS[4] = { "code_h", "code_l", "code_n", "code_r" };  // by (int)Gear

inline unsigned codeBit(uint8_t code, int shift) { return (unsigned)((code >> shift) & 1u); }

// "XYZ" for an assigned code, "---" for GEAR_CODE_UNSET. `out` must hold 4 chars.
void formatGearCode(uint8_t code, char out[4]) {
    if (code > 0b111) { out[0] = out[1] = out[2] = '-'; }
    else {
        out[0] = (char)('0' + codeBit(code, 2));
        out[1] = (char)('0' + codeBit(code, 1));
        out[2] = (char)('0' + codeBit(code, 0));
    }
    out[3] = '\0';
}
} // namespace

uint8_t TransmissionController::defaultGearCode(Gear gear) {
    switch (gear) {
        case Gear::GEAR_HIGH:    return GEAR_CODE_HIGH;
        case Gear::GEAR_LOW:     return GEAR_CODE_LOW;
        case Gear::GEAR_NEUTRAL: return GEAR_CODE_NEUTRAL;
        case Gear::GEAR_REVERSE: return GEAR_CODE_REVERSE;
        default:                 return 0;
    }
}

bool TransmissionController::isGearCodeTableValid(const uint8_t codes[4]) {
    // Invariant: every ASSIGNED code is 001..110 (000/111 reserved for UNKNOWN) and the
    // assigned codes are distinct. GEAR_CODE_UNSET entries are ignored (they never decode).
    for (int i = 0; i < 4; i++) {
        if (codes[i] == GEAR_CODE_UNSET) continue;
        if (codes[i] == 0b000 || codes[i] >= 0b111) return false;
        for (int k = i + 1; k < 4; k++) {
            if (codes[k] != GEAR_CODE_UNSET && codes[i] == codes[k]) return false;
        }
    }
    return true;
}

void TransmissionController::invalidateGearCache() {
    // Force the next getPhysicalGear() to decode with the new table and re-log the code.
    lastGearReadTime_ = millis() - TRANS_GEAR_READ_INTERVAL_MS;
    gearReadRetries_ = 0;
    lastLoggedGearCode_ = 0xFF;
}

TransmissionController::Gear TransmissionController::decodeGearCode(uint8_t code) const {
    if (code > 0b111) return Gear::GEAR_UNKNOWN;   // never matches an UNSET (0xFF) entry
    for (int i = 0; i < 4; i++) {
        if (gearCodes_[i] == code) return (Gear)i;
    }
    return Gear::GEAR_UNKNOWN;
}

uint8_t TransmissionController::getGearCodeFor(Gear gear) const {
    int idx = (int)gear;
    return (idx >= 0 && idx < 4) ? gearCodes_[idx] : GEAR_CODE_UNSET;
}

bool TransmissionController::isGearCodeDefault(Gear gear) const {
    return getGearCodeFor(gear) == defaultGearCode(gear);
}

void TransmissionController::loadGearCodes() {
    uint8_t codes[4];
    for (int i = 0; i < 4; i++) codes[i] = defaultGearCode((Gear)i);

    Preferences prefs;
    if (prefs.begin("transmission", true)) {
        for (int i = 0; i < 4; i++) {
            if (prefs.isKey(GEAR_CODE_KEYS[i])) {
                codes[i] = prefs.getUChar(GEAR_CODE_KEYS[i], codes[i]);
            }
        }
        prefs.end();
    }

    if (!isGearCodeTableValid(codes)) {
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] ERROR: stored gear codes invalid (R=%u N=%u H=%u L=%u) — using defaults\n",
            (unsigned)codes[(int)Gear::GEAR_REVERSE], (unsigned)codes[(int)Gear::GEAR_NEUTRAL],
            (unsigned)codes[(int)Gear::GEAR_HIGH],    (unsigned)codes[(int)Gear::GEAR_LOW]);
        for (int i = 0; i < 4; i++) codes[i] = defaultGearCode((Gear)i);
    }

    for (int i = 0; i < 4; i++) gearCodes_[i] = codes[i];
    invalidateGearCache();

    char r[4], n[4], h[4], l[4];
    formatGearCode(gearCodes_[(int)Gear::GEAR_REVERSE], r);
    formatGearCode(gearCodes_[(int)Gear::GEAR_NEUTRAL], n);
    formatGearCode(gearCodes_[(int)Gear::GEAR_HIGH], h);
    formatGearCode(gearCodes_[(int)Gear::GEAR_LOW], l);
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Gear codes: R=%s N=%s H=%s L=%s\n", r, n, h, l);
}

bool TransmissionController::captureGearCode(Gear gear, String& msg) {
    int idx = (int)gear;
    if (idx < 0 || idx >= 4) {
        msg = "Gear code capture failed: invalid gear (use R/N/L/H)";
        return false;
    }
    const char* letter = getGearLetter(gear);
    if (gearChangePhase_ != GearChangePhase::NONE) {
        msg = String("Gear code capture for ") + letter + " failed: gear change in progress";
        return false;
    }
    int8_t live = getGearCode();
    if (live < 0) {
        msg = String("Gear code capture for ") + letter + " failed: opto inputs unavailable or stale";
        return false;
    }
    uint8_t code = (uint8_t)live;
    char bits[4];
    formatGearCode(code, bits);
    if (code == 0b000 || code == 0b111) {
        msg = String("Gear code capture for ") + letter + " failed: code " + bits +
              (code == 0b000 ? " (all lines open / mid-shift)" : " (opto feed lost)") +
              " is reserved for UNKNOWN";
        return false;
    }
    // Take-over: a code owned by another gear moves to this gear and the other gear
    // becomes UNSET (never decodes) until it is captured again. The table invariant
    // (assigned codes distinct) holds, and swapped codes can be fixed in two captures.
    int previousOwner = -1;
    for (int i = 0; i < 4; i++) {
        if (i != idx && gearCodes_[i] == code) { previousOwner = i; break; }
    }

    Preferences prefs;
    if (!prefs.begin("transmission", false)) {
        msg = String("Gear code capture for ") + letter + " failed: NVS unavailable";
        return false;
    }
    // Unset the previous owner first: a reset between the two writes leaves the code
    // unassigned (UNKNOWN, fail-safe) rather than assigned twice.
    size_t wroteOwner = 1;
    if (previousOwner >= 0) wroteOwner = prefs.putUChar(GEAR_CODE_KEYS[previousOwner], GEAR_CODE_UNSET);
    size_t wrote = (wroteOwner != 0) ? prefs.putUChar(GEAR_CODE_KEYS[idx], code) : 0;
    prefs.end();
    if (wrote == 0) {
        // Fail closed: reload what NVS really holds (falls back to defaults if inconsistent).
        loadGearCodes();
        msg = String("Gear code capture for ") + letter + " failed: NVS write error";
        return false;
    }

    if (previousOwner >= 0) gearCodes_[previousOwner] = GEAR_CODE_UNSET;
    gearCodes_[idx] = code;
    invalidateGearCache();
    msg = String("Gear code for ") + letter + " saved: " + bits;
    if (previousOwner >= 0) {
        const char* other = getGearLetter((Gear)previousOwner);
        msg += String(" (was ") + other + "; " + other + " is now unset)";
    }
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Gear code captured: %s = 0b%s%s%s%s\n", letter, bits,
        previousOwner >= 0 ? " (taken from " : "",
        previousOwner >= 0 ? getGearLetter((Gear)previousOwner) : "",
        previousOwner >= 0 ? ", now unset)" : "");
    return true;
}

bool TransmissionController::resetGearCodes() {
    for (int i = 0; i < 4; i++) gearCodes_[i] = defaultGearCode((Gear)i);
    invalidateGearCache();

    bool ok = false;
    Preferences prefs;
    if (prefs.begin("transmission", false)) {
        for (int i = 0; i < 4; i++) {
            if (prefs.isKey(GEAR_CODE_KEYS[i])) prefs.remove(GEAR_CODE_KEYS[i]);
        }
        prefs.end();
        ok = true;
    }
    Debug::printfFeature(DebugFeature::TRANSMISSION,
        "[TRANS] Gear codes reset to defaults%s\n", ok ? "" : " (NVS unavailable — RAM only)");
    return ok;
}

const char* TransmissionController::getGearLetter(Gear gear) {
    switch (gear) {
        case Gear::GEAR_REVERSE: return "R";
        case Gear::GEAR_NEUTRAL: return "N";
        case Gear::GEAR_HIGH:    return "H";
        case Gear::GEAR_LOW:     return "L";
        default:                 return "?";
    }
}

int8_t TransmissionController::getGearCode() const {
    if (boardInputs_ == nullptr) return -1;
    BoardInputs::Snapshot snap = boardInputs_->getSnapshot();
    return snap.valid ? (int8_t)snap.gearCode : -1;
}

TransmissionController::Gear TransmissionController::getPhysicalGear() const {
    uint32_t now = millis();
    if (now - lastGearReadTime_ < TRANS_GEAR_READ_INTERVAL_MS) {
        return cachedPhysicalGear_;
    }

    // No input source, or the expander snapshot is stale: UNKNOWN is the intended
    // degraded mode (throttle capped at TRANS_UNKNOWN_GEAR_THROTTLE_MAX, gear-change
    // confirmation blocked) — no extra interlock is needed.
    if (boardInputs_ == nullptr) {
        lastGearReadTime_ = now;
        cachedPhysicalGear_ = Gear::GEAR_UNKNOWN;
        return cachedPhysicalGear_;
    }

    BoardInputs::Snapshot snap = boardInputs_->getSnapshot();
    if (!snap.valid) {
        lastGearReadTime_ = now;
        gearReadRetries_ = 0;
        cachedPhysicalGear_ = Gear::GEAR_UNKNOWN;
        return cachedPhysicalGear_;
    }

    uint8_t code = snap.gearCode;
    Gear decoded = decodeGearCode(code);

    if (code != lastLoggedGearCode_) {
        lastLoggedGearCode_ = code;
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] gear code=0b%u%u%u -> %s\n",
            (unsigned)((code >> 2) & 1), (unsigned)((code >> 1) & 1), (unsigned)(code & 1),
            decoded == Gear::GEAR_UNKNOWN ? "UNKNOWN" : getGearLetter(decoded));
    }

    // Code decodes to UNKNOWN while the servo is idle: consume the NEXT expander
    // poll instead of issuing back-to-back I2C reads (filters transient bounces
    // without blocking the control loop). At most TRANS_GEAR_READ_RETRY_COUNT deferrals.
    if (decoded == Gear::GEAR_UNKNOWN && gearChangePhase_ == GearChangePhase::NONE
        && gearReadRetries_ < TRANS_GEAR_READ_RETRY_COUNT) {
        if ((now - lastGearRetryTime_) >= BOARD_INPUT_POLL_MS) {
            lastGearRetryTime_ = now;
            gearReadRetries_++;
        }
        return cachedPhysicalGear_;
    }

    lastGearReadTime_ = now;
    gearReadRetries_ = 0;

    cachedPhysicalGear_ = decoded;
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
