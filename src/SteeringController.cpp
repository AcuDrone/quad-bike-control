#include "SteeringController.h"
#include "Debug.h"
#include <Preferences.h>

// NVS calibration layout version ("steering"/cal_ver). Version 2 = dual-sensor pairs.
static constexpr int32_t STEER_CAL_VERSION = 2;

SteeringController::SteeringController(IMotorDriver& motor, TwoWire& fineBus, TwoWire& coarseBus)
    : fine_(fineBus, "fine", true),        // sole device on I2C1: may restart its own bus
      coarse_(coarseBus, "coarse", false), // shares I2C2 with the expanders: never restarts it
      motor_(motor),
      fineRaw_(0),
      fineOk_(false),
      fineFresh_(false),
      fineBad_(STEER_SENSOR_FAULT_DEBOUNCE),
      fineReason_(SensorFault::FINE_I2C),
      haveFine_(false),
      prevFineRaw_(0),
      originRaw_(0),
      turn_(0),
      sessionTurn_(0),
      coarseRaw_(0),
      coarseOk_(false),
      coarseBad_(STEER_SENSOR_FAULT_DEBOUNCE),
      coarseReason_(SensorFault::COARSE_I2C),
      lastCoarsePollMs_(0),
      turnRecovered_(false),
      recoverCount_(0),
      recoverOffset_(0),
      mismatch_(false),
      mismatchLatched_(false),
      mismatchCount_(0),
      residual_(0),
      residualValid_(false),
      sensorOk_(false),
      fault_(SensorFault::FINE_I2C),
      fineCenterRaw_(-1),
      coarseCenterRaw_(-1),
      fineLeft_(POS_UNSET),
      coarseLeft_(-1),
      fineRight_(POS_UNSET),
      coarseRight_(-1),
      calSlot_{},
      calFirst_(-1),
      calLost_(false),
      sessionError_(nullptr),
      calibrated_(false),
      invert_(false),
      relLeft_(0),
      relRight_(0),
      seam_(0),
      ratio_(0.0f),
      linLeft_(0),
      calError_(nullptr),
      captureError_(nullptr),
      lastCapturePos_(0),
      lastCaptureCoarse_(0),
      targetRel_(0),
      isMoving_(false),
      moveStartTime_(0),
      lastMoveLogTime_(0),
      dutyBoost_(0),
      lastProgressPos_(0),
      lastProgressTime_(0),
      jogDir_(0),
      lastJogRefresh_(0),
      jogPulseUntil_(0),
      lastStallPosition_(0),
      lastStallCheckTime_(0),
      lastDriveDir_(0),
      stallLatched_(false),
      stallLatchDir_(0),
      stallLatchTime_(0),
      steerDriverOk_(false),
      bootCenterPending_(false),
      bootCenterRequestMs_(0) {
}

bool SteeringController::begin(gpio_num_t fineSda, gpio_num_t fineScl,
                               gpio_num_t coarseSda, gpio_num_t coarseScl) {
    // The motor driver is initialized by the caller (e.g. VESC UART begin())
    // before this point; command a coast so the actuator starts safe.
    motor_.stop();

    bool fineUp = fine_.begin(fineSda, fineScl);
    bool coarseUp = coarse_.begin(coarseSda, coarseScl);
    if (!fineUp) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] ERROR: fine AS5600 (I2C1/X14) init failed — re-probing");
    }
    if (!coarseUp) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] ERROR: coarse AS5600 (I2C2/X15) init failed — re-probing");
    }

    // Calibration is ALWAYS loaded: a sensor that answers later (re-probe) makes
    // steering available without a reboot, after turn recovery.
    loadCalibration();

    // Prime both sensors and attempt turn recovery right away, so setup() knows
    // whether to request boot centering. Long enough for the debounce (a single
    // bad sample at boot must not leave a sensor marked faulted) plus the
    // STEER_TURN_RECOVER_SAMPLES agreeing coarse polls a turn recovery needs.
    for (uint8_t i = 0; i < STEER_SENSOR_FAULT_DEBOUNCE + STEER_TURN_RECOVER_SAMPLES && !sensorOk_; i++) {
        refreshSensors(millis(), true);
    }

    Debug::printfFeature(DebugFeature::SERVO,
                         "[STEER] Initialized: pos=%ld turn=%ld fine=%ld coarse=%ld, sensor %s (%s)\n",
                         (long)getPosition(), (long)turn_, (long)getFineRaw(), (long)getCoarseRaw(),
                         sensorOk_ ? "OK" : "FAULT", faultToken(fault_));
    return fineUp && coarseUp;
}

// === Sensors ===

const char* SteeringController::faultToken(SensorFault f) {
    switch (f) {
        case SensorFault::NONE:          return "none";
        case SensorFault::FINE_I2C:      return "fine_i2c";
        case SensorFault::FINE_MAGNET:   return "fine_magnet";
        case SensorFault::COARSE_I2C:    return "coarse_i2c";
        case SensorFault::COARSE_MAGNET: return "coarse_magnet";
        case SensorFault::MISMATCH:      return "mismatch";
        case SensorFault::RECOVERING:    return "recovering";
    }
    return "none";
}

int32_t SteeringController::getPosition() const {
    int32_t origin = (fineCenterRaw_ >= 0) ? fineCenterRaw_ : originRaw_;
    return fineRaw_ + 4096 * turn_ - origin;
}

int32_t SteeringController::relPosition() const {
    int32_t pos = getPosition();
    return invert_ ? -pos : pos;
}

float SteeringController::expectedFine(int32_t coarseRaw) const {
    return (float)fineLeft_ + (float)(coarseLin(coarseRaw) - linLeft_) / ratio_;
}

void SteeringController::sampleFine() {
    uint16_t raw;
    bool magnetOk = false;
    bool readOk = fine_.read(raw, magnetOk);

    // A sample is good only when the bus read succeeded AND the magnet is seen;
    // a read right after a bus glitch can carry garbage with the MD bit clear,
    // so neither the position nor the turn counter is updated from bad samples.
    fineFresh_ = readOk && magnetOk;
    if (fineFresh_) {
        int32_t r = raw;
        if (!haveFine_) {
            haveFine_ = true;
            if (fineCenterRaw_ < 0) originRaw_ = r;   // position origin until a center is captured
        } else {
            // Incremental turn tracking: the shaft cannot move half a turn between
            // two loop samples, so a raw jump of more than 2048 counts is a wrap.
            // The session turn follows the same wraps but is never re-derived.
            int32_t d = r - prevFineRaw_;
            if (d > 2048) {
                turn_--;
                sessionTurn_--;
            } else if (d < -2048) {
                turn_++;
                sessionTurn_++;
            }
        }
        prevFineRaw_ = r;
        fineRaw_ = r;
        fineBad_ = 0;
        if (!fineOk_) {
            fineOk_ = true;
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Fine sensor OK");
        }
        return;
    }

    // Debounce: single EMI glitches keep the last position for a few loop
    // iterations (~ms) instead of stopping the motor and flapping fault logs.
    if (fineBad_ < 255) fineBad_++;
    if (fineBad_ >= STEER_SENSOR_FAULT_DEBOUNCE) {
        fineReason_ = readOk ? SensorFault::FINE_MAGNET : SensorFault::FINE_I2C;
        if (fineOk_) {
            fineOk_ = false;
            turnRecovered_ = false;   // the shaft may move unobserved: recover the turn on return
            resetRecovery();
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] Fine sensor fault (%s)\n",
                                 faultToken(fineReason_));
            // Fine continuity is lost: the session frame of unfinished points is gone.
            discardCalSession("fine sensor fault", true);
        }
    }
}

void SteeringController::sampleCoarse() {
    uint16_t raw;
    bool magnetOk = false;
    bool readOk = coarse_.read(raw, magnetOk);

    if (readOk && magnetOk) {
        coarseRaw_ = raw;
        coarseBad_ = 0;
        if (!coarseOk_) {
            coarseOk_ = true;
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Coarse sensor OK");
        }
        // The cross-check needs a fresh fine sample from this same iteration.
        if (calibrated_ && fineOk_ && fineFresh_) {
            crossCheck();
        } else {
            resetRecovery();   // a poll without a usable pair does not count as agreement
        }
        return;
    }

    resetRecovery();           // recovery needs CONSECUTIVE good coarse polls
    if (coarseBad_ < 255) coarseBad_++;
    if (coarseBad_ >= STEER_SENSOR_FAULT_DEBOUNCE) {
        coarseReason_ = readOk ? SensorFault::COARSE_MAGNET : SensorFault::COARSE_I2C;
        if (coarseOk_) {
            coarseOk_ = false;
            turnRecovered_ = false;
            resetRecovery();
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] Coarse sensor fault (%s)\n",
                                 faultToken(coarseReason_));
            discardCalSession("coarse sensor fault", true);
        }
    }
}

void SteeringController::crossCheck() {
    float expected = expectedFine(coarseRaw_);
    int32_t expectedCounts = (int32_t)lroundf(expected);

    if (!turnRecovered_) {
        // Turn recovery: pick the turn whose unwrapped fine position is closest to
        // the coarse-derived expectation. The 1024-count threshold is below half a
        // turn, so the choice of turn can never be ambiguous. A single sample is
        // never trusted: STEER_TURN_RECOVER_SAMPLES consecutive coarse polls must
        // agree on the same turn. Agreement is compared as the correction k − turn_
        // to the incremental counter, so a fine wrap between polls (wheel pushed
        // while the motor is stopped) does not count as a disagreement.
        int32_t base = fineRaw_ - fineCenterRaw_;
        int32_t k = (int32_t)lroundf((expected - (float)base) / 4096.0f);
        int32_t pos = base + 4096 * k;
        residual_ = pos - expectedCounts;
        residualValid_ = true;
        if (abs(residual_) > STEER_SENSOR_MISMATCH_COUNTS) {
            resetRecovery();
            if (mismatchCount_ < 255) mismatchCount_++;
            if (mismatchCount_ >= STEER_SENSOR_FAULT_DEBOUNCE && !mismatch_) {
                mismatch_ = true;
                Debug::printfFeature(DebugFeature::SERVO,
                                     "[STEER] Turn recovery: sensors disagree fine=%ld coarse=%ld resid=%ld — mismatch\n",
                                     (long)fineRaw_, (long)coarseRaw_, (long)residual_);
            }
            return;
        }
        mismatchCount_ = 0;
        mismatch_ = false;   // recovery disagreement is transient; only a runtime mismatch latches

        int32_t offset = k - turn_;
        if (recoverCount_ > 0 && offset == recoverOffset_) {
            recoverCount_++;
        } else {
            recoverOffset_ = offset;   // first sample, or a different turn: restart the count
            recoverCount_ = 1;
        }
        if (recoverCount_ < STEER_TURN_RECOVER_SAMPLES) return;

        turn_ = k;
        turnRecovered_ = true;
        resetRecovery();
        Debug::printfFeature(DebugFeature::SERVO,
                             "[STEER] Turn recovered (%d samples): pos=%ld turn=%ld coarse=%ld resid=%ld\n",
                             (int)STEER_TURN_RECOVER_SAMPLES, (long)getPosition(), (long)turn_,
                             (long)coarseRaw_, (long)residual_);
        return;
    }

    // Runtime consistency check: a persistent disagreement means a slipped gear or
    // a lost turn. Re-deriving the turn from one side cannot tell which side is
    // wrong, so the mismatch LATCHES (cleared only by a completed calibration or a reboot).
    // The residual keeps being reported while latched, for diagnosis.
    residual_ = getPosition() - expectedCounts;
    residualValid_ = true;
    if (mismatchLatched_) return;
    if (abs(residual_) > STEER_SENSOR_MISMATCH_COUNTS) {
        if (mismatchCount_ < 255) mismatchCount_++;
        if (mismatchCount_ >= STEER_SENSOR_FAULT_DEBOUNCE) {
            mismatchLatched_ = true;
            mismatchCount_ = 0;
            Debug::printfFeature(DebugFeature::SERVO,
                                 "[STEER] Sensor MISMATCH (latched until a completed calibration or reboot): pos=%ld turn=%ld coarse=%ld resid=%ld\n",
                                 (long)getPosition(), (long)turn_, (long)coarseRaw_, (long)residual_);
            // Points pre-filled from (or recorded against) a now-suspect geometry
            // must not complete a calibration that would clear this latch.
            discardCalSession("sensor mismatch", true);
        }
    } else {
        mismatchCount_ = 0;
    }
}

void SteeringController::updateCombinedState() {
    bool ok = fineOk_ && coarseOk_ &&
              (!calibrated_ || (turnRecovered_ && !mismatch_ && !mismatchLatched_));

    // Every !ok state maps to a token other than NONE.
    SensorFault f = SensorFault::NONE;
    if (!fineOk_) {
        f = fineReason_;
    } else if (!coarseOk_) {
        f = coarseReason_;
    } else if (calibrated_ && (mismatchLatched_ || mismatch_)) {
        f = SensorFault::MISMATCH;
    } else if (calibrated_ && !turnRecovered_) {
        f = SensorFault::RECOVERING;
    }
    fault_ = f;

    if (ok == sensorOk_) return;
    sensorOk_ = ok;
    if (!ok) {
        Debug::printfFeature(DebugFeature::SERVO, "[STEER] SENSOR FAULT (%s) — motor stopped\n",
                             faultToken(f));
    } else {
        Debug::printfFeature(DebugFeature::SERVO, "[STEER] Sensors OK: pos=%ld turn=%ld coarse=%ld\n",
                             (long)getPosition(), (long)turn_, (long)coarseRaw_);
    }
}

bool SteeringController::refreshSensors(uint32_t now, bool forceCoarse) {
    sampleFine();
    if (forceCoarse || now - lastCoarsePollMs_ >= STEER_COARSE_POLL_MS) {
        lastCoarsePollMs_ = now;
        sampleCoarse();
    }
    if (!calibrated_ || !fineOk_ || !coarseOk_) residualValid_ = false;
    updateCombinedState();

    if (!sensorOk_) {
        motor_.stop();
        isMoving_ = false;
        jogDir_ = 0;
        jogPulseUntil_ = 0;
        return false;
    }
    return true;   // within the debounce window: control continues on the last good position
}

// === Movement ===

void SteeringController::stop() {
    motor_.stop();
    isMoving_ = false;
    jogDir_ = 0;
    jogPulseUntil_ = 0;
}

void SteeringController::jog(int8_t direction, uint8_t duty) {
    if (direction != 0 && (!sensorOk_ || !steerDriverOk_)) return;   // no open-loop drive without stall backstop / driver link
    if (direction != 0) bootCenterPending_ = false;   // operator command supersedes boot centering

    if (direction == 0) {
        if (jogDir_ != 0) stop();
        return;
    }

    int8_t d = (direction > 0) ? 1 : -1;
    if (latchBlocks(d, millis())) return;   // stalled in this direction, still in cooldown
    stallLatched_ = false;                  // accepted move clears the latch

    if (duty < STEER_PWM_MIN_DUTY) duty = STEER_PWM_MIN_DUTY;   // below this the actuator stalls

    isMoving_ = false;   // jog overrides position control
    if (jogDir_ != direction) {
        lastStallPosition_ = relPosition();
        lastStallCheckTime_ = millis();
    }
    jogDir_ = d;
    lastDriveDir_ = d;
    lastJogRefresh_ = millis();
    jogPulseUntil_ = 0;
    motor_.setSpeed(jogDir_ * (int16_t)duty);
}

void SteeringController::nudge(int8_t direction) {
    if (direction == 0 || !sensorOk_ || !steerDriverOk_) return;
    bootCenterPending_ = false;   // operator command supersedes boot centering

    int8_t d = (direction > 0) ? 1 : -1;
    if (latchBlocks(d, millis())) return;   // same-direction refusal during cooldown
    stallLatched_ = false;

    isMoving_ = false;   // nudge overrides position control
    if (jogDir_ != direction) {
        lastStallPosition_ = relPosition();
        lastStallCheckTime_ = millis();
    }
    jogDir_ = d;
    lastDriveDir_ = d;
    lastJogRefresh_ = millis();
    jogPulseUntil_ = millis() + STEER_NUDGE_MS;
    motor_.setSpeed(jogDir_ * STEER_NUDGE_DUTY);
}

void SteeringController::update() {
    uint32_t now = millis();

    // Service the motor driver first: pump VESC UART, refresh telemetry, and run
    // the fault / over-current / comm-timeout monitors. Returns false when the
    // driver link is down (mirrors the sensor-fault path: stop + reject).
    // Sample the sensors FIRST, every iteration, even while the VESC link is down:
    // the fine-sensor turn counter only stays valid if no sample is skipped.
    bool sensorsOk = refreshSensors(now);
    bool driverOk = serviceDriver(now);

    serviceBootCenter(now, sensorsOk && driverOk);

    if (!driverOk) return;

    if (!sensorsOk) return;

    now = millis();

    // Stall detection — applies to any active drive (position move or jog)
    if (isMoving_ || jogDir_ != 0) {
        int32_t currentPos = relPosition();
        if (now - lastStallCheckTime_ >= STEER_STALL_TIMEOUT) {
            if (abs(currentPos - lastStallPosition_) < STEER_STALL_THRESHOLD) {
                // Direction that was being driven into the jam (jog dir, else
                // the sign of the remaining position error).
                int8_t dir = (jogDir_ != 0) ? jogDir_
                             : ((targetRel_ - currentPos) > 0 ? 1 : -1);
                triggerStallStop(dir, "position");
                return;
            }
            lastStallPosition_ = currentPos;
            lastStallCheckTime_ = now;
        }
    }

    // Jog mode (open loop, momentary)
    if (jogDir_ != 0) {
        if (jogPulseUntil_ != 0 && (int32_t)(now - jogPulseUntil_) >= 0) {
            stop();   // nudge pulse complete
            return;
        }
        if (now - lastJogRefresh_ >= STEER_JOG_TIMEOUT_MS) {
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Jog timeout — stopping");
            stop();
        }
        return;
    }

    if (!isMoving_) return;

    // Closed-loop proportional position control
    int32_t rel = relPosition();
    int32_t error = targetRel_ - rel;

    if (abs(error) <= STEER_POSITION_TOLERANCE) {
        stop();
        return;
    }

    if (now - moveStartTime_ >= STEER_MOVE_TIMEOUT) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Movement timeout — stopping");
        stop();
        return;
    }

    // Anti-stall boost: load rises toward the locks; if commanded motion makes
    // no progress, escalate duty above the P term until movement resumes.
    if (now - lastProgressTime_ >= STEER_BOOST_CHECK_MS) {
        if (abs(rel - lastProgressPos_) < STEER_BOOST_MIN_PROGRESS) {
            if (dutyBoost_ < STEER_PWM_MAX_DUTY) dutyBoost_ += STEER_BOOST_STEP;
        } else if (dutyBoost_ > 0) {
            dutyBoost_ -= (dutyBoost_ < STEER_BOOST_STEP) ? dutyBoost_ : STEER_BOOST_STEP;
        }
        lastProgressPos_ = rel;
        lastProgressTime_ = now;
    }

    float duty = fabsf((float)error) * STEER_KP + dutyBoost_;
    if (duty < STEER_PWM_MIN_DUTY) duty = STEER_PWM_MIN_DUTY;
    if (duty > STEER_PWM_MAX_DUTY) duty = STEER_PWM_MAX_DUTY;

    // Motor positive = wheel right = normalized position increasing
    int16_t speed = (int16_t)duty;
    lastDriveDir_ = (error > 0) ? 1 : -1;
    motor_.setSpeed(error > 0 ? speed : (int16_t)-speed);

    if (now - lastMoveLogTime_ >= 100) {
        lastMoveLogTime_ = now;
        Debug::printfFeature(DebugFeature::SERVO,
                             "[STEER] pos=%ld turn=%ld coarse=%ld resid=%ld rel=%ld target=%ld err=%ld duty=%d boost=%d\n",
                             (long)getPosition(), (long)turn_, (long)coarseRaw_,
                             (long)(residualValid_ ? residual_ : 0), (long)rel, (long)targetRel_,
                             (long)error, (int)(error > 0 ? speed : -speed), (int)dutyBoost_);
    }
}

// === Stall latch + VESC driver monitors ===

bool SteeringController::latchBlocks(int8_t d, uint32_t now) const {
    return stallLatched_ && d != 0 && d == stallLatchDir_ &&
           (now - stallLatchTime_) < STEER_STALL_COOLDOWN_MS;
}

void SteeringController::triggerStallStop(int8_t dir, const char* reason) {
    motor_.stop();
    isMoving_ = false;
    jogDir_ = 0;
    jogPulseUntil_ = 0;
    dutyBoost_ = 0;
    stallLatched_ = true;
    stallLatchDir_ = dir;
    stallLatchTime_ = millis();
    Debug::printfFeature(DebugFeature::SERVO,
                         "[STEER] Stall-stop (%s) dir=%d — latched for %d ms\n",
                         reason, (int)dir, STEER_STALL_COOLDOWN_MS);
}

bool SteeringController::serviceDriver(uint32_t now) {
    motor_.poll();   // non-blocking: pump VESC UART + periodic GET_VALUES (no-op for BTS7960)

    // Comm failsafe (mirror the sensor-fault path): no valid telemetry link ->
    // stop, reject moves, flag driver down. Auto-recovers when replies resume.
    bool ok = motor_.driverOk();
    if (!ok) {
        if (steerDriverOk_) {
            Debug::printlnFeature(DebugFeature::SERVO,
                                  "[STEER] VESC driver fault (no telemetry) — motor stopped");
        }
        steerDriverOk_ = false;
        motor_.stop();
        isMoving_ = false;
        jogDir_ = 0;
        jogPulseUntil_ = 0;
        return false;
    }
    if (!steerDriverOk_) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] VESC driver recovered");
    }
    steerDriverOk_ = true;

    // Nonzero VESC fault code -> immediate stop + latch (current drive direction).
    // This is the over-current backstop: the VESC raises a fault on Absolute Max
    // Current (and on over-temperature), and clamps at Motor Current Max below that.
    if (motor_.hasFault()) {
        if (isMoving_ || jogDir_ != 0) {
            triggerStallStop(lastDriveDir_, "vesc-fault");
        } else {
            motor_.stop();
        }
        return false;
    }

    return true;
}

// === Steering interface ===

bool SteeringController::setSteeringPercent(float percent) {
    if (!calibrated_ || !sensorOk_ || !steerDriverOk_) {
        return false;   // a rejected command leaves a pending boot centering in place
    }
    bootCenterPending_ = false;   // an accepted command supersedes boot centering

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

    // Re-command no-op guard: if a move is already in progress and the new
    // target is within tolerance of the current target, keep the in-flight move
    // untouched (do NOT reset moveStartTime_/stall timers/dutyBoost_). This lets
    // the 500 ms stall window and 7500 ms move timeout actually elapse under the
    // continuous ~25 Hz MAVLink command stream (the fixed re-command bug). Not a
    // rejection — the command is honored as a no-op.
    if (isMoving_ && abs(target - targetRel_) <= STEER_RETARGET_TOLERANCE) {
        return true;
    }

    // Stall latch: refuse a fresh move that would push further into a recent jam
    // (same direction, within cooldown). Opposite-direction and post-cooldown
    // moves are accepted and clear the latch.
    uint32_t now = millis();
    int32_t rel = relPosition();
    int8_t d = (target > rel) ? 1 : (target < rel ? -1 : 0);
    if (latchBlocks(d, now)) {
        return false;
    }
    stallLatched_ = false;   // accepted move clears the latch

    targetRel_ = target;
    jogDir_ = 0;
    isMoving_ = true;
    moveStartTime_ = now;
    lastStallPosition_ = rel;
    lastStallCheckTime_ = now;
    dutyBoost_ = 0;
    lastProgressPos_ = rel;
    lastProgressTime_ = now;
    return true;
}

void SteeringController::requestBootCenter() {
    bootCenterPending_ = true;
    bootCenterRequestMs_ = millis();
}

void SteeringController::serviceBootCenter(uint32_t now, bool ready) {
    if (!bootCenterPending_) return;
    // A VESC that comes up late must not make the vehicle steer by itself much
    // later: give up silently once the window has passed.
    if (now - bootCenterRequestMs_ >= STEER_BOOT_CENTER_TIMEOUT_MS) {
        bootCenterPending_ = false;
        return;
    }
    // sensorOk_ implies the absolute turn is recovered without a mismatch.
    if (!ready || !calibrated_ || !sensorOk_ || !steerDriverOk_) return;

    bootCenterPending_ = false;
    if (setSteeringPercent(0.0f)) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Moving to center");
    }
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
    return capturePoint(CAL_CENTER);
}

bool SteeringController::captureLeftLimit() {
    return capturePoint(CAL_LEFT);
}

bool SteeringController::captureRightLimit() {
    return capturePoint(CAL_RIGHT);
}

int32_t SteeringController::displayRef() const {
    if (calSlot_[CAL_CENTER].valid) return calSlot_[CAL_CENTER].sessionPos;
    if (calFirst_ >= 0 && calSlot_[calFirst_].valid) return calSlot_[calFirst_].sessionPos;
    return 0;
}

int32_t SteeringController::getDisplayPosition() const {
    return isCalPending() ? sessionPos() - displayRef() : getPosition();
}

bool SteeringController::getDisplayPoint(CalPoint p, int32_t& pos) const {
    if (p >= CAL_POINTS) return false;
    if (isCalPending()) {
        if (!calSlot_[p].valid) return false;
        pos = calSlot_[p].sessionPos - displayRef();
        return true;
    }
    switch (p) {
        case CAL_CENTER:
            if (!hasCenter()) return false;
            pos = 0;
            return true;
        case CAL_LEFT:
            if (fineLeft_ == POS_UNSET) return false;
            pos = fineLeft_;
            return true;
        case CAL_RIGHT:
            if (fineRight_ == POS_UNSET) return false;
            pos = fineRight_;
            return true;
        default:
            return false;
    }
}

void SteeringController::discardCalSession(const char* why, bool byFault) {
    bool had = isCalPending();
    for (uint8_t i = 0; i < CAL_POINTS; i++) calSlot_[i] = CalSlot{};
    calFirst_ = -1;
    sessionError_ = nullptr;
    if (had && byFault) {
        calLost_ = true;
        Debug::printfFeature(DebugFeature::SERVO,
                             "[STEER] Unfinished calibration points discarded (%s)\n", why);
    }
}

void SteeringController::prefillFromCalibration() {
    // Only a complete, valid calibration whose absolute turn is currently trusted
    // can be re-expressed in the session frame (sensorOk_ implies recovered turn
    // and no mismatch while calibrated).
    if (!calibrated_ || !sensorOk_ || !turnRecovered_ || mismatch_ || mismatchLatched_) return;

    const int32_t center = sessionPos() - getPosition();
    const CalSlot stored[CAL_POINTS] = {
        { true, center + fineLeft_,  coarseLeft_ },
        { true, center,              coarseCenterRaw_ },
        { true, center + fineRight_, coarseRight_ },
    };
    for (uint8_t i = 0; i < CAL_POINTS; i++) {
        if (!calSlot_[i].valid) calSlot_[i] = stored[i];
    }
    if (calFirst_ < 0) calFirst_ = CAL_CENTER;
}

bool SteeringController::finalizeCalibration() {
    const int32_t center = calSlot_[CAL_CENTER].sessionPos;
    const int32_t newCenterRaw = center & 4095;   // two's complement: mod 4096 for negatives too

    // Validate the candidate with the shared checks; on rejection the previously
    // active calibration is restored unchanged (nothing is persisted).
    const int32_t oFc = fineCenterRaw_, oCc = coarseCenterRaw_;
    const int32_t oFl = fineLeft_, oCl = coarseLeft_, oFr = fineRight_, oCr = coarseRight_;

    fineCenterRaw_ = newCenterRaw;
    coarseCenterRaw_ = calSlot_[CAL_CENTER].coarseRaw;
    fineLeft_ = calSlot_[CAL_LEFT].sessionPos - center;
    coarseLeft_ = calSlot_[CAL_LEFT].coarseRaw;
    fineRight_ = calSlot_[CAL_RIGHT].sessionPos - center;
    coarseRight_ = calSlot_[CAL_RIGHT].coarseRaw;
    revalidateCalibration();

    if (!calibrated_) {
        sessionError_ = calError_ ? calError_ : "calibration rejected";
        fineCenterRaw_ = oFc; coarseCenterRaw_ = oCc;
        fineLeft_ = oFl; coarseLeft_ = oCl; fineRight_ = oFr; coarseRight_ = oCr;
        revalidateCalibration();
        return false;
    }

    // The session frame defines the absolute turn: choose turn_ so that
    // getPosition() == sessionPos() − center. center − newCenterRaw is an exact
    // multiple of 4096.
    turn_ = sessionTurn_ - (center - newCenterRaw) / 4096;
    turnRecovered_ = true;
    mismatch_ = false;
    mismatchLatched_ = false;   // a complete new calibration resolves a latched mismatch
    mismatchCount_ = 0;
    resetRecovery();
    residualValid_ = false;
    saveCalibration();
    updateCombinedState();
    // The points now live in the active calibration; keeping them would let a
    // later single capture re-finalize against a geometry that may have gone stale.
    discardCalSession("finalized", false);
    return true;
}

bool SteeringController::capturePoint(CalPoint p) {
    static const char* const kName[CAL_POINTS] = { "Left limit", "Center", "Right limit" };
    captureError_ = nullptr;

    // Both sensors must be individually healthy with a fresh fine sample. A
    // mismatch / recovering state does not block: completing all three points
    // is the way out of a stale calibration.
    if (!fineOk_ || !coarseOk_ || !fineFresh_) {
        captureError_ = "sensor fault";
        return false;
    }

    // Pre-fill is provisional until the capture is accepted: a refused capture
    // leaves the session exactly as it was.
    CalSlot saved[CAL_POINTS];
    for (uint8_t i = 0; i < CAL_POINTS; i++) saved[i] = calSlot_[i];
    const int8_t savedFirst = calFirst_;
    prefillFromCalibration();

    const int32_t pos = sessionPos();
    // Early feedback: a limit must be clear of the center point (either order).
    if (p == CAL_CENTER) {
        if (calSlot_[CAL_LEFT].valid && abs(calSlot_[CAL_LEFT].sessionPos - pos) < STEER_CAL_MIN_SPAN) {
            captureError_ = "too close to left limit";
        } else if (calSlot_[CAL_RIGHT].valid && abs(calSlot_[CAL_RIGHT].sessionPos - pos) < STEER_CAL_MIN_SPAN) {
            captureError_ = "too close to right limit";
        }
    } else if (calSlot_[CAL_CENTER].valid &&
               abs(pos - calSlot_[CAL_CENTER].sessionPos) < STEER_CAL_MIN_SPAN) {
        captureError_ = "too close to center";
    }
    if (captureError_) {
        for (uint8_t i = 0; i < CAL_POINTS; i++) calSlot_[i] = saved[i];
        calFirst_ = savedFirst;
        Debug::printfFeature(DebugFeature::SERVO, "[STEER] %s %s — rejected\n", kName[p], captureError_);
        return false;
    }

    stop();   // the control frame may change: never keep driving on the old one
    bootCenterPending_ = false;
    calLost_ = false;
    sessionError_ = nullptr;
    calSlot_[p] = { true, pos, coarseRaw_ };
    if (calFirst_ < 0) calFirst_ = (int8_t)p;

    lastCapturePos_ = pos - displayRef();
    lastCaptureCoarse_ = coarseRaw_;

    bool done = calSlot_[CAL_LEFT].valid && calSlot_[CAL_CENTER].valid && calSlot_[CAL_RIGHT].valid;
    bool finalized = done && finalizeCalibration();
    Debug::printfFeature(DebugFeature::SERVO,
                         "[STEER] %s captured: pos=%ld coarse=%ld (points %c%c%c)%s\n",
                         kName[p], (long)lastCapturePos_, (long)lastCaptureCoarse_,
                         calSlot_[CAL_LEFT].valid || finalized ? 'L' : '-',
                         calSlot_[CAL_CENTER].valid || finalized ? 'C' : '-',
                         calSlot_[CAL_RIGHT].valid || finalized ? 'R' : '-',
                         finalized ? " — calibration complete and saved" : "");
    if (sessionError_) {
        Debug::printfFeature(DebugFeature::SERVO, "[STEER] Calibration rejected: %s (re-capture the bad point)\n",
                             sessionError_);
    }
    return true;
}

void SteeringController::revalidateCalibration() {
    calibrated_ = false;
    invert_ = false;
    relLeft_ = 0;
    relRight_ = 0;
    seam_ = 0;
    ratio_ = 0.0f;
    linLeft_ = 0;
    calError_ = nullptr;

    // Incomplete is not an error: the operator is still capturing.
    if (fineCenterRaw_ < 0 || coarseCenterRaw_ < 0 ||
        fineLeft_ == POS_UNSET || coarseLeft_ < 0 ||
        fineRight_ == POS_UNSET || coarseRight_ < 0) {
        return;
    }

    // 1. Limits on opposite sides of center, each at least the minimum span away
    if (fineLeft_ == 0 || fineRight_ == 0 || (fineLeft_ < 0) == (fineRight_ < 0)) {
        calError_ = "limits not on opposite sides of center";
        return;
    }
    if (abs(fineLeft_) < STEER_CAL_MIN_SPAN || abs(fineRight_) < STEER_CAL_MIN_SPAN) {
        calError_ = "limit too close to center";
        return;
    }

    // Coarse travel arc: the arc from the left to the right coarse capture that
    // contains the center capture. Its complement is the unused gap; the seam
    // sits mid-gap so that (c - seam) mod 4096 is continuous over the travel.
    const int32_t cL = coarseLeft_;
    const int32_t cC = coarseCenterRaw_;
    const int32_t cR = coarseRight_;
    const int32_t upLR = (cR - cL) & 4095;   // ascending distance left -> right
    const int32_t upLC = (cC - cL) & 4095;   // ascending distance left -> center
    int32_t span;
    if (upLC > 0 && upLC < upLR) {
        span = upLR;                          // travel ascends from cL to cR
        seam_ = (cR + (4096 - span) / 2) & 4095;
    } else {
        span = (cL - cR) & 4095;              // travel descends from cL to cR
        seam_ = (cL + (4096 - span) / 2) & 4095;
    }
    const int32_t gap = 4096 - span;

    const int32_t lL = coarseLin(cL);
    const int32_t lC = coarseLin(cC);
    const int32_t lR = coarseLin(cR);

    // 2. Coarse strictly ordered left, center, right (or reverse): both sensors
    //    move together in one direction
    if (!((lL < lC && lC < lR) || (lL > lC && lC > lR))) {
        calError_ = "coarse not monotonic (center not between limits)";
        return;
    }

    // 3. Enough coarse resolution per steering-shaft turn
    float ratio = (float)(lR - lL) / (float)(fineRight_ - fineLeft_);
    if (fabsf(ratio) * 4096.0f < (float)STEER_COARSE_MIN_PER_TURN) {
        calError_ = "ratio too small";
        return;
    }

    // 4. Enough unused coarse arc to place the seam
    if (gap < STEER_COARSE_MIN_GAP) {
        calError_ = "no coarse gap";
        return;
    }

    // 5. Linearity: the center capture must lie on the line through the limits
    ratio_ = ratio;
    linLeft_ = lL;
    float centerExp = expectedFine(cC);
    if (fabsf(centerExp) > (float)STEER_CAL_LINEARITY_COUNTS) {
        calError_ = "center not on line";
        ratio_ = 0.0f;
        linLeft_ = 0;
        return;
    }

    // Normalize sensor direction so that positive = right
    invert_ = (fineRight_ < 0);
    relLeft_ = invert_ ? -fineLeft_ : fineLeft_;
    relRight_ = invert_ ? -fineRight_ : fineRight_;
    calibrated_ = true;
}

void SteeringController::loadCalibration() {
    Preferences prefs;
    int32_t ver = 0;
    if (prefs.begin("steering", true)) {
        ver = prefs.getInt("cal_ver", 0);
        if (ver == STEER_CAL_VERSION) {
            fineCenterRaw_ = prefs.getInt("fc_raw", -1);
            coarseCenterRaw_ = prefs.getInt("cc_raw", -1);
            fineLeft_ = prefs.getInt("fl_pos", POS_UNSET);
            coarseLeft_ = prefs.getInt("cl_raw", -1);
            fineRight_ = prefs.getInt("fr_pos", POS_UNSET);
            coarseRight_ = prefs.getInt("cr_raw", -1);
        }
        prefs.end();
    }

    // Raw values outside 0..4095 are treated as unset
    auto rawOk = [](int32_t v) { return v >= 0 && v <= 4095; };
    if (!rawOk(fineCenterRaw_) || !rawOk(coarseCenterRaw_)) {
        fineCenterRaw_ = -1;
        coarseCenterRaw_ = -1;
    }
    if (!rawOk(coarseLeft_)) { fineLeft_ = POS_UNSET; coarseLeft_ = -1; }
    if (!rawOk(coarseRight_)) { fineRight_ = POS_UNSET; coarseRight_ = -1; }

    // Only complete calibrations are used (older firmware could persist a partial
    // one): an incomplete set is treated as no calibration at all.
    if (fineCenterRaw_ < 0 || fineLeft_ == POS_UNSET || fineRight_ == POS_UNSET) {
        fineCenterRaw_ = -1; coarseCenterRaw_ = -1;
        fineLeft_ = POS_UNSET; coarseLeft_ = -1;
        fineRight_ = POS_UNSET; coarseRight_ = -1;
    }

    // A reboot discards any unfinished calibration points; the absolute turn must be recovered.
    discardCalSession("reboot", false);
    calLost_ = false;
    turnRecovered_ = false;
    resetRecovery();
    mismatch_ = false;
    mismatchLatched_ = false;
    mismatchCount_ = 0;
    revalidateCalibration();

    if (ver != STEER_CAL_VERSION) {
        Debug::printfFeature(DebugFeature::SERVO,
                             "[STEER] No dual-sensor calibration (cal_ver=%ld) — recalibration required\n",
                             (long)ver);
    }
    Debug::printfFeature(DebugFeature::SERVO,
                         "[STEER] Calibration loaded: center=%ld/%ld left=%ld/%ld right=%ld/%ld -> %s%s%s\n",
                         (long)fineCenterRaw_, (long)coarseCenterRaw_,
                         (long)(fineLeft_ == POS_UNSET ? -1 : fineLeft_), (long)coarseLeft_,
                         (long)(fineRight_ == POS_UNSET ? -1 : fineRight_), (long)coarseRight_,
                         calibrated_ ? "CALIBRATED" : "NOT CALIBRATED",
                         calError_ ? " — rejected: " : "", calError_ ? calError_ : "");
    if (calibrated_) {
        Debug::printfFeature(DebugFeature::SERVO, "[STEER] Derived: ratio=%.1f coarse/turn seam=%ld invert=%d\n",
                             (double)(ratio_ * 4096.0f), (long)seam_, invert_);
    }
}

void SteeringController::saveCalibration() {
    Preferences prefs;
    if (prefs.begin("steering", false)) {
        prefs.putInt("cal_ver", STEER_CAL_VERSION);
        prefs.putInt("fc_raw", fineCenterRaw_);
        prefs.putInt("cc_raw", coarseCenterRaw_);
        prefs.putInt("fl_pos", fineLeft_);
        prefs.putInt("cl_raw", coarseLeft_);
        prefs.putInt("fr_pos", fineRight_);
        prefs.putInt("cr_raw", coarseRight_);
        // Remove the obsolete single-sensor keys one by one. NEVER clear() this
        // namespace: the steering speed-scaling base (steer_sca_base) lives here too.
        if (prefs.isKey("c_ang")) prefs.remove("c_ang");
        if (prefs.isKey("l_ang")) prefs.remove("l_ang");
        if (prefs.isKey("r_ang")) prefs.remove("r_ang");
        prefs.end();
    }
}
