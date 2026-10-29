#include "SteeringController.h"
#include "CANController.h"
#include "Debug.h"
#include <Preferences.h>

// NVS calibration layout version ("steering"/cal_ver). Version 4 = Bosch LWS limits +
// sensor-direction flag `inv`. Version 3 (limits only, fixed negation) migrates keeping the limits.
static constexpr int32_t STEER_CAL_VERSION = 4;
static const char* const NVS_NS = "steering";
static const char* const KEY_LIM_L = "lim_l";
static const char* const KEY_LIM_R = "lim_r";
static const char* const KEY_INV = "inv";

SteeringController::SteeringController(IMotorDriver& motor)
    : motor_(motor),
      can_(nullptr),
      lws_(STEER_LWS_TIMEOUT_MS, STEER_LWS_RECOVER_FRAMES),
      sensorOk_(false),
      fault_(SensorFault::CAN_DOWN),
      firstFrameLogged_(false),
      lastStateLogMs_(0),
      limLeft_(POS_UNSET),
      limRight_(POS_UNSET),
      calibrated_(false),
      inv_(false),
      captureError_(nullptr),
      lastCapturePos_(0),
      lastCaptureBeyondSafe_(false),
      lastCaptureToggled_(false),
      zeroState_(ZeroState::IDLE),
      zeroError_(nullptr),
      zeroRefusal_(nullptr),
      zeroLimitsErased_(false),
      targetRel_(0),
      isMoving_(false),
      moveStartTime_(0),
      dutyBoost_(0),
      lastProgressPos_(0),
      lastProgressTime_(0),
      jogDir_(0),
      lastJogRefresh_(0),
      jogPulseUntil_(0),
      jogOwner_(JogOwner::NONE),
      rateArmed_(false),
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

bool SteeringController::begin() {
    // The motor driver is initialized by the caller (e.g. VESC UART begin())
    // before this point; command a coast so the actuator starts safe.
    motor_.stop();

    // No I2C probe and no homing: the angle is absolute and arrives over CAN.
    loadCalibration();

    Debug::printfFeature(DebugFeature::SERVO,
                         "[STEER] Initialized: Bosch LWS on CAN 0x%03X (pos = %sangle per sensor direction: %s, "
                         "0.1 deg, + = right), waiting for frames\n", (unsigned)STEER_LWS_CAN_ID,
                         inv_ ? "+" : "-", inv_ ? "mirrored" : "normal");
    return true;
}

// === Sensor ===

bool SteeringController::hasAngle() const {
    return lws_.isFresh(millis()) && lws_.angleValid();
}

bool SteeringController::hasRate() const {
    return lws_.isFresh(millis()) && lws_.speedValid();
}

bool SteeringController::isRangeWarning() const {
    return (limLeft_ != POS_UNSET && abs(limLeft_) > STEER_LWS_SAFE_RANGE) ||
           (limRight_ != POS_UNSET && abs(limRight_) > STEER_LWS_SAFE_RANGE);
}

void SteeringController::logState(const char* suffix) {
    char ang[16];
    if (lws_.angleValid()) {
        snprintf(ang, sizeof(ang), "%+.1f", (double)lws_.angleRaw() / 10.0);
    } else {
        snprintf(ang, sizeof(ang), "--");
    }
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] angle=%s pos=%ld tok=%s fps=%u%s\n",
                         ang, (long)(lws_.angleValid() ? getPosition() : 0L), faultToken(fault_),
                         (unsigned)lws_.fps(), suffix ? suffix : "");
}

void SteeringController::refreshSensor(uint32_t now) {
    const bool zeroing = (zeroState_ == ZeroState::RUNNING);
    const bool canUp = (can_ != nullptr) && can_->isInitialized() && can_->hasLwsSink();
    const SensorFault prevFault = fault_;
    const bool prevOk = sensorOk_;

    fault_ = lws_.evaluate(now, zeroing, canUp);
    sensorOk_ = lws_.isHealthy();

    if (!firstFrameLogged_ && lws_.hasFrame()) {
        firstFrameLogged_ = true;
        // Raw sensor sign, shown once: the sensor's + is LEFT on a normal mount (RIGHT when
        // mirrored, inv=1); the firmware's + is always RIGHT.
        Debug::printfFeature(DebugFeature::SERVO,
                             "[STEER] LWS first frame: sensor angle=%+.1f deg (raw sensor sign) -> "
                             "pos=%ld (0.1 deg, firmware + = RIGHT), inv=%d OK=%d cal=%d adj=%d fault=%u\n",
                             lws_.angleValid() ? (double)lws_.angleRaw() / 10.0 : 0.0,
                             (long)(lws_.angleValid() ? getPosition() : 0L), (int)inv_,
                             (int)lws_.statusOk(), (int)lws_.statusCal(), (int)lws_.statusAdj(),
                             (unsigned)lws_.faultCode());
    }

    if (!prevOk && sensorOk_ && jogDir_ != 0 && jogOwner_ == JogOwner::RATE) {
        // Defence in depth: a rate drive never runs with a healthy sensor. The vehicle
        // layer also calls stopRate() on RATE -> POSITION; the wheel holds where it is.
        stop();
    }

    if (prevOk && !sensorOk_) {
        // Faults take effect at once: a closed-loop move is stopped, but an active
        // jog or nudge continues open loop under its own refresh / pulse bounds.
        if (isMoving_) {
            motor_.stop();
            isMoving_ = false;
            dutyBoost_ = 0;
        }
        logState(fault_ == SensorFault::RANGE_LATCHED
                     ? " — SENSOR FAULT: out-of-range latch, restart the vehicle (closed-loop stopped, jog/nudge available)"
                     : " — SENSOR FAULT (closed-loop stopped, jog/nudge available sensorless)");
    } else if (!prevOk && sensorOk_) {
        // Fresh stall baseline: a jog started sensorless has no valid one.
        lastStallPosition_ = getPosition();
        lastStallCheckTime_ = now;
        lastProgressPos_ = getPosition();
        lastProgressTime_ = now;
        logState(" — sensor OK");
    } else if (fault_ != prevFault) {
        logState(nullptr);
    }
}

// === Movement ===

void SteeringController::stop() {
    motor_.stop();
    isMoving_ = false;
    jogDir_ = 0;
    jogPulseUntil_ = 0;
    jogOwner_ = JogOwner::NONE;
}

void SteeringController::stopRate() {
    if (jogDir_ != 0 && jogOwner_ == JogOwner::RATE) {
        stop();
    }
}

bool SteeringController::webJogActive(uint32_t now) const {
    if (jogDir_ == 0 || jogOwner_ != JogOwner::WEB) return false;
    if (jogPulseUntil_ != 0) return (int32_t)(now - jogPulseUntil_) < 0;   // nudge pulse running
    return (now - lastJogRefresh_) < STEER_JOG_TIMEOUT_MS;                  // hold-jog refreshed
}

bool SteeringController::driveJog(int8_t d, uint8_t duty, JogOwner owner, uint32_t pulseMs) {
    const uint32_t now = millis();
    if (latchBlocks(d, now)) return false;   // stalled in this direction, still in cooldown
    stallLatched_ = false;                   // accepted move clears the latch

    if (duty < STEER_PWM_MIN_DUTY) duty = STEER_PWM_MIN_DUTY;   // below this the actuator stalls

    isMoving_ = false;   // jog overrides position control
    if (jogDir_ != d) {
        lastStallPosition_ = getPosition();
        lastStallCheckTime_ = now;
    }
    jogDir_ = d;
    jogOwner_ = owner;
    lastDriveDir_ = d;
    lastJogRefresh_ = now;
    jogPulseUntil_ = pulseMs ? now + pulseMs : 0;
    motor_.setSpeed(jogDir_ * (int16_t)duty);
    return true;
}

void SteeringController::jog(int8_t direction, uint8_t duty) {
    // Sensorless allowed: only the VESC link (and below, the stall latch) gates a jog.
    if (direction != 0 && !steerDriverOk_) return;
    if (direction != 0) bootCenterPending_ = false;   // operator command supersedes boot centering

    if (direction == 0) {
        if (jogDir_ != 0) stop();   // stops whatever jog is active, a rate drive included
        return;
    }

    driveJog((direction > 0) ? 1 : -1, duty, JogOwner::WEB, 0);
}

void SteeringController::nudge(int8_t direction) {
    // Sensorless allowed: only the VESC link (and the stall latch) gates a nudge.
    if (direction == 0 || !steerDriverOk_) return;
    bootCenterPending_ = false;   // operator command supersedes boot centering

    driveJog((direction > 0) ? 1 : -1, STEER_NUDGE_DUTY, JogOwner::WEB, STEER_NUDGE_MS);
}

bool SteeringController::setRateCommand(float pct) {
    // Rate driving is impossible while a position is available (defence in depth: the
    // vehicle layer only calls this in RATE mode, which requires an unhealthy sensor).
    if (sensorOk_ || !steerDriverOk_) return false;

    const int16_t duty = steerRateDuty(pct, RC_STEERING_DEADBAND, STEER_PWM_MIN_DUTY, STEER_PWM_MAX_DUTY);
    if (duty == 0) {
        // Centred stick: arms the path and stops a rate drive — never a web jog/nudge.
        rateArmed_ = true;
        stopRate();
        return true;
    }
    if (!rateArmed_) return false;   // centre-to-arm pending

    // A web jog/nudge has priority: rate commands neither stop nor override it.
    if (webJogActive(millis())) return true;

    const int8_t d = (duty > 0) ? 1 : -1;
    if (!driveJog(d, (uint8_t)(duty > 0 ? duty : -duty), JogOwner::RATE, 0)) return false;
    bootCenterPending_ = false;   // an accepted drive supersedes boot centering
    return true;
}

void SteeringController::update() {
    uint32_t now = millis();

    // Sensor health first (frames were dispatched by CANController::update() earlier in
    // this loop iteration), then the zero-set, then the motor driver monitors.
    refreshSensor(now);
    serviceZero(now);
    bool driverOk = serviceDriver(now);

    serviceBootCenter(now, driverOk);

    if (!driverOk) return;

    now = millis();

    // Stall detection — any active drive (position move or jog), ONLY with a healthy
    // sensor. Sensorless jogs rely on the refresh timeout, the nudge pulse length and
    // the VESC current clamp / fault stop.
    // Rate mode: stall detection is bypassed BY DESIGN. A RATE-owned drive exists only
    // while the sensor is unhealthy (setRateCommand() refuses otherwise, and
    // refreshSensor() stops it on recovery), so this check, the anti-stall boost, the
    // move timeout and the range guard below (closed loop only, isMoving_) never run
    // for it. End stops are left to the VESC current limit; the VESC fault / comm stops
    // in serviceDriver(), the stall latch and the jog refresh timeout still apply.
    if (sensorOk_ && (isMoving_ || (jogDir_ != 0 && jogOwner_ != JogOwner::RATE))) {
        int32_t currentPos = getPosition();
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

    // Jog mode (open loop, momentary; never range-guarded)
    if (jogDir_ != 0) {
        if (now - lastStateLogMs_ >= 250) {
            lastStateLogMs_ = now;
            logState(jogOwner_ == JogOwner::RATE ? " rate"
                     : jogPulseUntil_ != 0 ? " nudge"
                     : (sensorOk_ ? " jog" : " jog (sensorless)"));
        }
        if (jogPulseUntil_ != 0 && (int32_t)(now - jogPulseUntil_) >= 0) {
            stop();   // nudge pulse complete
            return;
        }
        if (now - lastJogRefresh_ >= STEER_JOG_TIMEOUT_MS) {
            Debug::printlnFeature(DebugFeature::SERVO, jogOwner_ == JogOwner::RATE
                                  ? "[STEER] Rate timeout (no fresh frame) — stopping"
                                  : "[STEER] Jog timeout — stopping");
            stop();
        }
        return;
    }

    if (!isMoving_) return;
    if (!sensorOk_) {   // refreshSensor() already stopped it; never drive closed loop blind
        stop();
        return;
    }

    // Closed-loop proportional position control (0.1°, positive = right)
    int32_t rel = getPosition();
    int32_t error = targetRel_ - rel;

    if (abs(error) <= STEER_POSITION_TOLERANCE) {
        stop();
        return;
    }

    // Range guard: never drive further outward at or beyond the safe range (e.g. after
    // an overshoot) — keeps automatic steering out of the sensor's ~600° latch.
    // Driving back toward zero is always allowed.
    if (abs(rel) >= STEER_LWS_SAFE_RANGE && ((rel > 0) == (error > 0))) {
        logState(" — range guard: outward move stopped at the safe range");
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

    // Motor positive = wheel right = position increasing. THE sign-critical spot: this relies
    // on the sensor sign (direction flag inv_) matching the motor sign. It is gated by
    // calibrated_, i.e. both limits captured under the same flag (SteerSensorDir.h).
    int16_t speed = (int16_t)duty;
    lastDriveDir_ = (error > 0) ? 1 : -1;
    motor_.setSpeed(error > 0 ? speed : (int16_t)-speed);

    if (now - lastStateLogMs_ >= 100) {
        lastStateLogMs_ = now;
        char extra[64];
        snprintf(extra, sizeof(extra), " target=%ld err=%ld duty=%d boost=%d",
                 (long)targetRel_, (long)error, (int)(error > 0 ? speed : -speed), (int)dutyBoost_);
        logState(extra);
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
    (void)now;
    motor_.poll();   // non-blocking: pump VESC UART + periodic GET_VALUES (no-op for BTS7960)

    // Comm failsafe: no valid telemetry link -> stop, reject moves (incl. jog/nudge),
    // flag driver down. Auto-recovers when replies resume.
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

    // Nonzero VESC fault code -> immediate stop + latch (current drive direction),
    // also for a sensorless jog. This is the over-current backstop: the VESC raises a
    // fault on Absolute Max Current (and on over-temperature), and clamps at Motor
    // Current Max below that.
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

    // Asymmetric mapping: -100 -> limLeft_ (<0), 0 -> sensor zero, +100 -> limRight_ (>0)
    int32_t target;
    if (percent >= 0.0f) {
        target = (int32_t)((percent / 100.0f) * (float)limRight_);
    } else {
        target = (int32_t)((-percent / 100.0f) * (float)limLeft_);
    }

    // Clamp to the calibrated limits, then to the safe range (a limit stored beyond
    // 550° maps ±100 % to ±550° instead).
    if (target < limLeft_) target = limLeft_;
    if (target > limRight_) target = limRight_;
    if (target < -STEER_LWS_SAFE_RANGE) target = -STEER_LWS_SAFE_RANGE;
    if (target > STEER_LWS_SAFE_RANGE) target = STEER_LWS_SAFE_RANGE;

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
    int32_t rel = getPosition();
    int8_t d = (target > rel) ? 1 : (target < rel ? -1 : 0);
    if (latchBlocks(d, now)) {
        return false;
    }
    stallLatched_ = false;   // accepted move clears the latch

    targetRel_ = target;
    jogDir_ = 0;
    jogPulseUntil_ = 0;
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

void SteeringController::serviceBootCenter(uint32_t now, bool driverReady) {
    if (!bootCenterPending_) return;
    // The LWS, the ECU and the VESC come up on the ignition key after the ESP32 may
    // have booted: wait for them, but never make the vehicle steer by itself much
    // later — give up silently once the window has passed.
    if (now - bootCenterRequestMs_ >= STEER_BOOT_CENTER_TIMEOUT_MS) {
        bootCenterPending_ = false;
        return;
    }
    if (!driverReady || !calibrated_ || !sensorOk_ || !steerDriverOk_) return;

    bootCenterPending_ = false;
    if (setSteeringPercent(0.0f)) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Moving to center");
    }
}

float SteeringController::getSteeringPercent() const {
    if (!calibrated_ || !hasAngle()) return 0.0f;

    int32_t rel = getPosition();
    float percent;
    if (rel >= 0) {
        percent = (limRight_ != 0) ? ((float)rel / (float)limRight_) * 100.0f : 0.0f;
    } else {
        percent = (limLeft_ != 0) ? -((float)rel / (float)limLeft_) * 100.0f : 0.0f;
    }
    if (percent < -100.0f) percent = -100.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

// === Calibration ===

bool SteeringController::captureLeftLimit() {
    return captureLimit(true);
}

bool SteeringController::captureRightLimit() {
    return captureLimit(false);
}

bool SteeringController::captureLimit(bool left) {
    const char* name = left ? "Left limit" : "Right limit";
    captureError_ = nullptr;
    lastCaptureBeyondSafe_ = false;
    lastCaptureToggled_ = false;

    SteerDirResult r = {SteerDirAction::REFUSE_TOO_CLOSE, inv_, 0, false};
    if (!sensorOk_) {
        captureError_ = "sensor fault";
    } else {
        const bool oppositeSet = (left ? limRight_ : limLeft_) != POS_UNSET;
        r = steerDirCapture(left, getPosition(), inv_, oppositeSet, STEER_CAL_MIN_SPAN);
        if (r.action == SteerDirAction::REFUSE_TOO_CLOSE) {
            captureError_ = "too close to zero";
        } else if (r.action == SteerDirAction::REFUSE_CONTRADICTS) {
            captureError_ = left
                ? "reads right of zero, contradicting the stored right limit — sensor zero off-centre "
                  "or sensor re-mounted: jog straight ahead, Set sensor zero, capture both limits"
                : "reads left of zero, contradicting the stored left limit — sensor zero off-centre "
                  "or sensor re-mounted: jog straight ahead, Set sensor zero, capture both limits";
        }
    }
    if (captureError_) {
        Debug::printfFeature(DebugFeature::SERVO, "[STEER] %s capture refused: %s\n", name, captureError_);
        return false;   // stored limits and the direction flag unchanged
    }

    // The safe range is deliberately NOT checked: calibration must reach the real locks.
    stop();
    bootCenterPending_ = false;

    const char* oppositeKey = nullptr;
    if (r.action == SteerDirAction::TOGGLE_AND_STORE) {
        // The capture is on the wrong side and the opposite limit is unset: the sensor is
        // mounted the other way. Apply the flag; the position of the same raw angle is −pos.
        inv_ = r.inv;
        lws_.setInverted(inv_);
        if (r.clearOpposite) {   // defensive: with the strict rule the opposite is already unset
            if (left) limRight_ = POS_UNSET; else limLeft_ = POS_UNSET;
            oppositeKey = left ? KEY_LIM_R : KEY_LIM_L;
        }
        // Baselines holding old-sign values would fake progress / a stall.
        const uint32_t now = millis();
        lastStallPosition_ = getPosition();
        lastStallCheckTime_ = now;
        lastProgressPos_ = getPosition();
        lastProgressTime_ = now;
        lastCaptureToggled_ = true;
        Debug::printfFeature(DebugFeature::SERVO,
                             "[STEER] sensor direction: %s (inferred from the %s limit capture) — "
                             "capture the %s limit again\n",
                             inv_ ? "mirrored" : "normal", left ? "left" : "right", left ? "right" : "left");
    }

    const int32_t pos = r.pos;
    if (left) {
        limLeft_ = pos;
        saveLimit(KEY_LIM_L, pos, lastCaptureToggled_, oppositeKey);
    } else {
        limRight_ = pos;
        saveLimit(KEY_LIM_R, pos, lastCaptureToggled_, oppositeKey);
    }
    revalidateCalibration();
    lastCapturePos_ = pos;
    lastCaptureBeyondSafe_ = abs(pos) > STEER_LWS_SAFE_RANGE;

    Debug::printfFeature(DebugFeature::SERVO, "[STEER] %s saved: %+.1f deg%s%s\n", name,
                         (double)pos / 10.0,
                         lastCaptureBeyondSafe_ ? " — WARNING beyond the safe range, near the ~600 deg sensor latch" : "",
                         calibrated_ ? " — CALIBRATED" : "");
    return true;
}

bool SteeringController::requestSensorZero() {
    zeroRefusal_ = nullptr;
    if (zeroState_ == ZeroState::RUNNING) {
        zeroRefusal_ = "busy";
        return false;
    }
    if (can_ == nullptr || !can_->isInitialized()) {
        zeroRefusal_ = "can_unavailable";
        finishZero(false, zeroRefusal_);
        return false;
    }

    // No ignition / RPM / standstill precondition. Motion stops and boot centering is
    // cancelled before the sequence is asked for.
    stop();
    bootCenterPending_ = false;

    switch (can_->startLwsZero()) {
        case CANController::LwsZeroStart::STARTED:
            break;
        case CANController::LwsZeroStart::NOT_INITIALIZED:
            zeroRefusal_ = "can_unavailable";
            finishZero(false, zeroRefusal_);
            return false;
        case CANController::LwsZeroStart::BUSY_PROBE:
        case CANController::LwsZeroStart::BUSY:
        default:
            zeroRefusal_ = "busy";
            finishZero(false, zeroRefusal_);
            return false;
    }

    zeroState_ = ZeroState::RUNNING;
    zeroError_ = nullptr;
    zeroLimitsErased_ = false;
    Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Sensor zero-set started");
    return true;
}

void SteeringController::finishZero(bool ok, const char* reason) {
    zeroState_ = ok ? ZeroState::OK : ZeroState::FAILED;
    zeroError_ = ok ? nullptr : reason;
    if (ok) {
        Debug::printlnFeature(DebugFeature::SERVO,
                              "[STEER] Sensor zero confirmed (angle 0.0) — capture Left and Right again");
    } else {
        Debug::printfFeature(DebugFeature::SERVO, "[STEER] Sensor zero-set failed: %s\n", reason ? reason : "?");
    }
}

void SteeringController::serviceZero(uint32_t now) {
    if (zeroState_ != ZeroState::RUNNING) return;
    if (can_ == nullptr) {
        finishZero(false, "can_unavailable");
        return;
    }

    // From the reset frame on, the old zero is no longer guaranteed: limits measured
    // against it are erased whatever the outcome.
    if (!zeroLimitsErased_ && can_->lwsZeroResetSent()) {
        zeroLimitsErased_ = true;
        eraseLimits();
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Reset frame sent — Left/Right limits erased");
    }

    if (can_->isLwsZeroRunning()) return;

    if (can_->lwsZeroAbortReason() != nullptr) {
        finishZero(false, can_->lwsZeroAbortReason());
        return;
    }
    if (!can_->lwsZeroZeroSent()) {
        finishZero(false, "tx_failed");   // defensive: sequence ended without the zero frame
        return;
    }

    // Confirmation: an accepted frame after [53 30] with OK=1, fault=0, cal=1, |angle| ≤ tol.
    const uint32_t sentMs = can_->lwsZeroSentMs();
    const bool frameSince = lws_.hasFrame() && (int32_t)(lws_.lastFrameMs() - sentMs) > 0;
    if (frameSince) {
        if (lws_.faultCode() == LWS_FAULT_RANGE_LATCHED) {
            finishZero(false, "range_latched");   // the latched sensor ignores 0x7C0: restart the vehicle
            return;
        }
        if (lws_.statusOk() && lws_.faultCode() == 0 && lws_.statusCal() && lws_.angleValid() &&
            abs((int32_t)lws_.angleRaw()) <= STEER_LWS_ZERO_TOLERANCE) {
            finishZero(true, nullptr);
            return;
        }
    }
    if (now - sentMs >= STEER_LWS_ZERO_CONFIRM_MS) {
        finishZero(false, frameSince ? "not_confirmed" : "no_frames");
    }
}

const char* SteeringController::zeroStateToken(ZeroState s) {
    switch (s) {
        case ZeroState::IDLE:    return "idle";
        case ZeroState::RUNNING: return "running";
        case ZeroState::OK:      return "ok";
        case ZeroState::FAILED:  return "failed";
    }
    return "idle";
}

void SteeringController::revalidateCalibration() {
    calibrated_ = limLeft_ != POS_UNSET && limRight_ != POS_UNSET &&
                  limLeft_ <= -STEER_CAL_MIN_SPAN && limRight_ >= STEER_CAL_MIN_SPAN;
}

void SteeringController::loadCalibration() {
    limLeft_ = POS_UNSET;
    limRight_ = POS_UNSET;
    inv_ = false;

    Preferences prefs;
    int32_t ver = 0;
    if (prefs.begin(NVS_NS, false)) {
        ver = prefs.getInt("cal_ver", 0);
        if (ver == STEER_CAL_VERSION) {
            limLeft_ = prefs.getInt(KEY_LIM_L, POS_UNSET);
            limRight_ = prefs.getInt(KEY_LIM_R, POS_UNSET);
            inv_ = prefs.getBool(KEY_INV, false);
        } else if (ver == 3) {
            // v3 limits were captured with the fixed negation (= normal mount): keep them.
            limLeft_ = prefs.getInt(KEY_LIM_L, POS_UNSET);
            limRight_ = prefs.getInt(KEY_LIM_R, POS_UNSET);
            prefs.putBool(KEY_INV, false);
            prefs.putInt("cal_ver", STEER_CAL_VERSION);
            Debug::printfFeature(DebugFeature::SERVO,
                                 "[STEER] NVS calibration v3 -> v4: limits kept, sensor direction normal\n");
        } else {
            // Migration: remove every obsolete key ONE BY ONE. NEVER clear() this
            // namespace: the steering speed-scaling base (steer_sca_base) lives here too.
            static const char* const kObsolete[] = {
                "fc_raw", "cc_raw", "fl_pos", "cl_raw", "fr_pos", "cr_raw",   // v2 (dual AS5600)
                "c_ang", "l_ang", "r_ang",                                      // v1 (single sensor)
                KEY_LIM_L, KEY_LIM_R, KEY_INV                                   // no limits after migration
            };
            unsigned removed = 0;
            for (const char* key : kObsolete) {
                if (prefs.isKey(key) && prefs.remove(key)) removed++;
            }
            prefs.putInt("cal_ver", STEER_CAL_VERSION);
            prefs.putBool(KEY_INV, false);
            Debug::printfFeature(DebugFeature::SERVO,
                                 "[STEER] NVS calibration v%ld -> v%ld: %u old key(s) removed — "
                                 "recalibration required (sensor zero + Left/Right)\n",
                                 (long)ver, (long)STEER_CAL_VERSION, removed);
        }
        prefs.end();
    }
    lws_.setInverted(inv_);

    // A stored limit that no longer passes the capture checks is treated as unset.
    if (limLeft_ != POS_UNSET && limLeft_ > -STEER_CAL_MIN_SPAN) limLeft_ = POS_UNSET;
    if (limRight_ != POS_UNSET && limRight_ < STEER_CAL_MIN_SPAN) limRight_ = POS_UNSET;
    revalidateCalibration();

    char l[16], r[16];
    if (limLeft_ != POS_UNSET) snprintf(l, sizeof(l), "%+.1f", (double)limLeft_ / 10.0);
    else snprintf(l, sizeof(l), "--");
    if (limRight_ != POS_UNSET) snprintf(r, sizeof(r), "%+.1f", (double)limRight_ / 10.0);
    else snprintf(r, sizeof(r), "--");
    Debug::printfFeature(DebugFeature::SERVO,
                         "[STEER] Calibration loaded: left=%s right=%s deg -> %s, sensor direction: %s%s\n", l, r,
                         calibrated_ ? "CALIBRATED" : "NOT CALIBRATED", inv_ ? "mirrored" : "normal",
                         isRangeWarning() ? " (a limit is beyond the 550 deg safe range)" : "");
}

void SteeringController::saveLimit(const char* key, int32_t value, bool writeInv, const char* oppositeKey) {
    Preferences prefs;
    if (prefs.begin(NVS_NS, false)) {
        // Order on a direction toggle: the limit, then `inv`, then remove the opposite.
        // A power loss in between leaves at most one limit valid under the stored flag,
        // so the steering is not calibrated and the closed loop cannot run.
        prefs.putInt("cal_ver", STEER_CAL_VERSION);
        prefs.putInt(key, value);
        if (writeInv) prefs.putBool(KEY_INV, inv_);
        if (oppositeKey != nullptr && prefs.isKey(oppositeKey)) prefs.remove(oppositeKey);
        prefs.end();
    }
}

void SteeringController::eraseLimits() {
    Preferences prefs;
    if (prefs.begin(NVS_NS, false)) {
        // Key by key — never clear() the namespace (steer_sca_base lives here).
        // The sensor-direction flag `inv` is kept; the next capture re-confirms or toggles it.
        if (prefs.isKey(KEY_LIM_L)) prefs.remove(KEY_LIM_L);
        if (prefs.isKey(KEY_LIM_R)) prefs.remove(KEY_LIM_R);
        prefs.putInt("cal_ver", STEER_CAL_VERSION);
        prefs.end();
    }
    limLeft_ = POS_UNSET;
    limRight_ = POS_UNSET;
    revalidateCalibration();
}
