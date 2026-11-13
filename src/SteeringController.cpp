#include "SteeringController.h"
#include "CANController.h"
#include "Debug.h"
#include "EventJournal.h"
#include <Preferences.h>

// NVS calibration layout version ("steering"/cal_ver). Version 4 = Bosch LWS limits +
// sensor-direction flag `inv`. Version 3 (limits only, fixed negation) migrates keeping the limits.
static constexpr int32_t STEER_CAL_VERSION = 4;
static const char* const NVS_NS = "steering";
static const char* const KEY_LIM_L = "lim_l";
static const char* const KEY_LIM_R = "lim_r";
static const char* const KEY_INV = "inv";

// Closed-loop position law tunables (include/SteerPositionLaw.h, design D7)
static const SteerPosParams POS_PARAMS = {
    STEER_POSITION_TOLERANCE,
    STEER_RESTART_TOLERANCE,
    STEER_KP,
    STEER_PWM_MIN_DUTY,
    STEER_PWM_TRACK_MIN_DUTY,
    STEER_PWM_MAX_DUTY,
    STEER_STALL_THRESHOLD,
    STEER_STALL_TIMEOUT,
    STEER_MOVE_TIMEOUT,
    STEER_BOOST_CHECK_MS,
    STEER_BOOST_MIN_PROGRESS,
    STEER_BOOST_STEP,
    STEER_BOOST_DECAY,
    STEER_BOOST_HOLD_ERR,
};

// Park correction defaults (include/SteerParkLaw.h); doneTol / maxPulses / rungs are web settings
static const SteerParkParams PARK_DEFAULTS = {
    STEER_PARK_TRIGGER_TOL,   // 0.4°, independent of the 0.2° park window (fix-steering-jerk-and-centre-undershoot D5)
    STEER_PARK_DONE_TOL_DEFAULT,
    STEER_PARK_MOVE_MIN,
    STEER_PARK_SETTLE_MS,
    STEER_PARK_PULSE_GAP_MS,
    STEER_PARK_MAX_PULSES_DEFAULT,
    {{STEER_PARK_DUTY1_DEFAULT, STEER_PARK_MS1_DEFAULT},
     {STEER_PARK_DUTY2_DEFAULT, STEER_PARK_MS2_DEFAULT},
     {STEER_PARK_DUTY3_DEFAULT, STEER_PARK_MS3_DEFAULT}},
};
static const char* const PARK_DUTY_KEYS[STEER_PARK_RUNGS] = {
    STEER_PARK_D1_NVS_KEY, STEER_PARK_D2_NVS_KEY, STEER_PARK_D3_NVS_KEY};
static const char* const PARK_MS_KEYS[STEER_PARK_RUNGS] = {
    STEER_PARK_T1_NVS_KEY, STEER_PARK_T2_NVS_KEY, STEER_PARK_T3_NVS_KEY};

static inline double parkDeg(int32_t v) { return (double)v / 10.0; }

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
      posState_(SteerPosState::IDLE),
      posTimers_(),
      parkParams_(PARK_DEFAULTS),
      parkEnabled_(STEER_PARK_ENABLED_DEFAULT),
      parkState_(),
      parkRefTarget_(0),
      parkSettleStartMs_(0),
      parkPulseActive_(false),
      parkPulseEndMs_(0),
      parkBlocked_(false),
      parkLast_(),
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
      linkDownSinceMs_(0),
      linkDownSnapMs_(0),
      lastFaultCode_(0),
      diagLock_(false),
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

    if (fault_ != prevFault) {
        // Event journal: every LWS health change, with the fault token and the frame rate.
        EventJournal::log(JSrc::STEER, fault_ == SensorFault::NONE ? JLvl::I : JLvl::W, "lws",
                          "fault=%s prev=%s fps=%u", faultToken(fault_), faultToken(prevFault),
                          (unsigned)lws_.fps());
    }

    if (prevOk && !sensorOk_) {
        // Faults take effect at once: a closed-loop move is stopped, but an active
        // jog or nudge continues open loop under its own refresh / pulse bounds.
        loopToIdle();
        logState(fault_ == SensorFault::RANGE_LATCHED
                     ? " — SENSOR FAULT: out-of-range latch, restart the vehicle (closed-loop stopped, jog/nudge available)"
                     : " — SENSOR FAULT (closed-loop stopped, jog/nudge available sensorless)");
    } else if (!prevOk && sensorOk_) {
        // Fresh jog stall baseline: a jog started sensorless has no valid one.
        lastStallPosition_ = getPosition();
        lastStallCheckTime_ = now;
        logState(" — sensor OK");
    } else if (fault_ != prevFault) {
        logState(nullptr);
    }
}

// === Movement ===

void SteeringController::loopToIdle() {
    parkCancel();   // leaving PARKED (or a jog / nudge / rate drive taking over)
    if (posState_ == SteerPosState::IDLE) return;
    motor_.stop();
    posState_ = SteerPosState::IDLE;
}

void SteeringController::stop() {
    parkCancel();
    motor_.stop();
    posState_ = SteerPosState::IDLE;
    jogDir_ = 0;
    jogPulseUntil_ = 0;
    jogOwner_ = JogOwner::NONE;
}

void SteeringController::stopRate() {
    if (jogDir_ != 0 && jogOwner_ == JogOwner::RATE) {
        stop();
    }
}

// === USB steering diagnostics ===

void SteeringController::setDiagLock(bool on) {
    stop();                       // coast, closed loop idle, jog cancelled
    bootCenterPending_ = false;   // never centre by itself after a diagnostics session
    diagLock_ = on;
    EventJournal::log(JSrc::STEER, JLvl::I, "diag_lock", "on=%d", on ? 1 : 0);
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] USB diagnostics %s\n",
                         on ? "own the steering (external commands refused)" : "released the steering");
}

bool SteeringController::diagDrive(int16_t speed) {
    if (!diagLock_ || !steerDriverOk_) return false;
    if (speed > STEER_PWM_MAX_DUTY) speed = STEER_PWM_MAX_DUTY;
    if (speed < -STEER_PWM_MAX_DUTY) speed = -STEER_PWM_MAX_DUTY;
    parkCancel();
    posState_ = SteerPosState::IDLE;   // update() must not drive the loop or a jog meanwhile
    jogDir_ = 0;
    jogPulseUntil_ = 0;
    jogOwner_ = JogOwner::NONE;
    if (speed != 0) lastDriveDir_ = speed > 0 ? 1 : -1;
    motor_.setSpeed(speed);
    return true;
}

const char* SteeringController::posStateToken(SteerPosState s) {
    switch (s) {
        case SteerPosState::IDLE:    return "idle";
        case SteerPosState::DRIVING: return "drive";
        case SteerPosState::PARKED:  return "park";
    }
    return "idle";
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

    loopToIdle();   // jog overrides position control (coast, then the jog duty below)
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
    if (diagLock_) return;   // USB diagnostics own the steering
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
    if (diagLock_) return;   // USB diagnostics own the steering
    // Sensorless allowed: only the VESC link (and the stall latch) gates a nudge.
    if (direction == 0 || !steerDriverOk_) return;
    bootCenterPending_ = false;   // operator command supersedes boot centering

    driveJog((direction > 0) ? 1 : -1, STEER_NUDGE_DUTY, JogOwner::WEB, STEER_NUDGE_MS);
}

bool SteeringController::setRateCommand(float pct) {
    // Rate driving is impossible while a position is available (defence in depth: the
    // vehicle layer only calls this in RATE mode, which requires an unhealthy sensor).
    if (diagLock_ || sensorOk_ || !steerDriverOk_) return false;

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

    // Jog stall detection, ONLY with a healthy sensor (the closed loop has its own in
    // steerPosStep()). Sensorless jogs rely on the refresh timeout, the nudge pulse length
    // and the VESC current clamp / fault stop.
    // Rate mode: stall detection is bypassed BY DESIGN. A RATE-owned drive exists only
    // while the sensor is unhealthy (setRateCommand() refuses otherwise, and
    // refreshSensor() stops it on recovery), so this check and the closed-loop stall,
    // boost, move timeout and range guard never run for it. End stops are left to the
    // VESC current limit; the VESC fault / comm stops in serviceDriver(), the stall latch
    // and the jog refresh timeout still apply.
    if (sensorOk_ && jogDir_ != 0 && jogOwner_ != JogOwner::RATE) {
        int32_t currentPos = getPosition();
        if (now - lastStallCheckTime_ >= STEER_STALL_TIMEOUT) {
            if (abs(currentPos - lastStallPosition_) < STEER_STALL_THRESHOLD) {
                triggerStallStop(jogDir_, "position");
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

    if (!loopActive()) return;
    if (!sensorOk_ || !calibrated_) {   // never drive closed loop blind or with erased limits
        stop();
        return;
    }

    // Closed-loop position control (0.1°, positive = right): IDLE / DRIVING / PARKED,
    // continuous tracking, stop hysteresis, break-away / tracking floors (SteerPositionLaw.h).
    const int32_t rel = getPosition();
    const int32_t error = targetRel_ - rel;

    // Range guard: never drive further outward at or beyond the safe range (e.g. after
    // an overshoot) — keeps automatic steering out of the sensor's ~600° latch.
    // Driving back toward zero is always allowed.
    if (posState_ == SteerPosState::DRIVING && abs(error) > STEER_POSITION_TOLERANCE &&
        abs(rel) >= STEER_LWS_SAFE_RANGE && ((rel > 0) == (error > 0))) {
        logState(" — range guard: outward move stopped at the safe range");
        stop();
        return;
    }

    int16_t duty = 0;
    const SteerPosEvent ev = steerPosStep(posState_, posTimers_, POS_PARAMS, targetRel_, rel, now, &duty);
    switch (ev) {
        case SteerPosEvent::PARKED:
            motor_.stop();   // coast once; the self-locking worm holds the shaft
            parkCancel();
            parkRestartSettle(now);   // new park entry: settle clock, ladder and budget re-armed
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] parked err=%ld\n", (long)error);
            return;
        case SteerPosEvent::STALL:
            // Direction that was being driven into the jam: the sign of the remaining error.
            triggerStallStop(error > 0 ? 1 : -1, "position");
            return;
        case SteerPosEvent::TIMEOUT:
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Movement timeout — stopping");
            stop();
            return;
        case SteerPosEvent::RESTARTED:
            // The main loop keeps priority over the park correction: cancel it, then drive from
            // rest. A restart after a park pulse (its overshoot beyond STEER_RESTART_TOLERANCE)
            // disables the correction until the next stick command — re-arming here would loop
            // pulse → restart → re-park → pulse and shake the wheels.
            if (parkState_.pulses > 0) {
                parkLast_.outcome = ParkOutcome::GAVE_UP;
                parkLast_.note = SteerParkNote::OVERSHOOT_RESTART;
                parkLast_.pulses = parkState_.pulses;
                parkLast_.residual = error;
                parkLast_.duty = parkState_.lastDuty;
                parkLast_.ms = parkState_.lastMs;
                parkBlocked_ = true;
                Debug::printlnFeature(DebugFeature::SERVO,
                                      "[STEER] park correction disabled until the next stick command: "
                                      "pulse overshot the restart band");
                parkCancel();
            } else {
                parkCancel(true, error);
            }
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] parked -> drive err=%ld\n", (long)error);
            break;
        case SteerPosEvent::NONE:
            break;
    }
    if (posState_ != SteerPosState::DRIVING) {
        // PARKED: no motor commands, except the bounded park correction
        if (posState_ == SteerPosState::PARKED) serviceParkCorrection(now, rel, error);
        return;
    }

    // Motor positive = wheel right = position increasing. THE sign-critical spot: this relies
    // on the sensor sign (direction flag inv_) matching the motor sign. It is gated by
    // calibrated_, i.e. both limits captured under the same flag (SteerSensorDir.h).
    lastDriveDir_ = (duty > 0) ? 1 : -1;
    motor_.setSpeed(duty);

    if (now - lastStateLogMs_ >= 100) {
        lastStateLogMs_ = now;
        char extra[80];
        snprintf(extra, sizeof(extra), " target=%ld err=%ld duty=%d boost=%d floor=%u",
                 (long)targetRel_, (long)error, (int)duty, (int)posTimers_.boost,
                 (unsigned)steerPosFloor(posTimers_, POS_PARAMS));
        logState(extra);
    }
}

// === Stall latch + VESC driver monitors ===

bool SteeringController::latchBlocks(int8_t d, uint32_t now) const {
    return stallLatched_ && d != 0 && d == stallLatchDir_ &&
           (now - stallLatchTime_) < STEER_STALL_COOLDOWN_MS;
}

void SteeringController::triggerStallStop(int8_t dir, const char* reason) {
    parkCancel();
    motor_.stop();
    posState_ = SteerPosState::IDLE;
    jogDir_ = 0;
    jogPulseUntil_ = 0;
    stallLatched_ = true;
    stallLatchDir_ = dir;
    stallLatchTime_ = millis();
    Debug::printfFeature(DebugFeature::SERVO,
                         "[STEER] Stall-stop (%s) dir=%d — latched for %d ms\n",
                         reason, (int)dir, STEER_STALL_COOLDOWN_MS);
    EventJournal::log(JSrc::STEER, JLvl::W, "stall", "dir=%d reason=%s", (int)dir, reason ? reason : "?");
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
            journalLink(JLvl::E, "link_lost", nullptr);
            linkDownSinceMs_ = millis();
            linkDownSnapMs_ = linkDownSinceMs_;
        } else if (linkDownSinceMs_ != 0 && (millis() - linkDownSnapMs_) >= VESC_LINK_DOWN_SNAP_MS) {
            // Still down: a parser snapshot every VESC_LINK_DOWN_SNAP_MS (only after a loss —
            // a VESC that never answered since boot is not a link-down episode).
            linkDownSnapMs_ = millis();
            char extra[32];
            snprintf(extra, sizeof(extra), "down_ms=%lu", (unsigned long)(linkDownSnapMs_ - linkDownSinceMs_));
            journalLink(JLvl::W, "link_down_snap", extra);
        }
        steerDriverOk_ = false;
        parkCancel();
        motor_.stop();
        posState_ = SteerPosState::IDLE;
        jogDir_ = 0;
        jogPulseUntil_ = 0;
        return false;
    }
    if (!steerDriverOk_) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] VESC driver recovered");
        char extra[32];
        snprintf(extra, sizeof(extra), "down_ms=%lu", (unsigned long)(millis() - linkDownSinceMs_));
        journalLink(JLvl::I, "link_up", extra);
    }
    steerDriverOk_ = true;

    // Fault-code change (I code=0 when it clears)
    const uint8_t fc = motor_.faultCode();
    if (fc != lastFaultCode_) {
        EventJournal::log(JSrc::VESC, fc ? JLvl::W : JLvl::I, "fault", "code=%u prev=%u",
                          (unsigned)fc, (unsigned)lastFaultCode_);
        lastFaultCode_ = fc;
    }

    // Nonzero VESC fault code -> immediate stop + latch (current drive direction),
    // also for a sensorless jog. This is the over-current backstop: the VESC raises a
    // fault on Absolute Max Current (and on over-temperature), and clamps at Motor
    // Current Max below that.
    if (motor_.hasFault()) {
        // A park pulse is a drive in progress too: latch its direction (lastDriveDir_).
        if (posState_ == SteerPosState::DRIVING || jogDir_ != 0 || parkPulseActive_) {
            triggerStallStop(lastDriveDir_, "vesc-fault");
        } else {
            parkCancel();
            motor_.stop();   // idle or parked: coast, no latch (no drive was in progress)
            posState_ = SteerPosState::IDLE;
        }
        return false;
    }

    return true;
}

void SteeringController::journalLink(JLvl lvl, const char* event, const char* extra) {
    // Compact keys keep the line within JOURNAL_LINE_MAX: st = parser state (at the loss for
    // link_lost), bsg = bytes since the last good packet, crc / stop = dropped packets, rs = resyncs.
    const IMotorDriver::LinkStats ls = motor_.linkStats();
    const bool lost = strcmp(event, "link_lost") == 0;
    EventJournal::log(JSrc::VESC, lvl, event,
                      "st=%s bsg=%lu good=%lu crc=%lu stop=%lu rs=%lu vin=%.1f fet=%.1f fault=%u%s%s",
                      lost ? ls.stateAtLoss : ls.state, (unsigned long)ls.bytesSinceGood,
                      (unsigned long)ls.good, (unsigned long)ls.dropCrc, (unsigned long)ls.dropStop,
                      (unsigned long)ls.resync, (double)motor_.inputVoltageV(), (double)motor_.fetTempC(),
                      (unsigned)motor_.faultCode(), extra ? " " : "", extra ? extra : "");
}

// === Steering interface ===

bool SteeringController::setSteeringPercent(float percent) {
    // USB diagnostics own the steering: MAVLink, web, failsafe and boot centring are refused.
    if (diagLock_) return false;
    if (!calibrated_ || !sensorOk_ || !steerDriverOk_) {
        return false;   // a rejected command leaves a pending boot centering in place
    }

    if (percent < -100.0f) percent = -100.0f;
    if (percent > 100.0f) percent = 100.0f;

    // Asymmetric mapping: -100 -> limLeft_ (<0), 0 -> sensor zero, +100 -> limRight_ (>0)
    int32_t target;
    if (percent >= 0.0f) {
        target = (int32_t)((percent / 100.0f) * (float)limRight_);
    } else {
        target = (int32_t)((-percent / 100.0f) * (float)limLeft_);
    }
    return commandTarget(target);
}

bool SteeringController::setSteeringAngleDeg(float deg) {
    // Same gates as setSteeringPercent(); commandTarget() clamps to the limits / safe range.
    if (diagLock_) return false;
    if (!calibrated_ || !sensorOk_ || !steerDriverOk_) return false;
    if (!isfinite(deg)) return false;
    // Pre-clamp before the int conversion (commandTarget() clamps precisely afterwards)
    if (deg < -1000.0f) deg = -1000.0f;
    if (deg > 1000.0f) deg = 1000.0f;
    int32_t target = (int32_t)lroundf(deg * 10.0f);   // 0.1° units
    return commandTarget(target);
}

bool SteeringController::diagGoto(int32_t targetPos) {
    if (!diagLock_) return false;
    if (!calibrated_ || !sensorOk_ || !steerDriverOk_) return false;
    return commandTarget(targetPos);
}

bool SteeringController::commandTarget(int32_t target) {
    bootCenterPending_ = false;   // an accepted command supersedes boot centering

    // Clamp to the calibrated limits, then to the safe range (a limit stored beyond
    // 550° maps ±100 % to ±550° instead).
    if (target < limLeft_) target = limLeft_;
    if (target > limRight_) target = limRight_;
    if (target < -STEER_LWS_SAFE_RANGE) target = -STEER_LWS_SAFE_RANGE;
    if (target > STEER_LWS_SAFE_RANGE) target = STEER_LWS_SAFE_RANGE;

    // Continuous tracking: while driving or parked a command only replaces the target —
    // no timer, boost or break-away reset, so the 500 ms stall window and the move timeout
    // elapse under the ~25 Hz MAVLink stream. Whether a parked loop drives is decided in
    // update() by the stop hysteresis. The latch is never set while tracking (a stall ends
    // in IDLE), so it is not checked here.
    if (loopActive()) {
        if (abs(target - parkRefTarget_) > STEER_PARK_TARGET_STABLE) {
            // Target moved beyond the stable band: cancel a correction (recorded when it had
            // pulsed, residual against the old target) and restart the settle clock.
            parkCancel(true, targetRel_ - getPosition());
            parkRestartSettle(millis());
            parkRefTarget_ = target;
            parkBlocked_ = false;   // a new stick command re-arms the correction
        }
        targetRel_ = target;
        return true;
    }

    // From idle. Stall latch: refuse a fresh move that would push further into a recent
    // jam (same direction, within cooldown). Opposite-direction and post-cooldown moves
    // are accepted and clear the latch.
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
    parkCancel();
    parkRestartSettle(now);   // a command from idle is a new park entry
    parkBlocked_ = false;     // ... and re-arms a correction disabled by an overshoot restart
    if (steerPosStart(posState_, posTimers_, POS_PARAMS, target, rel, now) == SteerPosState::PARKED) {
        motor_.stop();   // already there: coast (also ends a cancelled jog's drive)
        Debug::printfFeature(DebugFeature::SERVO, "[STEER] parked err=%ld\n", (long)(target - rel));
    }
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
    if (diagLock_) {
        captureError_ = "steering held by USB diagnostics";
    } else if (!sensorOk_) {
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
        // A jog stall baseline holding an old-sign value would fake progress / a stall
        // (the closed loop is IDLE after stop() and restarts its own from rest).
        lastStallPosition_ = getPosition();
        lastStallCheckTime_ = millis();
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
    if (diagLock_) {
        zeroRefusal_ = "diag";   // USB diagnostics own the steering
        return false;
    }
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
        loadParkSettings(prefs);   // park_* keys: never touched by the migration above
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

// === Park correction (include/SteerParkLaw.h, change add-steering-park-correction) ===

const char* SteeringController::parkOutcomeToken(ParkOutcome o) {
    switch (o) {
        case ParkOutcome::NONE:      return "none";
        case ParkOutcome::DONE:      return "done";
        case ParkOutcome::GAVE_UP:   return "gave_up";
        case ParkOutcome::CANCELLED: return "cancelled";
    }
    return "none";
}

void SteeringController::parkCancel(bool record, int32_t residual) {
    if (parkPulseActive_) {
        motor_.stop();   // end a running pulse at once (coast)
        parkPulseActive_ = false;
    }
    if (record && parkState_.phase == SteerParkPhase::WAIT && parkState_.pulses > 0) {
        parkLast_.outcome = ParkOutcome::CANCELLED;
        parkLast_.pulses = parkState_.pulses;
        parkLast_.residual = residual;
        parkLast_.duty = parkState_.lastDuty;
        parkLast_.ms = parkState_.lastMs;
        parkLast_.note = SteerParkNote::NONE;
        Debug::printfFeature(DebugFeature::SERVO,
                             "[STEER] park correction cancelled at %.2f deg (%u pulses)\n",
                             parkDeg(residual), (unsigned)parkState_.pulses);
    }
    parkState_.reset();
}

void SteeringController::parkRestartSettle(uint32_t now) {
    parkRefTarget_ = targetRel_;
    parkSettleStartMs_ = now;
}

void SteeringController::serviceParkCorrection(uint32_t now, int32_t pos, int32_t err) {
    // Position mode is implied here (update() reaches PARKED only with a healthy, calibrated
    // sensor and the VESC link up); the switch and the USB diagnostics lock gate the rest.
    if (!parkEnabled_ || diagLock_ || parkBlocked_) return;

    if (parkPulseActive_) {
        if ((int32_t)(now - parkPulseEndMs_) < 0) return;   // pulse still running
        motor_.stop();   // pulse length reached: coast; the worm holds the shaft
        parkPulseActive_ = false;
    }

    // Stall latch: a pulse toward a direction latched within STEER_STALL_COOLDOWN_MS is not
    // sent and not counted — the law is simply not stepped until the cooldown has passed.
    const int8_t errDir = err > 0 ? 1 : (err < 0 ? -1 : 0);
    if (latchBlocks(errDir, now)) return;

    const SteerParkResult r = steerParkStep(parkState_, parkParams_, now, err, pos,
                                            now - parkSettleStartMs_);
    if (r.evaluated) {
        Debug::printfFeature(DebugFeature::SERVO, "[STEER] park nudge %ux%u -> moved %.2f deg (err %.2f)\n",
                             (unsigned)r.evalDuty, (unsigned)r.evalMs, parkDeg(r.moved), parkDeg(err));
        if (r.note == SteerParkNote::OVERSHOOT) {
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] park overshoot -> rung 1, reversing (err %.2f)\n",
                                 parkDeg(err));
        } else if (r.note == SteerParkNote::TOO_STRONG) {
            Debug::printfFeature(DebugFeature::SERVO,
                                 "[STEER] park pulse too strong (overshot to %.2f deg) -> giving up until the next stick command\n",
                                 parkDeg(err));
        } else if (r.note == SteerParkNote::WRONG_WAY) {
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] park nudge moved the wrong way -> rung 1 (err %.2f)\n",
                                 parkDeg(err));
        }
    }

    switch (r.action) {
        case SteerParkAction::PULSE:
            // Straight to the driver, NOT driveJog(): no STEER_PWM_MIN_DUTY floor, no jog state.
            lastDriveDir_ = r.dir;   // a VESC fault during the pulse latches this direction
            motor_.setSpeed((int16_t)(r.dir * (int16_t)r.duty));
            parkPulseActive_ = true;
            parkPulseEndMs_ = now + r.ms;
            break;
        case SteerParkAction::DONE:
        case SteerParkAction::GIVE_UP: {
            const bool done = (r.action == SteerParkAction::DONE);
            parkLast_.outcome = done ? ParkOutcome::DONE : ParkOutcome::GAVE_UP;
            parkLast_.pulses = parkState_.pulses;
            parkLast_.residual = err;
            parkLast_.duty = parkState_.lastDuty;
            parkLast_.ms = parkState_.lastMs;
            parkLast_.note = r.note;
            if (r.note == SteerParkNote::TOO_STRONG) parkBlocked_ = true;   // until the next stick command
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] park correction %s at %.2f deg (%u pulses)\n",
                                 done ? "done" : "gave up", parkDeg(err), (unsigned)parkState_.pulses);
            break;
        }
        case SteerParkAction::NONE:
            break;
    }
}

void SteeringController::loadParkSettings(Preferences& prefs) {
    // Missing key → default (NVS not written). Out of range / non-monotonic → logged, default
    // in RAM, NVS not rewritten.
    SteerParkParams p = PARK_DEFAULTS;
    parkEnabled_ = prefs.isKey(STEER_PARK_EN_NVS_KEY) ? prefs.getBool(STEER_PARK_EN_NVS_KEY, STEER_PARK_ENABLED_DEFAULT)
                                                     : STEER_PARK_ENABLED_DEFAULT;

    for (uint8_t i = 0; i < STEER_PARK_RUNGS; i++) {
        if (prefs.isKey(PARK_DUTY_KEYS[i])) {
            const uint8_t d = prefs.getUChar(PARK_DUTY_KEYS[i], p.rung[i].duty);
            if (d >= STEER_PARK_DUTY_MIN && d <= STEER_PARK_DUTY_MAX) {
                p.rung[i].duty = d;
            } else {
                Debug::printfFeature(DebugFeature::SERVO, "[STEER] NVS %s=%u out of range (%d-%d) — default %u used\n",
                                     PARK_DUTY_KEYS[i], (unsigned)d, STEER_PARK_DUTY_MIN, STEER_PARK_DUTY_MAX,
                                     (unsigned)p.rung[i].duty);
            }
        }
        if (prefs.isKey(PARK_MS_KEYS[i])) {
            const uint16_t t = prefs.getUShort(PARK_MS_KEYS[i], p.rung[i].ms);
            if (t >= STEER_PARK_MS_MIN && t <= STEER_PARK_MS_MAX) {
                p.rung[i].ms = t;
            } else {
                Debug::printfFeature(DebugFeature::SERVO, "[STEER] NVS %s=%u out of range (%d-%d ms) — default %u used\n",
                                     PARK_MS_KEYS[i], (unsigned)t, STEER_PARK_MS_MIN, STEER_PARK_MS_MAX,
                                     (unsigned)p.rung[i].ms);
            }
        }
    }
    if (!steerParkLadderValid(p)) {
        Debug::printfFeature(DebugFeature::SERVO,
                             "[STEER] NVS park ladder %ux%u / %ux%u / %ux%u decreases — default ladder used\n",
                             (unsigned)p.rung[0].duty, (unsigned)p.rung[0].ms, (unsigned)p.rung[1].duty,
                             (unsigned)p.rung[1].ms, (unsigned)p.rung[2].duty, (unsigned)p.rung[2].ms);
        for (uint8_t i = 0; i < STEER_PARK_RUNGS; i++) p.rung[i] = PARK_DEFAULTS.rung[i];
    }

    if (prefs.isKey(STEER_PARK_TOL_NVS_KEY)) {
        const int32_t tol = prefs.getInt(STEER_PARK_TOL_NVS_KEY, p.doneTol);
        if (tol >= STEER_PARK_DONE_TOL_MIN && tol <= STEER_PARK_DONE_TOL_MAX) {
            p.doneTol = tol;
        } else {
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] NVS %s=%ld out of range (%d-%d) — default %ld used\n",
                                 STEER_PARK_TOL_NVS_KEY, (long)tol, STEER_PARK_DONE_TOL_MIN, STEER_PARK_DONE_TOL_MAX,
                                 (long)p.doneTol);
        }
    }
    if (prefs.isKey(STEER_PARK_MAX_NVS_KEY)) {
        const uint8_t m = prefs.getUChar(STEER_PARK_MAX_NVS_KEY, p.maxPulses);
        if (m >= STEER_PARK_MAX_PULSES_MIN && m <= STEER_PARK_MAX_PULSES_MAX) {
            p.maxPulses = m;
        } else {
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] NVS %s=%u out of range (%d-%d) — default %u used\n",
                                 STEER_PARK_MAX_NVS_KEY, (unsigned)m, STEER_PARK_MAX_PULSES_MIN,
                                 STEER_PARK_MAX_PULSES_MAX, (unsigned)p.maxPulses);
        }
    }
    parkParams_ = p;

    Debug::printfFeature(DebugFeature::SERVO,
                         "[STEER] Park correction %s: ladder %ux%u / %ux%u / %ux%u, done %.1f deg, max %u pulses\n",
                         parkEnabled_ ? "ON" : "OFF", (unsigned)p.rung[0].duty, (unsigned)p.rung[0].ms,
                         (unsigned)p.rung[1].duty, (unsigned)p.rung[1].ms, (unsigned)p.rung[2].duty,
                         (unsigned)p.rung[2].ms, parkDeg(p.doneTol), (unsigned)p.maxPulses);
}

SteeringController::ParkSetResult SteeringController::setParkEnabled(bool on) {
    // Persist first: RAM changes only once NVS holds the value. One key in the shared
    // "steering" namespace — never clear() it.
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(NVS_NS, false)) {
        ok = prefs.putBool(STEER_PARK_EN_NVS_KEY, on) == sizeof(uint8_t);
        prefs.end();
    }
    if (!ok) return ParkSetResult::SAVE_FAILED;
    parkCancel();                  // a running correction ends (pulse coasts) ...
    parkRestartSettle(millis());   // ... and the next one waits a full settle time
    parkEnabled_ = on;
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] Park correction %s\n", on ? "enabled" : "disabled");
    return ParkSetResult::OK;
}

SteeringController::ParkSetResult SteeringController::setParkRung(uint8_t rung, int32_t duty, int32_t ms) {
    if (rung < 1 || rung > STEER_PARK_RUNGS ||
        duty < STEER_PARK_DUTY_MIN || duty > STEER_PARK_DUTY_MAX ||
        ms < STEER_PARK_MS_MIN || ms > STEER_PARK_MS_MAX) {
        return ParkSetResult::OUT_OF_RANGE;
    }
    SteerParkParams candidate = parkParams_;
    candidate.rung[rung - 1].duty = (uint8_t)duty;
    candidate.rung[rung - 1].ms = (uint16_t)ms;
    if (!steerParkLadderValid(candidate)) return ParkSetResult::NOT_MONOTONIC;

    Preferences prefs;
    bool ok = false;
    if (prefs.begin(NVS_NS, false)) {
        ok = prefs.putUChar(PARK_DUTY_KEYS[rung - 1], (uint8_t)duty) == sizeof(uint8_t);
        ok = ok && prefs.putUShort(PARK_MS_KEYS[rung - 1], (uint16_t)ms) == sizeof(uint16_t);
        prefs.end();
    }
    if (!ok) return ParkSetResult::SAVE_FAILED;
    parkCancel();                  // a running correction ends (pulse coasts) ...
    parkRestartSettle(millis());   // ... and the next one waits a full settle time
    parkParams_ = candidate;
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] Park rung %u set to %ldx%ld\n",
                         (unsigned)rung, (long)duty, (long)ms);
    return ParkSetResult::OK;
}

SteeringController::ParkSetResult SteeringController::setParkDoneTol(int32_t tol) {
    if (tol < STEER_PARK_DONE_TOL_MIN || tol > STEER_PARK_DONE_TOL_MAX) return ParkSetResult::OUT_OF_RANGE;
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(NVS_NS, false)) {
        ok = prefs.putInt(STEER_PARK_TOL_NVS_KEY, tol) == sizeof(int32_t);
        prefs.end();
    }
    if (!ok) return ParkSetResult::SAVE_FAILED;
    parkCancel();                  // a running correction ends (pulse coasts) ...
    parkRestartSettle(millis());   // ... and the next one waits a full settle time
    parkParams_.doneTol = tol;
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] Park tolerance set to %.1f deg\n", parkDeg(tol));
    return ParkSetResult::OK;
}

SteeringController::ParkSetResult SteeringController::setParkMaxPulses(int32_t count) {
    if (count < STEER_PARK_MAX_PULSES_MIN || count > STEER_PARK_MAX_PULSES_MAX) return ParkSetResult::OUT_OF_RANGE;
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(NVS_NS, false)) {
        ok = prefs.putUChar(STEER_PARK_MAX_NVS_KEY, (uint8_t)count) == sizeof(uint8_t);
        prefs.end();
    }
    if (!ok) return ParkSetResult::SAVE_FAILED;
    parkCancel();                  // a running correction ends (pulse coasts) ...
    parkRestartSettle(millis());   // ... and the next one waits a full settle time
    parkParams_.maxPulses = (uint8_t)count;
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] Park max pulses set to %ld\n", (long)count);
    return ParkSetResult::OK;
}
