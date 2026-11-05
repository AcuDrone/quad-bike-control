#include "SteerDiag.h"

// Constants for the pure law (include/SteerDiagLaw.h)
static const SteerDiagParams DIAG_PARAMS = {
    STEER_DIAG_MOVE_POS,
    STEER_DIAG_MOVE_RATE,
    STEER_DIAG_SETTLE_MS,
    STEER_DIAG_RAMP_TIMEOUT_MS,
    STEER_DIAG_RAMP_MAXHOLD_MS,
    STEER_DIAG_GOTO_TIMEOUT_MS,
};

static const SteerDiagLimits DIAG_LIMITS = {
    STEER_PWM_MAX_DUTY,
    STEER_DIAG_PULSE_MAX_MS,
    STEER_DIAG_HOLD_MAX_MS,
    STEER_DIAG_GOTO_MAX_DEG,
};

SteerDiag::SteerDiag(SteeringController& steering, IMotorDriver& motor, const SpeedSensor& speed)
    : steering_(steering),
      motor_(motor),
      speed_(speed),
      active_(false),
      lastCmdMs_(0),
      lineSeen_(false),
      lastLineMs_(0),
      lastKeepaliveMs_(0),
      lineLen_(0),
      lineOverflow_(false),
      exp_(),
      lastLogMs_(0),
      lastDriveMs_(0),
      lastDuty_(0),
      dropped_(0),
      hostWatch_() {
    line_[0] = '\0';
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

float SteerDiag::speedKmh() const { return speed_.getSpeedMs() * 3.6f; }

bool SteerDiag::moving() const {
    return steerDiagMoving(speed_.isValid(), speed_.getSpeedMs(), STEER_DIAG_MAX_SPEED_KMH);
}

static SteerDiagLoop toDiagLoop(SteerPosState s) {
    switch (s) {
        case SteerPosState::DRIVING: return SteerDiagLoop::DRIVE;
        case SteerPosState::PARKED:  return SteerDiagLoop::PARK;
        default:                     return SteerDiagLoop::IDLE;
    }
}

SteerDiagSample SteerDiag::sample(uint32_t now) const {
    SteerDiagSample s;
    s.now = now;
    s.pos = steering_.getPosition();
    s.rateValid = steering_.hasRate();
    s.rate = s.rateValid ? steering_.getRateDegS() : 0;
    s.currentA = steering_.getMotorCurrent();
    s.loop = toDiagLoop(steering_.getPosState());
    return s;
}

const char* SteerDiag::motionRefusal() const {
    if (!active_) return "diag_off";
    if (moving()) return "speed";
    if (!steering_.isSensorOk() || !steering_.hasAngle()) return "angle";
    if (!steering_.isDriverOk()) return "driver";
    if (steering_.getVescFault() != 0) return "fault";
    if (!steering_.isCalibrated()) return "not_calibrated";
    return nullptr;
}

const char* SteerDiag::abortReason() const {
    if (moving()) return "speed";
    if (!steering_.isSensorOk() || !steering_.hasAngle()) return "angle";
    if (!steering_.isDriverOk()) return "driver";
    if (steering_.getVescFault() != 0) return "fault";
    const int16_t duty = steering_.getLastDriveCommand();
    const int sign = duty > 0 ? 1 : (duty < 0 ? -1 : 0);
    if (steerDiagNearLimit(steering_.getPosition(), sign, steering_.getLeftLimit(), steering_.getRightLimit(),
                           STEER_DIAG_LIMIT_MARGIN, SteeringController::POS_UNSET)) {
        return "limit";
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// USB TX keepalive
// ---------------------------------------------------------------------------

// HWCDC (core 3.3.x) moves ring-buffer bytes to the USB FIFO only from its SERIAL_IN_EMPTY ISR,
// and that interrupt can stay disarmed after a short reply, which then sits in the ring buffer
// until the next write() (seen live: a status line arrived together with the next answer).
// write() itself re-arms the interrupt, so inside the keepalive window a 2-byte `K` line is
// written every STEER_DIAG_KEEPALIVE_MS through the public API only (no LL register access
// from task context). Skipped while D-lines flow, never outside the window, and only when it
// fits the ring buffer: a write that does not fit can block up to the TX timeout (100 ms).
void SteerDiag::keepalive(uint32_t now) {
    if (!steerDiagKeepaliveDue(active_, exp_.running(), lineSeen_, lastLineMs_, lastKeepaliveMs_, now,
                               STEER_DIAG_KEEPALIVE_WINDOW_MS, STEER_DIAG_KEEPALIVE_MS)) {
        return;
    }
    lastKeepaliveMs_ = now;
    static const uint8_t K_LINE[2] = {'K', '\n'};
    if (Serial.availableForWrite() < 16) return;   // never block the control loop on USB
    Serial.write(K_LINE, sizeof(K_LINE));
}

// ---------------------------------------------------------------------------
// Main poll
// ---------------------------------------------------------------------------

void SteerDiag::update() {
    readSerial();
    keepalive(millis());

    if (!active_) return;
    const uint32_t now = millis();

    // USB host gone: never keep driving unattended. Debounced — HWCDC's flag can read false
    // transiently (TX not drained yet, write timeout), see steerDiagHostLost().
    if (steerDiagHostLost(hostWatch_, (bool)Serial, now, STEER_DIAG_HOST_LOST_MS)) {
        if (exp_.running()) abortExperiment("host");
        setActive(false, "host");
        return;
    }
    if (now - lastCmdMs_ >= STEER_DIAG_IDLE_TIMEOUT_MS) {
        if (exp_.running()) abortExperiment("idle_timeout");
        setActive(false, "idle_timeout");
        return;
    }

    if (exp_.running()) serviceExperiment(now);
}

void SteerDiag::readSerial() {
    // Bounded per call: never stalls the loop on a flood of input.
    for (int i = 0; i < 128 && Serial.available() > 0; ++i) {
        const int ch = Serial.read();
        if (ch < 0) break;
        if (ch == '\r' || ch == '\n') {
            if (lineOverflow_) {
                Serial.println("E,line,line_too_long");
            } else if (lineLen_ > 0) {
                line_[lineLen_] = '\0';
                handleLine(line_);
            }
            lineLen_ = 0;
            lineOverflow_ = false;
            continue;
        }
        if (lineLen_ < STEER_DIAG_LINE_MAX) {
            line_[lineLen_++] = (char)ch;
        } else {
            lineOverflow_ = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

void SteerDiag::handleLine(const char* line) {
    lastCmdMs_ = millis();
    lastLineMs_ = lastCmdMs_;   // keepalive window (lastCmdMs_ is also reset by setActive)
    lineSeen_ = true;
    const SteerDiagCmd c = steerDiagParse(line, DIAG_LIMITS);

    switch (c.type) {
        case SteerDiagCmdType::NONE:
            return;
        case SteerDiagCmdType::INVALID:
            Serial.printf("E,cmd,%s\n", c.error ? c.error : "bad_arg");
            return;
        case SteerDiagCmdType::HELP:
            printHelp();
            return;
        case SteerDiagCmdType::STATUS:
            printStatus();
            return;
        case SteerDiagCmdType::STOP:
            if (exp_.running()) abortExperiment("stop");
            if (active_) steering_.diagDrive(0);
            else steering_.stop();
            Serial.println("OK stop");
            return;
        case SteerDiagCmdType::DIAG_ON:
            if (active_) { Serial.println("OK diag_on"); return; }
            if (moving()) { Serial.println("E,diag_on,speed"); return; }
            setActive(true, "cmd");
            Serial.println("OK diag_on");
            return;
        case SteerDiagCmdType::DIAG_OFF:
            if (exp_.running()) abortExperiment("stop");
            if (active_) setActive(false, "cmd");
            Serial.println("OK diag_off");
            return;
        case SteerDiagCmdType::RAMP:
        case SteerDiagCmdType::PULSE:
        case SteerDiagCmdType::HOLD:
        case SteerDiagCmdType::GOTO:
            startExperiment(c);
            return;
    }
}

void SteerDiag::setActive(bool on, const char* reason) {
    exp_ = SteerDiagExp();
    hostWatch_ = SteerDiagHostWatch();   // re-armed by the first true reading after diag on
    steering_.setDiagLock(on);   // stops the motor either way
    motor_.setTelemetryPeriodMs(on ? STEER_DIAG_VESC_TELEM_MS : STEER_VESC_TELEM_MS);
    active_ = on;
    lastCmdMs_ = millis();
    Serial.printf("I,diag,%s,reason=%s\n", on ? "on" : "off", reason);
}

void SteerDiag::startExperiment(const SteerDiagCmd& c) {
    const char* name = c.type == SteerDiagCmdType::RAMP ? "ramp"
                     : c.type == SteerDiagCmdType::PULSE ? "pulse"
                     : c.type == SteerDiagCmdType::HOLD ? "hold" : "goto";
    if (exp_.running()) abortExperiment("superseded");

    const char* why = motionRefusal();
    if (why) {
        Serial.printf("E,%s,%s\n", name, why);
        return;
    }

    const uint32_t now = millis();
    const int32_t pos = steering_.getPosition();
    const int32_t limL = steering_.getLeftLimit();
    const int32_t limR = steering_.getRightLimit();

    if (c.type == SteerDiagCmdType::GOTO) {
        if (!steerDiagTargetOk(c.target, limL, limR, STEER_DIAG_LIMIT_MARGIN)) {
            Serial.println("E,goto,near_limit");
            return;
        }
        steerDiagBeginGoto(exp_, c.target, pos, now);
        if (!steering_.diagGoto(c.target)) {
            exp_ = SteerDiagExp();
            Serial.println("E,goto,refused");
            return;
        }
    } else {
        if (steerDiagNearLimit(pos, c.dir, limL, limR, STEER_DIAG_LIMIT_MARGIN, SteeringController::POS_UNSET)) {
            Serial.printf("E,%s,near_limit\n", name);
            return;
        }
        if (c.type == SteerDiagCmdType::RAMP) {
            steerDiagBeginRamp(exp_, c.dir, c.startDuty, c.maxDuty, c.step, c.stepMs, pos, now);
        } else {
            steerDiagBeginTimed(exp_, c.type == SteerDiagCmdType::PULSE ? SteerDiagKind::PULSE : SteerDiagKind::HOLD,
                                c.dir, c.duty, c.ms, pos, now);
        }
    }

    dropped_ = 0;
    lastDuty_ = 0;
    Serial.printf("OK %s\n", name);
    if (exp_.kind != SteerDiagKind::GOTO) {
        applyDuty((int16_t)(exp_.curDuty * exp_.dir), now, true);
    }
    lastLogMs_ = now;
    logLine(now);
}

// ---------------------------------------------------------------------------
// Experiment
// ---------------------------------------------------------------------------

void SteerDiag::applyDuty(int16_t duty, uint32_t now, bool force) {
    // Re-sent on change and every STEER_DIAG_REFRESH_MS so the VESC never times out mid-hold.
    if (!force && duty == lastDuty_ && now - lastDriveMs_ < STEER_DIAG_REFRESH_MS) return;
    steering_.diagDrive(duty);
    lastDuty_ = duty;
    lastDriveMs_ = now;
}

void SteerDiag::serviceExperiment(uint32_t now) {
    const char* why = abortReason();
    if (why) {
        abortExperiment(why);
        return;
    }

    const SteerDiagOut out = steerDiagStep(exp_, DIAG_PARAMS, sample(now));
    if (exp_.kind != SteerDiagKind::GOTO) {
        applyDuty(out.duty, now, out.phaseChanged);
    }
    if (out.phaseChanged || now - lastLogMs_ >= STEER_DIAG_LOG_MS) {
        lastLogMs_ = now;
        logLine(now);
    }
    if (out.done) finishExperiment();
}

void SteerDiag::abortExperiment(const char* reason) {
    if (active_) steering_.diagDrive(0);
    steering_.stop();   // also drops a goto's closed loop to idle
    lastDuty_ = 0;
    exp_ = SteerDiagExp();
    Serial.printf("E,abort,%s\n", reason);
}

void SteerDiag::finishExperiment() {
    // Motor off in every case; a goto's parked loop is dropped to idle so nothing restarts it.
    steering_.diagDrive(0);
    lastDuty_ = 0;
    printSummary();
    exp_ = SteerDiagExp();
}

void SteerDiag::logLine(uint32_t now) {
    char buf[112];
    char rate[12];
    if (steering_.hasRate()) snprintf(rate, sizeof(rate), "%ld", (long)steering_.getRateDegS());
    else rate[0] = '\0';
    const bool isGoto = exp_.kind == SteerDiagKind::GOTO;
    const char* state = isGoto ? SteeringController::posStateToken(steering_.getPosState())
                               : steerDiagPhaseToken(exp_);
    const int n = snprintf(buf, sizeof(buf), "D,%lu,%d,%ld,%s,%.2f,%.1f,%s,%ld\n",
                           (unsigned long)(now - exp_.t0), (int)steering_.getLastDriveCommand(),
                           (long)steering_.getPosition(), rate, (double)steering_.getMotorCurrent(),
                           (double)steering_.getInputVoltage(), state,
                           (long)(isGoto ? exp_.target : exp_.startPos));
    if (n <= 0) return;
    const size_t len = (size_t)(n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1);
    if ((size_t)Serial.availableForWrite() < len) {   // never block the control loop on USB
        dropped_++;
        return;
    }
    Serial.write(reinterpret_cast<const uint8_t*>(buf), len);
}

void SteerDiag::printSummary() {
    const char* dir = exp_.dir > 0 ? "R" : (exp_.dir < 0 ? "L" : "-");
    const double start = exp_.startPos / 10.0;
    switch (exp_.kind) {
        case SteerDiagKind::RAMP:
            Serial.printf("S,ramp,dir=%s,result=%s,break_duty=%d,break_cur=%.2f,break_ms=%lu,moved_cut=%.1f,"
                          "moved_end=%.1f,coast=%.1f,peak_cur=%.2f,start=%.1f,drop=%lu\n",
                          dir, exp_.endReason ? exp_.endReason : "?", exp_.moved ? (int)exp_.breakDuty : 0,
                          (double)exp_.breakCurrentA, (unsigned long)exp_.breakMs,
                          (double)steerDiagMovedAtCutDeg(exp_), (double)steerDiagMovedEndDeg(exp_),
                          (double)steerDiagCoastDeg(exp_), (double)exp_.peakCurrentA, start,
                          (unsigned long)dropped_);
            break;
        case SteerDiagKind::PULSE:
            Serial.printf("S,pulse,dir=%s,duty=%d,ms=%lu,actual_ms=%lu,moved_cut=%.1f,moved_end=%.1f,coast=%.1f,"
                          "peak_cur=%.2f,start=%.1f,drop=%lu\n",
                          dir, (int)exp_.duty, (unsigned long)exp_.ms, (unsigned long)exp_.cutMs,
                          (double)steerDiagMovedAtCutDeg(exp_), (double)steerDiagMovedEndDeg(exp_),
                          (double)steerDiagCoastDeg(exp_), (double)exp_.peakCurrentA, start,
                          (unsigned long)dropped_);
            break;
        case SteerDiagKind::HOLD:
            Serial.printf("S,hold,dir=%s,duty=%d,ms=%lu,rate_pos=%.1f,rate_lws=%.1f,mean_cur=%.2f,peak_cur=%.2f,"
                          "moved_cut=%.1f,coast=%.1f,start=%.1f,drop=%lu\n",
                          dir, (int)exp_.duty, (unsigned long)exp_.ms, (double)steerDiagHoldPosRate(exp_),
                          (double)steerDiagHoldLwsRate(exp_), (double)steerDiagHoldMeanCurrent(exp_),
                          (double)exp_.peakCurrentA, (double)steerDiagMovedAtCutDeg(exp_),
                          (double)steerDiagCoastDeg(exp_), start, (unsigned long)dropped_);
            break;
        case SteerDiagKind::GOTO:
            Serial.printf("S,goto,target=%.1f,start=%.1f,end=%.1f,result=%s,overshoot=%.1f,residual=%.1f,"
                          "t_stop=%lu,t_end=%lu,restarts=%u,peak_cur=%.2f,drop=%lu\n",
                          exp_.target / 10.0, start, exp_.endPos / 10.0, exp_.endReason ? exp_.endReason : "?",
                          (double)steerDiagOvershootDeg(exp_), (double)steerDiagResidualDeg(exp_),
                          (unsigned long)exp_.firstStopMs, (unsigned long)exp_.endMs, (unsigned)exp_.restarts,
                          (double)exp_.peakCurrentA, (unsigned long)dropped_);
            break;
        default:
            break;
    }
}

void SteerDiag::printStatus() {
    char angle[12], rate[12], limL[12], limR[12];
    if (steering_.hasAngle()) snprintf(angle, sizeof(angle), "%.1f", (double)steering_.getAngleDeg());
    else snprintf(angle, sizeof(angle), "nan");
    if (steering_.hasRate()) snprintf(rate, sizeof(rate), "%ld", (long)steering_.getRateDegS());
    else snprintf(rate, sizeof(rate), "nan");
    const int32_t l = steering_.getLeftLimit(), r = steering_.getRightLimit();
    if (l != SteeringController::POS_UNSET) snprintf(limL, sizeof(limL), "%.1f", l / 10.0);
    else snprintf(limL, sizeof(limL), "nan");
    if (r != SteeringController::POS_UNSET) snprintf(limR, sizeof(limR), "%.1f", r / 10.0);
    else snprintf(limR, sizeof(limR), "nan");
    const char* expName = exp_.running() ? (exp_.kind == SteerDiagKind::GOTO ? "goto" : steerDiagPhaseToken(exp_))
                                         : "none";
    Serial.printf("I,status,diag=%d,exp=%s,angle=%s,rate=%s,limL=%s,limR=%s,cal=%d,sensor=%d,drv=%d,fault=%u,"
                  "duty=%d,cur=%.2f,vin=%.1f,speed=%.1f,speedValid=%d,loop=%s,fw=%s\n",
                  active_ ? 1 : 0, expName, angle, rate, limL, limR, steering_.isCalibrated() ? 1 : 0,
                  steering_.isSensorOk() ? 1 : 0, steering_.isDriverOk() ? 1 : 0,
                  (unsigned)steering_.getVescFault(), (int)steering_.getLastDriveCommand(),
                  (double)steering_.getMotorCurrent(), (double)steering_.getInputVoltage(), (double)speedKmh(),
                  speed_.isValid() ? 1 : 0, SteeringController::posStateToken(steering_.getPosState()),
                  FIRMWARE_VERSION);
}

void SteerDiag::printHelp() {
    Serial.println("I,help,diag on|diag off|status|stop|s|ramp L|R [max=220] [step=2] [stepms=150] [start=40]|"
                   "pulse L|R <duty> <ms>|hold L|R <duty> <ms>|goto <deg>");
}
