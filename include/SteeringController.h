#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "IMotorDriver.h"
#include "LwsSensor.h"
#include "SteerParkLaw.h"
#include "EventJournalLaw.h"
#include "SteerPositionLaw.h"
#include "SteerRate.h"
#include "SteerSensorDir.h"

class CANController;
class Preferences;

/**
 * @brief Steering actuator controller: drives an IMotorDriver (VESC over UART)
 * with proportional PWM and absolute multi-turn position feedback from a Bosch LWS
 * steering-angle sensor 0 265 005 486 mounted directly on the steering shaft and
 * read over the shared MCP2515 CAN bus (frame 0x305, 100 Hz).
 *
 *  - Position = −angle for a normal mount (the sensor's positive is LEFT; the firmware's
 *    positive is RIGHT), +angle for a mirrored mount (NVS flag `inv`), in 0.1° of
 *    steering-shaft angle (int32), relative to the sensor's own stored zero. No homing,
 *    no turn tracking: every frame is absolute. The flag is applied inside LwsSensor, so
 *    every consumer of getPosition() / getRateDegS() sees the firmware sign only.
 *  - Sensor direction (SteerSensorDir.h): inferred at limit capture, never set by hand.
 *    A capture on the wrong side of zero (by ≥ STEER_CAL_MIN_SPAN) toggles the flag only
 *    while the opposite limit is unset, and clears it; with the opposite limit stored it
 *    is refused (no ping-pong). Below the span it never toggles. A zero-set keeps the flag.
 *  - Health (fault token, design D5): zeroing > can_down > timeout > range_latched >
 *    sensor_fault > not_calibrated > angle_invalid > none. Faults take effect at once;
 *    recovery needs STEER_LWS_RECOVER_FRAMES good frames. A fault stops a closed-loop
 *    move and rejects percent commands; jog and nudge keep working (sensorless, open
 *    loop, no stall backstop) — they only need the VESC link and no stall latch.
 *  - Calibration: center = sensor zero (set by requestSensorZero()); NVS namespace
 *    "steering" v4 keeps lim_l / lim_r (0.1°, firmware sign) and inv (bool). v3 limits
 *    are kept with inv = false. Each capture is saved at once; calibrated = both limits
 *    stored and valid (always under one flag).
 *  - Closed loop (SteerPositionLaw.h): states IDLE / DRIVING / PARKED. Continuous target
 *    tracking — while driving or parked a percent command only replaces the target (the
 *    former 5° re-command guard, STEER_RETARGET_TOLERANCE, is removed). Stop hysteresis —
 *    park (one coast; the self-locking 1:40 worm holds) within STEER_POSITION_TOLERANCE
 *    (0.2° on the quad), drive again only beyond STEER_RESTART_TOLERANCE (1.0°). Two duty
 *    floors — STEER_PWM_MIN_DUTY (break-away) from rest / after a reversal until 0.3° of
 *    progress, then STEER_PWM_TRACK_MIN_DUTY; duty = clamp(max(|err| × STEER_KP, floor) +
 *    boost, floor, STEER_PWM_MAX_DUTY), the anti-stall boost adds on top of the floor
 *    (+STEER_BOOST_STEP per stalled check, −STEER_BOOST_DECAY per moving check, ≥ 0).
 *    Every stop / fault path coasts and goes to IDLE.
 *  - Range guard: percent-driven (closed-loop) targets are clamped to the limits, then to
 *    ±STEER_LWS_SAFE_RANGE, and a closed-loop move driving outward at |pos| ≥ that range
 *    is stopped. Jog, nudge and captures are not guarded.
 *  - Steering modes (selected by VehicleController, see SteerRate.h): POSITION while the
 *    sensor is healthy (closed loop, setSteeringPercent()); RATE while it is not, the
 *    source is MAVLink and the autopilot is in MANUAL/unknown; BLOCKED otherwise.
 *  - Rate drive (RATE mode only): setRateCommand(pct) is a continuous jog with variable
 *    duty (steerRateDuty(): deadband → 0, else STEER_PWM_MIN_DUTY..255 quadratic). Refused while the
 *    sensor is healthy, the VESC is down, or the path is disarmed (centre-to-arm: one
 *    zero-duty command arms it after disarmRate()). It reuses the jog refresh timeout
 *    (STEER_JOG_TIMEOUT_MS), the stall latch and the VESC fault / comm stops; position
 *    stall detection, boost, move timeout and range guard never apply to it.
 *  - USB diagnostics lock (SteerDiag, change add-steering-serial-diagnostics): while
 *    setDiagLock(true) is in force every external steering entry point — setSteeringPercent()
 *    (MAVLink, web slider, failsafe centre, boot centring), setRateCommand(), jog(), nudge(),
 *    the limit captures and the sensor zero-set — is refused. The diagnostics drive the motor
 *    open loop through diagDrive() or run one closed-loop move through diagGoto(). Stop paths
 *    (stop(), stopRate(), the sensor / VESC monitors) are never gated.
 *  - Park correction (SteerParkLaw.h, change add-steering-park-correction): a sub-mode of
 *    PARKED (the state stays PARKED, no jog). Once the loop has been parked with a target stable
 *    within STEER_PARK_TARGET_STABLE for STEER_PARK_SETTLE_MS and |err| exceeds
 *    max(STEER_PARK_TRIGGER_TOL (0.4°, independent of the park window), done tolerance), short pulses from a 3-rung ladder
 *    (motor_.setSpeed(dir × duty) then motor_.stop(), no duty floor) finish the move to the done
 *    tolerance, at most max-pulses per parking. Any target move beyond the stable band, any exit
 *    from PARKED, jog / nudge / rate, captures, zero-set, the diag lock or a park setting change
 *    cancels it (a running pulse coasts at once). Settings persist in NVS "steering" (park_*).
 *  - Jog ownership: an active jog is owned by WEB (jog()/nudge()) or RATE
 *    (setRateCommand()). Rate commands never stop or override a WEB jog/nudge; a zero
 *    rate command and stopRate() stop only a RATE-owned drive; jog(0) and stop() stop
 *    any jog.
 */
class SteeringController {
public:
    /** @brief Steering-angle sensor fault reason (telemetry token via faultToken()) */
    using SensorFault = LwsFault;

    /** @brief Sentinel for an unset limit position (positions may be negative) */
    static constexpr int32_t POS_UNSET = INT32_MIN;

    /** @brief Owner of the active open-loop jog drive */
    enum class JogOwner : uint8_t { NONE, WEB, RATE };

    /** @brief Zero-set progress (telemetry token via zeroStateToken()) */
    enum class ZeroState : uint8_t { IDLE, RUNNING, OK, FAILED };

    /**
     * @brief Construct bound to an already-constructed motor driver.
     * The driver's own begin() (e.g. VESC UART init) must be called by the
     * caller before SteeringController::begin().
     */
    explicit SteeringController(IMotorDriver& motor);

    /**
     * @brief Coast the motor and load the limits from NVS (migrating older layouts).
     * The sensor starts as `timeout` (or `can_down`) until frames arrive.
     */
    bool begin();

    /**
     * @brief Bind the CAN controller that feeds lwsSensor() and runs the zero-set.
     * Without it the sensor state is `can_down`.
     */
    void attachCan(CANController* can) { can_ = can; }

    /** @brief The frame sink to attach to the CAN controller */
    LwsSensor& lwsSensor() { return lws_; }
    const LwsSensor& lwsSensor() const { return lws_; }

    /**
     * @brief Sensor health, zero-set and position control — call every loop
     * iteration, after CANController::update().
     *
     * Closed loop (steerPosStep()): DRIVING applies duty = clamp(max(|err| × STEER_KP, floor)
     * + boost, floor, STEER_PWM_MAX_DUTY), floor = STEER_PWM_MIN_DUTY until the wheel has moved
     * 0.3° since the drive started or last reversed, then STEER_PWM_TRACK_MIN_DUTY; the boost
     * rises by STEER_BOOST_STEP per stalled check and decays by STEER_BOOST_DECAY per moving
     * check. At |err| ≤ STEER_POSITION_TOLERANCE (0.2°) it parks (one stop(), no further commands); a parked
     * loop restarts only at |err| > STEER_RESTART_TOLERANCE. Stall (DRIVING only; window
     * restarts from rest or on a direction flip), move timeout (time continuously behind by
     * more than STEER_RESTART_TOLERANCE) and the range guard end in a coast and IDLE.
     */
    void update();

    /**
     * @brief Stop the motor and cancel any active move/jog
     */
    void stop();

    // === Steering interface ===

    /**
     * @brief Set steering by percentage: -100 (left limit) .. 0 (sensor zero) .. +100
     * (right limit), then clamped to ±STEER_LWS_SAFE_RANGE.
     *
     * Continuous tracking: while the loop is DRIVING or PARKED the call only replaces the
     * target (any delta) — the move timer, stall window, progress baseline, boost and
     * break-away state keep running, so the stall and timeout checks work under the 25 Hz
     * MAVLink stream (there is no re-command guard any more). From IDLE the stall latch is
     * checked; an accepted command cancels a jog and starts a drive from rest, or parks at
     * once (coast) when already within STEER_POSITION_TOLERANCE.
     * @return false if rejected (limits missing, sensor unhealthy, VESC down, stall latch from idle)
     */
    bool setSteeringPercent(float percent);
    float getSteeringPercent() const;

    /**
     * @brief Closed-loop move to an absolute angle in degrees (+ = right), for the web
     * "Target angle" control. Converted to 0.1° and sent through the same path as
     * setSteeringPercent() (commandTarget(): clamped to the captured limits and the safe
     * range, same gates, same stall latch, continuous tracking).
     * @return false if rejected (diag lock, not calibrated, sensor unhealthy, VESC down,
     * non-finite value, stall latch from idle)
     */
    bool setSteeringAngleDeg(float deg);

    /**
     * @brief Request a one-shot move to center once it becomes possible
     * (calibrated, sensor healthy, VESC link up). Issued from update();
     * dropped silently after STEER_BOOT_CENTER_TIMEOUT_MS, and cancelled by any
     * accepted steering/jog/nudge command, capture or zero-set made before it fires.
     */
    void requestBootCenter();

    // === USB steering diagnostics (SteerDiag) ===

    /**
     * @brief Take (true) / release (false) exclusive ownership for the USB diagnostics.
     * Both directions stop the motor and cancel any move, jog or pending boot centring.
     */
    void setDiagLock(bool on);
    bool isDiagLocked() const { return diagLock_; }

    /**
     * @brief Open-loop drive for the diagnostics: setSpeed(speed) with no duty floor, no stall
     * latch and no refresh timeout (the caller refreshes and bounds it). Drops the closed loop
     * and any jog to idle first. Needs the diag lock and the VESC link.
     * @return false if refused
     */
    bool diagDrive(int16_t speed);

    /**
     * @brief One closed-loop move to an absolute target (0.1°, firmware sign) under the diag
     * lock, through the same path as setSteeringPercent() (clamped to the limits and the safe
     * range, same gates, same stall latch).
     * @return false if refused
     */
    bool diagGoto(int32_t targetPos);

    /** @brief Last effort sent to the motor driver (-255..+255, 0 = coast) */
    int16_t getLastDriveCommand() const { return motor_.lastSpeed(); }

    /** @brief Closed-loop target, 0.1° (meaningful while the loop is active) */
    int32_t getTargetPos() const { return targetRel_; }

    /** @brief Closed-loop state */
    SteerPosState getPosState() const { return posState_; }

    /** @brief Closed-loop state token: idle, drive, park */
    static const char* posStateToken(SteerPosState s);

    // === Park correction (SteerParkLaw.h) ===

    /** @brief Result of a park setting change (the web layer maps it to its reply) */
    enum class ParkSetResult : uint8_t { OK, OUT_OF_RANGE, NOT_MONOTONIC, SAVE_FAILED };

    /** @brief How the last park correction ended (RAM only, "none" after boot) */
    enum class ParkOutcome : uint8_t { NONE, DONE, GAVE_UP, CANCELLED };

    /** @brief Last park correction result for telemetry */
    struct ParkLastResult {
        ParkOutcome outcome = ParkOutcome::NONE;
        uint8_t pulses = 0;      // pulses sent in that parking
        int32_t residual = 0;    // target − position at the end, 0.1°, signed
        uint8_t duty = 0;        // duty × ms of the last pulse
        uint16_t ms = 0;
        SteerParkNote note = SteerParkNote::NONE;   // why it ended (TOO_STRONG, OVERSHOOT_RESTART, …)
    };

    /**
     * @brief Park correction settings: validated, written to NVS "steering" first, applied only
     * after the write succeeded; any running correction is cancelled (a pulse coasts).
     * setParkRung(): rung 1..3, duty STEER_PARK_DUTY_MIN..MAX, ms STEER_PARK_MS_MIN..MAX, and the
     * ladder must stay non-decreasing (NOT_MONOTONIC otherwise). setParkDoneTol(): 0.1° units,
     * STEER_PARK_DONE_TOL_MIN..MAX. setParkMaxPulses(): STEER_PARK_MAX_PULSES_MIN..MAX.
     */
    ParkSetResult setParkEnabled(bool on);
    ParkSetResult setParkRung(uint8_t rung, int32_t duty, int32_t ms);
    ParkSetResult setParkDoneTol(int32_t tol);
    ParkSetResult setParkMaxPulses(int32_t count);

    bool isParkEnabled() const { return parkEnabled_; }
    /** @brief Ladder rung, index 0..2 */
    const SteerParkRung& getParkRung(uint8_t i) const { return parkParams_.rung[i < STEER_PARK_RUNGS ? i : 0]; }
    /** @brief Done tolerance, 0.1° */
    int32_t getParkDoneTol() const { return parkParams_.doneTol; }
    uint8_t getParkMaxPulses() const { return parkParams_.maxPulses; }
    const ParkLastResult& getParkLast() const { return parkLast_; }
    /** @brief Telemetry token: none, done, gave_up, cancelled */
    static const char* parkOutcomeToken(ParkOutcome o);

    // === Calibration ===

    /**
     * @brief Momentary open-loop jog: -1 = left, +1 = right, 0 = stop.
     * Needs only the VESC link (sensorless allowed). Auto-stops if not refreshed
     * within STEER_JOG_TIMEOUT_MS. Not limited by the range guard.
     * @param duty PWM duty 0-255; clamped up to STEER_PWM_MIN_DUTY
     */
    void jog(int8_t direction, uint8_t duty = STEER_JOG_DUTY);

    /**
     * @brief Precision nudge: one STEER_NUDGE_MS pulse at STEER_NUDGE_DUTY.
     * Same gating as jog().
     */
    void nudge(int8_t direction);

    // === Sensorless rate drive (RATE mode) ===

    /**
     * @brief Rate command from the autopilot stick (−100..+100, + = right), RATE mode only.
     * Duty = steerRateDuty(pct, RC_STEERING_DEADBAND, STEER_PWM_MIN_DUTY, STEER_PWM_MAX_DUTY),
     * driven as a RATE-owned jog refreshed by every call (stops after STEER_JOG_TIMEOUT_MS
     * without one). Duty 0 arms the path and stops a RATE-owned drive only. Ignored
     * (returns true) while a WEB jog/nudge is active.
     * @return false if refused (sensor healthy, VESC down, disarmed with nonzero duty,
     *         direction stall-latched)
     */
    bool setRateCommand(float pct);

    /** @brief Stop a RATE-owned drive at once; a WEB jog/nudge or a closed-loop move is untouched */
    void stopRate();

    /** @brief Disarm the rate path: nonzero rate commands are refused until a zero one */
    void disarmRate() { rateArmed_ = false; }

    /** @brief The rate path has seen a centred stick since the last disarmRate() */
    bool isRateArmed() const { return rateArmed_; }

    /** @brief Owner of the active jog (NONE when no jog/nudge/rate drive runs) */
    JogOwner getJogOwner() const { return jogDir_ != 0 ? jogOwner_ : JogOwner::NONE; }

    /**
     * @brief Store the current position as the left / right limit and save it at once.
     * Needs a healthy sensor and |pos| ≥ STEER_CAL_MIN_SPAN. A capture on the wrong side
     * toggles the sensor-direction flag while the opposite limit is unset (and clears it),
     * or is refused while it is stored (steerDirCapture()). A limit beyond ±STEER_LWS_SAFE_RANGE is accepted with a warning (isLastCaptureBeyondSafe()).
     * @return true if saved (else see getCaptureError())
     */
    bool captureLeftLimit();
    bool captureRightLimit();

    /** @brief Why the last capture was refused (nullptr if it was accepted) */
    const char* getCaptureError() const { return captureError_; }
    /** @brief Position stored by the last successful capture, 0.1° */
    int32_t getLastCapturePos() const { return lastCapturePos_; }
    /** @brief The last successful capture lies beyond ±STEER_LWS_SAFE_RANGE */
    bool isLastCaptureBeyondSafe() const { return lastCaptureBeyondSafe_; }
    /** @brief The last successful capture toggled the sensor-direction flag */
    bool isLastCaptureToggled() const { return lastCaptureToggled_; }

    /**
     * @brief Run the sensor zero-set: stop motion, cancel boot centering, ask the CAN
     * controller for 0x7C0 [05 00] / [53 30], erase both limits once the reset frame is
     * sent, and confirm within STEER_LWS_ZERO_CONFIRM_MS. No ignition/RPM/standstill
     * precondition. Progress: getZeroState()/getZeroError().
     * @return true if started (else getZeroRefusal(): "busy" or "can_unavailable")
     */
    bool requestSensorZero();
    const char* getZeroRefusal() const { return zeroRefusal_; }
    ZeroState getZeroState() const { return zeroState_; }
    /** @brief Failure reason of the last zero-set (nullptr unless FAILED) */
    const char* getZeroError() const { return zeroError_; }
    static const char* zeroStateToken(ZeroState s);

    /** @brief Load the limits from NVS, migrating older layouts (called by begin()). */
    void loadCalibration();

    // === State ===

    bool isCalibrated() const { return calibrated_; }

    /** @brief Sensor-direction flag: true = sensor mounted mirrored (NVS `inv`) */
    bool isSensorInverted() const { return inv_; }

    /** @brief Steering-angle sensor healthy (token `none` for the recovery streak) */
    bool isSensorOk() const { return sensorOk_; }

    /** @brief Current position, 0.1°, firmware sign (+ = right); meaningful while isSensorOk() */
    int32_t getPosition() const { return lws_.position(); }

    /** @brief Stored limits (0.1°, firmware sign), POS_UNSET if unset */
    int32_t getLeftLimit() const { return limLeft_; }
    int32_t getRightLimit() const { return limRight_; }

    /** @brief Either stored limit lies beyond ±STEER_LWS_SAFE_RANGE (near the ~600° latch) */
    bool isRangeWarning() const;

    /** @brief Fresh accepted frame with a valid angle: getAngleDeg() is meaningful */
    bool hasAngle() const;
    /** @brief Steering angle in degrees, firmware sign (+ = right) */
    float getAngleDeg() const { return (float)getPosition() / 10.0f; }
    /** @brief Fresh accepted frame with a valid speed byte: getRateDegS() is meaningful */
    bool hasRate() const;
    /** @brief Signed rotation rate, deg/s (+ = turning right) */
    int32_t getRateDegS() const { return lws_.rateDegS(); }

    /** @brief Current fault reason */
    SensorFault getSensorFault() const { return fault_; }

    /** @brief Telemetry token: none, zeroing, can_down, timeout, range_latched, sensor_fault, not_calibrated, angle_invalid */
    static const char* faultToken(SensorFault f) { return lwsFaultToken(f); }

    // === VESC driver telemetry (for the web/telemetry layer) ===

    /** @brief True while the steering motor driver link is healthy */
    bool isDriverOk() const { return steerDriverOk_; }

    /**
     * @brief The steering motor is being driven right now: closed-loop DRIVING, a jog / nudge /
     * rate drive, a park-correction pulse, or any nonzero effort on the driver (e.g. a USB diag
     * drive). The event journal defers its flash flush while this is true (at most
     * JOURNAL_FLUSH_DEFER_MAX_MS).
     */
    bool isDriving() const {
        return posState_ == SteerPosState::DRIVING || jogDir_ != 0 || parkPulseActive_ ||
               motor_.lastSpeed() != 0;
    }
    /** @brief Steering motor current (A) from the last VESC telemetry poll */
    float getMotorCurrent() const { return motor_.motorCurrentA(); }
    /** @brief VESC FET temperature (°C) from the last telemetry poll */
    float getFetTemp() const { return motor_.fetTempC(); }
    /** @brief VESC input voltage (V) from the last telemetry poll */
    float getInputVoltage() const { return motor_.inputVoltageV(); }
    /** @brief VESC fault code (0 = no fault) */
    uint8_t getVescFault() const { return motor_.faultCode(); }
    /** @brief Steering driver packet-link counters (all zero without a packet link) */
    IMotorDriver::LinkStats getLinkStats() const { return motor_.linkStats(); }

private:
    IMotorDriver& motor_;
    CANController* can_;
    LwsSensor lws_;

    // Sensor health
    bool sensorOk_;
    SensorFault fault_;
    bool firstFrameLogged_;      // raw sensor sign shown once at boot
    uint32_t lastStateLogMs_;    // rate limit for the while-moving debug log

    // Calibration (NVS v4)
    int32_t limLeft_;            // 0.1°, firmware sign, < 0; POS_UNSET = unset
    int32_t limRight_;           // 0.1°, firmware sign, > 0; POS_UNSET = unset
    bool calibrated_;
    bool inv_;                   // sensor mounted mirrored (RAM copy of NVS `inv`, pushed into lws_)

    // Capture bookkeeping (for the web response)
    const char* captureError_;
    int32_t lastCapturePos_;
    bool lastCaptureBeyondSafe_;
    bool lastCaptureToggled_;

    // Zero-set
    ZeroState zeroState_;
    const char* zeroError_;      // reason when FAILED
    const char* zeroRefusal_;    // reason of the last refused requestSensorZero()
    bool zeroLimitsErased_;      // limits erased for the running sequence

    // Position control state (0.1°, firmware sign). The loop "owns the target" while
    // posState_ is DRIVING or PARKED (loopActive()).
    int32_t targetRel_;
    SteerPosState posState_;
    SteerPosTimers posTimers_;   // move timer, stall window, boost, break-away (reset only from rest)

    // Park correction (sub-mode of PARKED). The settle clock restarts on a target move beyond
    // STEER_PARK_TARGET_STABLE (reference = that target) and on every entry into PARKED.
    SteerParkParams parkParams_;   // ladder, done tolerance, max pulses + compile-time timings
    bool parkEnabled_;
    SteerParkState parkState_;
    int32_t parkRefTarget_;        // target the stable band is measured from
    uint32_t parkSettleStartMs_;   // settle clock start
    bool parkPulseActive_;         // a pulse is being driven (ends with motor_.stop() at parkPulseEndMs_)
    uint32_t parkPulseEndMs_;
    // Correction disabled until the next stick command: the main loop restarted after a park
    // pulse (overshoot beyond STEER_RESTART_TOLERANCE). Cleared only by a target move beyond
    // STEER_PARK_TARGET_STABLE or a command from idle — never by RESTARTED / PARKED events,
    // otherwise pulse → restart → re-park → pulse loops forever (wheels shaking).
    bool parkBlocked_;
    ParkLastResult parkLast_;

    // Jog state
    int8_t jogDir_;
    uint32_t lastJogRefresh_;
    uint32_t jogPulseUntil_;    // nudge mode: absolute ms to stop at (0 = normal hold-jog)
    JogOwner jogOwner_;          // who started the active jog (WEB jog/nudge or RATE)
    bool rateArmed_;             // centre-to-arm: a zero rate command was seen since disarmRate()

    // Jog stall detection (only while the sensor is healthy; the closed loop keeps its own in posTimers_)
    int32_t lastStallPosition_;
    uint32_t lastStallCheckTime_;
    int8_t lastDriveDir_;        // sign of the last nonzero drive command (+1 right, -1 left)

    // Stall latch: after a stall-stop, further motion in the stalled direction is
    // refused for STEER_STALL_COOLDOWN_MS (opposite direction always allowed).
    bool stallLatched_;
    int8_t stallLatchDir_;       // direction that stalled (+1/-1)
    uint32_t stallLatchTime_;

    // VESC driver health / monitors
    bool steerDriverOk_;         // mirrors driver link health (false until first good reply)
    uint32_t linkDownSinceMs_;   // event journal: when the link went down (0 = boot)
    uint32_t linkDownSnapMs_;    // event journal: last link_down_snap line
    uint8_t lastFaultCode_;      // event journal: last journaled VESC fault code

    // USB diagnostics lock: external steering commands refused while true
    bool diagLock_;

    // One-shot boot centering (deferred until sensor + VESC link are up)
    bool bootCenterPending_;
    uint32_t bootCenterRequestMs_;

    /**
     * @brief Shared open-loop jog start/refresh (stall-latch check, lastDriveDir_,
     * lastJogRefresh_, setSpeed(d·duty)). Caller has checked the VESC link.
     * @param pulseMs 0 = hold-jog (refresh timeout), else nudge pulse length
     * @return false if refused by the stall latch
     */
    bool driveJog(int8_t d, uint8_t duty, JogOwner owner, uint32_t pulseMs);

    /** @brief A WEB-owned jog (refreshed within STEER_JOG_TIMEOUT_MS) or nudge (pulse running) is active */
    bool webJogActive(uint32_t now) const;

    /** @brief The closed loop owns the target (DRIVING or PARKED) */
    bool loopActive() const { return posState_ != SteerPosState::IDLE; }

    /** @brief Coast and drop the closed loop to IDLE (no-op when already idle) */
    void loopToIdle();

    /**
     * @brief Cancel the park correction: coast a running pulse, reset the law state. With
     * record, a cycle in progress that sent ≥ 1 pulse is stored as "cancelled" with residual.
     */
    void parkCancel(bool record = false, int32_t residual = 0);

    /** @brief Restart the park settle clock at the current target (new park entry / target move) */
    void parkRestartSettle(uint32_t now);

    /** @brief Run the park correction while PARKED (called from update()) */
    void serviceParkCorrection(uint32_t now, int32_t pos, int32_t err);

    /** @brief Load the park settings from an open "steering" Preferences session (missing → default) */
    void loadParkSettings(Preferences& prefs);

    /** @brief True if a new move in direction d must be refused due to the stall latch */
    bool latchBlocks(int8_t d, uint32_t now) const;

    /** @brief Stop + engage the stall latch in direction dir (shared stall-stop path) */
    void triggerStallStop(int8_t dir, const char* reason);

    /** @brief Poll the VESC driver, update steerDriverOk_, apply the fault-code and comm monitors.
     *  @return false if the driver is down (caller should skip driving this cycle) */
    bool serviceDriver(uint32_t now);

    /** @brief Evaluate the LWS health (D5); on a new fault stop a closed-loop move only */
    void refreshSensor(uint32_t now);

    /** @brief Advance a running zero-set (limit erase, CAN result, confirmation) */
    void serviceZero(uint32_t now);
    void finishZero(bool ok, const char* reason);

    /** @brief Issue the pending boot centering once possible, or drop it on timeout */
    void serviceBootCenter(uint32_t now, bool driverReady);

    /**
     * @brief Shared closed-loop command path (setSteeringPercent(), diagGoto()): clamp the
     * target to the limits and the safe range, then track it or start from idle.
     */
    bool commandTarget(int32_t target);

    /** @brief Shared capture path for the two limits */
    bool captureLimit(bool left);

    /** @brief Recompute calibrated_ from the limits */
    void revalidateCalibration();

    /**
     * @brief Persist one limit key (v4 layout). With a flag toggle, in the same session:
     * the limit, then `inv`, then remove the opposite key (oppositeKey, nullptr = none).
     */
    void saveLimit(const char* key, int32_t value, bool writeInv = false, const char* oppositeKey = nullptr);

    /** @brief Erase both limits (NVS + RAM); the sensor-direction flag is kept */
    void eraseLimits();

    /** @brief One debug line: angle, pos, token, fps (+ suffix) */
    void logState(const char* suffix);

    /** @brief Event journal: VESC link event with the parser snapshot (extra = trailing fields) */
    void journalLink(JLvl lvl, const char* event, const char* extra);
};

#endif // STEERING_CONTROLLER_H
