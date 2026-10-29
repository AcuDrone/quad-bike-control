#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "IMotorDriver.h"
#include "LwsSensor.h"
#include "SteerRate.h"
#include "SteerSensorDir.h"

class CANController;

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
 *  - Range guard: percent-driven (closed-loop) targets are clamped to the limits, then to
 *    ±STEER_LWS_SAFE_RANGE, and a closed-loop move driving outward at |pos| ≥ that range
 *    is stopped. Jog, nudge and captures are not guarded.
 *  - Steering modes (selected by VehicleController, see SteerRate.h): POSITION while the
 *    sensor is healthy (closed loop, setSteeringPercent()); RATE while it is not, the
 *    source is MAVLink and the autopilot is in MANUAL/unknown; BLOCKED otherwise.
 *  - Rate drive (RATE mode only): setRateCommand(pct) is a continuous jog with variable
 *    duty (steerRateDuty(): deadband → 0, else 140..255 quadratic). Refused while the
 *    sensor is healthy, the VESC is down, or the path is disarmed (centre-to-arm: one
 *    zero-duty command arms it after disarmRate()). It reuses the jog refresh timeout
 *    (STEER_JOG_TIMEOUT_MS), the stall latch and the VESC fault / comm stops; position
 *    stall detection, boost, move timeout and range guard never apply to it.
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
     * @return false if rejected (limits missing, sensor unhealthy, VESC down, stall latch)
     */
    bool setSteeringPercent(float percent);
    float getSteeringPercent() const;

    /**
     * @brief Request a one-shot move to center once it becomes possible
     * (calibrated, sensor healthy, VESC link up). Issued from update();
     * dropped silently after STEER_BOOT_CENTER_TIMEOUT_MS, and cancelled by any
     * accepted steering/jog/nudge command, capture or zero-set made before it fires.
     */
    void requestBootCenter();

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
    /** @brief Steering motor current (A) from the last VESC telemetry poll */
    float getMotorCurrent() const { return motor_.motorCurrentA(); }
    /** @brief VESC FET temperature (°C) from the last telemetry poll */
    float getFetTemp() const { return motor_.fetTempC(); }
    /** @brief VESC input voltage (V) from the last telemetry poll */
    float getInputVoltage() const { return motor_.inputVoltageV(); }
    /** @brief VESC fault code (0 = no fault) */
    uint8_t getVescFault() const { return motor_.faultCode(); }

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

    // Position control state (0.1°, firmware sign)
    int32_t targetRel_;
    bool isMoving_;
    uint32_t moveStartTime_;

    // Anti-stall boost (extra duty above the P term while progress stalls under load)
    int16_t dutyBoost_;
    int32_t lastProgressPos_;
    uint32_t lastProgressTime_;

    // Jog state
    int8_t jogDir_;
    uint32_t lastJogRefresh_;
    uint32_t jogPulseUntil_;    // nudge mode: absolute ms to stop at (0 = normal hold-jog)
    JogOwner jogOwner_;          // who started the active jog (WEB jog/nudge or RATE)
    bool rateArmed_;             // centre-to-arm: a zero rate command was seen since disarmRate()

    // Stall detection (only while the sensor is healthy)
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
};

#endif // STEERING_CONTROLLER_H
