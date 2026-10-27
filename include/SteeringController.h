#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "IMotorDriver.h"
#include "LwsSensor.h"

class CANController;

/**
 * @brief Steering actuator controller: drives an IMotorDriver (VESC over UART)
 * with proportional PWM and absolute multi-turn position feedback from a Bosch LWS
 * steering-angle sensor 0 265 005 486 mounted directly on the steering shaft and
 * read over the shared MCP2515 CAN bus (frame 0x305, 100 Hz).
 *
 *  - Position = −angle (the sensor's positive is LEFT; the firmware's positive is
 *    RIGHT), in 0.1° of steering-shaft angle (int32), relative to the sensor's own
 *    stored zero. No homing, no turn tracking: every frame is absolute.
 *  - Health (fault token, design D5): zeroing > can_down > timeout > range_latched >
 *    sensor_fault > not_calibrated > angle_invalid > none. Faults take effect at once;
 *    recovery needs STEER_LWS_RECOVER_FRAMES good frames. A fault stops a closed-loop
 *    move and rejects percent commands; jog and nudge keep working (sensorless, open
 *    loop, no stall backstop) — they only need the VESC link and no stall latch.
 *  - Calibration: center = sensor zero (set by requestSensorZero()); NVS namespace
 *    "steering" v3 keeps only lim_l / lim_r (0.1°, firmware sign). Each capture is saved
 *    at once; calibrated = both limits stored and valid.
 *  - Range guard: percent-driven (closed-loop) targets are clamped to the limits, then to
 *    ±STEER_LWS_SAFE_RANGE, and a closed-loop move driving outward at |pos| ≥ that range
 *    is stopped. Jog, nudge and captures are not guarded.
 */
class SteeringController {
public:
    /** @brief Steering-angle sensor fault reason (telemetry token via faultToken()) */
    using SensorFault = LwsFault;

    /** @brief Sentinel for an unset limit position (positions may be negative) */
    static constexpr int32_t POS_UNSET = INT32_MIN;

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

    /**
     * @brief Store the current position as the left / right limit and save it at once.
     * Needs a healthy sensor; left must be ≤ −STEER_CAL_MIN_SPAN, right ≥ +STEER_CAL_MIN_SPAN.
     * A limit beyond ±STEER_LWS_SAFE_RANGE is accepted with a warning (isLastCaptureBeyondSafe()).
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

    // Calibration (NVS v3)
    int32_t limLeft_;            // 0.1°, firmware sign, < 0; POS_UNSET = unset
    int32_t limRight_;           // 0.1°, firmware sign, > 0; POS_UNSET = unset
    bool calibrated_;

    // Capture bookkeeping (for the web response)
    const char* captureError_;
    int32_t lastCapturePos_;
    bool lastCaptureBeyondSafe_;

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

    /** @brief Persist one limit key (v3 layout) */
    void saveLimit(const char* key, int32_t value);

    /** @brief Erase both limits (NVS + RAM) */
    void eraseLimits();

    /** @brief One debug line: angle, pos, token, fps (+ suffix) */
    void logState(const char* suffix);
};

#endif // STEERING_CONTROLLER_H
