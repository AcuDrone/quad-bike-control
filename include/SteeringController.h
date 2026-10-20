#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "AS5600Sensor.h"
#include "IMotorDriver.h"

/**
 * @brief Steering actuator controller: drives an IMotorDriver (VESC over UART)
 * with proportional PWM and absolute MULTI-TURN position feedback from two
 * AS5600 magnetic angle sensors (aztec buggy, ±1.7 steering-shaft turns).
 *
 *  - FINE sensor on the steering shaft (I2C1 `Wire`, X14): one turn = 4096 counts.
 *    Its turns are tracked incrementally (a raw jump > 2048 counts is a wrap).
 *  - COARSE sensor on a geared-down shaft (I2C2 `Wire1`, X15) sweeping < 360°
 *    lock to lock: absolute but coarse. It is linearised with a "seam" placed in
 *    its unused arc and mapped onto the fine position through a line fitted at
 *    calibration (the ratio is derived, never stored).
 *
 * All positions are UNWRAPPED fine counts relative to the calibrated center
 * (int32): pos = fineRaw + 4096·turn − fineCenterRaw, in sensor direction; the
 * control loop uses rel = invert ? −pos : pos (positive = right).
 *
 * No homing: at boot (and after any sensor fault) the absolute turn is recovered
 * from the coarse sensor, accepted only after STEER_TURN_RECOVER_SAMPLES
 * consecutive coarse polls agree on it. While calibrated the two sensors are
 * cross-checked on every coarse sample; a persistent disagreement LATCHES a
 * mismatch fault that only a completed calibration or a reboot clears. Any fault on
 * either sensor (I2C failure, magnet lost, or fine/coarse mismatch) stops the
 * motor and rejects commands; there is no single-sensor degraded mode.
 *
 * Calibration (NVS namespace "steering", keys cal_ver/fc_raw/cc_raw/fl_pos/
 * cl_raw/fr_pos/cr_raw): the center, left and right captures may be made in ANY
 * order. Each capture records only its own point (unwrapped fine position in a
 * session frame counted from boot + coarse raw) into a session slot; nothing is
 * cleared by another capture. Once all three slots are set the calibration is
 * validated and, if accepted, becomes active and is persisted (only complete
 * calibrations are stored). The session frame needs fine-sensor continuity, so a
 * reboot or a sensor fault (fine/coarse fault, latched mismatch) discards the
 * slots of an unfinished calibration. When the stored calibration is complete
 * and its absolute turn is trusted, empty slots are pre-filled from it, so a
 * single point can be re-captured on an already calibrated vehicle.
 */
class SteeringController {
public:
    /** @brief Combined steering-sensor fault reason (telemetry token via faultToken()) */
    enum class SensorFault : uint8_t {
        NONE,
        FINE_I2C,
        FINE_MAGNET,
        COARSE_I2C,
        COARSE_MAGNET,
        MISMATCH,
        RECOVERING      // both sensors healthy and calibrated, absolute turn not yet recovered
    };

    /** @brief Sentinel for an unset limit position (positions may be negative) */
    static constexpr int32_t POS_UNSET = INT32_MIN;

    /**
     * @brief Construct bound to an already-constructed motor driver and the two
     * I2C buses (opened later by main.cpp).
     * The driver's own begin() (e.g. VESC UART init) must be called by the
     * caller before SteeringController::begin().
     * @param fineBus   Bus of the fine sensor (`Wire`, I2C1) — the sensor may restart it
     * @param coarseBus Bus of the coarse sensor (`Wire1`, I2C2) — never restarted here
     */
    SteeringController(IMotorDriver& motor, TwoWire& fineBus, TwoWire& coarseBus);

    /**
     * @brief Probe both AS5600 sensors, load the calibration (always, even if a
     * sensor is missing) and prime the sensor state, including turn recovery.
     * A sensor that does not answer is re-probed every STEER_SENSOR_REPROBE_MS.
     * @return true if both sensors answered (a silent VESC does not block boot;
     *         see driver-ok gating below)
     */
    bool begin(gpio_num_t fineSda, gpio_num_t fineScl, gpio_num_t coarseSda, gpio_num_t coarseScl);

    /**
     * @brief Update sensor readings and position control — call every loop iteration
     */
    void update();

    /**
     * @brief Stop the motor and cancel any active move/jog
     */
    void stop();

    // === Steering interface ===

    /**
     * @brief Set steering by percentage: -100 (left limit) .. 0 (center) .. +100 (right limit).
     * Left and right ranges may be asymmetric.
     * @return false if rejected (not calibrated or sensor invalid)
     */
    bool setSteeringPercent(float percent);
    float getSteeringPercent() const;

    /**
     * @brief Request a one-shot move to center once it becomes possible
     * (calibrated, combined sensor state OK, VESC link up). Issued from update();
     * dropped silently after STEER_BOOT_CENTER_TIMEOUT_MS, and cancelled by any
     * steering command or capture made before it fires.
     */
    void requestBootCenter();

    // === Calibration ===

    /**
     * @brief Momentary open-loop jog for calibration: -1 = left, +1 = right, 0 = stop.
     * Auto-stops if not refreshed within STEER_JOG_TIMEOUT_MS.
     * @param duty PWM duty 0-255; clamped up to STEER_PWM_MIN_DUTY (below it the actuator stalls)
     */
    void jog(int8_t direction, uint8_t duty = STEER_JOG_DUTY);

    /**
     * @brief Precision nudge: one STEER_NUDGE_MS pulse at STEER_NUDGE_DUTY,
     * moving a few counts per call. For landing exactly on center during calibration.
     */
    void nudge(int8_t direction);

    /** @brief Calibration point / session slot index */
    enum CalPoint : uint8_t { CAL_LEFT = 0, CAL_CENTER = 1, CAL_RIGHT = 2, CAL_POINTS = 3 };

    /**
     * @brief Record the center point (session fine position + coarse raw) into
     * its session slot. Requires both sensors healthy and a fresh fine sample (a
     * mismatch does not block: completing a calibration is the way out of it).
     * Must be at least STEER_CAL_MIN_SPAN away from an already recorded limit.
     * Clears nothing else. When all three slots are set the calibration is
     * finalized (validated, activated, persisted); a rejected finalization keeps
     * the slots and getCalError() names the failed check.
     * @return true if the point was recorded (else see getCaptureError())
     */
    bool captureCenter();

    /**
     * @brief Record a limit point into its session slot; same rules as
     * captureCenter(), and at least STEER_CAL_MIN_SPAN away from a recorded center.
     * @return true if the point was recorded (else see getCaptureError())
     */
    bool captureLeftLimit();
    bool captureRightLimit();

    /** @brief Why the last capture was refused (nullptr if it was accepted) */
    const char* getCaptureError() const { return captureError_; }

    /**
     * @brief Failed check of the last finalization attempt (all three points set
     * but rejected), or nullptr. Cleared by the next capture and when the
     * session is discarded.
     */
    const char* getCalError() const { return sessionError_; }

    /** @brief True while at least one session point is recorded (unfinished or rejected calibration) */
    bool isCalPending() const { return calSlot_[CAL_LEFT].valid || calSlot_[CAL_CENTER].valid || calSlot_[CAL_RIGHT].valid; }

    /** @brief True if session point p is recorded */
    bool hasCalPoint(CalPoint p) const { return p < CAL_POINTS && calSlot_[p].valid; }

    /** @brief True if unfinished session points were discarded by a sensor fault (cleared by the next capture) */
    bool isCalSessionLost() const { return calLost_; }

    /**
     * @brief Position shown to the operator: while session points are pending,
     * in the session display frame (see getDisplayPoint()); otherwise getPosition().
     */
    int32_t getDisplayPosition() const;

    /**
     * @brief Calibration point shown to the operator, in the same frame as
     * getDisplayPosition(). While session points are pending these are the
     * session points relative to the center point when it is recorded, otherwise
     * relative to the FIRST recorded point (so the first capture reads 0 and the
     * others read their distance from it). Without a pending session these are
     * the stored calibration (center = 0, limits from center).
     * @return false if that point is not set
     */
    bool getDisplayPoint(CalPoint p, int32_t& pos) const;

    /** @brief Fine position (display frame) / coarse raw recorded by the last successful capture */
    int32_t getLastCapturePos() const { return lastCapturePos_; }
    int32_t getLastCaptureCoarse() const { return lastCaptureCoarse_; }

    /**
     * @brief Load calibration from NVS (called by begin()).
     */
    void loadCalibration();

    // === State ===

    bool isCalibrated() const { return calibrated_; }

    /**
     * @brief Combined sensor health: both sensors healthy and, when calibrated,
     * the absolute turn recovered without a fine/coarse mismatch.
     */
    bool isSensorOk() const { return sensorOk_; }

    /** @brief True once the absolute turn has been recovered from the coarse sensor */
    bool isTurnRecovered() const { return turnRecovered_; }

    /** @brief True while a runtime mismatch is latched (cleared by a completed calibration or reboot) */
    bool isMismatchLatched() const { return mismatchLatched_; }

    /**
     * @brief Current unwrapped fine position in counts, SENSOR direction,
     * relative to the captured center (or to the boot-time fine sample when no
     * center is captured).
     */
    int32_t getPosition() const;

    /** @brief True when a center pair is captured (its position is 0 by definition) */
    bool hasCenter() const { return fineCenterRaw_ >= 0; }

    /** @brief Stored limit positions (fine counts from center, sensor direction), POS_UNSET if unset */
    int32_t getLeftLimit() const { return fineLeft_; }
    int32_t getRightLimit() const { return fineRight_; }

    /** @brief Last fine / coarse raw angle (0-4095), -1 while that sensor is faulted */
    int32_t getFineRaw() const { return fineOk_ ? fineRaw_ : -1; }
    int32_t getCoarseRaw() const { return coarseOk_ ? coarseRaw_ : -1; }

    /** @brief Turn index of the fine sensor relative to the center turn */
    int32_t getTurn() const { return turn_; }

    /** @brief Derived coarse counts per fine-shaft turn (signed), NAN when not calibrated */
    float getRatio() const { return calibrated_ ? ratio_ * 4096.0f : NAN; }

    /** @brief True when getResidual() is meaningful (calibrated, both sensors healthy) */
    bool hasResidual() const { return calibrated_ && fineOk_ && coarseOk_ && residualValid_; }

    /** @brief Last fine-vs-coarse disagreement in fine counts */
    int32_t getResidual() const { return residual_; }

    /** @brief Current combined fault reason */
    SensorFault getSensorFault() const { return fault_; }

    /** @brief Telemetry token for a fault reason: none, fine_i2c, fine_magnet, coarse_i2c, coarse_magnet, mismatch, recovering */
    static const char* faultToken(SensorFault f);

    /** @brief Consecutive failed transactions of the coarse sensor (main.cpp I2C2 recovery) */
    uint16_t getCoarseConsecutiveFailures() const { return coarse_.consecutiveFailures(); }

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
    AS5600Sensor fine_;         // steering shaft (I2C1, may restart its own bus)
    AS5600Sensor coarse_;       // geared-down shaft (I2C2, never restarts the bus)
    IMotorDriver& motor_;

    // Fine sensor state
    int32_t fineRaw_;           // last good fine raw angle (0-4095)
    bool fineOk_;               // debounced fine health
    bool fineFresh_;            // the most recent fine sample was good
    uint8_t fineBad_;           // consecutive bad fine samples (fault debounce)
    SensorFault fineReason_;    // FINE_I2C / FINE_MAGNET
    bool haveFine_;             // a previous good fine sample exists (turn tracking)
    int32_t prevFineRaw_;       // previous good fine sample (wrap carry)
    int32_t originRaw_;         // boot-time fine sample: position origin while no center is captured
    int32_t turn_;              // signed turn counter relative to the center turn
    int32_t sessionTurn_;       // fine wraps counted since boot (session frame, never recovered/reset by calibration)

    // Coarse sensor state
    int32_t coarseRaw_;         // last good coarse raw angle (0-4095)
    bool coarseOk_;             // debounced coarse health
    uint8_t coarseBad_;         // consecutive bad coarse samples (fault debounce)
    SensorFault coarseReason_;  // COARSE_I2C / COARSE_MAGNET
    uint32_t lastCoarsePollMs_;

    // Cross-check / turn recovery
    bool turnRecovered_;        // absolute turn known (recovered or defined by a capture)
    uint8_t recoverCount_;      // consecutive agreeing recovery samples (STEER_TURN_RECOVER_SAMPLES)
    int32_t recoverOffset_;     // candidate turn correction (k − turn_) shared by those samples
    bool mismatch_;             // recovery samples keep disagreeing (transient: clears when they agree)
    bool mismatchLatched_;      // runtime mismatch: cleared only by a completed calibration or reboot
    uint8_t mismatchCount_;     // consecutive mismatching coarse samples (debounce)
    int32_t residual_;          // last pos − expected(coarse), fine counts
    bool residualValid_;

    // Combined state
    bool sensorOk_;
    SensorFault fault_;

    // Calibration pairs (stored)
    int32_t fineCenterRaw_;     // 0-4095, -1 = unset
    int32_t coarseCenterRaw_;   // 0-4095, -1 = unset
    int32_t fineLeft_;          // fine counts from center (sensor dir), POS_UNSET = unset
    int32_t coarseLeft_;        // 0-4095, -1 = unset
    int32_t fineRight_;
    int32_t coarseRight_;

    // Calibration session slots (RAM only): points recorded this power cycle, in
    // the session frame sessionPos = fineRaw + 4096·sessionTurn.
    struct CalSlot {
        bool valid;
        int32_t sessionPos;
        int32_t coarseRaw;
    };
    CalSlot calSlot_[CAL_POINTS];
    int8_t calFirst_;           // first recorded slot (display reference without a center), -1 = none
    bool calLost_;              // unfinished points were discarded by a sensor fault
    const char* sessionError_;  // failed check of the last finalization attempt (nullptr = none)

    // Calibration (derived by revalidateCalibration, never stored)
    bool calibrated_;
    bool invert_;               // sensor counts decrease when the wheel moves right
    int32_t relLeft_;           // normalized left limit relative to center (< 0)
    int32_t relRight_;          // normalized right limit relative to center (> 0)
    int32_t seam_;              // coarse raw value in the middle of the unused arc
    float ratio_;               // coarse counts per fine count (signed)
    int32_t linLeft_;           // seam-linearised coarse value at the left limit
    const char* calError_;      // failed check of the ACTIVE calibration (nullptr = valid/incomplete)

    // Capture bookkeeping (for the web response)
    const char* captureError_;
    int32_t lastCapturePos_;
    int32_t lastCaptureCoarse_;

    // Position control state (normalized counts relative to center)
    int32_t targetRel_;
    bool isMoving_;
    uint32_t moveStartTime_;
    uint32_t lastMoveLogTime_;   // rate limit for the while-moving debug log

    // Anti-stall boost (extra duty above the P term while progress stalls under load)
    int16_t dutyBoost_;
    int32_t lastProgressPos_;
    uint32_t lastProgressTime_;

    // Jog state
    int8_t jogDir_;
    uint32_t lastJogRefresh_;
    uint32_t jogPulseUntil_;    // nudge mode: absolute ms to stop at (0 = normal hold-jog)

    // Stall detection
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

    // One-shot boot centering (deferred until sensors + VESC link are up)
    bool bootCenterPending_;
    uint32_t bootCenterRequestMs_;

    /** @brief Position normalized so that positive = right (control frame) */
    int32_t relPosition() const;

    /** @brief True if a new move in direction d must be refused due to the stall latch */
    bool latchBlocks(int8_t d, uint32_t now) const;

    /** @brief Stop + engage the stall latch in direction dir (shared stall-stop path) */
    void triggerStallStop(int8_t dir, const char* reason);

    /** @brief Poll the VESC driver, update steerDriverOk_, apply the fault-code and comm monitors.
     *  @return false if the driver is down (caller should skip driving this cycle) */
    bool serviceDriver(uint32_t now);

    /**
     * @brief Sample the fine sensor (every call) and the coarse sensor (every
     * STEER_COARSE_POLL_MS, or now if forceCoarse), update the combined state and
     * stop the motor on fault.
     * @return false while the combined sensor state is faulted
     */
    bool refreshSensors(uint32_t now, bool forceCoarse = false);

    /** @brief One fine sample: debounce + incremental turn tracking */
    void sampleFine();

    /** @brief One coarse sample: debounce + turn recovery / consistency check */
    void sampleCoarse();

    /** @brief Turn recovery (D4) or runtime residual check (D5) on a fresh coarse sample */
    void crossCheck();

    /** @brief Forget a partially agreed turn recovery (any poll that does not agree) */
    void resetRecovery() { recoverCount_ = 0; }

    /** @brief Issue the pending boot centering once possible, or drop it on timeout */
    void serviceBootCenter(uint32_t now, bool ready);

    /** @brief Recompute sensorOk_ / fault_ */
    void updateCombinedState();

    /** @brief Seam-linearised coarse value: (c − seam) mod 4096 */
    int32_t coarseLin(int32_t c) const { return (c - seam_) & 4095; }

    /** @brief Expected fine position (sensor dir, from center) for a coarse raw value */
    float expectedFine(int32_t coarseRaw) const;

    /** @brief Shared capture path for the three calibration points */
    bool capturePoint(CalPoint p);

    /** @brief Current position in the session frame: fineRaw + 4096·sessionTurn */
    int32_t sessionPos() const { return fineRaw_ + 4096 * sessionTurn_; }

    /** @brief Session-frame reference for display: center slot, else first slot */
    int32_t displayRef() const;

    /** @brief Fill empty slots from the stored calibration when its absolute turn is trusted */
    void prefillFromCalibration();

    /** @brief Try to turn three valid slots into the active calibration; false + sessionError_ if rejected */
    bool finalizeCalibration();

    /** @brief Drop all session slots; byFault marks the loss for the UI (only if any slot existed) */
    void discardCalSession(const char* why, bool byFault);

    /** @brief Recompute calibrated_/invert_/relLeft_/relRight_/seam_/ratio_ and calError_ */
    void revalidateCalibration();

    /** @brief Persist the calibration pairs to NVS (v2 layout) */
    void saveCalibration();
};

#endif // STEERING_CONTROLLER_H
