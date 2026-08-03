#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "AS5600Sensor.h"
#include "IMotorDriver.h"

/**
 * @brief Steering actuator controller: drives an IMotorDriver (VESC over UART)
 * with proportional PWM, absolute position feedback from an AS5600 magnetic
 * angle sensor.
 *
 * All positions are AS5600 raw counts (0-4095, ~0.088°/LSB). Position control
 * works in wrap-safe counts relative to the calibrated center; center and
 * left/right travel limits are captured at the current wheel position via the
 * web UI and persisted in NVS ("steering"/"c_ang","l_ang","r_ang").
 *
 * No homing: position is absolute. On sensor fault (I2C failure or magnet
 * lost) the motor stops immediately and position commands are ignored until
 * valid readings return.
 */
class SteeringController {
public:
    /**
     * @brief Construct bound to an already-constructed motor driver.
     * The driver's own begin() (e.g. VESC UART init) must be called by the
     * caller before SteeringController::begin().
     */
    explicit SteeringController(IMotorDriver& motor);

    /**
     * @brief Initialize the AS5600 sensor and prime driver state.
     * @return true if the sensor initializes (a silent VESC does not block boot;
     *         see driver-ok gating below)
     */
    bool begin(gpio_num_t sdaPin, gpio_num_t sclPin);

    /**
     * @brief Update sensor reading and position control — call every loop iteration
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

    /**
     * @brief Capture the current AS5600 angle as center / left limit / right limit
     * and persist to NVS. Limits are validated against center (opposite sides,
     * minimum STEER_CAL_MIN_SPAN counts).
     * @return true if captured and saved
     */
    bool captureCenter();
    bool captureLeftLimit();
    bool captureRightLimit();

    /**
     * @brief Load calibration from NVS. Call at init.
     */
    void loadCalibration();

    // === State ===

    bool isCalibrated() const { return calibrated_; }
    bool isSensorOk() const { return sensorOk_; }

    /** @brief Last AS5600 raw angle reading (0-4095) */
    int32_t getRawAngle() const { return rawAngle_; }

    /** @brief Calibrated raw angles (-1 if not set) */
    int32_t getCenter() const { return center_; }
    int32_t getLeftLimit() const { return leftLimit_; }
    int32_t getRightLimit() const { return rightLimit_; }

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
    AS5600Sensor sensor_;
    IMotorDriver& motor_;

    // Sensor state
    int32_t rawAngle_;          // last valid raw reading (0-4095)
    bool sensorOk_;
    uint8_t badSampleCount_;    // consecutive bad samples (fault debounce)

    // Calibration (raw angles, -1 = not set)
    int32_t center_;
    int32_t leftLimit_;
    int32_t rightLimit_;
    bool calibrated_;
    bool invert_;               // sensor counts decrease when the wheel moves right
    int32_t relLeft_;           // normalized left limit relative to center (< 0)
    int32_t relRight_;          // normalized right limit relative to center (> 0)

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
    uint32_t overCurrentStart_;  // millis() when motor current first crossed the threshold (0 = not over)

    /** @brief Signed shortest delta from center, sensor-direction normalized (positive = right) */
    int32_t relPosition() const;

    /** @brief True if a new move in direction d must be refused due to the stall latch */
    bool latchBlocks(int8_t d, uint32_t now) const;

    /** @brief Stop + engage the stall latch in direction dir (shared stall-stop path) */
    void triggerStallStop(int8_t dir, const char* reason);

    /** @brief Poll the VESC driver, update steerDriverOk_, apply fault/over-current/comm monitors.
     *  @return false if the driver is down (caller should skip driving this cycle) */
    bool serviceDriver(uint32_t now);

    /** @brief Signed shortest delta between two raw angles: ((a - b + 2048) & 4095) - 2048 */
    static int32_t wrapDelta(int32_t a, int32_t b);

    /** @brief Refresh sensor reading + magnet status (single I2C burst); stop the motor on fault */
    bool refreshSensor();

    /** @brief Recompute calibrated_/invert_/relLeft_/relRight_ from stored angles */
    void revalidateCalibration();

    /** @brief Persist calibration angles to NVS */
    void saveCalibration();
};

#endif // STEERING_CONTROLLER_H
