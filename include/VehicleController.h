#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "ServoController.h"
#include "ThrottleController.h"
#include "SteeringController.h"
#include "BTS7960Controller.h"
#include "TransmissionController.h"
#include "WebPortal.h"
#include "MavlinkInterface.h"
#include "RelayController.h"
#include "CANController.h"
#include "SpeedSensor.h"
#include "EngineHourMeter.h"

/**
 * @brief Vehicle control coordination layer
 *
 * Handles input source priority (MAVLINK > WEB > FAILSAFE), processes commands,
 * and coordinates all vehicle actuators (steering, throttle, brake, transmission).
 */
class VehicleController {
public:
    VehicleController(SteeringController& steering,
                      ThrottleController& throttle,
                      TransmissionController& transmission,
                      BTS7960Controller& brake,
                      MavlinkInterface& mavlink,
                      RelayController& relayController,
                      SpeedSensor& speedSensor);

    /**
     * @brief Load the engine hour meter from NVS. Call once from setup().
     * Cannot live in the constructor: this controller is a global built before NVS is ready.
     */
    void initEngineHourMeter();

    /**
     * @brief Initialize CAN controller
     * @return true if initialization successful
     */
    bool initCAN();

    /**
     * @brief Update control loop - call every loop iteration
     */
    void update();

    /**
     * @brief Set current input source
     * @param source Input source (MAVLINK, WEB, or FAILSAFE)
     */
    void setInputSource(InputSource source);

    /**
     * @brief Get current input source
     * @return Current input source
     */
    InputSource getInputSource() const { return currentInputSource_; }

    /**
     * @brief Latched web-control override. When engaged, MAVLink commands are ignored and
     * all control comes from the web portal. Not persisted (boots false = MAVLink in charge).
     * Engaging snaps to a safe state (throttle idle, hold steering/gear/brake).
     */
    void setWebControl(bool on);
    bool isWebControl() const { return webControl_; }

    /**
     * @brief Process web command from web portal
     * @param cmd Web command structure
     * @param webPortal Reference to web portal for sending responses
     */
    void processWebCommand(const WebPortal::WebCommand& cmd, WebPortal& webPortal);

    /**
     * @brief Get current gear as string (R/N/L/H)
     * @return Gear string
     */
    String getCurrentGearString() const;

    /**
     * @brief Direction of travel implied by the gearbox: +1 forward, -1 reverse, 0 unknown.
     *
     * Signs the (unsigned) wheel-speed reading for the MAVLink external-navigation
     * velocity. Derived from the PHYSICAL gear (the gear switches), never from the
     * assumed/commanded one: the transmission command path is sensorless and time-based,
     * and an assumption must never sign a measurement the autopilot's EKF will fuse.
     * An ambiguous or faulted reading yields 0, which suppresses the sample.
     * @return +1 (LOW/HIGH), -1 (REVERSE) or 0 (NEUTRAL/UNKNOWN)
     */
    int8_t getTravelDirection() const;

    /**
     * @brief Get the current target/step gear as string (R/N/L/H).
     * During a multi-step sequence this is the gear currently being moved to.
     * @return Gear string
     */
    String getTargetGearString() const;

    /**
     * @brief Get the gear the current step is moving away from, as string (R/N/L/H).
     * @return Gear string
     */
    String getFromGearString() const;

    /**
     * @brief Get steering percentage
     * @return Steering percentage (-100 to +100)
     */
    float getSteeringPercent() const { return steering_.getSteeringPercent(); }

    /**
     * @brief Get steering controller reference
     */
    SteeringController& getSteering() { return steering_; }

    /**
     * @brief Get the calibrated steering center (AS5600 raw counts, -1 if unset)
     */
    int32_t getSteerCenter() const { return steering_.getCenter(); }

    /**
     * @brief Steering controller (const access for telemetry)
     */
    const SteeringController& getSteering() const { return steering_; }

    /**
     * @brief Get current throttle servo pulse width (µs)
     */
    uint16_t getThrottleUs() const { return throttle_.getCurrentUs(); }

    /**
     * @brief Get the commanded throttle as a percent of the calibrated window (0-100).
     * Derived from the servo's live pulse width, so this is the ARBITRATED output —
     * whatever actually won among autopilot command, web command, gear-change boost
     * override, speed-limit cap and fail-safe idle — not any single input's demand.
     */
    uint8_t getThrottlePercent() const { return throttle_.usToPercent(throttle_.getCurrentUs()); }

    /**
     * @brief Get calibrated throttle idle/full endpoints (µs) and calibration state
     */
    uint16_t getThrottleIdleUs() const { return throttle_.getIdleUs(); }
    uint16_t getThrottleFullUs() const { return throttle_.getFullUs(); }
    bool isThrottleCalibrating() const { return throttle_.isCalibrating(); }

    /**
     * @brief Get transmission controller reference
     * @return Reference to transmission controller
     */
    TransmissionController& getTransmission() { return transmission_; }
    const TransmissionController& getTransmission() const { return transmission_; }

    /**
     * @brief Check brake sensor state
     * @return true if brake is released (HIGH signal, no pressure)
     */
    bool isBrakeReleased() const { return digitalRead(PIN_BRAKE_SENSOR); }

    /**
     * @brief Get vehicle data from CAN bus
     * @return VehicleData structure with engine RPM, speed, temperatures, etc.
     */
    CANController::VehicleData getVehicleData() const { return canController_.getVehicleData(); }

    /**
     * @brief Get the latest ECU capability probe snapshot (for telemetry)
     */
    CANController::ProbeResults getProbeResults() const { return canController_.getProbeResults(); }

    /**
     * @brief Hall-sensor vehicle speed (m/s) and its health flag.
     * Independent of CAN status — a CAN outage does not blank these.
     */
    float getVehicleSpeedMs() const { return speedSensor_.getSpeedMs(); }
    bool isVehicleSpeedValid() const { return speedSensor_.isValid(); }

    /** @brief Speed-sensor calibration (telemetry / web UI) */
    uint16_t getSpeedPulsesPerRev() const { return speedSensor_.getPulsesPerRev(); }
    float getSpeedWheelCircumferenceMm() const { return speedSensor_.getWheelCircumferenceMm(); }

    /**
     * @brief Distance counters in km (telemetry / MAVLink). READ-ONLY.
     * Always valid — distance already driven depends on neither CAN health nor the current
     * speed reading's validity. The odometer is a vehicle-lifetime counter with no reset path;
     * the trip counter is cleared only by an acknowledged MAVLink trip-reset command.
     */
    float getOdoKm() const { return speedSensor_.getOdoKm(); }
    float getTripKm() const { return speedSensor_.getTripKm(); }

    /**
     * @brief Engine hour meter ("мотогодини"). READ-ONLY.
     * Always valid: the TOTAL only ever increases and has NO reset path on any interface
     * (no command, no web control, no constant — only an NVS erase); the TRIP total
     * ("мотогодини місії") counts in lockstep with it and is zeroed only by the SAME
     * latched trip reset that clears the trip DISTANCE. Both stop growing rather than going
     * unknown while CAN data is invalid.
     */
    uint64_t getEngineSeconds() const { return engineHours_.getEngineSeconds(); }
    float getEngineHours() const { return engineHours_.getEngineHours(); }
    uint64_t getEngineTripSeconds() const { return engineHours_.getTripSeconds(); }
    float getEngineTripHours() const { return engineHours_.getTripHours(); }

    /**
     * @brief The ceiling the limiter is enforcing, in m/s. 0 = NO LIMIT.
     *
     * There is exactly one source: the autopilot's `SPEED_MAX` parameter, held in RAM only
     * (never persisted — there is no local ceiling any more). When no usable value is
     * available — never received, zero, out of range, stale, or the link is down — this
     * returns 0 and the limiter does nothing, exactly as ArduPilot reads `SPEED_MAX = 0`.
     */
    float getSpeedLimitMs() const;

    /**
     * @brief Throttle ceiling the proportional taper is currently applying (%, 100 = inactive)
     */
    float getSpeedLimitCeilingPct() const { return limiterCeilingPct_; }

    /**
     * @brief Where the steering speed-scaling base in use came from.
     * Reported to the portal so "the steering feels weak" can be diagnosed without a console.
     */
    enum class SteerScaleSource : uint8_t {
        NONE = 0,       // no base at all — not scaling
        LOCAL = 1,      // the ESP32's own NVS value (steer_sca_base)
        AUTOPILOT = 2   // the autopilot's MOT_SPD_SCA_BASE
    };

    /**
     * @brief The steering speed-scaling base in use, m/s. 0 = none, i.e. no scaling.
     *
     * Local NVS value first, the autopilot's `MOT_SPD_SCA_BASE` second, nothing third. The
     * priority is fixed and one-directional: there is no arbitration and no compile-time
     * default, because a base nobody chose would silently restrict the steering of an
     * unconfigured vehicle.
     */
    float getSteerScaleBaseMs() const;

    /** @brief Source of the base returned by getSteerScaleBaseMs(). */
    SteerScaleSource getSteerScaleSource() const;

    /**
     * @brief The scale actually applied to autopilot steering commands after rate limiting,
     * 0..1. ALWAYS finite and never NaN — 1.0 is the definite answer "not scaling".
     */
    float getSteerScale() const { return steerScaleApplied_; }

    /**
     * @brief Live bench test-speed override in m/s, 0 when none is in force.
     * RAM only, never persisted, and it reaches NOTHING but the steering-scale calculation.
     */
    float getSteerTestSpeedMs() const;

    /**
     * @brief Set ignition state with safety interlocks
     * @param state Ignition state string (OFF/ACC/IGNITION/START)
     * @param errorMsg Output parameter for error message if operation fails
     * @return true if ignition state changed successfully, false if rejected by safety interlock
     */
    bool setIgnitionState(const String& state, String& errorMsg);

    /**
     * @brief Get current ignition state
     * @return Current ignition state (OFF/ACC/IGNITION/CRANKING)
     */
    RelayController::IgnitionState getIgnitionState() const { return relayController_.getIgnitionState(); }

    /**
     * @brief Set front light state
     * @param on true to turn light on, false to turn off
     */
    void setFrontLight(bool on);

    /**
     * @brief Get current front light state
     * @return true if light is on, false if off
     */
    bool getFrontLight() const { return relayController_.getFrontLight(); }

    /**
     * @brief Set front-wheel lock state
     * @param locked true to engage the lock, false to release
     */
    void setWheelLock(bool locked);

    /**
     * @brief Get current front-wheel lock state
     * @return true if locked
     */
    bool getWheelLock() const { return relayController_.getWheelLock(); }

    /**
     * @brief Get current boost RPM target (NVS-backed, runtime-editable)
     */
    int32_t getBoostTargetRpm() const { return boostTargetRpm_; }

    /**
     * @brief Check if engine is running based on CAN RPM data
     * @return true if CAN data is valid and RPM >= 1000
     */
    bool isEngineRunning() const {
        CANController::VehicleData d = canController_.getVehicleData();
        return d.dataValid && d.engineRPM >= ENGINE_RUNNING_RPM_THRESHOLD;
    }

private:
    // Actuator references
    SteeringController& steering_;
    ThrottleController& throttle_;
    TransmissionController& transmission_;
    BTS7960Controller& brake_;

    // Input and output references
    MavlinkInterface& mavlink_;
    RelayController& relayController_;
    SpeedSensor& speedSensor_;     // hall speed sensor (updated from main.cpp)
    EngineHourMeter engineHours_;  // OWNED by value: a counter, not a shared device
    CANController canController_;  // CAN bus controller (owned, not reference)

    // State tracking
    InputSource currentInputSource_;
    bool failsafeApplied_;
    bool webControl_;   // latched web-control override (MAVLink ignored while true)

    // Brake actuator tracking
    float currentBrakeTarget_;          // Current brake percentage target (0-100)
    float currentBrakePosition_;        // Estimated brake position (0-100)
    uint32_t brakeMovementStartTime_;   // Time when brake started moving
    uint32_t lastBrakeUpdateTime_;      // Last position update timestamp
    uint32_t brakeSensorTriggerTime_;   // Time when brake sensor detected release (0 = not triggered)
    bool brakeIsMoving_;

    // Gear boost configuration (NVS-backed, runtime-editable)
    int32_t boostTargetRpm_;            // RPM target for gear boost PID (default TRANS_GEAR_BOOST_TARGET_RPM)

    // Gear boost PID state
    bool gearBoostActive_;              // True while PID is holding RPM during gear change
    bool boostManualActive_;            // True when boost is triggered manually via web for testing
    uint32_t gearBoostStartTime_;       // millis() when boost activated (for timeout)
    uint32_t pidLastUpdateTime_;        // millis() of last PID compute (for dt)
    uint32_t lastCanUpdateTime_;        // CAN lastUpdateTime seen on previous PID iteration
    float pidIntegral_;                 // Accumulated integral term
    float pidPrevError_;                // Previous error (for derivative)
    uint16_t lastPIDThrottleUs_;        // Last PID-computed throttle µs (held on CAN stale)

    // Ignition state tracking
    MavlinkInterface::IgnitionState previousIgnitionState_;  // Track previous state for transition detection
    TransmissionController::Gear lastCommandedGear_;         // Last gear requested via MAVLink (dedup guard)

    bool transmissionInitialized_;  // True after first engine-running restore

    uint32_t lastSpeedLimitWarnMs_; // rate limit for the "limiter armed, speed invalid" log

    // Limiter ceiling tracking (log on change only)
    float    lastSpeedLimitMs_;     // NAN sentinel = nothing logged yet
    uint32_t lastSpeedLimitLogMs_;

    // Proportional taper state. The ceiling is what gets rate-limited, never the demand.
    float    limiterCeilingPct_;    // currently applied throttle ceiling (%, 100 = inactive)
    uint32_t limiterLastMs_;        // millis() of the last taper evaluation (0 = first call)

    // Steering speed scaling. `steerScaBaseMs_` is the LOCAL (NVS-backed) base in m/s; 0 means
    // "not set", which falls through to the autopilot's MOT_SPD_SCA_BASE and then to no scaling
    // at all. The applied scale is what gets rate-limited, never the steering command.
    float    steerScaBaseMs_;       // local base from NVS "steering"/steer_sca_base (0 = not set)
    float    steerScaleApplied_;    // scale actually applied, 0..1 (1 = not scaling) — never NaN
    uint32_t steerScaleLastMs_;     // millis() of the last slew evaluation (0 = first call)
    uint32_t lastSteerScaleWarnMs_; // rate limit for the "no base" / "speed invalid" warnings
    float    lastSteerScaleLogged_; // NAN sentinel = nothing logged yet
    uint32_t lastSteerScaleLogMs_;

    // Bench test-speed override. RAM ONLY, never persisted, cleared by a reboot and by its own
    // STEER_SCALE_TEST_SPEED_MS fuse. It substitutes a speed for the SCALING CALCULATION ONLY —
    // the odometry, VFR_HUD, the SPEED_MAX limiter, the transmission interlock and the odometer
    // all keep using the real measured speed, so a bench setting cannot become a driving one.
    float    steerTestSpeedMs_;     // 0 = no override in force
    uint32_t steerTestSpeedSetMs_;  // millis() the override was set (0 = none)

    // Standing web throttle demand (%), re-limited every loop so a vehicle accelerating past
    // the ceiling on an unchanged web command is still clamped. Reset to 0 on every path that
    // idles the throttle — a demand that outlives an idle command would reopen the throttle.
    float    webThrottleDemandPct_;

    /**
     * @brief Apply fail-safe commands (center steering, idle throttle, stop actuators)
     */
    void applyFailsafe();

    /**
     * @brief Process MAVLink commands from the autopilot
     */
    void processMavlinkCommands();

    /**
     * @brief Apply brake control with percentage (0-100%)
     * @param brakePct Brake percentage (0 = released, 100 = fully applied)
     */
    void applyBrake(float brakePct);

    /**
     * @brief Update brake actuator control and tracking
     */
    void updateBrakeControl();

    /**
     * @brief PID-controlled RPM hold during gear changes
     * Overrides MAVLink/web throttle while gear change is active.
     */
    void updateGearBoostPID();

    /**
     * @brief Process gear change command
     * @param gearStr Gear string ("R", "N", "L", "H")
     * @param webPortal Reference to web portal for sending responses
     */
    void processGearCommand(const String& gearStr, WebPortal& webPortal);

    /**
     * @brief Process steering command
     * @param value Steering percentage (-100 to +100)
     * @param webPortal Reference to web portal for sending responses
     */
    void processSteeringCommand(float value, WebPortal& webPortal);

    /**
     * @brief Process throttle command
     * @param value Throttle percentage (0 to 100)
     * @param webPortal Reference to web portal for sending responses
     */
    void processThrottleCommand(float value, WebPortal& webPortal);

    /**
     * @brief Throttle calibration commands (RC-independent; drive the servo directly).
     * Begin is refused while the engine is running.
     */
    void processThrottleCalBegin(WebPortal& webPortal);
    void processThrottleCalJog(bool isFull, uint16_t us, WebPortal& webPortal);
    void processThrottleCalSave(WebPortal& webPortal);
    void processThrottleCalCancel(WebPortal& webPortal);

    /**
     * @brief Process brake command
     * @param value Brake percentage (0 to 100, 0=released, 100=fully applied)
     * @param webPortal Reference to web portal for sending responses
     */
    void processBrakeCommand(float value, WebPortal& webPortal);

    /**
     * @brief Process ignition command
     * @param state Ignition state string (OFF/ACC/IGNITION/START)
     * @param webPortal Reference to web portal for sending responses
     */
    void processIgnitionCommand(const String& state, WebPortal& webPortal);

    /**
     * @brief Process light command
     * @param on true to turn light on, false to turn off
     * @param webPortal Reference to web portal for sending responses
     */
    void processLightCommand(bool on, WebPortal& webPortal);

    /**
     * @brief Process front-wheel lock command
     * @param locked true to engage the lock, false to release
     * @param webPortal Reference to web portal for sending responses
     */
    void processWheelLockCommand(bool locked, WebPortal& webPortal);

    /**
     * @brief Steering calibration commands (web-only).
     * Capture commands persist the current AS5600 angle as center/left/right;
     * jog drives the actuator open-loop (momentary, sign = direction, |value| = duty);
     * nudge fires one short slow pulse for precise positioning.
     */
    void processSteerCalCommand(const String& which, WebPortal& webPortal);
    void processSteerJogCommand(float value, WebPortal& webPortal);
    void processSteerNudgeCommand(int8_t direction, WebPortal& webPortal);

    /**
     * @brief Process web take/release control command (latched MAVLink override)
     */
    void processWebControlCommand(bool on, WebPortal& webPortal);

    /**
     * @brief Process set_gear_default command
     * @param gear Gear string ("R", "N", "L", "H")
     * @param positionPct Servo position 0.0–100.0 % to save as default
     * @param webPortal Reference to web portal for sending responses
     */
    void processSetGearDefaultCommand(const String& gear, float positionPct, WebPortal& webPortal);
    void processMoveToPositionCommand(float positionPct, WebPortal& webPortal);
    void processBoostTestCommand(bool enable, WebPortal& webPortal);
    void processSetBoostRpmCommand(int32_t rpm, WebPortal& webPortal);

    /**
     * @brief Process can_probe command — trigger a one-shot ECU capability probe.
     * Deferred while a gear change is active; short-circuits with a no-ECU result when
     * CAN data is invalid.
     */
    void processCanProbeCommand(WebPortal& webPortal);

    /**
     * @brief Speed-sensor calibration commands (`speed_cal_ppr`, `speed_cal_circ`).
     * Accepted regardless of the active input source, like the other calibration commands.
     * There is deliberately no limiter command: the ceiling is the autopilot's alone.
     */
    void processSpeedCalPprCommand(float value, WebPortal& webPortal);
    void processSpeedCalCircCommand(float value, WebPortal& webPortal);

    /**
     * @brief Steering speed-scaling commands (`set_steer_sca_base`, `set_test_speed`).
     * Accepted regardless of the active input source, like the calibration commands above.
     * The base is persisted to NVS; the test speed is RAM-only and expires by itself.
     */
    void processSetSteerScaBaseCommand(float value, WebPortal& webPortal);
    void processSetTestSpeedCommand(float value, WebPortal& webPortal);

    /**
     * @brief Speed-scale the AUTOPILOT steering command. ArduPilot's own formula,
     * `scale = min(1, base / speed)`, applied as `steeringPct *= scale` — the percentage is
     * already signed about a centre of zero, so this scales the DEVIATION FROM CENTRE and
     * leaves a centred command centred.
     *
     * ANY fault gives scale = 1 and an unmodified command: no base, an invalid speed reading,
     * or an autopilot mode that is known and is not Rover MANUAL. There is deliberately NO
     * hold, NO substitute speed and NO minimum floor — every one of those is a way for a sensor
     * fault to leave the driver with restricted steering, which is the failure this exists to
     * eliminate. An unknown or stale mode is treated as MANUAL: it is the mode this vehicle is
     * driven in, and being wrong only means the assist keeps working.
     *
     * The applied scale is slew-limited (STEER_SCALE_SLEW_PER_S, both directions); the rate
     * limit is on the SCALE, never on the command, so the driver's own steering movements are
     * never slowed by the assist.
     *
     * @return the steering percentage to command
     */
    float applySteeringScale(float steeringPct);

    /**
     * @brief Log a scale change once, then hold off.
     * Suppressed logs deliberately do NOT update the remembered state, so whatever the scale
     * settles on is logged on the next opportunity rather than being lost — the
     * logSpeedLimitChange() rule.
     */
    void logSteerScaleChange(float scale, float speedMs, bool speedIsTest,
                             float baseMs, SteerScaleSource source);

    /**
     * @brief Max-speed throttle limiter, applied to the arbitrated driver/MAVLink/web
     * throttle command only (the gear-boost PID owns throttle during a shift and is
     * never touched here). Fails OPEN: an invalid/stale speed reading does not clamp.
     *
     * A PROPORTIONAL taper, not a step: the ceiling is 100 % at
     * `limit - SPEED_LIMIT_TAPER_BAND_MS`, falls linearly to SPEED_LIMIT_FLOOR_PCT at the
     * limit, and holds the floor above it — never a hard cut mid-corner. The ceiling itself
     * is slew-limited so engagement cannot snap the servo. With no limit (0 m/s) the demand
     * passes through untouched.
     * @return the throttle percentage to command
     */
    float applySpeedLimit(float throttlePct);

    /**
     * @brief Log a limit change once, then hold off.
     * Suppressed logs deliberately do NOT update the remembered state, so the settled value
     * is logged on the next opportunity rather than being lost.
     */
    void logSpeedLimitChange(float limitMs);

    // Returns true when throttle should be clamped to TRANS_UNKNOWN_GEAR_THROTTLE_MAX.
    // Clips when gear position is invalid and physical gear is not neutral
    // (neutral is safe — drivetrain disconnected).
    bool shouldClipThrottle() const;
};

#endif // VEHICLE_CONTROLLER_H
