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
#include "BoardInputs.h"
#include "CANController.h"
#include "SpeedSensor.h"

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
                      BoardInputs& boardInputs,
                      SpeedSensor& speedSensor);

    /**
     * @brief Record that the 24V boost rail was enabled in setup() and prepare the
     * rail ADC. `main.cpp` drives `PIN_BOOST_EN` HIGH as its first hardware action;
     * this starts the startup grace window and the sampling schedule.
     */
    void initBoostRail();

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
     * @brief Check the brake "fully released" endstop (opto input In5).
     *
     * Read from the `BoardInputs` snapshot — never a `digitalRead()` and never an
     * I2C transaction of its own. A stale snapshot returns false ("not confirmed
     * released"), so retraction stays bounded by BRAKE_SENSOR_OVERRUN_TIME /
     * BRAKE_FULL_TRAVEL_TIME instead of stopping early on a failed read.
     *
     * @return true only if the endstop is confirmed reached
     */
    bool isBrakeReleased() const {
        BoardInputs::Snapshot snap = boardInputs_.getSnapshot();
        return snap.valid && snap.brakeReleased;
    }

    /** @brief Latest averaged 24V rail voltage (V) */
    float getRail24V() const { return rail24V_; }

    /** @brief Commanded state of the 24V boost enable line */
    bool isBoostOn() const { return boostEnabled_; }

    /** @brief True when the rail is below RAIL_24V_LOW_THRESHOLD (warning only) */
    bool isRailLow() const { return railLow_; }

    /** @brief Opto-input expander fault state (telemetry `io_input_fault`) */
    bool isInputFault() const { return boardInputs_.isFaulted(); }

    /** @brief Raw opto-input port byte of the last good read (telemetry `io_raw`) */
    uint8_t getInputRaw() const { return boardInputs_.getRawInputs(); }

    /** @brief Relay expander fault state (telemetry `io_relay_fault`) */
    bool isRelayFault() const { return relayController_.isFaulted(); }

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
     * @brief Hall-sensor vehicle speed (km/h) and its health flag.
     * Independent of CAN status — a CAN outage does not blank these.
     */
    float getVehicleSpeedKmh() const { return speedSensor_.getSpeedKmh(); }
    bool isVehicleSpeedValid() const { return speedSensor_.isValid(); }

    /** @brief Speed-sensor calibration and limiter settings (telemetry / web UI) */
    uint16_t getSpeedPulsesPerRev() const { return speedSensor_.getPulsesPerRev(); }
    float getSpeedWheelCircumferenceMm() const { return speedSensor_.getWheelCircumferenceMm(); }
    bool isSpeedLimiterEnabled() const { return speedSensor_.isLimiterEnabled(); }
    float getSpeedLimitMaxKmh() const { return speedSensor_.getLimitMaxKmh(); }

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
    BoardInputs& boardInputs_;     // opto inputs (gear switches + brake endstop)
    SpeedSensor& speedSensor_;     // hall speed sensor (updated from main.cpp)
    CANController canController_;  // CAN bus controller (owned, not reference)

    // 24V boost rail state
    bool     boostEnabled_;        // commanded state of PIN_BOOST_EN
    uint32_t boostEnabledAtMs_;    // millis() of the last enable (startup grace)
    float    rail24V_;             // last averaged rail voltage (V)
    bool     railLow_;             // rail below threshold outside the grace window
    uint32_t lastRailSampleMs_;
    uint32_t lastRailWarnMs_;      // rate limit for the low-rail log

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
     * @brief Process the 24V boost rail command (`set_boost`).
     * Enabling is always allowed and restarts the startup grace window; disabling
     * is refused unless ignition is OFF, because the rail powers the steering VESC.
     */
    void processBoostCommand(bool on, WebPortal& webPortal);

    /**
     * @brief Sample and average the 24V rail (non-blocking, every RAIL_SAMPLE_INTERVAL_MS)
     */
    void updateRailMonitor();

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
     * @brief Speed-sensor calibration and max-speed limiter commands
     * (`speed_cal_ppr`, `speed_cal_circ`, `speed_limit_enable`, `speed_limit_set`).
     * Accepted regardless of the active input source, like the other calibration commands.
     */
    void processSpeedCalPprCommand(float value, WebPortal& webPortal);
    void processSpeedCalCircCommand(float value, WebPortal& webPortal);
    void processSpeedLimitEnableCommand(bool enable, WebPortal& webPortal);
    void processSpeedLimitSetCommand(float kmh, WebPortal& webPortal);

    /**
     * @brief Max-speed throttle limiter, applied to the arbitrated driver/MAVLink/web
     * throttle command only (the gear-boost PID owns throttle during a shift and is
     * never touched here). Fails OPEN: an invalid/stale speed reading does not clamp.
     * @return the throttle percentage to command
     */
    float applySpeedLimit(float throttlePct);

    // Returns true when throttle should be clamped to TRANS_UNKNOWN_GEAR_THROTTLE_MAX.
    // Clips when gear position is invalid and physical gear is not neutral
    // (neutral is safe — drivetrain disconnected).
    bool shouldClipThrottle() const;
};

#endif // VEHICLE_CONTROLLER_H
