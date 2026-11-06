#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "BatteryVoltage.h"
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
#include "EngineHourMeter.h"
#include "SpeedLimiterLaw.h"
#include "TachoLaw.h"
#include "TachoSensor.h"

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
     * @brief Prepare the 12 V battery sense on X10 / GPIO9 (ADC attenuation) and load the
     * user calibration factor from NVS (absent / out of range → 1.0). `main.cpp` still
     * drives `PIN_BOOST_EN` HIGH (no boost board on aztec — harmless).
     */
    void initBatterySense();

    /**
     * @brief Start the ECU tacho input on GPIO14 (ppr from NVS "tacho", RISING-edge ISR).
     * Call once from setup(), after NVS is up.
     */
    void initTacho() { tacho_.begin(); }

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
     * velocity. Derived from the PHYSICAL gear (the opto switches), never from the
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

    /** @brief Latest calibrated 12 V battery voltage (V); meaningful only when isBatteryValid() */
    float getBatteryV() const { return batteryV_; }

    /** @brief Latest averaged battery voltage BEFORE the user calibration factor (V) */
    float getBatteryRawV() const { return batteryRawV_; }

    /** @brief User calibration factor in use (1.0 = uncalibrated) */
    float getBatteryCal() const { return batteryCal_; }

    /** @brief True once a sample shows something wired to X10 (raw > BAT12_PRESENT_MIN_V) */
    bool isBatteryValid() const { return batteryValid_; }

    /** @brief Low-battery warning (hysteresis, suppressed while cranking + 2 s); warning only */
    bool isBatteryLow() const { return batteryLow_; }

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
     * @brief Latest CAN sniffer snapshot (reference into the CAN controller, for telemetry)
     */
    const CANController::SniffResults& getSniffResults() const { return canController_.getSniffResults(); }

    /**
     * @brief Hall-sensor vehicle speed (m/s) and its health flag.
     * Independent of CAN status — a CAN outage does not blank these.
     */
    float getVehicleSpeedMs() const { return speedSensor_.getSpeedMs(); }
    bool isVehicleSpeedValid() const { return speedSensor_.isValid(); }

    /** @brief Speed-sensor calibration (telemetry / web UI) */
    uint16_t getSpeedPulsesPerRev() const { return speedSensor_.getPulsesPerRev(); }
    /** @brief Running pulse total since boot, for the web ppr measurement; counts up in either rotation direction */
    uint32_t getSpeedPulseTotal() const { return speedSensor_.getPulseTotal(); }
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
     * @brief Throttle ceiling the limiter is currently applying (%, 100 = inactive): the
     * proportional taper plus the integral trim, after the floor/100 clamps and the slew.
     */
    float getSpeedLimitCeilingPct() const { return limiter_.ceilPct; }

    /**
     * @brief Speed-limiter throttle floor in use (%, 0 = calibrated idle), NVS "speed"/thr_floor
     */
    float getSpeedLimitFloorPct() const { return limiterFloorPct_; }

    /**
     * @brief Speed-limiter ceiling slew in use (%/s, both directions), NVS "speed"/thr_slew
     */
    float getSpeedLimitSlewPctS() const { return limiterSlewPctS_; }

    /**
     * @brief Throttle opening rate limit in use (%/s, 0 = off), NVS "throttle"/open_slew
     */
    float getThrottleOpenSlewPctS() const { return throttle_.getOpenSlewPctS(); }

    /**
     * @brief Live saturation: ceiling at the floor while the speed stays more than
     * SPEED_LIMIT_SAT_MARGIN_MS above the limit, continuously for SPEED_LIMIT_SAT_HOLD_MS.
     */
    bool isSpeedLimitSaturated() const { return satActive_; }

    /**
     * @brief Saturation display hold: true while saturated and for SPEED_LIMIT_SAT_UI_HOLD_MS
     * after it last cleared. Evaluated from millis() at call time, so the hold expires on time
     * even while applySpeedLimit() is not being called.
     */
    bool isSpeedLimitSatHeld() const {
        return satActive_ ||
               (satClearedMs_ != 0 && (millis() - satClearedMs_) < SPEED_LIMIT_SAT_UI_HOLD_MS);
    }

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
     * @brief Steering control mode selected in the latest update() (SteerRate.h):
     * POSITION while the steering sensor is healthy; RATE while it is not, the source is
     * MAVLink and the autopilot is in MANUAL (or its mode is unknown/stale); BLOCKED otherwise.
     */
    SteerMode getSteerMode() const { return steerMode_; }

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
     * @brief Set front-wheel lock state (NOT fitted on aztec — a LOCK is rejected)
     * @param locked true to engage the lock, false to release
     * @return false if the request could not be honoured
     */
    bool setWheelLock(bool locked);

    /**
     * @brief Get current front-wheel lock state
     * @return true if locked (always false on aztec)
     */
    bool getWheelLock() const { return relayController_.getWheelLock(); }

    /** @brief True if the front-wheel lock is fitted (false on aztec) */
    bool isWheelLockFitted() const { return RelayController::isWheelLockFitted(); }

    /**
     * @brief Web relay test state, relay-number order (bit0 = Relay_1 … bit7 = Relay_8),
     * from the relay output shadow. Published in telemetry as `relay_test`.
     */
    uint8_t getRelayTestBits() const { return relayController_.getRelayTestBits(); }

    /**
     * @brief Get current boost RPM target (NVS-backed, runtime-editable)
     */
    int32_t getBoostTargetRpm() const { return boostTargetRpm_; }

    /**
     * @brief Gear-change RPM boost switch in use (NVS "boost"/enabled, default true).
     * Off: a gear change holds the throttle at idle instead of running the PID.
     */
    bool isGearBoostEnabled() const { return boostGearEnabled_; }

    /**
     * @brief Engine running, from the EFFECTIVE RPM (tacho when fitted, CAN otherwise).
     * @return true if the effective RPM is valid and >= ENGINE_RUNNING_RPM_THRESHOLD (800 on
     *         aztec: above cranking speed, below the ≈1500 RPM idle)
     */
    bool isEngineRunning() const {
        return effRpm_.valid && effRpm_.rpm >= ENGINE_RUNNING_RPM_THRESHOLD;
    }

    /** @brief Effective engine RPM (0 when no source is valid) — the ONE value every consumer reads */
    uint16_t getEngineRpm() const { return effRpm_.valid ? effRpm_.rpm : 0; }

    /** @brief True when the effective RPM came from a valid source (TACHO or CAN) */
    bool isEngineRpmValid() const { return effRpm_.valid; }

    /** @brief Where the effective RPM came from (TACHO / CAN / NONE) */
    RpmSource getRpmSource() const { return effRpm_.source; }

    /** @brief ECU tacho input (live Hz / ppr / RPM for telemetry) */
    const TachoSensor& getTacho() const { return tacho_; }
    TachoSensor& getTacho() { return tacho_; }

private:
    // Actuator references
    SteeringController& steering_;
    ThrottleController& throttle_;
    TransmissionController& transmission_;
    BTS7960Controller& brake_;

    // Input and output references
    MavlinkInterface& mavlink_;
    RelayController& relayController_;
    BoardInputs& boardInputs_;     // opto inputs (gear code lines + brake endstop)
    SpeedSensor& speedSensor_;     // hall speed sensor (updated from main.cpp)
    EngineHourMeter engineHours_;  // OWNED by value: a counter, not a shared device
    CANController canController_;  // CAN bus controller (owned, not reference)
    TachoSensor tacho_;            // ECU tacho input on GPIO14 (owned; single instance — file-static ISR)
    EffectiveRpm effRpm_;          // effective engine RPM, recomputed once per update()

    // 12 V battery sense (X10 / GPIO9)
    float    batteryV_;            // last calibrated battery voltage (V)
    float    batteryRawV_;         // last averaged voltage before calibration (V)
    bool     batteryValid_;        // last sample showed something wired (raw > BAT12_PRESENT_MIN_V)
    bool     batteryLow_;          // published low flag (hysteresis, crank-suppressed)
    bool     batteryLowRaw_;       // hysteresis state before the crank suppression
    float    batteryCal_;          // user calibration factor (NVS "bat12"/"cal")
    uint32_t lastBatterySampleMs_;
    uint32_t lastStarterOnMs_;     // millis() the starter relay was last seen energized (0 = never)
    uint32_t lastBatteryWarnMs_;   // rate limit for the low-battery log

    // State tracking
    InputSource currentInputSource_;

    // Steering control mode (change add-aztec-steering-rate-mode)
    SteerMode steerMode_;           // re-derived every update() before the MAVLink commands
    uint32_t lastMavCmdCount_;      // MavlinkInterface::getCommandCount() seen last iteration
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
    bool boostGearEnabled_;             // Gear-change boost switch (NVS "boost"/enabled, default BOOST_ENABLED_DEFAULT)

    // Gear boost PID state
    bool gearBoostActive_;              // True while PID is holding RPM during gear change
    bool boostManualActive_;            // True when boost is triggered manually via web for testing
    bool shiftIdleHold_;                // True while a gear change holds the throttle at idle (boost switched off)
    uint32_t gearBoostStartTime_;       // millis() when boost activated (for timeout)
    uint32_t pidLastUpdateTime_;        // millis() of last PID compute (for dt)
    uint32_t lastRpmSampleMs_;          // effective-RPM sample timestamp seen on previous PID iteration
    float pidIntegral_;                 // Accumulated integral term
    float pidPrevError_;                // Previous error (for derivative)
    uint16_t lastPIDThrottleUs_;        // Last PID-computed throttle µs (held on CAN stale)

    // Ignition state tracking
    MavlinkInterface::IgnitionState previousIgnitionState_;  // Track previous state for transition detection
    TransmissionController::Gear lastCommandedGear_;         // Last gear requested via MAVLink (dedup guard)

    uint32_t lastSpeedLimitWarnMs_; // rate limit for the "limiter armed, speed invalid" log

    // Limiter ceiling tracking (log on change only)
    float    lastSpeedLimitMs_;     // NAN sentinel = nothing logged yet
    uint32_t lastSpeedLimitLogMs_;

    // Limiter law state (include/SpeedLimiterLaw.h). The ceiling is what gets rate-limited,
    // never the demand.
    SpeedLimiterState limiter_;     // applied ceiling (%, 100 = inactive), integral trim, primed
    uint32_t limiterLastMs_;        // millis() of the last limiter evaluation (0 = first call)
    float    limiterFloorPct_;      // floor in use (%), from NVS "speed"/thr_floor or the default
    float    limiterSlewPctS_;      // ceiling slew in use (%/s), from NVS "speed"/thr_slew or the default

    // Saturation indicator (web console only — no log line, no MAVLink message)
    uint32_t satSinceMs_;           // millis() the raw condition started holding (0 = not holding)
    bool     satActive_;            // live flag: raw condition held for SPEED_LIMIT_SAT_HOLD_MS
    uint32_t satClearedMs_;         // millis() satActive_ last cleared (0 = never) — display hold

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

    // Portal waiting for the final result of a "Set sensor zero" (steer_lws_zero): the
    // command is answered once when it starts and again when the zero-set ends.
    WebPortal* steerZeroResponsePortal_;

    /**
     * @brief Apply fail-safe commands (center steering, idle throttle, stop actuators)
     */
    void applyFailsafe();

    /**
     * @brief Re-derive steerMode_ (steerModeSelect()) and apply the transition actions:
     * one serial line per change, stopRate() when leaving RATE, disarmRate() when entering it.
     * Called every update() before processMavlinkCommands().
     */
    void updateSteerMode();

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
     * @brief Throttle owned by the gear change: the boost PID is active or the idle hold is on.
     * MAVLink throttle, the web re-limit and the web `throttle` command stay clear while true.
     */
    bool isThrottleLockedForShift() const { return gearBoostActive_ || shiftIdleHold_; }

    /**
     * @brief Idle hold for a gear change while the boost switch is off: holds throttle_.idle()
     * as long as needsThrottleBoost() is true (no CAN/RPM precondition, poll rate untouched),
     * then releases with throttle_.idle() and the web demand zeroed.
     */
    void updateShiftIdleHold();

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
     * @brief Process exact steering angle command (steer_angle, web control only)
     * @param deg Target angle in degrees (+ = right), clamped to the captured limits
     * @param webPortal Reference to web portal for sending responses
     */
    void processSteerAngleCommand(float deg, WebPortal& webPortal);

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
     * @brief Process the web relay test command (`set_relay_test`).
     * Toggles one relay (1..8) for a wiring check. Not gated by input source — the
     * gate is in RelayController: ON only with ignition OFF and no function relay on.
     * @param relayNumber relay 1..8 (anything else is rejected)
     * @param on true to energize, false to release
     * @param webPortal Reference to web portal for sending responses
     */
    void processRelayTestCommand(int32_t relayNumber, bool on, WebPortal& webPortal);

    /**
     * @brief `set_boost`: still accepted, but aztec has no boost board on X10 — replies
     * "boost rail not fitted" and does nothing.
     */
    void processBoostCommand(bool on, WebPortal& webPortal);

    /**
     * @brief `cal_bat_v`: compute the battery calibration factor from the user's multimeter
     * reading and the CURRENT averaged raw value, validate (measured 8–16 V, battery present,
     * factor 0.80–1.20), persist to NVS, then apply.
     */
    void processBatteryCalCommand(float measuredV, WebPortal& webPortal);

    /** @brief `reset_bat_v_cal`: calibration factor back to 1.0 (persisted) */
    void processBatteryCalResetCommand(WebPortal& webPortal);

    /**
     * @brief `cal_tacho_ppr`: raw = CURRENT tacho Hz × 60 / reference RPM, snapped to
     * {0.5,1,2,3,4}; refused without pulses, with a reference outside 300–8000 RPM or a raw
     * ratio more than ±25 % off. Persisted, then applied. Replies start with "Tacho:".
     */
    void processTachoCalCommand(float refRpm, WebPortal& webPortal);

    /** @brief `set_tacho_ppr`: accept only {0.5,1,2,3,4}; persisted, then applied */
    void processTachoSetPprCommand(float ppr, WebPortal& webPortal);

    /** @brief `reset_tacho_ppr`: remove the stored key, ppr back to 1 */
    void processTachoResetPprCommand(WebPortal& webPortal);

    /** @brief Compose effRpm_ from the tacho and the CAN snapshot (TachoLaw.h tachoSelectRpm) */
    void updateEffectiveRpm();

    /**
     * @brief Sample and average the 12 V battery on X10 (non-blocking, every
     * BAT12_SAMPLE_INTERVAL_MS); low flag with hysteresis, suppressed while cranking + 2 s.
     */
    void updateBatteryMonitor();

    /** @brief BatteryVoltage.h parameters from Constants.h */
    static BatParams batteryParams();

    /**
     * @brief Steering calibration commands (web-only, any input source).
     * steer_cal_left / steer_cal_right store the current angle as that limit and save it
     * at once (reply in degrees, with the refusal reason or the beyond-safe-range warning);
     * steer_cal_center is answered with a failure pointing to "Set sensor zero";
     * jog drives the actuator open-loop (momentary, sign = direction, |value| = duty);
     * nudge fires one short slow pulse for precise positioning. Jog and nudge work
     * without a healthy sensor.
     */
    void processSteerCalCommand(const String& which, WebPortal& webPortal);

    /**
     * @brief "Set sensor zero" (steer_lws_zero): start the LWS zero-set sequence with no
     * precondition. Answered when it starts (or is refused) and again with the final
     * result (zero confirmed / failed + reason) from update().
     */
    void processSteerZeroCommand(WebPortal& webPortal);
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

    /**
     * @brief Gear code calibration commands (web-only, bench).
     * `set_gear_code` (value "R"/"N"/"L"/"H") stores the CURRENT live gear code as that
     * gear's code; `reset_gear_codes` restores the Constants.h defaults. Both are refused
     * while the engine is running; capture validation lives in TransmissionController.
     */
    void processGearCodeCommand(const String& which, const String& gear, WebPortal& webPortal);
    void processBoostTestCommand(bool enable, WebPortal& webPortal);
    void processSetBoostRpmCommand(int32_t rpm, WebPortal& webPortal);
    /** @brief Persist NVS "boost"/enabled, then apply; turning off mid-boost releases the PID into the idle hold. */
    void processSetBoostEnabledCommand(bool enabled, WebPortal& webPortal);

    /**
     * @brief Process can_probe command — trigger a one-shot ECU capability probe.
     * Deferred while a gear change is active; short-circuits with a no-ECU result when
     * CAN data is invalid.
     */
    void processCanProbeCommand(WebPortal& webPortal);

    /**
     * @brief Process can_sniff command — accept every CAN ID for CAN_SNIFF_WINDOW_MS and
     * record a per-ID table. Standstill diagnostic; refused while the ECU probe or the LWS
     * zero-set runs, or during a gear change.
     */
    void processCanSniffCommand(WebPortal& webPortal);

    /**
     * @brief Speed-sensor calibration commands (`speed_cal_ppr`, `speed_cal_circ`) and the
     * speed-limiter floor (`set_speed_limit_floor`) and throttle rate (`set_speed_limit_slew`).
     * Accepted regardless of the active input source, like the other calibration commands.
     * There is deliberately no limiter CEILING command: the ceiling is the autopilot's alone.
     * The floor and the throttle rate are the limiter settings: they shape how the limit is
     * enforced and never impose a limit by themselves.
     */
    void processSpeedCalPprCommand(float value, WebPortal& webPortal);
    void processSpeedCalCircCommand(float value, WebPortal& webPortal);
    /** @brief Validate (NaN, 0–20 %), persist to NVS "speed"/thr_floor, then apply in RAM. */
    void processSetSpeedLimitFloorCommand(float value, WebPortal& webPortal);
    /** @brief Validate (NaN, 10–200 %/s), persist to NVS "speed"/thr_slew, then apply in RAM. */
    void processSetSpeedLimitSlewCommand(float value, WebPortal& webPortal);
    /**
     * @brief `set_throttle_open_slew`: round to a whole %/s, validate (NaN, 0 or 30–100 %/s),
     * persist to NVS "throttle"/open_slew, then apply. Accepted from any input source.
     */
    void processSetThrottleOpenSlewCommand(float value, WebPortal& webPortal);

    /**
     * @brief Steering speed-scaling commands (`set_steer_sca_base`, `set_test_speed`).
     * Accepted regardless of the active input source, like the calibration commands above.
     * The base is persisted to NVS; the test speed is RAM-only and expires by itself.
     */
    void processSetSteerScaBaseCommand(float value, WebPortal& webPortal);
    void processSetTestSpeedCommand(float value, WebPortal& webPortal);

    /**
     * @brief Steering park correction settings (`set_steer_park_enabled` bool,
     * `set_steer_park_rung` value = rung 1–3 + strValue "D,T", `set_steer_park_tol` degrees,
     * `set_steer_park_max` count). Accepted from any input source; SteeringController validates,
     * persists to NVS "steering" first, then applies.
     */
    void processSetSteerParkEnabledCommand(bool on, WebPortal& webPortal);
    void processSetSteerParkRungCommand(float rung, const String& strValue, WebPortal& webPortal);
    void processSetSteerParkTolCommand(float deg, WebPortal& webPortal);
    void processSetSteerParkMaxCommand(float count, WebPortal& webPortal);

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
     * The law (include/SpeedLimiterLaw.h, design D2/D3): a PROPORTIONAL taper over a band of
     * clamp(SPEED_LIMIT_TAPER_FRACTION × limit, SPEED_LIMIT_TAPER_MIN_MS, SPEED_LIMIT_TAPER_MAX_MS)
     * — 100 % at `limit - band`, linear down to the floor at the limit — plus an INTEGRAL TRIM.
     * The trim falls at SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS × excess above the limit (step capped at
     * the slew), so the throttle keeps closing toward the floor; below the limit it rises at
     * Ki × deficit, capped at SPEED_LIMIT_TRIM_RISE_PCT_S, so the speed settles AT the limit
     * instead of drooping inside the band. The ceiling clamp(taper + trim, floor, 100) is then
     * slew-limited at the per-vehicle slew (limiterSlewPctS_, dt capped at 1 s), so it never steps —
     * it reaches the floor (default 0 % = calibrated idle) only gradually.
     *
     * Anti-windup: the trim is clamped to [0, 100 − floor]; it rises only while the limiter
     * binds (demand ≥ ceiling − SPEED_LIMIT_BIND_MARGIN_PCT) and is held otherwise; it is reset
     * below the band, on "no limit", on an invalid sensor and on the first evaluation.
     *
     * Fail-open: with no limit (0 m/s) or an invalid speed reading the state is reset (ceiling
     * 100 %, trim 0) and the demand passes through untouched.
     *
     * Also updates the saturation indicator (design D6) after each step.
     * @return the throttle percentage to command
     */
    float applySpeedLimit(float throttlePct);

    /**
     * @brief Log a limit change once, then hold off.
     * Suppressed logs deliberately do NOT update the remembered state, so the settled value
     * is logged on the next opportunity rather than being lost.
     */
    void logSpeedLimitChange(float limitMs);

    /** @brief End the live saturation (no-limit / fail-open paths); starts the display hold. */
    void clearSpeedLimitSaturation(uint32_t now);

    // Returns true when throttle should be clamped to TRANS_UNKNOWN_GEAR_THROTTLE_MAX:
    // while a gear change is active, the expected gear is NEUTRAL, or the transmission is
    // not initialised (expected gear GEAR_UNKNOWN).
    bool shouldClipThrottle() const;
};

#endif // VEHICLE_CONTROLLER_H
