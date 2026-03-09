#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "ServoController.h"
#include "BTS7960Controller.h"
#include "TransmissionController.h"
#include "WebPortal.h"
#include "SBusInput.h"
#include "RelayController.h"
#include "CANController.h"

/**
 * @brief Vehicle control coordination layer
 *
 * Handles input source priority (SBUS > WEB > FAILSAFE), processes commands,
 * and coordinates all vehicle actuators (steering, throttle, brake, transmission).
 */
class VehicleController {
public:
    VehicleController(ServoController& steering,
                      ServoController& throttle,
                      TransmissionController& transmission,
                      BTS7960Controller& brake,
                      SBusInput& sbusInput,
                      RelayController& relayController);

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
     * @param source Input source (SBUS, WEB, or FAILSAFE)
     */
    void setInputSource(InputSource source);

    /**
     * @brief Get current input source
     * @return Current input source
     */
    InputSource getInputSource() const { return currentInputSource_; }

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
     * @brief Get steering angle
     * @return Steering angle in degrees
     */
    float getSteeringAngle() const { return steering_.getAngle(); }

    /**
     * @brief Get throttle angle
     * @return Throttle angle in degrees
     */
    float getThrottleAngle() const { return throttle_.getAngle(); }

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
    bool isBrakeReleased() const { return digitalRead(PIN_BRAKE_SENSOR) == HIGH; }

    /**
     * @brief Get vehicle data from CAN bus
     * @return VehicleData structure with engine RPM, speed, temperatures, etc.
     */
    CANController::VehicleData getVehicleData() const { return canController_.getVehicleData(); }

private:
    // Actuator references
    ServoController& steering_;
    ServoController& throttle_;
    TransmissionController& transmission_;
    BTS7960Controller& brake_;

    // Input and output references
    SBusInput& sbusInput_;
    RelayController& relayController_;
    CANController canController_;  // CAN bus controller (owned, not reference)

    // State tracking
    InputSource currentInputSource_;
    bool failsafeApplied_;

    // Brake actuator tracking
    float currentBrakeTarget_;          // Current brake percentage target (0-100)
    float currentBrakePosition_;        // Estimated brake position (0-100)
    uint32_t brakeMovementStartTime_;   // Time when brake started moving
    bool brakeIsMoving_;

    // Throttle boost tracking
    uint32_t throttleBoostStartTime_;   // Time when throttle boost started
    bool throttleBoostActive_;          // True if boost is currently active

    // Ignition state tracking
    SBusInput::IgnitionState previousSBusIgnitionState_;  // Track previous state for transition detection

    /**
     * @brief Apply fail-safe commands (center steering, idle throttle, stop actuators)
     */
    void applyFailsafe();

    /**
     * @brief Process SBUS commands from ArduPilot
     */
    void processSBusCommands();

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
     * @brief Update throttle boost during gear changes
     * Temporarily increases throttle to maintain RPM
     */
    void updateThrottleBoost();

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
     * @brief Process brake command
     * @param value Brake percentage (0 to 100, 0=released, 100=fully applied)
     * @param webPortal Reference to web portal for sending responses
     */
    void processBrakeCommand(float value, WebPortal& webPortal);

    /**
     * @brief Process transmission calibration command
     *
     * Runs full transmission calibration using physical gear sensors.
     * Uses max speed (255) and 20 second timeout.
     *
     * @param webPortal Reference to web portal for sending responses
     */
    void processCalibrationCommand(WebPortal& webPortal);

    /**
     * @brief Process clear calibration command
     *
     * Clears saved calibration data from NVS. Next boot will require recalibration.
     *
     * @param webPortal Reference to web portal for sending responses
     */
    void processClearCalibrationCommand(WebPortal& webPortal);
};

#endif // VEHICLE_CONTROLLER_H
