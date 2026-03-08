#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "ServoController.h"
#include "BTS7960Controller.h"
#include "TransmissionController.h"
#include "WebPortal.h"

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
                      BTS7960Controller& brake);

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

private:
    // Actuator references
    ServoController& steering_;
    ServoController& throttle_;
    TransmissionController& transmission_;
    BTS7960Controller& brake_;

    // State tracking
    InputSource currentInputSource_;
    bool failsafeApplied_;

    /**
     * @brief Apply fail-safe commands (center steering, idle throttle, stop actuators)
     */
    void applyFailsafe();

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
};

#endif // VEHICLE_CONTROLLER_H
