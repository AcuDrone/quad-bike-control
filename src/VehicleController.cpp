#include "VehicleController.h"
#include "Debug.h"

VehicleController::VehicleController(ServoController& steering,
                                     ServoController& throttle,
                                     TransmissionController& transmission,
                                     BTS7960Controller& brake,
                                     SBusInput& sbusInput,
                                     RelayController& relayController)
    : steering_(steering),
      throttle_(throttle),
      transmission_(transmission),
      brake_(brake),
      sbusInput_(sbusInput),
      relayController_(relayController),
      currentInputSource_(InputSource::FAILSAFE),
      failsafeApplied_(false),
      currentBrakeTarget_(0.0f),
      currentBrakePosition_(0.0f),
      brakeMovementStartTime_(0),
      brakeIsMoving_(false),
      throttleBoostStartTime_(0),
      throttleBoostActive_(false),
      previousSBusIgnitionState_(SBusInput::IgnitionState::OFF) {
}

bool VehicleController::initCAN() {
    return canController_.begin();
}

void VehicleController::update() {
    // Update CAN controller (read vehicle data from ECU)
    canController_.update();

    // Pass vehicle data to transmission for safety checks
    CANController::VehicleData canData = canController_.getVehicleData();
    TransmissionVehicleData transData;
    transData.vehicleSpeed = canData.vehicleSpeed;
    transData.lastUpdateTime = canData.lastUpdateTime;
    transData.dataValid = canData.dataValid;
    transmission_.setVehicleData(transData);

    // Update relay controller with engine RPM (for automatic cranking stop)
    relayController_.update(canData.engineRPM);

    // Process SBUS commands if SBUS is active
    if (currentInputSource_ == InputSource::SBUS) {
        processSBusCommands();
    }

    // Apply fail-safe if needed
    applyFailsafe();

    // Check if throttle boost is needed for gear changes
    if (transmission_.needsThrottleBoost() || throttleBoostActive_) {
        updateThrottleBoost();
    }

    // Update position control for transmission actuator
    transmission_.update();

    // Update brake actuator control
    updateBrakeControl();
}

void VehicleController::setInputSource(InputSource source) {
    if (currentInputSource_ != source) {
        Debug::printf("[INPUT] Source changed: %s -> %s\n",
                     (currentInputSource_ == InputSource::SBUS) ? INPUT_SOURCE_NAME_SBUS :
                     (currentInputSource_ == InputSource::WEB) ? INPUT_SOURCE_NAME_WEB :
                     INPUT_SOURCE_NAME_FAILSAFE,
                     (source == InputSource::SBUS) ? INPUT_SOURCE_NAME_SBUS :
                     (source == InputSource::WEB) ? INPUT_SOURCE_NAME_WEB :
                     INPUT_SOURCE_NAME_FAILSAFE);
    }
    currentInputSource_ = source;
}

void VehicleController::processWebCommand(const WebPortal::WebCommand& cmd, WebPortal& webPortal) {
    if (!cmd.hasCommand) {
        return;
    }

    // Special commands that work regardless of input source
    if (cmd.cmd == "calibrate_transmission") {
        processCalibrationCommand(webPortal);
        return;
    } else if (cmd.cmd == "clear_calibration") {
        processClearCalibrationCommand(webPortal);
        return;
    } else if (cmd.cmd == "set_ignition") {
        // Ignition control always available (not restricted by input source)
        processIgnitionCommand(cmd.strValue, webPortal);
        return;
    } else if (cmd.cmd == "set_light") {
        // Light control always available (not restricted by input source)
        processLightCommand(cmd.boolValue, webPortal);
        return;
    }

    // Normal control commands require WEB input source
    if (currentInputSource_ != InputSource::WEB) {
        // Web control not active, ignore control commands
        return;
    }

    // Process control commands
    if (cmd.cmd == "set_gear") {
        processGearCommand(cmd.strValue, webPortal);
    } else if (cmd.cmd == "set_steering") {
        processSteeringCommand(cmd.floatValue, webPortal);
    } else if (cmd.cmd == "set_throttle") {
        processThrottleCommand(cmd.floatValue, webPortal);
    } else if (cmd.cmd == "set_brake") {
        processBrakeCommand(cmd.floatValue, webPortal);
    }
}

String VehicleController::getCurrentGearString() const {
    TransmissionController::Gear gear = transmission_.getCurrentGear();
    switch (gear) {
        case TransmissionController::Gear::GEAR_REVERSE: return "R";
        case TransmissionController::Gear::GEAR_NEUTRAL: return "N";
        case TransmissionController::Gear::GEAR_LOW: return "L";
        case TransmissionController::Gear::GEAR_HIGH: return "H";
        default: return "N";
    }
}

void VehicleController::applyFailsafe() {
    if (currentInputSource_ == InputSource::FAILSAFE && !failsafeApplied_) {
        Debug::println("[FAILSAFE] Entering safe state");
        steering_.setAngle(STEERING_CENTER_ANGLE);
        throttle_.setAngle(THROTTLE_IDLE_ANGLE);
        brake_.stop();         // Stop brake actuator (hold position)
        transmission_.stop();  // Stop transmission actuator
        relayController_.allOff();  // Turn off ignition and lights
        previousSBusIgnitionState_ = SBusInput::IgnitionState::OFF;  // Reset ignition tracking
        failsafeApplied_ = true;
    } else if (currentInputSource_ != InputSource::FAILSAFE && failsafeApplied_) {
        Debug::println("[FAILSAFE] Exiting safe state");
        brake_.stop();  // Ensure brake is stopped before handing control
        failsafeApplied_ = false;
    }
}

void VehicleController::processSBusCommands() {
    if (!sbusInput_.isSignalValid()) {
        return;  // Safety check
    }

    // Apply steering
    float steeringPct = sbusInput_.getSteering();
    int steeringAngle = map((int)steeringPct, -100, 100, STEERING_MIN_ANGLE, STEERING_MAX_ANGLE);
    steering_.setAngle(steeringAngle);

    // Apply throttle
    float throttlePct = sbusInput_.getThrottle();
    int throttleAngle = map((int)throttlePct, 0, 100, THROTTLE_MIN_ANGLE, THROTTLE_MAX_ANGLE);
    throttle_.setAngle(throttleAngle);

    // Apply gear selection
    TransmissionController::Gear gear = sbusInput_.getGear();
    transmission_.setGear(gear);

    // Apply brake
    float brakePct = sbusInput_.getBrake();
    applyBrake(brakePct);

    // Apply ignition state with automatic cranking
    SBusInput::IgnitionState ignitionState = sbusInput_.getIgnitionState();
    RelayController::IgnitionState relayIgnitionState;

    switch (ignitionState) {
        case SBusInput::IgnitionState::OFF:
            relayIgnitionState = RelayController::IgnitionState::OFF;
            break;
        case SBusInput::IgnitionState::ACC:
            relayIgnitionState = RelayController::IgnitionState::ACC;
            break;
        case SBusInput::IgnitionState::IGNITION:
            // Check if this is a fresh transition to IGNITION (from OFF or ACC)
            if (previousSBusIgnitionState_ != SBusInput::IgnitionState::IGNITION) {
                // Fresh transition - start automatic cranking
                relayIgnitionState = RelayController::IgnitionState::CRANKING;
                Debug::println("[VEHICLE] IGNITION detected - starting automatic cranking");
            } else {
                // Already in IGNITION state - maintain current relay state
                // (RelayController will auto-transition from CRANKING to IGNITION when ready)
                relayIgnitionState = relayController_.getIgnitionState();
            }
            break;
    }

    // Update relay state
    relayController_.setIgnitionState(relayIgnitionState);

    // Track state for next iteration
    previousSBusIgnitionState_ = ignitionState;

    // Apply front light
    bool frontLightOn = sbusInput_.getFrontLight();
    relayController_.setFrontLight(frontLightOn);
}

void VehicleController::processGearCommand(const String& gearStr, WebPortal& webPortal) {
    TransmissionController::Gear targetGear;
    String gearName;

    if (gearStr == "R") {
        targetGear = TransmissionController::Gear::GEAR_REVERSE;
        gearName = "REVERSE";
    } else if (gearStr == "N") {
        targetGear = TransmissionController::Gear::GEAR_NEUTRAL;
        gearName = "NEUTRAL";
    } else if (gearStr == "L") {
        targetGear = TransmissionController::Gear::GEAR_LOW;
        gearName = "LOW";
    } else if (gearStr == "H") {
        targetGear = TransmissionController::Gear::GEAR_HIGH;
        gearName = "HIGH";
    } else {
        webPortal.sendResponse(false, "Invalid gear selection");
        return;
    }

    if (transmission_.setGear(targetGear)) {
        webPortal.sendResponse(true, "Moving to " + gearName + " gear");
    } else {
        webPortal.sendResponse(false, "Already at " + gearName + " gear");
    }
}

void VehicleController::processSteeringCommand(float value, WebPortal& webPortal) {
    // Convert -100 to +100 percentage to servo angle (0-180 degrees)
    int angle = map((int)value, -100, 100, STEERING_MIN_ANGLE, STEERING_MAX_ANGLE);
    steering_.setAngle(angle);
    webPortal.sendResponse(true, "Steering set");
}

void VehicleController::processThrottleCommand(float value, WebPortal& webPortal) {
    // Convert 0 to 100 percentage to servo angle (0-180 degrees)
    int angle = map((int)value, 0, 100, THROTTLE_MIN_ANGLE, THROTTLE_MAX_ANGLE);
    throttle_.setAngle(angle);
    webPortal.sendResponse(true, "Throttle set");
}

void VehicleController::processBrakeCommand(float value, WebPortal& webPortal) {
    // Validate brake percentage range
    if (value < 0.0f || value > 100.0f) {
        webPortal.sendResponse(false, "Invalid brake value (must be 0-100)");
        return;
    }

    // Apply brake percentage (uses same mechanism as SBUS control)
    applyBrake(value);
    webPortal.sendResponse(true, "Brake set to " + String((int)value) + "%");
}

void VehicleController::processCalibrationCommand(WebPortal& webPortal) {
    Debug::println("[WEB] Calibration command received");
    webPortal.sendResponse(true, "Starting transmission calibration...");

    // Run calibration with max speed (255) and 20 second timeout
    bool success = transmission_.calibrateAllGearPositions(255, 20000);

    if (success) {
        Debug::println("[WEB] Calibration completed successfully");
        webPortal.sendResponse(true, "Calibration completed successfully");
    } else {
        Debug::println("[WEB] Calibration failed");
        webPortal.sendResponse(false, "Calibration failed - check serial output");
    }
}

void VehicleController::processClearCalibrationCommand(WebPortal& webPortal) {
    Debug::println("[WEB] Clear calibration command received");

    transmission_.clearCalibration();

    webPortal.sendResponse(true, "Calibration cleared - reboot to recalibrate");
}

bool VehicleController::setIgnitionState(const String& state, String& errorMsg) {
    RelayController::IgnitionState currentState = relayController_.getIgnitionState();
    RelayController::IgnitionState targetState;

    // Parse target state
    if (state == "OFF") {
        targetState = RelayController::IgnitionState::OFF;
    } else if (state == "ACC") {
        targetState = RelayController::IgnitionState::ACC;
    } else if (state == "IGNITION") {
        targetState = RelayController::IgnitionState::IGNITION;
    } else if (state == "START") {
        targetState = RelayController::IgnitionState::CRANKING;
    } else {
        errorMsg = "Invalid ignition state";
        return false;
    }

    // Safety interlock: require brake applied before powering on from OFF
    if (currentState == RelayController::IgnitionState::OFF &&
        targetState != RelayController::IgnitionState::OFF) {
        // if (currentBrakeTarget_ < 20.0f) {
        //     errorMsg = "Apply brake before ignition";
        //     Debug::println("[IGNITION] Rejected: brake not applied (need >= 20%)");
        //     return false;
        // }
    }

    // Safety interlock: prevent cranking if engine already running
    if (targetState == RelayController::IgnitionState::CRANKING) {
        CANController::VehicleData canData = canController_.getVehicleData();
        if (canData.dataValid && canData.engineRPM >= ENGINE_RUNNING_RPM_THRESHOLD) {
            errorMsg = "Engine already running";
            Debug::printf("[IGNITION] Rejected: engine already running (RPM: %d)\n", canData.engineRPM);
            return false;
        }
    }

    // Apply ignition state
    relayController_.setIgnitionState(targetState);
    Debug::printf("[IGNITION] State changed: %s\n", state.c_str());
    return true;
}

void VehicleController::setFrontLight(bool on) {
    relayController_.setFrontLight(on);
    Debug::printf("[LIGHT] Front light: %s\n", on ? "ON" : "OFF");
}

void VehicleController::processIgnitionCommand(const String& state, WebPortal& webPortal) {
    String errorMsg;
    if (setIgnitionState(state, errorMsg)) {
        webPortal.sendResponse(true, "Ignition set to " + state);
    } else {
        webPortal.sendResponse(false, errorMsg);
    }
}

void VehicleController::processLightCommand(bool on, WebPortal& webPortal) {
    setFrontLight(on);
    webPortal.sendResponse(true, String("Front light ") + (on ? "ON" : "OFF"));
}

void VehicleController::applyBrake(float brakePct) {
    // Clamp brake percentage to valid range
    if (brakePct < 0.0f) brakePct = 0.0f;
    if (brakePct > 100.0f) brakePct = 100.0f;

    // Update target
    currentBrakeTarget_ = brakePct;
}

void VehicleController::updateBrakeControl() {
    // Special case: If target is 0 (released), check sensor for confirmation
    if (currentBrakeTarget_ == 0.0f && isBrakeReleased()) {
        // Sensor confirms brake is fully released
        if (currentBrakePosition_ != 0.0f) {
            Debug::println("[BRAKE] Sensor confirms released, syncing position to 0%");
            currentBrakePosition_ = 0.0f;
        }

        // Stop actuator if moving
        if (brakeIsMoving_) {
            uint32_t movementDuration = millis() - brakeMovementStartTime_;
            Debug::printf("[BRAKE] Stopped by sensor. Movement: %lums\n", movementDuration);
            brakeIsMoving_ = false;
        }
        brake_.stop();
        return;  // Exit early, brake is confirmed released
    }

    // Calculate position error
    float positionError = currentBrakeTarget_ - currentBrakePosition_;

    // Check if we need to move
    if (abs(positionError) > BRAKE_TOLERANCE) {
        // Brake needs to move
        if (!brakeIsMoving_) {
            // Just started moving
            brakeMovementStartTime_ = millis();
            brakeIsMoving_ = true;
        }

        // Calculate speed based on error magnitude
        // Map 0-100% error to 0-255 speed
        float speedFloat = abs(positionError) * 2.55f;  // Convert % to 0-255 scale

        // Apply minimum speed to overcome friction
        if (speedFloat < BRAKE_MIN_SPEED) {
            speedFloat = BRAKE_MIN_SPEED;
        }

        // Clamp to motor limits
        if (speedFloat > 255.0f) {
            speedFloat = 255.0f;
        }

        int16_t speed = (int16_t)speedFloat;

        // Determine direction
        // Positive = extend (apply brake)
        // Negative = retract (release brake)
        if (positionError < 0.0f) {
            speed = -speed;  // Need to retract (release)
        }

        // Apply speed to actuator
        brake_.setSpeed(speed);

        // Estimate position based on time and speed
        // This is a rough estimate - in reality you'd need position feedback
        // Assume actuator takes ~3 seconds to go from 0 to 100%
        const float FULL_TRAVEL_TIME = BRAKE_FULL_TRAVEL_TIME;  // ms for 0-100%
        float movementRate = (100.0f / FULL_TRAVEL_TIME) * abs(speed) / 255.0f;  // %/ms

        // Update estimated position
        static uint32_t lastUpdateTime = millis();
        uint32_t now = millis();
        uint32_t dt = now - lastUpdateTime;
        lastUpdateTime = now;

        if (positionError > 0.0f) {
            currentBrakePosition_ += movementRate * dt;
        } else {
            currentBrakePosition_ -= movementRate * dt;
        }

        // Clamp position
        if (currentBrakePosition_ < 0.0f) currentBrakePosition_ = 0.0f;
        if (currentBrakePosition_ > 100.0f) currentBrakePosition_ = 100.0f;

    } else {
        // Brake is at target position (within tolerance)
        if (brakeIsMoving_) {
            // Just stopped moving
            uint32_t movementDuration = millis() - brakeMovementStartTime_;
            Debug::printf("[BRAKE] Stopped. Movement: %lums\n", movementDuration);
            brakeIsMoving_ = false;
        }

        // Stop actuator
        brake_.stop();
    }
}

void VehicleController::updateThrottleBoost() {
    // Safety check 1: disable boost if brake is applied
    if (currentBrakeTarget_ > TRANS_THROTTLE_BOOST_BRAKE_THRESHOLD) {
        if (throttleBoostActive_) {
            Debug::println("[BOOST] Disabled due to brake application");
            throttleBoostActive_ = false;
        }
        // Set throttle to idle when brake is applied
        throttle_.setAngle(THROTTLE_IDLE_ANGLE);
        return;
    }

    // Safety check 2: disable boost if SBUS is commanding throttle
    if (currentInputSource_ == InputSource::SBUS && sbusInput_.isSignalValid()) {
        float sbusThrottle = sbusInput_.getThrottle();
        if (sbusThrottle > 5.0f) {  // SBUS commanding throttle, let it take priority
            if (throttleBoostActive_) {
                Debug::println("[BOOST] Disabled due to SBUS throttle command");
                throttleBoostActive_ = false;
            }
            return;  // SBUS will set throttle, don't override
        }
    }

    // Check if we need to start boost
    if (!throttleBoostActive_) {
        throttleBoostStartTime_ = millis();
        throttleBoostActive_ = true;
        Debug::printf("[BOOST] Activated at %d%%\n", TRANS_THROTTLE_BOOST_PERCENT);
    }

    // Check timeout
    uint32_t boostDuration = millis() - throttleBoostStartTime_;
    if (boostDuration > TRANS_THROTTLE_BOOST_DURATION) {
        Debug::println("[BOOST] Timeout, releasing boost");
        throttleBoostActive_ = false;
        throttle_.setAngle(THROTTLE_IDLE_ANGLE);
        return;
    }

    // Apply boost throttle
    int boostAngle = map(TRANS_THROTTLE_BOOST_PERCENT, 0, 100,
                        THROTTLE_MIN_ANGLE, THROTTLE_MAX_ANGLE);
    throttle_.setAngle(boostAngle);
}
