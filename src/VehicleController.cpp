#include "VehicleController.h"

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
      brakeIsMoving_(false) {
}

void VehicleController::update() {
    // Process SBUS commands if SBUS is active
    if (currentInputSource_ == InputSource::SBUS) {
        processSBusCommands();
    }

    // Apply fail-safe if needed
    applyFailsafe();

    // Update position control for transmission actuator
    transmission_.update();

    // Update brake actuator control
    updateBrakeControl();
}

void VehicleController::setInputSource(InputSource source) {
    if (currentInputSource_ != source) {
        Serial.printf("[INPUT] Source changed: %s -> %s\n",
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
    if (currentInputSource_ != InputSource::WEB) {
        // Web control not active, ignore commands
        return;
    }

    if (!cmd.hasCommand) {
        return;
    }

    // Process command based on type
    if (cmd.cmd == "set_gear") {
        processGearCommand(cmd.strValue, webPortal);
    } else if (cmd.cmd == "set_steering") {
        processSteeringCommand(cmd.floatValue, webPortal);
    } else if (cmd.cmd == "set_throttle") {
        processThrottleCommand(cmd.floatValue, webPortal);
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
        Serial.println("[FAILSAFE] Entering safe state");
        steering_.setAngle(STEERING_CENTER_ANGLE);
        throttle_.setAngle(THROTTLE_IDLE_ANGLE);
        brake_.stop();         // Stop brake actuator (hold position)
        transmission_.stop();  // Stop transmission actuator
        relayController_.allOff();  // Turn off ignition and lights
        failsafeApplied_ = true;
    } else if (currentInputSource_ != InputSource::FAILSAFE && failsafeApplied_) {
        Serial.println("[FAILSAFE] Exiting safe state");
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

    // Apply ignition state
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
            relayIgnitionState = RelayController::IgnitionState::IGNITION;
            break;
    }
    relayController_.setIgnitionState(relayIgnitionState);

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
            Serial.println("[BRAKE] Sensor confirms released, syncing position to 0%");
            currentBrakePosition_ = 0.0f;
        }

        // Stop actuator if moving
        if (brakeIsMoving_) {
            uint32_t movementDuration = millis() - brakeMovementStartTime_;
            Serial.printf("[BRAKE] Stopped by sensor. Movement: %lums\n", movementDuration);
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
            Serial.printf("[BRAKE] Stopped. Movement: %lums\n", movementDuration);
            brakeIsMoving_ = false;
        }

        // Stop actuator
        brake_.stop();
    }
}
