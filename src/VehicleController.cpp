#include "VehicleController.h"

VehicleController::VehicleController(ServoController& steering,
                                     ServoController& throttle,
                                     TransmissionController& transmission,
                                     BTS7960Controller& brake)
    : steering_(steering),
      throttle_(throttle),
      transmission_(transmission),
      brake_(brake),
      currentInputSource_(InputSource::FAILSAFE),
      failsafeApplied_(false) {
}

void VehicleController::update() {
    // Apply fail-safe if needed
    applyFailsafe();

    // Update position control for transmission actuator
    transmission_.update();
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
        failsafeApplied_ = true;
    } else if (currentInputSource_ != InputSource::FAILSAFE && failsafeApplied_) {
        Serial.println("[FAILSAFE] Exiting safe state - brake released for manual control");
        brake_.stop();  // Ensure brake is stopped before handing control
        failsafeApplied_ = false;
    }
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
