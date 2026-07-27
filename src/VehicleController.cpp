#include "VehicleController.h"
#include "Debug.h"
#include <Preferences.h>

VehicleController::VehicleController(SteeringController& steering,
                                     ThrottleController& throttle,
                                     TransmissionController& transmission,
                                     BTS7960Controller& brake,
                                     MavlinkInterface& mavlink,
                                     RelayController& relayController)
    : steering_(steering),
      throttle_(throttle),
      transmission_(transmission),
      brake_(brake),
      mavlink_(mavlink),
      relayController_(relayController),
      currentInputSource_(InputSource::FAILSAFE),
      failsafeApplied_(false),
      webControl_(false),
      currentBrakeTarget_(0.0f),
      currentBrakePosition_(100.0f),
      brakeMovementStartTime_(0),
      lastBrakeUpdateTime_(0),
      brakeSensorTriggerTime_(0),
      brakeIsMoving_(false),
      boostTargetRpm_(TRANS_GEAR_BOOST_TARGET_RPM),
      gearBoostActive_(false),
      boostManualActive_(false),
      gearBoostStartTime_(0),
      pidLastUpdateTime_(0),
      lastCanUpdateTime_(0),
      pidIntegral_(0.0f),
      pidPrevError_(0.0f),
      lastPIDThrottleUs_(THROTTLE_DEFAULT_IDLE_US),
      previousIgnitionState_(MavlinkInterface::IgnitionState::OFF),
      lastCommandedGear_(TransmissionController::Gear::GEAR_UNKNOWN),
      transmissionInitialized_(false) {
}

bool VehicleController::initCAN() {
    Preferences prefs;
    if (prefs.begin("boost", true)) {
        boostTargetRpm_ = prefs.getInt("target_rpm", TRANS_GEAR_BOOST_TARGET_RPM);
        prefs.end();
    }
    bool result = canController_.begin();
    canController_.setRPMPollInterval(CAN_POLL_INTERVAL_RPM);
    return result;
}

void VehicleController::update() {
    // Update CAN controller (read vehicle data from ECU)
    canController_.update();

    // Restore transmission state once, on first engine-running detection
    if (!transmissionInitialized_ && isEngineRunning()) {
        transmissionInitialized_ = true;
        if (!transmission_.restoreStateIfValid()) {
            transmission_.setGear(TransmissionController::Gear::GEAR_NEUTRAL);
        }
        Debug::printlnFeature(DebugFeature::VEHICLE, "[VEHICLE] Transmission initialized on engine start");
    }

    // Pass vehicle data to transmission for safety checks
    CANController::VehicleData canData = canController_.getVehicleData();
    TransmissionVehicleData transData;
    transData.vehicleSpeed = canData.vehicleSpeed;
    transData.lastUpdateTime = canData.lastUpdateTime;
    transData.dataValid = canData.dataValid;
    transmission_.setVehicleData(transData);

    // Update relay controller with engine RPM (for automatic cranking stop)
    relayController_.update(canData.engineRPM);

    // Process MAVLink commands if MAVLink is active
    if (currentInputSource_ == InputSource::MAVLINK) {
        processMavlinkCommands();
    }

    // Apply fail-safe if needed
    applyFailsafe();

    // PID-controlled RPM boost during gear changes or manual test (overrides MAVLink/web throttle)
    if (transmission_.needsThrottleBoost() || gearBoostActive_ || boostManualActive_) {
        updateGearBoostPID();
    }

    // Update steering position control
    steering_.update();

    // Update position control for transmission actuator
    transmission_.update();

    // Update brake actuator control
    updateBrakeControl();
}

void VehicleController::setInputSource(InputSource source) {
    if (currentInputSource_ != source) {
        Debug::printfFeature(DebugFeature::VEHICLE, "[INPUT] Source changed: %s -> %s\n",
                     (currentInputSource_ == InputSource::MAVLINK) ? INPUT_SOURCE_NAME_MAVLINK :
                     (currentInputSource_ == InputSource::WEB) ? INPUT_SOURCE_NAME_WEB :
                     INPUT_SOURCE_NAME_FAILSAFE,
                     (source == InputSource::MAVLINK) ? INPUT_SOURCE_NAME_MAVLINK :
                     (source == InputSource::WEB) ? INPUT_SOURCE_NAME_WEB :
                     INPUT_SOURCE_NAME_FAILSAFE);
    }
    currentInputSource_ = source;
}

void VehicleController::setWebControl(bool on) {
    if (webControl_ == on) return;
    webControl_ = on;
    Debug::printfFeature(DebugFeature::VEHICLE, "[INPUT] Web control %s\n", on ? "ENGAGED (MAVLink ignored)" : "RELEASED");
    if (on) {
        // Snap to a safe state on takeover so we don't inherit the autopilot's live throttle.
        // Steering / gear / brake hold their current positions (no lurch).
        throttle_.idle();
        gearBoostActive_ = false;
        boostManualActive_ = false;
    }
}

void VehicleController::processWebCommand(const WebPortal::WebCommand& cmd, WebPortal& webPortal) {
    if (!cmd.hasCommand) {
        return;
    }
    Debug::printfFeature(DebugFeature::VEHICLE, "[WEB] Command %s\n", cmd.cmd);
    // Special commands that work regardless of input source
    if (cmd.cmd == "set_gear_default") {
        processSetGearDefaultCommand(cmd.strValue, cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "move_to_position") {
        processMoveToPositionCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_ignition") {
        // Ignition control always available (not restricted by input source)
        processIgnitionCommand(cmd.strValue, webPortal);
        return;
    } else if (cmd.cmd == "set_light") {
        // Light control always available (not restricted by input source)
        processLightCommand(cmd.boolValue, webPortal);
        return;
    } else if (cmd.cmd == "set_wheel_lock") {
        // Front-wheel lock always available (not restricted by input source)
        processWheelLockCommand(cmd.boolValue, webPortal);
        return;
    } else if (cmd.cmd == "steer_cal_center" || cmd.cmd == "steer_cal_left" || cmd.cmd == "steer_cal_right") {
        processSteerCalCommand(cmd.cmd, webPortal);
        return;
    } else if (cmd.cmd == "steer_jog") {
        processSteerJogCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "steer_nudge") {
        processSteerNudgeCommand((int8_t)cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_web_control") {
        processWebControlCommand(cmd.boolValue, webPortal);
        return;
    } else if (cmd.cmd == "test_boost") {
        processBoostTestCommand(cmd.boolValue, webPortal);
        return;
    } else if (cmd.cmd == "set_boost_rpm") {
        processSetBoostRpmCommand((int32_t)cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "throttle_cal_begin") {
        processThrottleCalBegin(webPortal);
        return;
    } else if (cmd.cmd == "throttle_cal_idle") {
        processThrottleCalJog(false, (uint16_t)cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "throttle_cal_full") {
        processThrottleCalJog(true, (uint16_t)cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "throttle_cal_save") {
        processThrottleCalSave(webPortal);
        return;
    } else if (cmd.cmd == "throttle_cal_cancel") {
        processThrottleCalCancel(webPortal);
        return;
    }

    Debug::printfFeature(DebugFeature::VEHICLE, "[WEB] Command end %s\n", cmd.cmd);
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

String VehicleController::getTargetGearString() const {
    TransmissionController::Gear gear = transmission_.getTargetGear();
    switch (gear) {
        case TransmissionController::Gear::GEAR_REVERSE: return "R";
        case TransmissionController::Gear::GEAR_NEUTRAL: return "N";
        case TransmissionController::Gear::GEAR_LOW: return "L";
        case TransmissionController::Gear::GEAR_HIGH: return "H";
        default: return "N";
    }
}

String VehicleController::getFromGearString() const {
    TransmissionController::Gear gear = transmission_.getFromGear();
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
        Debug::printlnFeature(DebugFeature::VEHICLE, "[FAILSAFE] Entering safe state");
        steering_.setSteeringPercent(0.0f);
        throttle_.idle();
        brake_.stop();         // Stop brake actuator (hold position)
        brakeIsMoving_ = false;
        brakeSensorTriggerTime_ = 0;  // Reset sensor trigger
        transmission_.stop();  // Stop transmission actuator
        relayController_.allOff();  // Turn off ignition and lights
        previousIgnitionState_ = MavlinkInterface::IgnitionState::OFF;  // Reset ignition tracking
        lastCommandedGear_ = TransmissionController::Gear::GEAR_UNKNOWN;  // Force re-eval on restore
        failsafeApplied_ = true;
    } else if (currentInputSource_ != InputSource::FAILSAFE && failsafeApplied_) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[FAILSAFE] Exiting safe state");
        brake_.stop();  // Ensure brake is stopped before handing control
        failsafeApplied_ = false;
    }
}

void VehicleController::processMavlinkCommands() {
    if (!mavlink_.isSignalValid()) {
        return;  // Safety check
    }

    // Apply steering
    float steeringPct = mavlink_.getSteering();
    steering_.setSteeringPercent(steeringPct);

    // Apply throttle — skipped when gear boost PID is active (PID overrides)
    if (!gearBoostActive_) {
        float throttlePct = mavlink_.getThrottle();
        if (shouldClipThrottle()) {
            throttlePct = min(TRANS_UNKNOWN_GEAR_THROTTLE_MAX, throttlePct);
        }
        throttle_.setThrottlePercent(throttlePct);
    }

    // Apply gear selection — retry until setGear() accepts the command
    TransmissionController::Gear gear = mavlink_.getGear();
    if (gear != lastCommandedGear_ && isEngineRunning()) {
        bool accepted = transmission_.setGear(gear);
        if (accepted || transmission_.getTargetGear() == gear) {
            lastCommandedGear_ = gear;
        }
        // If not accepted (e.g. speed interlock), lastCommandedGear_ stays unchanged → retry next loop
    }

    // Apply brake
    float brakePct = mavlink_.getBrake();
    applyBrake(brakePct);

    // Apply ignition state. Cranking is sequenced inside RelayController, which enforces a
    // pre-crank dwell: the starter (R2) does not engage until the ignition/ECU line (R1) has
    // been powered for ACC_PRECRANK_DWELL_MS (immediately if R1 was already up that long).
    MavlinkInterface::IgnitionState ignitionState = mavlink_.getIgnitionState();

    switch (ignitionState) {
        case MavlinkInterface::IgnitionState::OFF:
            relayController_.setIgnitionState(RelayController::IgnitionState::OFF);
            break;
        case MavlinkInterface::IgnitionState::ACC:
            relayController_.setIgnitionState(RelayController::IgnitionState::ACC);
            break;
        case MavlinkInterface::IgnitionState::IGNITION:
            // Arm the dwell-gated auto-crank only on a FRESH transition into IGNITION.
            // Holding IGNITION does not re-crank; RelayController::update() runs the sequence
            // (ACC hold -> dwell -> CRANKING -> IGNITION) and reports ACC while waiting.
            if (previousIgnitionState_ != MavlinkInterface::IgnitionState::IGNITION) {
                Debug::printlnFeature(DebugFeature::VEHICLE, "[VEHICLE] IGNITION selected - arming pre-crank dwell");
                relayController_.requestCrank();
            }
            break;
    }

    // Track state for next iteration (fresh-transition detection)
    previousIgnitionState_ = ignitionState;

    // Apply front light
    bool frontLightOn = mavlink_.getFrontLight();
    relayController_.setFrontLight(frontLightOn);

    // Apply front-wheel lock (channel 7)
    relayController_.setWheelLock(mavlink_.getWheelLock());
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
        Debug::printfFeature(DebugFeature::VEHICLE, "[WEB] Invalid gear selection %s\n", gearStr);
        webPortal.sendResponse(false, "Invalid gear selection");
        return;
    }

    if (transmission_.setGear(targetGear)) {
        Debug::printfFeature(DebugFeature::VEHICLE, "[WEB] Moving %s\n", gearName);
        webPortal.sendResponse(true, "Moving to " + gearName + " gear");
    } else {
        Debug::printfFeature(DebugFeature::VEHICLE, "[WEB] Already at %s\n", gearName);
        webPortal.sendResponse(false, "Already at " + gearName + " gear");
    }
}

void VehicleController::processSteeringCommand(float value, WebPortal& webPortal) {
    if (!steering_.setSteeringPercent(value)) {
        webPortal.sendResponse(false, "Steering rejected (not calibrated or sensor fault)");
        return;
    }
    webPortal.sendResponse(true, "Steering set");
}

bool VehicleController::shouldClipThrottle() const {
    if (transmission_.isGearChangeActive())
        return true;
    if (transmission_.getTargetGear() == TransmissionController::Gear::GEAR_NEUTRAL)
        return true;
    return false;
}

void VehicleController::processThrottleCommand(float value, WebPortal& webPortal) {
    if (gearBoostActive_) {
        webPortal.sendResponse(false, "Gear boost active");
        return;
    }
    if (shouldClipThrottle()) {
        value = min(TRANS_UNKNOWN_GEAR_THROTTLE_MAX, value);
    }
    throttle_.setThrottlePercent(value);
    webPortal.sendResponse(true, "Throttle set");
}

void VehicleController::processThrottleCalBegin(WebPortal& webPortal) {
    if (!throttle_.beginCalibration(isEngineRunning())) {
        webPortal.sendResponse(false, "Cannot calibrate while engine is running");
        return;
    }
    webPortal.sendResponse(true, "Throttle calibration started");
}

void VehicleController::processThrottleCalJog(bool isFull, uint16_t us, WebPortal& webPortal) {
    if (!throttle_.isCalibrating()) {
        webPortal.sendResponse(false, "Throttle calibration not active");
        return;
    }
    if (isFull) throttle_.jogFull(us);
    else        throttle_.jogIdle(us);
    webPortal.sendResponse(true, "Throttle preview");
}

void VehicleController::processThrottleCalSave(WebPortal& webPortal) {
    String err;
    if (!throttle_.saveCalibration(err)) {
        webPortal.sendResponse(false, err);
        return;
    }
    webPortal.sendResponse(true, "Throttle calibration saved");
}

void VehicleController::processThrottleCalCancel(WebPortal& webPortal) {
    throttle_.cancelCalibration();
    webPortal.sendResponse(true, "Throttle calibration cancelled");
}

void VehicleController::processBrakeCommand(float value, WebPortal& webPortal) {
    // Validate brake percentage range
    if (value < 0.0f || value > 100.0f) {
        webPortal.sendResponse(false, "Invalid brake value (must be 0-100)");
        return;
    }

    // Apply brake percentage (uses same mechanism as MAVLink control)
    applyBrake(value);
    webPortal.sendResponse(true, "Brake set to " + String((int)value) + "%");
}

void VehicleController::processSetGearDefaultCommand(const String& gearStr, float positionPct, WebPortal& webPortal) {
    TransmissionController::Gear gear;
    if      (gearStr == "R") gear = TransmissionController::Gear::GEAR_REVERSE;
    else if (gearStr == "N") gear = TransmissionController::Gear::GEAR_NEUTRAL;
    else if (gearStr == "L") gear = TransmissionController::Gear::GEAR_LOW;
    else if (gearStr == "H") gear = TransmissionController::Gear::GEAR_HIGH;
    else {
        webPortal.sendResponse(false, "Invalid gear (use R/N/L/H)");
        return;
    }

    if (!transmission_.setDefaultPosition(gear, positionPct)) {
        webPortal.sendResponse(false, "Position out of range (0.0-100.0)");
        return;
    }

    String msg = "Default for " + gearStr + " set to " + String(positionPct, 1) + "%";
    Debug::printfFeature(DebugFeature::VEHICLE, "[WEB] %s\n", msg.c_str());
    webPortal.sendResponse(true, msg);
}

void VehicleController::processMoveToPositionCommand(float positionPct, WebPortal& webPortal) {
    if (positionPct < 0.0f || positionPct > 100.0f) {
        webPortal.sendResponse(false, "Position out of range (0.0-100.0)");
        return;
    }
    transmission_.moveToPercent(positionPct);
    Debug::printfFeature(DebugFeature::VEHICLE, "[WEB] Moving transmission to %.1f%%\n", positionPct);
    webPortal.sendResponse(true, "Moving to " + String(positionPct, 1) + "%");
}

void VehicleController::processBoostTestCommand(bool enable, WebPortal& webPortal) {
    if (enable) {
        boostManualActive_ = true;
        Debug::printlnFeature(DebugFeature::VEHICLE, "[BOOST] Manual test activated");
        webPortal.sendResponse(true, "Boost test started");
    } else {
        boostManualActive_ = false;
        if (gearBoostActive_) {
            canController_.setRPMPollInterval(CAN_POLL_INTERVAL_RPM);
            gearBoostActive_ = false;
            throttle_.idle();
        }
        Debug::printlnFeature(DebugFeature::VEHICLE, "[BOOST] Manual test stopped");
        webPortal.sendResponse(true, "Boost test stopped");
    }
}

void VehicleController::processSetBoostRpmCommand(int32_t rpm, WebPortal& webPortal) {
    if (rpm < 1700 || rpm > 2500) {
        webPortal.sendResponse(false, "Boost RPM out of range (1700-2500)");
        return;
    }
    boostTargetRpm_ = rpm;
    Preferences prefs;
    if (prefs.begin("boost", false)) {
        prefs.putInt("target_rpm", rpm);
        prefs.end();
    }
    Debug::printfFeature(DebugFeature::VEHICLE, "[BOOST] Target RPM set to %ld\n", rpm);
    webPortal.sendResponse(true, "Boost RPM set to " + String(rpm));
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

    // Safety interlock: prevent cranking if engine already running
    if (targetState == RelayController::IgnitionState::CRANKING) {
        CANController::VehicleData canData = canController_.getVehicleData();
        if (canData.dataValid && canData.engineRPM >= ENGINE_RUNNING_RPM_THRESHOLD) {
            errorMsg = "Engine already running";
            Debug::printfFeature(DebugFeature::VEHICLE, "[IGNITION] Rejected: engine already running (RPM: %d)\n", canData.engineRPM);
            return false;
        }
    }

    // Apply ignition state
    relayController_.setIgnitionState(targetState);
    Debug::printfFeature(DebugFeature::VEHICLE, "[IGNITION] State changed: %s\n", state.c_str());
    return true;
}

void VehicleController::setFrontLight(bool on) {
    relayController_.setFrontLight(on);
    Debug::printfFeature(DebugFeature::VEHICLE, "[LIGHT] Front light: %s\n", on ? "ON" : "OFF");
}

void VehicleController::setWheelLock(bool locked) {
    relayController_.setWheelLock(locked);
    Debug::printfFeature(DebugFeature::VEHICLE, "[WHEEL_LOCK] Front wheels: %s\n", locked ? "LOCKED" : "UNLOCKED");
}

void VehicleController::processWheelLockCommand(bool locked, WebPortal& webPortal) {
    setWheelLock(locked);
    webPortal.sendResponse(true, String("Front wheels ") + (locked ? "LOCKED" : "UNLOCKED"));
}

void VehicleController::processSteerCalCommand(const String& which, WebPortal& webPortal) {
    bool ok;
    String what;
    if (which == "steer_cal_center") {
        ok = steering_.captureCenter();
        what = "center";
    } else if (which == "steer_cal_left") {
        ok = steering_.captureLeftLimit();
        what = "left limit";
    } else {
        ok = steering_.captureRightLimit();
        what = "right limit";
    }

    if (!ok) {
        webPortal.sendResponse(false, "Capture " + what + " failed (sensor fault, no center, or too close to center)");
        return;
    }
    webPortal.sendResponse(true, "Steering " + what + " saved: " + String(steering_.getRawAngle()) +
                                 (steering_.isCalibrated() ? "" : " (calibration incomplete)"));
}

void VehicleController::processSteerJogCommand(float value, WebPortal& webPortal) {
    // value encodes direction and speed: sign = direction, |value| = PWM duty.
    // |value| <= 1 (legacy -1/0/1 form) uses the default jog duty.
    int8_t direction = (value > 0.0f) ? 1 : (value < 0.0f) ? -1 : 0;
    float mag = fabsf(value);
    uint8_t duty = (mag > 1.0f) ? (uint8_t)min(mag, 255.0f) : STEER_JOG_DUTY;
    steering_.jog(direction, duty);
    webPortal.sendResponse(true, direction == 0 ? "Jog stop" : (direction > 0 ? "Jog right" : "Jog left"));
}

void VehicleController::processSteerNudgeCommand(int8_t direction, WebPortal& webPortal) {
    if (direction == 0) {
        webPortal.sendResponse(false, "Nudge needs a direction");
        return;
    }
    steering_.nudge(direction);
    webPortal.sendResponse(true, direction > 0 ? "Nudge right" : "Nudge left");
}

void VehicleController::processWebControlCommand(bool on, WebPortal& webPortal) {
    setWebControl(on);
    webPortal.sendResponse(true, on ? "Web control engaged" : "Web control released");
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
    if (currentBrakeTarget_ == brakePct) { return ;}
    // Clamp brake percentage to valid range
    if (brakePct < 5.0f) brakePct = 0.0f;
    if (brakePct > 100.0f) brakePct = 100.0f;

    // Update target
    currentBrakeTarget_ = brakePct;
}

void VehicleController::updateBrakeControl() {
    float positionError = currentBrakeTarget_ - currentBrakePosition_;

    // Special case: If target is 0% (released), check sensor for confirmation with overrun
    if (currentBrakeTarget_ == 0.0f && 
        isBrakeReleased() && 
        abs(positionError) < BRAKE_TOLERANCE) {

        // Sensor detected release
        if (brakeSensorTriggerTime_ == 0) {
            // First detection - start overrun timer
            brakeSensorTriggerTime_ = millis();
            Debug::printfFeature(DebugFeature::BRAKE,
                "[BRAKE] Sensor triggered at %.1f%%, continuing for %dms overrun\n",
                currentBrakePosition_, BRAKE_SENSOR_OVERRUN_TIME);
        }

        // Check if overrun period has elapsed
        uint32_t now = millis();
        uint32_t overrunDuration = now - brakeSensorTriggerTime_;

        if (overrunDuration >= BRAKE_SENSOR_OVERRUN_TIME) {
            // Overrun complete - stop and sync position
            if (currentBrakePosition_ != 0.0f) {
                Debug::printfFeature(DebugFeature::BRAKE,
                    "[BRAKE] Overrun complete (%lums), syncing position to 0%%\n", overrunDuration);
                currentBrakePosition_ = 0.0f;
            }

            // Stop actuator
            if (brakeIsMoving_) {
                uint32_t movementDuration = millis() - brakeMovementStartTime_;
                Debug::printfFeature(DebugFeature::BRAKE,
                    "[BRAKE] Stopped by sensor. Movement: %lums\n", movementDuration);
                brakeIsMoving_ = false;
                lastBrakeUpdateTime_ = 0;
            }
            brake_.stop();
            return;
        } else {
            // Still in overrun period - keep retracting
            brake_.setSpeed(-255);
            static uint32_t lastOverrunLog = 0;
            if (now - lastOverrunLog > 100) {
                Debug::printfFeature(DebugFeature::BRAKE,
                    "[BRAKE] Sensor overrun: %lums / %ldms remaining\n",
                    overrunDuration, (long)(BRAKE_SENSOR_OVERRUN_TIME - overrunDuration));
                lastOverrunLog = now;
            }
            return;
        }
    } else {
        // Not releasing or sensor not triggered - reset trigger time
        brakeSensorTriggerTime_ = 0;
    }

    // Check if we need to move
    if (abs(positionError) > BRAKE_TOLERANCE) {
        // Brake needs to move
        if (!brakeIsMoving_) {
            // Just started moving
            brakeMovementStartTime_ = millis();
            lastBrakeUpdateTime_ = brakeMovementStartTime_;  // Reset position tracking baseline
            brakeIsMoving_ = true;
            Debug::printfFeature(DebugFeature::BRAKE,
                "[BRAKE] Starting movement: %.1f%% -> %.1f%% (error: %.1f%%)\n",
                currentBrakePosition_, currentBrakeTarget_, positionError);
        }

        // Positive = extend (apply brake), negative = retract (release brake)
        float direction = (positionError > 0.0f) ? 1.0f : -1.0f;
        int16_t speed = (int16_t)(255 * direction);

        // Apply speed to actuator
        brake_.setSpeed(speed);

        // Update estimated position based on time and speed
        uint32_t now = millis();
        uint32_t dt = min(now - lastBrakeUpdateTime_, 100UL);  // Cap at 100ms

        float oldPosition = currentBrakePosition_;

        // Calculate movement rate based on speed
        float speedFraction = abs(speed) / 255.0f;
        float movementRate = (100.0f / BRAKE_FULL_TRAVEL_TIME) * speedFraction;  // %/ms
        float expectedMovement = movementRate * dt;

        currentBrakePosition_ += direction * expectedMovement;

        // Clamp to valid range
        if (currentBrakePosition_ < 0.0f) currentBrakePosition_ = 0.0f;
        if (currentBrakePosition_ > 100.0f) currentBrakePosition_ = 100.0f;

        // Detailed logging every 500ms
        static uint32_t lastProgressLog = 0;
        if (now - lastProgressLog > 500) {
            float actualMovement = currentBrakePosition_ - oldPosition;
            Debug::printfFeature(DebugFeature::BRAKE,
                "[BRAKE] Progress: %.1f%%→%.1f%% (Δ%.2f%% in %lums, rate=%.3f%%/ms)\n",
                oldPosition, currentBrakePosition_, actualMovement, dt, movementRate);
            Debug::printfFeature(DebugFeature::BRAKE,
                "[BRAKE] Target: %.1f%%, Error: %.1f%%, Duration: %lums, Speed: %d PWM, Dir: %s\n",
                currentBrakeTarget_, positionError, now - brakeMovementStartTime_, speed,
                direction > 0 ? "PRESS" : "RELEASE");
            lastProgressLog = now;
        }

        // Update timestamp
        lastBrakeUpdateTime_ = now;

    } else if (currentBrakeTarget_ != 0.0f) {
        // Brake is at target position (within tolerance)
        if (brakeIsMoving_) {
            // Just stopped moving
            uint32_t movementDuration = millis() - brakeMovementStartTime_;
            Debug::printfFeature(DebugFeature::BRAKE,
                "[BRAKE] Stopped at %.1f%%. Movement: %lums\n",
                currentBrakePosition_, movementDuration);
            brakeIsMoving_ = false;
            lastBrakeUpdateTime_ = 0;
            brakeSensorTriggerTime_ = 0;  // Reset sensor trigger
        }

        // Hold position against cylinder hydraulic preassure return
        brake_.setSpeed(BRAKE_HOLD_SPEED);
    }
}

void VehicleController::updateGearBoostPID() {
    uint32_t now = millis();
    CANController::VehicleData canData = canController_.getVehicleData();

    // Deactivate when gear change completes (skip if in manual test mode)
    if (gearBoostActive_ && !boostManualActive_ && !transmission_.needsThrottleBoost()) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[BOOST] Gear change complete, releasing PID");
        canController_.setRPMPollInterval(CAN_POLL_INTERVAL_RPM);
        throttle_.idle();
        gearBoostActive_ = false;
        return;
    }

    // Activate on first call (gear change just started, or manual test triggered)
    if (!gearBoostActive_) {
        if (!canData.dataValid || canData.engineRPM < ENGINE_RUNNING_RPM_THRESHOLD) {
            return;  // Engine not running — unsafe to boost throttle
        }
        gearBoostActive_ = true;
        gearBoostStartTime_ = now;
        pidLastUpdateTime_ = now;
        lastCanUpdateTime_ = 0;
        pidIntegral_ = 0.0f;
        pidPrevError_ = 0.0f;
        lastPIDThrottleUs_ = throttle_.getIdleUs();
        canController_.setRPMPollInterval(CAN_POLL_INTERVAL_RPM_BOOST);
        Debug::printfFeature(DebugFeature::VEHICLE, "[BOOST] PID activated, target=%ld RPM\n", boostTargetRpm_);
    }

    // Safety timeout
    if (now - gearBoostStartTime_ > TRANS_GEAR_BOOST_TIMEOUT) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[BOOST] Timeout, releasing gear boost PID");
        canController_.setRPMPollInterval(CAN_POLL_INTERVAL_RPM);
        gearBoostActive_ = false;
        throttle_.idle();
        return;
    }

    // CAN stale: hold last µs, don't accumulate integral
    if (!canData.dataValid) {
        throttle_.idle();
        return;
    }

    // No new CAN sample since last compute: hold last µs, skip integral accumulation
    if (canData.lastUpdateTime == lastCanUpdateTime_) {
        throttle_.setThrottleUs(lastPIDThrottleUs_);
        return;
    }
    lastCanUpdateTime_ = canData.lastUpdateTime;

    // Compute PID
    float dt = (now - pidLastUpdateTime_) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    pidLastUpdateTime_ = now;

    float error = (float)boostTargetRpm_ - (float)canData.engineRPM;

    // Reset integral on zero-crossing to prevent windup from fighting recovery
    if (pidPrevError_ != 0.0f && ((error > 0.0f) != (pidPrevError_ > 0.0f))) {
        pidIntegral_ = 0.0f;
    }

    // Anti-windup: clamp integral so its contribution is ≤ half of max output
    const float integralLimit = (TRANS_GEAR_BOOST_PID_KI > 0.0f)
        ? (TRANS_GEAR_BOOST_MAX_PCT * 0.5f / TRANS_GEAR_BOOST_PID_KI) : 0.0f;
    pidIntegral_ += error * dt;
    pidIntegral_ = constrain(pidIntegral_, -integralLimit, integralLimit);

    float derivative = (error - pidPrevError_) / dt;
    pidPrevError_ = error;

    float outputPct = TRANS_GEAR_BOOST_PID_KP * error
                    + TRANS_GEAR_BOOST_PID_KI * pidIntegral_
                    + TRANS_GEAR_BOOST_PID_KD * derivative;
    outputPct = constrain(outputPct, 0.0f, TRANS_GEAR_BOOST_MAX_PCT);

    uint16_t targetUs = throttle_.percentToUs(outputPct);
    int delta = constrain((int)targetUs - (int)lastPIDThrottleUs_, -TRANS_GEAR_BOOST_SLEW_RATE_US, TRANS_GEAR_BOOST_SLEW_RATE_US);
    uint16_t us = (uint16_t)((int)lastPIDThrottleUs_ + delta);
    lastPIDThrottleUs_ = us;
    throttle_.setThrottleUs(us);

    static uint32_t lastProgressLog = 0;
    if (now - lastProgressLog > 250) {
        Debug::printfFeature(DebugFeature::VEHICLE, "[BOOST] Us=%d Error=%.0f RPM=%d CAN_age=%lums\n", us, error, canData.engineRPM, now - canData.lastUpdateTime);
        lastProgressLog = now;
    }
}
