#include "VehicleController.h"
#include "Debug.h"
#include <Preferences.h>

VehicleController::VehicleController(SteeringController& steering,
                                     ThrottleController& throttle,
                                     TransmissionController& transmission,
                                     BTS7960Controller& brake,
                                     MavlinkInterface& mavlink,
                                     RelayController& relayController,
                                     SpeedSensor& speedSensor)
    : steering_(steering),
      throttle_(throttle),
      transmission_(transmission),
      brake_(brake),
      mavlink_(mavlink),
      relayController_(relayController),
      speedSensor_(speedSensor),
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
      transmissionInitialized_(false),
      lastSpeedLimitWarnMs_(0),
      lastSpeedLimitMs_(NAN),
      lastSpeedLimitLogMs_(0),
      limiterCeilingPct_(100.0f),
      limiterLastMs_(0),
      steerScaBaseMs_(0.0f),
      steerScaleApplied_(1.0f),
      steerScaleLastMs_(0),
      lastSteerScaleWarnMs_(0),
      lastSteerScaleLogged_(NAN),
      lastSteerScaleLogMs_(0),
      steerTestSpeedMs_(0.0f),
      steerTestSpeedSetMs_(0),
      webThrottleDemandPct_(0.0f) {
}

void VehicleController::initEngineHourMeter() {
    engineHours_.begin();
}

bool VehicleController::initCAN() {
    Preferences prefs;
    if (prefs.begin("boost", true)) {
        boostTargetRpm_ = prefs.getInt("target_rpm", TRANS_GEAR_BOOST_TARGET_RPM);
        prefs.end();
    }

    // The LOCAL steering speed-scaling base, from the SAME "steering" namespace as the steering
    // calibration. Read here rather than in the constructor: this controller is a global built
    // before NVS is ready. An absent key, a NaN, a negative value or anything above
    // STEER_SCALE_BASE_MAX_MS is treated as "not set" (0) — the autopilot's MOT_SPD_SCA_BASE
    // then supplies the base, and if it cannot, there is no scaling at all. A stored value is
    // never "repaired" into a plausible number: a base nobody chose must not restrict steering.
    if (prefs.begin("steering", true)) {
        float base = prefs.getFloat(STEER_SCALE_BASE_NVS_KEY, 0.0f);
        prefs.end();
        if (isnan(base) || base < 0.0f || base > STEER_SCALE_BASE_MAX_MS) {
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[STEER] Stored speed-scaling base %.2f out of range (0-%.1f m/s) — ignored\n",
                base, STEER_SCALE_BASE_MAX_MS);
            base = 0.0f;
        }
        steerScaBaseMs_ = base;
    }
    Debug::printfFeature(DebugFeature::VEHICLE,
        "[STEER] Speed-scaling base (local): %.2f m/s%s\n",
        steerScaBaseMs_,
        (steerScaBaseMs_ > 0.0f) ? "" : " — not set, will use the autopilot's MOT_SPD_SCA_BASE");

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

    // Pass vehicle data to transmission for safety checks. Speed comes from the hall
    // sensor (the CAN speed field is dead); the CAN fields still drive the timeout
    // fallback used when the sensor reading is unavailable.
    CANController::VehicleData canData = canController_.getVehicleData();
    TransmissionVehicleData transData;
    transData.vehicleSpeed = canData.vehicleSpeed;
    transData.lastUpdateTime = canData.lastUpdateTime;
    transData.dataValid = canData.dataValid;
    transData.sensorSpeedMs = speedSensor_.getSpeedMs();
    transData.sensorSpeedValid = speedSensor_.isValid();
    transData.sensorSpeedSuspicious = speedSensor_.isSuspicious();
    transmission_.setVehicleData(transData);

    // Engine hour meter, from the SAME snapshot the transmission just got — no second
    // getVehicleData() call, so the two consumers can never see different data in one
    // iteration. An invalid CAN reading means the engine state is UNKNOWN, not "off": the
    // interval is discarded rather than counted, because an unknown state must not invent
    // hours. ENGINE_HOURS_MIN_RPM, not ENGINE_RUNNING_RPM_THRESHOLD — an hour meter must
    // count idling, which the 1500 RPM gear-change safety threshold sits above.
    engineHours_.update(canData.dataValid && canData.engineRPM >= ENGINE_HOURS_MIN_RPM, millis());

    // Update relay controller with engine RPM (for automatic cranking stop)
    relayController_.update(canData.engineRPM);

    // Expire the bench test-speed override. The fuse is the whole point: an override left on
    // must not survive as a silent steering restriction, so it clears itself whether or not
    // anyone is looking at the portal.
    if (steerTestSpeedSetMs_ != 0 &&
        (millis() - steerTestSpeedSetMs_) >= STEER_SCALE_TEST_SPEED_MS) {
        steerTestSpeedMs_ = 0.0f;
        steerTestSpeedSetMs_ = 0;
        Debug::printlnFeature(DebugFeature::VEHICLE,
            "[STEER] TEST speed override expired — back to the measured speed");
    }

    // Process MAVLink commands if MAVLink is active
    if (currentInputSource_ == InputSource::MAVLINK) {
        processMavlinkCommands();
    } else {
        // The scale is a property of the AUTOPILOT command path alone. With the web or the
        // fail-safe in charge nothing is being scaled, so the reported scale must say so rather
        // than freeze at whatever the autopilot path last computed — a stale 0.18 on the portal
        // and on STEER_SCA would read as a restriction that is not actually in force. The slew
        // baseline is cleared too, so a return to MAVLink adopts its first target directly.
        steerScaleApplied_ = 1.0f;
        steerScaleLastMs_ = 0;
    }

    // Perform a latched trip reset. The transport validates and acknowledges the COMMAND_LONG
    // but never holds a SpeedSensor& — the vehicle layer owns the counters, so the request is
    // consumed here regardless of which input source is active.
    //
    // ONE gesture clears BOTH trip readings. The trip distance and the trip hours describe the
    // same "since the operator last pressed reset" interval in two units, so a command that
    // zeroed one and left the other would leave the pair permanently incomparable. This is the
    // ONLY call site of EngineHourMeter::resetTrip(): there is no second command, no extra
    // param1 magic and no web control — deliberately, so the two can never diverge.
    if (mavlink_.consumeTripResetRequest()) {
        speedSensor_.resetTrip();
        engineHours_.resetTrip();   // TRIP hours only — the TOTAL hour meter is untouched
    }

    // Apply fail-safe if needed
    applyFailsafe();

    // Re-apply the limiter to the STANDING web throttle demand every loop. The MAVLink path
    // already re-reads its channel every iteration, so it was always re-limited; the web path
    // used to clamp only at command time, which meant a vehicle accelerating past the ceiling on
    // an unchanged 80 % slider was never clamped until the operator touched the slider again.
    // Calibration owns the servo directly, and the gear-boost PID writes µs — both stay clear.
    if (currentInputSource_ == InputSource::WEB && !gearBoostActive_ && !throttle_.isCalibrating()) {
        float demandPct = webThrottleDemandPct_;
        if (shouldClipThrottle()) {
            demandPct = min(TRANS_UNKNOWN_GEAR_THROTTLE_MAX, demandPct);
        }
        throttle_.setThrottlePercent(applySpeedLimit(demandPct));
    }

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
        if (source == InputSource::WEB) {
            webThrottleDemandPct_ = 0.0f;
            throttle_.idle();
        }
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
        webThrottleDemandPct_ = 0.0f;   // never inherit a stale demand into the next loop
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
    } else if (cmd.cmd == "speed_cal_ppr") {
        processSpeedCalPprCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "speed_cal_circ") {
        processSpeedCalCircCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_steer_sca_base") {
        processSetSteerScaBaseCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_test_speed") {
        processSetTestSpeedCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "can_probe") {
        // ECU capability probe — available regardless of input source (diagnostic)
        processCanProbeCommand(webPortal);
        return;
    }

    Debug::printfFeature(DebugFeature::VEHICLE, "[WEB] Command end %s\n", cmd.cmd);
    if (currentInputSource_ != InputSource::WEB) {
        webPortal.sendResponse(false, "MAVLink control active");
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
        // Physical gear unknown (ambiguous switches or none active):
        // report it as unknown rather than presenting a plausible gear as current.
        default: return "?";
    }
}

int8_t VehicleController::getTravelDirection() const {
    // PHYSICAL gear only — see the header. Both forward ratios drive the same direction.
    switch (transmission_.getPhysicalGear()) {
        case TransmissionController::Gear::GEAR_REVERSE: return -1;
        case TransmissionController::Gear::GEAR_LOW:     return  1;
        case TransmissionController::Gear::GEAR_HIGH:    return  1;
        // NEUTRAL, and GEAR_UNKNOWN (ambiguous switches, none active, or the
        // brief mid-shift window): no direction to report, so the sample is suppressed
        // rather than signed by a guess.
        default: return 0;
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
        webThrottleDemandPct_ = 0.0f;  // a demand that outlives the idle would reopen the throttle
        brake_.stop();         // Stop brake actuator (hold position)
        brakeIsMoving_ = false;
        brakeSensorTriggerTime_ = 0;  // Reset sensor trigger
        transmission_.stop();  // Stop transmission actuator
        relayController_.allOff();  // Turn off ignition and lights
        // A fail-safe is a power-down in every respect that matters to the persisted counters
        // (distance and engine hours), so flush both once here — on entry only, guarded by
        // !failsafeApplied_ above.
        speedSensor_.persistDistance();
        engineHours_.persist();
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

    // Apply steering, speed-scaled. applySteeringScale() sits BETWEEN the transport's decoded
    // command and the actuator — the same place ArduPilot's own MANUAL_OPTIONS scaling sat,
    // only on the side of the link that owns the speed measurement. The web set_steering path
    // below is deliberately NOT scaled: it is a bench control with no road speed behind it.
    float steeringPct = applySteeringScale(mavlink_.getSteering());
    steering_.setSteeringPercent(steeringPct);

    // Apply throttle — skipped when gear boost PID is active (PID overrides)
    if (!gearBoostActive_) {
        float throttlePct = mavlink_.getThrottle();
        if (shouldClipThrottle()) {
            throttlePct = min(TRANS_UNKNOWN_GEAR_THROTTLE_MAX, throttlePct);
        }
        throttle_.setThrottlePercent(applySpeedLimit(throttlePct));
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
            // Flush the persisted counters (distance and engine hours) on the TRANSITION into
            // OFF only, so a normal shutdown loses nothing — not on every iteration OFF is
            // merely being held.
            if (previousIgnitionState_ != MavlinkInterface::IgnitionState::OFF) {
                speedSensor_.persistDistance();
                engineHours_.persist();
            }
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
    // Record the operator's standing demand; update() re-limits it every loop from here on.
    webThrottleDemandPct_ = value;
    throttle_.setThrottlePercent(applySpeedLimit(value));
    webPortal.sendResponse(true, "Throttle set");
}

// ============================================================================
// MAX-SPEED LIMITER — SPEED_MAX ceiling and proportional taper
// ============================================================================

float VehicleController::getSpeedLimitMs() const {
    // ONE source, no arbitration and no master switch: the autopilot's SPEED_MAX. Anything that
    // makes that value untrustworthy — never received, zero, out of range, stale, link down —
    // is reported as 0, which means NO LIMITING. That matches ArduPilot's own reading of a zero
    // SPEED_MAX and the value Mission Planner's speed-limit sign writes to clear a limit.
    if (!mavlink_.hasSpeedMaxParam()) {
        return 0.0f;
    }
    return mavlink_.getSpeedMaxMs();
}

void VehicleController::logSpeedLimitChange(float limitMs) {
    bool changed = isnan(lastSpeedLimitMs_) ||
                   fabsf(limitMs - lastSpeedLimitMs_) > SPEED_LIMIT_LOG_EPSILON_MS;
    if (!changed) {
        return;
    }
    uint32_t now = millis();
    // Hold-off. While suppressed the remembered state is deliberately NOT updated, so whatever
    // the value settles on is logged on the next opportunity instead of being swallowed.
    if (lastSpeedLimitLogMs_ != 0 && (now - lastSpeedLimitLogMs_) < SPEED_LIMIT_LOG_MIN_MS) {
        return;
    }
    lastSpeedLimitLogMs_ = now;
    lastSpeedLimitMs_ = limitMs;
    if (limitMs <= 0.0f) {
        Debug::printlnFeature(DebugFeature::VEHICLE,
            "[SPEED] Speed limit: none (no usable SPEED_MAX) — not limiting");
    } else {
        Debug::printfFeature(DebugFeature::VEHICLE,
            "[SPEED] Speed limit: %.2f m/s (%.1f km/h) from SPEED_MAX\n",
            limitMs, limitMs * MS_TO_KMH);
    }
}

float VehicleController::applySpeedLimit(float throttlePct) {
    float limitMs = getSpeedLimitMs();
    logSpeedLimitChange(limitMs);

    if (limitMs <= 0.0f) {
        // No usable SPEED_MAX => no limiting at all. Reset the taper so the next limit that
        // arrives never resumes from a stale low ceiling.
        limiterCeilingPct_ = 100.0f;
        limiterLastMs_ = 0;
        return throttlePct;
    }

    // Fail OPEN on sensor loss. The limiter only ever acts NEAR AND ABOVE the maximum speed, and a
    // lost sensor reads 0 m/s — clamping throttle on a reading we do not trust, mid-manoeuvre,
    // is the more dangerous failure. Over-speed protection assumes a working sensor.
    if (!speedSensor_.isValid()) {
        uint32_t now = millis();
        if (lastSpeedLimitWarnMs_ == 0 || (now - lastSpeedLimitWarnMs_) >= SPEED_LIMIT_WARN_MS) {
            lastSpeedLimitWarnMs_ = now;
            Debug::printlnFeature(DebugFeature::VEHICLE,
                "[SPEED] WARNING: speed limit active but speed reading is invalid — not clamping (fail open)");
        }
        limiterCeilingPct_ = 100.0f;
        limiterLastMs_ = 0;
        return throttlePct;
    }
    lastSpeedLimitWarnMs_ = 0;

    // Proportional taper: full authority up to `limit - band`, falling linearly to the floor at
    // the limit, floor above it. A floor rather than zero — cutting all drive mid-corner is a
    // stability event, not a safety feature.
    float over = speedSensor_.getSpeedMs() - (limitMs - SPEED_LIMIT_TAPER_BAND_MS);
    float targetPct;
    if (over <= 0.0f) {
        targetPct = 100.0f;
    } else if (over >= SPEED_LIMIT_TAPER_BAND_MS) {
        targetPct = SPEED_LIMIT_FLOOR_PCT;
    } else {
        targetPct = 100.0f - (100.0f - SPEED_LIMIT_FLOOR_PCT) * (over / SPEED_LIMIT_TAPER_BAND_MS);
    }

    // Slew-limit the CEILING (not the demand — the driver's own stick moves must pass through at
    // full rate). dt is capped at 1 s and the first evaluation adopts the target directly, so a
    // stalled or just-restarted loop cannot integrate an unknown interval into a step change.
    uint32_t now = millis();
    if (limiterLastMs_ == 0) {
        limiterCeilingPct_ = targetPct;
    } else {
        float dt = (now - limiterLastMs_) / 1000.0f;
        if (dt > 1.0f) dt = 1.0f;
        float maxStep = SPEED_LIMIT_CEILING_SLEW_PCT_S * dt;
        float delta = constrain(targetPct - limiterCeilingPct_, -maxStep, maxStep);
        limiterCeilingPct_ += delta;
    }
    limiterLastMs_ = now;
    limiterCeilingPct_ = constrain(limiterCeilingPct_, SPEED_LIMIT_FLOOR_PCT, 100.0f);

    return min(throttlePct, limiterCeilingPct_);
}

void VehicleController::processSpeedCalPprCommand(float value, WebPortal& webPortal) {
    int32_t ppr = (int32_t)lroundf(value);
    if (!speedSensor_.setPulsesPerRev(ppr)) {
        webPortal.sendResponse(false, "Pulses/rev out of range (" + String(SPEED_PPR_MIN) + "-" +
                                      String(SPEED_PPR_MAX) + ")");
        return;
    }
    webPortal.sendResponse(true, "Pulses/rev set to " + String(ppr));
}

void VehicleController::processSpeedCalCircCommand(float value, WebPortal& webPortal) {
    if (!speedSensor_.setWheelCircumferenceMm(value)) {
        webPortal.sendResponse(false, "Wheel circumference out of range (" +
                                      String((int)SPEED_CIRC_MIN_MM) + "-" +
                                      String((int)SPEED_CIRC_MAX_MM) + " mm)");
        return;
    }
    webPortal.sendResponse(true, "Wheel circumference set to " + String(value, 0) + " mm");
}

// ============================================================================
// STEERING SPEED SCALING — ArduPilot's formula, on the side that owns the speed
// ============================================================================

float VehicleController::getSteerScaleBaseMs() const {
    // Fixed, one-directional priority — deliberately not an arbitration. The LOCAL value wins so
    // a vehicle can be commissioned and driven with the scaling working before anyone has
    // touched the autopilot; MOT_SPD_SCA_BASE follows because it is the number this feature was
    // tuned with and it is visible from the ground station. Neither → 0, which means no scaling.
    // There is no compile-time default: a base nobody chose would silently restrict the steering
    // of an unconfigured vehicle, which is the opposite of what this change is for.
    if (steerScaBaseMs_ > 0.0f) {
        return steerScaBaseMs_;
    }
    if (mavlink_.hasSpdScaBaseParam()) {
        return mavlink_.getSpdScaBaseMs();   // already m/s — the firmware's internal unit
    }
    return 0.0f;
}

VehicleController::SteerScaleSource VehicleController::getSteerScaleSource() const {
    if (steerScaBaseMs_ > 0.0f) {
        return SteerScaleSource::LOCAL;
    }
    if (mavlink_.hasSpdScaBaseParam()) {
        return SteerScaleSource::AUTOPILOT;
    }
    return SteerScaleSource::NONE;
}

float VehicleController::getSteerTestSpeedMs() const {
    return (steerTestSpeedSetMs_ != 0) ? steerTestSpeedMs_ : 0.0f;
}

void VehicleController::logSteerScaleChange(float scale, float speedMs, bool speedIsTest,
                                            float baseMs, SteerScaleSource source) {
    // "Changed between scaling and not scaling" is a change even when the numbers are within
    // epsilon of each other, because crossing that boundary is the event the operator cares
    // about — it is what turns the assist on and off.
    bool wasScaling = !isnan(lastSteerScaleLogged_) && lastSteerScaleLogged_ < 1.0f;
    bool isScaling = scale < 1.0f;
    bool changed = isnan(lastSteerScaleLogged_) ||
                   fabsf(scale - lastSteerScaleLogged_) > STEER_SCALE_LOG_EPSILON ||
                   wasScaling != isScaling;
    if (!changed) {
        return;
    }
    uint32_t now = millis();
    // Hold-off. While suppressed the remembered state is deliberately NOT updated, so whatever
    // the scale settles on is logged on the next opportunity instead of being swallowed.
    if (lastSteerScaleLogMs_ != 0 && (now - lastSteerScaleLogMs_) < STEER_SCALE_LOG_MIN_MS) {
        return;
    }
    lastSteerScaleLogMs_ = now;
    lastSteerScaleLogged_ = scale;

    const char* srcName = (source == SteerScaleSource::LOCAL)     ? "local" :
                          (source == SteerScaleSource::AUTOPILOT) ? "MOT_SPD_SCA_BASE" : "none";
    Debug::printfFeature(DebugFeature::VEHICLE,
        "[STEER] Speed scale: %.2f (speed %.2f m/s%s, base %.2f m/s from %s)\n",
        scale, speedMs, speedIsTest ? " TEST" : "", baseMs, srcName);
}

float VehicleController::applySteeringScale(float steeringPct) {
    const float baseMs = getSteerScaleBaseMs();
    const SteerScaleSource source = getSteerScaleSource();
    const bool testLive = (steerTestSpeedSetMs_ != 0);
    const float measuredMs = speedSensor_.getSpeedMs();
    const float speedMs = testLive ? steerTestSpeedMs_ : measuredMs;

    // ---- The fault gates. EVERY one of them resolves to scale = 1, and to a command that
    // passes through untouched. Speed scaling is a comfort and assist feature: a vehicle that
    // loses it steers the way it did before the feature existed, while a vehicle that KEEPS a
    // restriction after a fault is a vehicle whose driver cannot turn. There is deliberately no
    // hold, no last-known-good speed, no conservative substitute and no minimum-scale floor —
    // this is the operator's explicit choice (2026-09-24) and must not be "improved" later.
    uint32_t now = millis();
    bool warn = false;
    const char* warnText = nullptr;
    float targetScale = 1.0f;

    if (baseMs <= 0.0f) {
        // Nothing to scale by. Matches ArduPilot's own is_positive(MOT_SPD_SCA_BASE) reading.
        warn = true;
        warnText = "[STEER] WARNING: no speed-scaling base (local steer_sca_base and the "
                   "autopilot's MOT_SPD_SCA_BASE are both unavailable) — not scaling";
    } else if (!speedSensor_.isValid()) {
        // The speed is UNKNOWN — never pulsed since boot, latched suspicious, or the sensor is
        // not initialised. A guess is exactly how ArduPilot got into trouble here. The gate
        // applies even with a test override live, so the bench cannot scale on an unhealthy
        // sensor, and even at speed: a fault must never leave the driver with restricted steering.
        warn = true;
        warnText = "[STEER] WARNING: speed reading invalid — not scaling (full steering authority)";
    } else if (mavlink_.hasAutopilotMode() &&
               mavlink_.getAutopilotCustomMode() != ROVER_CUSTOM_MODE_MANUAL) {
        // In every other mode the autopilot computes steering from speed itself; a second,
        // invisible reduction on top of it would be double limiting. An UNKNOWN or stale mode
        // falls through this branch and IS scaled — the single place where an unavailable input
        // does not disable the scaling, because MANUAL is the mode this vehicle is driven in.
        targetScale = 1.0f;
    } else {
        // ArduPilot's formula, unchanged: scale = min(1, base / v). v <= base gives 1 by the
        // formula itself, so a parked vehicle and a reading decayed to zero need no special case.
        targetScale = (speedMs > baseMs) ? (baseMs / speedMs) : 1.0f;
    }

    if (warn) {
        if (lastSteerScaleWarnMs_ == 0 || (now - lastSteerScaleWarnMs_) >= STEER_SCALE_WARN_MS) {
            lastSteerScaleWarnMs_ = now;
            Debug::printlnFeature(DebugFeature::VEHICLE, warnText);
        }
    } else {
        lastSteerScaleWarnMs_ = 0;
    }

    // ---- Slew-limit the SCALE (not the command — the driver's own steering movements must
    // pass through at full rate). dt is capped at 1 s and the first evaluation adopts the target
    // directly, so a stalled or just-restarted loop cannot integrate an unknown interval into a
    // step change. Same shape as applySpeedLimit()'s ceiling slew, for the same reason.
    if (steerScaleLastMs_ == 0) {
        steerScaleApplied_ = targetScale;
    } else {
        float dt = (now - steerScaleLastMs_) / 1000.0f;
        if (dt > 1.0f) dt = 1.0f;
        float maxStep = STEER_SCALE_SLEW_PER_S * dt;
        float delta = constrain(targetScale - steerScaleApplied_, -maxStep, maxStep);
        steerScaleApplied_ += delta;
    }
    steerScaleLastMs_ = now;
    steerScaleApplied_ = constrain(steerScaleApplied_, 0.0f, 1.0f);

    logSteerScaleChange(steerScaleApplied_, speedMs, testLive, baseMs, source);

    // steeringPct is already signed about a centre of ZERO, so multiplying scales the DEVIATION
    // FROM CENTRE and leaves a centred command centred: a driver going straight feels nothing.
    return steeringPct * steerScaleApplied_;
}

void VehicleController::processSetSteerScaBaseCommand(float value, WebPortal& webPortal) {
    if (isnan(value) || value < 0.0f || value > STEER_SCALE_BASE_MAX_MS) {
        webPortal.sendResponse(false, "Steering scale base out of range (0-" +
                                      String(STEER_SCALE_BASE_MAX_MS, 1) + " m/s)");
        return;
    }

    steerScaBaseMs_ = value;

    // Persisted into the EXISTING "steering" namespace, beside the steering calibration. Zero is
    // a legitimate stored value and means "not set": fall back to the autopilot's
    // MOT_SPD_SCA_BASE, or to no scaling at all.
    Preferences prefs;
    if (prefs.begin("steering", false)) {
        prefs.putFloat(STEER_SCALE_BASE_NVS_KEY, value);
        prefs.end();
    }

    if (value > 0.0f) {
        webPortal.sendResponse(true, "Steering scale base set to " + String(value, 2) + " m/s");
    } else {
        webPortal.sendResponse(true, "Steering scale base cleared — using the autopilot's "
                                     "MOT_SPD_SCA_BASE, or no scaling");
    }
    Debug::printfFeature(DebugFeature::VEHICLE,
        "[STEER] Speed-scaling base (local) set to %.2f m/s\n", value);
}

void VehicleController::processSetTestSpeedCommand(float value, WebPortal& webPortal) {
    if (isnan(value) || value < 0.0f || value > STEER_SCALE_TEST_SPEED_MAX_MS) {
        webPortal.sendResponse(false, "Test speed out of range (0-" +
                                      String(STEER_SCALE_TEST_SPEED_MAX_MS, 1) + " m/s)");
        return;
    }

    if (value <= 0.0f) {
        steerTestSpeedMs_ = 0.0f;
        steerTestSpeedSetMs_ = 0;
        webPortal.sendResponse(true, "Test speed cleared — using the measured speed");
        Debug::printlnFeature(DebugFeature::VEHICLE, "[STEER] TEST speed override cleared");
        return;
    }

    // RAM ONLY — never persisted, cleared by a reboot, and fused to STEER_SCALE_TEST_SPEED_MS.
    // It substitutes a speed for the SCALING CALCULATION ONLY: the wheel odometry sent to the
    // autopilot, VFR_HUD, the SPEED_MAX limiter, the transmission interlock and the odometer all
    // keep using the real measured speed, so the bench configuration is structurally incapable
    // of becoming a driving configuration.
    steerTestSpeedMs_ = value;
    steerTestSpeedSetMs_ = millis();
    webPortal.sendResponse(true, "TEST speed " + String(value, 2) +
                                 " m/s — temporary, clears itself in " +
                                 String(STEER_SCALE_TEST_SPEED_MS / 1000) + " s");
    Debug::printfFeature(DebugFeature::VEHICLE,
        "[STEER] TEST speed override %.2f m/s (steering scale only, expires in %lu s)\n",
        value, (unsigned long)(STEER_SCALE_TEST_SPEED_MS / 1000));
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


void VehicleController::processCanProbeCommand(WebPortal& webPortal) {
    // Defer while a gear change is active to protect the high-rate RPM feedback the shift relies on.
    bool gearChangeActive = transmission_.isGearChangeActive() || transmission_.needsThrottleBoost();
    CANController::ProbeStart result = canController_.startProbe(gearChangeActive);

    switch (result) {
        case CANController::ProbeStart::STARTED:
            Debug::printlnFeature(DebugFeature::VEHICLE, "[WEB] ECU probe started");
            webPortal.sendResponse(true, "ECU probe started");
            break;
        case CANController::ProbeStart::BUSY:
            Debug::printlnFeature(DebugFeature::VEHICLE, "[WEB] ECU probe deferred (busy)");
            webPortal.sendResponse(false, "ECU probe busy (gear change or probe in progress)");
            break;
        case CANController::ProbeStart::NO_ECU:
            Debug::printlnFeature(DebugFeature::VEHICLE, "[WEB] ECU probe: no ECU / disconnected");
            webPortal.sendResponse(false, "No ECU / CAN disconnected");
            break;
    }
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

    // Flush the persisted counters (distance and engine hours) on the TRANSITION into OFF
    // only — the web twin of the MAVLink ignition-OFF flush; re-selecting OFF while already
    // OFF writes nothing.
    if (targetState == RelayController::IgnitionState::OFF &&
        currentState != RelayController::IgnitionState::OFF) {
        speedSensor_.persistDistance();
        engineHours_.persist();
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
        webThrottleDemandPct_ = 0.0f;  // the boost released to IDLE — don't let the old web demand undo it
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
        webThrottleDemandPct_ = 0.0f;  // same reasoning as the normal release above
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
