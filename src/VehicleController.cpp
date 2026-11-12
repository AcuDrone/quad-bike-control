#include "VehicleController.h"
#include "Debug.h"
#include "EventJournal.h"
#include "BrakeLaw.h"

// BrakeLaw.h cannot include Constants.h (native tests): keep its literals in sync.
static_assert(BRAKE_LAW_TRAVEL_MS_MIN == BRAKE_TRAVEL_MS_MIN, "BrakeLaw.h travel min out of sync");
static_assert(BRAKE_LAW_TRAVEL_MS_MAX == BRAKE_TRAVEL_MS_MAX, "BrakeLaw.h travel max out of sync");
static_assert(BRAKE_LAW_TRAVEL_MS_DEFAULT == BRAKE_FULL_TRAVEL_TIME, "BrakeLaw.h travel default out of sync");
static_assert(BRAKE_LAW_HOLD_PWM_MIN == BRAKE_HOLD_PWM_MIN, "BrakeLaw.h hold min out of sync");
static_assert(BRAKE_LAW_HOLD_PWM_MAX == BRAKE_HOLD_PWM_MAX, "BrakeLaw.h hold max out of sync");
static_assert(BRAKE_LAW_HOLD_PWM_DEFAULT == BRAKE_HOLD_SPEED, "BrakeLaw.h hold default out of sync");
#include <Preferences.h>

// Vehicle-command gear code (VehicleCommandLaw.h: R -1, N 0, H 1, L 2) → transmission gear.
static TransmissionController::Gear vcmdGearToTrans(int8_t code) {
    switch (code) {
        case VCMD_GEAR_R: return TransmissionController::Gear::GEAR_REVERSE;
        case VCMD_GEAR_N: return TransmissionController::Gear::GEAR_NEUTRAL;
        case VCMD_GEAR_H: return TransmissionController::Gear::GEAR_HIGH;
        case VCMD_GEAR_L: return TransmissionController::Gear::GEAR_LOW;
        default:          return TransmissionController::Gear::GEAR_UNKNOWN;
    }
}

VehicleController::VehicleController(SteeringController& steering,
                                     ThrottleController& throttle,
                                     TransmissionController& transmission,
                                     BTS7960Controller& brake,
                                     BTS7960Controller& parkBrake,
                                     MavlinkInterface& mavlink,
                                     RelayController& relayController,
                                     BoardInputs& boardInputs,
                                     SpeedSensor& speedSensor)
    : steering_(steering),
      throttle_(throttle),
      transmission_(transmission),
      brake_(brake),
      parkBrake_(parkBrake),
      mavlink_(mavlink),
      relayController_(relayController),
      boardInputs_(boardInputs),
      speedSensor_(speedSensor),
      batteryV_(0.0f),
      batteryRawV_(0.0f),
      batteryValid_(false),
      batteryLow_(false),
      batteryLowRaw_(false),
      batteryCal_(1.0f),
      lastBatterySampleMs_(0),
      lastStarterOnMs_(0),
      lastBatteryWarnMs_(0),
      currentInputSource_(InputSource::FAILSAFE),
      prevInputSource_(InputSource::FAILSAFE),
      steerMode_(SteerMode::BLOCKED),   // the sensor starts unhealthy (no frames yet)
      lastMavCmdCount_(0),
      failsafeApplied_(false),
      webControl_(false),
      currentBrakeTarget_(0.0f),
      currentBrakePosition_(100.0f),
      brakeMovementStartTime_(0),
      lastBrakeUpdateTime_(0),
      brakeSensorTriggerTime_(0),
      brakeIsMoving_(false),
      brakeTravelMs_(BRAKE_FULL_TRAVEL_TIME),
      brakeHoldPwm_(BRAKE_HOLD_SPEED),
      boostTargetRpm_(TRANS_GEAR_BOOST_TARGET_RPM),
      boostGearEnabled_(BOOST_ENABLED_DEFAULT),
      gearBoostActive_(false),
      boostManualActive_(false),
      shiftIdleHold_(false),
      gearBoostStartTime_(0),
      pidLastUpdateTime_(0),
      lastRpmSampleMs_(0),
      pidIntegral_(0.0f),
      pidPrevError_(0.0f),
      lastPIDThrottleUs_(THROTTLE_DEFAULT_IDLE_US),
      lastSpeedLimitWarnMs_(0),
      lastSpeedLimitMs_(NAN),
      lastSpeedLimitLogMs_(0),
      limiter_(),
      limiterLastMs_(0),
      limiterFloorPct_(SPEED_LIMIT_FLOOR_DEFAULT_PCT),
      limiterSlewPctS_(SPEED_LIMIT_SLEW_DEFAULT_PCT_S),
      satSinceMs_(0),
      satActive_(false),
      satClearedMs_(0),
      steerScaBaseMs_(0.0f),
      steerScaleApplied_(1.0f),
      steerScaleLastMs_(0),
      lastSteerScaleWarnMs_(0),
      lastSteerScaleLogged_(NAN),
      lastSteerScaleLogMs_(0),
      steerTestSpeedMs_(0.0f),
      steerTestSpeedSetMs_(0),
      webThrottleDemandPct_(0.0f),
      pbArmed_(false),
      pbDemandPct_(0.0f),
      pbBlockLogged_(ParkBrakeDaWait::NONE),
      steerZeroResponsePortal_(nullptr) {
    effRpm_ = EffectiveRpm{0, false, RpmSource::NONE, 0};
}

BatParams VehicleController::batteryParams() {
    return BatParams{
        BAT12_DIVIDER_RATIO, BAT12_PRESENT_MIN_V, BAT12_LOW_V, BAT12_LOW_CLEAR_V,
        BAT12_CAL_MIN, BAT12_CAL_MAX, BAT12_CAL_MEASURED_MIN_V, BAT12_CAL_MEASURED_MAX_V};
}

void VehicleController::initBatterySense() {
    // aztec has NO Buck-Boost board: X10 pin 3 carries the 12 V battery + into the on-board
    // 200K/24K divider → GPIO9. main.cpp still drives PIN_BOOST_EN (GPIO46) HIGH — harmless
    // with nothing fitted, and the same pin state as master.
    batteryValid_ = false;
    batteryLow_ = false;
    batteryLowRaw_ = false;
    lastBatterySampleMs_ = 0;

    // User calibration factor (cal_bat_v). Absent key / NaN / out of range → 1.0, never rewritten.
    Preferences prefs;
    float cal = 1.0f;
    if (prefs.begin(BAT12_CAL_NVS_NS, true)) {
        cal = prefs.getFloat(BAT12_CAL_NVS_KEY, 1.0f);
        prefs.end();
    }
    if (!batCalValid(cal, batteryParams())) {
        Debug::printfFeature(DebugFeature::VEHICLE,
            "[BAT] Stored calibration %.4f out of range (%.2f-%.2f) — using 1.000\n",
            cal, BAT12_CAL_MIN, BAT12_CAL_MAX);
        cal = 1.0f;
    }
    batteryCal_ = cal;

    // GPIO9 is ADC1 (unaffected by WiFi). 16 V through the 200K/24K divider is ~1.7 V,
    // so the widest attenuation is required.
    //
    // Order matters: analogSetPinAttenuation() only RE-configures a channel that
    // is already attached to the ADC oneshot unit. Called on a fresh pin the core
    // logs "[E] __analogChannelConfig(): Pin is not configured as analog channel"
    // and the attenuation is silently not applied. One throwaway analogRead()
    // attaches the pin (via __analogInit) first; the per-pin attenuation then
    // takes effect, and the discarded first sample is never used for telemetry.
    (void)analogRead(PIN_ADC_24V);
    analogSetPinAttenuation(PIN_ADC_24V, ADC_11db);

    Debug::printfFeature(DebugFeature::VEHICLE,
        "[BAT] 12 V battery sense on X10/GPIO%d every %d ms, cal %.4f\n",
        (int)PIN_ADC_24V, BAT12_SAMPLE_INTERVAL_MS, batteryCal_);
}

void VehicleController::updateBatteryMonitor() {
    uint32_t now = millis();

    // Track the starter relay on every call (not only at sample time) so a short crank
    // is never missed: the low warning is suppressed while it is on and BAT12_CRANK_HOLDOFF_MS after.
    if (relayController_.isStarterEnergized()) {
        lastStarterOnMs_ = (now == 0) ? 1 : now;
    }

    if (lastBatterySampleMs_ != 0 && (now - lastBatterySampleMs_) < BAT12_SAMPLE_INTERVAL_MS) {
        return;
    }
    lastBatterySampleMs_ = (now == 0) ? 1 : now;

    // ADC1 on the S3 is noisy and the divider is high-impedance, so average a few
    // calibrated readings. Eight conversions are a few hundred microseconds — short
    // enough for the cooperative loop.
    uint32_t sumMv = 0;
    for (uint8_t i = 0; i < BAT12_ADC_SAMPLE_COUNT; i++) {
        sumMv += analogReadMilliVolts(PIN_ADC_24V);
    }
    const BatParams p = batteryParams();
    batteryRawV_ = batRawVolts(sumMv / (float)BAT12_ADC_SAMPLE_COUNT, p);
    batteryV_ = batVolts(batteryRawV_, batteryCal_);
    batteryValid_ = batPresent(batteryRawV_, p);

    if (!batteryValid_) {
        batteryLowRaw_ = false;   // nothing wired is not a low battery
        batteryLow_ = false;
        lastBatteryWarnMs_ = 0;
        return;
    }

    batteryLowRaw_ = batLowStep(batteryLowRaw_, batteryV_, p);
    bool cranking = relayController_.isStarterEnergized() ||
                    (lastStarterOnMs_ != 0 && (now - lastStarterOnMs_) < BAT12_CRANK_HOLDOFF_MS);
    batteryLow_ = batteryLowRaw_ && !cranking;

    if (batteryLow_ && (lastBatteryWarnMs_ == 0 || (now - lastBatteryWarnMs_) >= 5000)) {
        lastBatteryWarnMs_ = now;
        Debug::printfFeature(DebugFeature::VEHICLE,
            "[BAT] WARNING: 12 V battery low (%.2f V < %.1f V)\n", batteryV_, BAT12_LOW_V);
    } else if (!batteryLow_) {
        lastBatteryWarnMs_ = 0;
    }
}

void VehicleController::processBatteryCalCommand(float measuredV, WebPortal& webPortal) {
    float cal = batteryCal_;
    const float rawV = batteryRawV_;   // the CURRENT averaged reading
    BatCalResult r = batComputeCal(measuredV, rawV, batteryParams(), cal);
    if (r == BatCalResult::MEASURED_OUT_OF_RANGE) {
        webPortal.sendResponse(false, "Battery: measured voltage out of range (" +
                                      String(BAT12_CAL_MEASURED_MIN_V, 0) + "-" +
                                      String(BAT12_CAL_MEASURED_MAX_V, 0) + " V)");
        return;
    }
    if (r == BatCalResult::NOT_CONNECTED) {
        webPortal.sendResponse(false, "Battery: nothing on X10 (raw " + String(rawV, 2) + " V) - nothing to calibrate");
        return;
    }
    if (r == BatCalResult::FACTOR_OUT_OF_RANGE) {
        webPortal.sendResponse(false, "Battery: correction " + String(measuredV / rawV, 3) + " out of range (" +
                                      String(BAT12_CAL_MIN, 2) + "-" + String(BAT12_CAL_MAX, 2) +
                                      ") - check the value or the wiring");
        return;
    }

    // Persist first: if the write fails, the RAM value is left as it was.
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(BAT12_CAL_NVS_NS, false)) {
        ok = prefs.putFloat(BAT12_CAL_NVS_KEY, cal) == sizeof(float);
        prefs.end();
    }
    if (!ok) {
        webPortal.sendResponse(false, "Battery: failed to save the calibration");
        return;
    }

    batteryCal_ = cal;
    batteryV_ = batVolts(batteryRawV_, batteryCal_);
    webPortal.sendResponse(true, "Battery calibrated: cal " + String(cal, 4) + ", " +
                                 String(batteryV_, 2) + " V");
    Debug::printfFeature(DebugFeature::VEHICLE,
        "[BAT] Calibrated: measured %.2f V, raw %.3f V -> cal %.4f\n", measuredV, rawV, cal);
}

void VehicleController::processBatteryCalResetCommand(WebPortal& webPortal) {
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(BAT12_CAL_NVS_NS, false)) {
        ok = prefs.putFloat(BAT12_CAL_NVS_KEY, 1.0f) == sizeof(float);
        prefs.end();
    }
    if (!ok) {
        webPortal.sendResponse(false, "Battery: failed to save the calibration");
        return;
    }
    batteryCal_ = 1.0f;
    batteryV_ = batteryRawV_;
    webPortal.sendResponse(true, "Battery calibration reset: cal 1.0000, " + String(batteryV_, 2) + " V");
    Debug::printlnFeature(DebugFeature::VEHICLE, "[BAT] Calibration reset to 1.0");
}

static String tachoPprText(float ppr) {
    // 0.5 keeps its decimal, the whole values print as integers ("ppr 1", not "ppr 1.0")
    return (fabsf(ppr - roundf(ppr)) > 0.01f) ? String(ppr, 1) : String((int)lroundf(ppr));
}

void VehicleController::processTachoCalCommand(float refRpm, WebPortal& webPortal) {
    const float hz = tacho_.getHz();   // the CURRENT averaged frequency (0 when timed out)
    float raw = NAN;
    float ppr = tacho_.getPpr();
    const TachoCalParams p{TACHO_CAL_REF_MIN_RPM, TACHO_CAL_REF_MAX_RPM, TACHO_CAL_TOLERANCE};
    const TachoCalResult r = tachoComputeCal(hz, refRpm, p, raw, ppr);
    if (r == TachoCalResult::NO_PULSES) {
        webPortal.sendResponse(false, "Tacho: no pulses - start the engine and check the wiring");
        return;
    }
    if (r == TachoCalResult::REF_OUT_OF_RANGE) {
        webPortal.sendResponse(false, "Tacho: reference RPM out of range (" +
                                      String((int)TACHO_CAL_REF_MIN_RPM) + "-" +
                                      String((int)TACHO_CAL_REF_MAX_RPM) + ")");
        return;
    }
    const String ratio = String(hz, 1) + " Hz × 60 / " + String((int)lroundf(refRpm)) + " = " + String(raw, 2);
    if (r == TachoCalResult::RATIO_OFF) {
        webPortal.sendResponse(false, "Tacho: " + ratio +
                                      " - not within 25% of an allowed ppr (0.5, 1, 2, 3, 4), check the reference");
        return;
    }
    if (!tacho_.setPpr(ppr)) {
        webPortal.sendResponse(false, "Tacho: failed to save ppr");
        return;
    }
    webPortal.sendResponse(true, "Tacho: " + ratio + " → ppr " + tachoPprText(ppr));
    Debug::printfFeature(DebugFeature::VEHICLE, "[TACHO] Calibrated: %.2f Hz, ref %.0f RPM -> raw %.3f -> ppr %.1f\n",
                         hz, refRpm, raw, ppr);
}

void VehicleController::processTachoSetPprCommand(float ppr, WebPortal& webPortal) {
    if (!tachoPprAllowed(ppr)) {
        webPortal.sendResponse(false, "Tacho: ppr " + String(ppr, 2) + " not allowed (0.5, 1, 2, 3, 4)");
        return;
    }
    if (!tacho_.setPpr(ppr)) {
        webPortal.sendResponse(false, "Tacho: failed to save ppr");
        return;
    }
    webPortal.sendResponse(true, "Tacho: ppr set to " + tachoPprText(tacho_.getPpr()));
}

void VehicleController::processTachoResetPprCommand(WebPortal& webPortal) {
    if (!tacho_.resetPpr()) {
        webPortal.sendResponse(false, "Tacho: failed to reset ppr");
        return;
    }
    webPortal.sendResponse(true, "Tacho: ppr reset to " + tachoPprText(tacho_.getPpr()));
}

void VehicleController::initEngineHourMeter() {
    engineHours_.begin();
}

bool VehicleController::initCAN() {
    Preferences prefs;
    if (prefs.begin("boost", true)) {
        boostTargetRpm_ = prefs.getInt("target_rpm", TRANS_GEAR_BOOST_TARGET_RPM);
        boostGearEnabled_ = prefs.getBool(BOOST_ENABLED_NVS_KEY, BOOST_ENABLED_DEFAULT);
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

    // The speed-limiter throttle floor, from the "speed" namespace (beside the speed-sensor
    // calibration, which SpeedSensor owns; this key is VehicleController's). An absent key reads as
    // the default; NaN or anything outside 0–20 % is ignored (and never rewritten).
    if (prefs.begin("speed", true)) {
        float floorPct = prefs.getFloat(SPEED_LIMIT_FLOOR_NVS_KEY, SPEED_LIMIT_FLOOR_DEFAULT_PCT);
        float slewPctS = prefs.getFloat(SPEED_LIMIT_SLEW_NVS_KEY, SPEED_LIMIT_SLEW_DEFAULT_PCT_S);
        prefs.end();
        if (isnan(floorPct) || floorPct < SPEED_LIMIT_FLOOR_MIN_PCT ||
            floorPct > SPEED_LIMIT_FLOOR_MAX_PCT) {
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[SPEED] Stored limiter floor %.1f out of range (%.0f-%.0f %%) — ignored\n",
                floorPct, SPEED_LIMIT_FLOOR_MIN_PCT, SPEED_LIMIT_FLOOR_MAX_PCT);
            floorPct = SPEED_LIMIT_FLOOR_DEFAULT_PCT;
        }
        limiterFloorPct_ = floorPct;
        // The ceiling slew, same rules: absent = default, NaN or outside 10–200 %/s ignored.
        if (isnan(slewPctS) || slewPctS < SPEED_LIMIT_SLEW_MIN_PCT_S ||
            slewPctS > SPEED_LIMIT_SLEW_MAX_PCT_S) {
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[SPEED] Stored limiter slew %.1f out of range (%.0f-%.0f %%/s) — ignored\n",
                slewPctS, SPEED_LIMIT_SLEW_MIN_PCT_S, SPEED_LIMIT_SLEW_MAX_PCT_S);
            slewPctS = SPEED_LIMIT_SLEW_DEFAULT_PCT_S;
        }
        limiterSlewPctS_ = slewPctS;
    }
    Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] Limiter floor: %.1f %%, slew: %.1f %%/s\n",
                         limiterFloorPct_, limiterSlewPctS_);

    // Brake full travel time and hold drive, NVS "brake" (VehicleController owns the brake
    // estimate). A missing key reads as the default; an out-of-range stored value is logged and
    // ignored (default used, never rewritten).
    if (prefs.begin(BRAKE_NVS_NAMESPACE, true)) {
        int32_t travel = prefs.getUShort(BRAKE_TRAVEL_NVS_KEY, BRAKE_FULL_TRAVEL_TIME);
        int32_t hold = prefs.getUChar(BRAKE_HOLD_NVS_KEY, BRAKE_HOLD_SPEED);
        prefs.end();
        if (!brakeTravelMsValid(travel)) {
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[BRAKE] Stored travel time %ld ms out of range (%d-%d ms) — ignored\n",
                (long)travel, BRAKE_TRAVEL_MS_MIN, BRAKE_TRAVEL_MS_MAX);
        }
        if (!brakeHoldPwmValid(hold)) {
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[BRAKE] Stored hold drive %ld out of range (%d-%d) — ignored\n",
                (long)hold, BRAKE_HOLD_PWM_MIN, BRAKE_HOLD_PWM_MAX);
        }
        brakeTravelMs_ = (uint16_t)brakeTravelMsOrDefault(travel);
        brakeHoldPwm_ = (uint8_t)brakeHoldPwmOrDefault(hold);
    }
    Debug::printfFeature(DebugFeature::VEHICLE, "[BRAKE] Travel time: %u ms, hold drive: %u\n",
                         (unsigned)brakeTravelMs_, (unsigned)brakeHoldPwm_);

    bool result = canController_.begin();
    canController_.setRPMPollInterval(CAN_POLL_INTERVAL_RPM);
    // The Bosch LWS steering-angle sensor shares this bus: every 0x305 frame goes to the
    // steering controller's decoder, and the zero-set runs here. Attached even if init
    // failed, so the steering reports `can_down` rather than `timeout`.
    canController_.attachLwsSink(&steering_.lwsSensor());
    steering_.attachCan(&canController_);
    return result;
}

void VehicleController::updateEffectiveRpm() {
    const CANController::VehicleData d = canController_.getVehicleData();
    effRpm_ = tachoSelectRpm(tacho_.isFitted(), tacho_.isLive(), tacho_.getRpm(), tacho_.lastEdgeMs(),
                             d.dataValid, d.engineRPM, d.lastUpdateTime);
}

static const char* inputSourceName(InputSource s) {
    return (s == InputSource::MAVLINK) ? INPUT_SOURCE_NAME_MAVLINK :
           (s == InputSource::WEB) ? INPUT_SOURCE_NAME_WEB : INPUT_SOURCE_NAME_FAILSAFE;
}

void VehicleController::update() {
    // The Bosch LWS is powered from the ACC line (ignition Relay_1): its CAN silence watchdog
    // runs only while the ignition is ACC or IGNITION (decision 1 of change
    // add-event-journal-and-link-recovery). Read-only use of the relay state.
    {
        const RelayController::IgnitionState ign = relayController_.getIgnitionState();
        canController_.setLwsPowered(ign == RelayController::IgnitionState::ACC ||
                                     ign == RelayController::IgnitionState::IGNITION);
    }

    // Update CAN controller (read vehicle data from ECU)
    canController_.update();

    // ECU tacho, then the ONE effective engine RPM every consumer below reads
    // (live TACHO → CAN rpm > 0 → fitted TACHO 0 → CAN → NONE). VehicleData.engineRPM
    // keeps the raw CAN value.
    tacho_.update();
    updateEffectiveRpm();

    // Sample the 12 V battery on X10 (rate-limited internally)
    updateBatteryMonitor();

    // Initialise the transmission once, on first engine-running detection (unless a web
    // gear command already did): sensor, else valid NVS state, else a step into NEUTRAL.
    if (!transmission_.isInitialised() && isEngineRunning()) {
        transmission_.initialise();
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

    // Engine hour meter, from the effective RPM computed once at the top of update(). No
    // valid source (CAN invalid and no fitted tacho) means the engine state is UNKNOWN, not
    // "off": the interval is discarded rather than counted, because an unknown state must not
    // invent hours. A fitted tacho that timed out reads a valid 0 (nothing counted either).
    // ENGINE_HOURS_MIN_RPM, not ENGINE_RUNNING_RPM_THRESHOLD — an hour meter counts every
    // second the crank turns, independent of the gear-change safety threshold.
    engineHours_.update(effRpm_.valid && effRpm_.rpm >= ENGINE_HOURS_MIN_RPM, millis());

    // Update relay controller with the effective engine RPM (automatic cranking stop and the
    // armed-crank suppression); 0 when no source is valid.
    relayController_.update(getEngineRpm());

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

    // Steering control mode for this iteration — BEFORE the MAVLink commands, so a change of
    // source (web takeover, FAILSAFE) or autopilot mode stops a rate drive in this same loop.
    updateSteerMode();

    // The park-brake operator demand is re-fed by whichever throttle path runs below (0 when
    // none does). processThrottleCommand() ran earlier in this loop; the web re-apply re-feeds it.
    pbDemandPct_ = 0.0f;

    // Process MAVLink commands if MAVLink is active
    if (currentInputSource_ == InputSource::MAVLINK) {
        processMavlinkCommands();
    } else {
        // No autopilot command path: keep the frame counter in step so a later return to
        // MAVLink never treats an old frame as a fresh rate command.
        lastMavCmdCount_ = mavlink_.getCommandCount();
        // RC is not in control (fail-safe or web portal): the first valid frame after the
        // return re-applies gear / ignition / light / lock, exactly as at boot (owner decisions
        // D-A, D-H). A GCS / web command given meanwhile is not restored.
        rc_.resync();
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

    // Ignition OFF always disarms the park-brake drive-away latch, whether or not a throttle path
    // runs this loop (a throttle already up when the ignition comes on then needs a new press).
    if (relayController_.getIgnitionState() == RelayController::IgnitionState::OFF) {
        pbArmed_ = false;
    }

    // Re-apply the limiter to the STANDING web throttle demand every loop. The MAVLink path
    // already re-reads its channel every iteration, so it was always re-limited; the web path
    // used to clamp only at command time, which meant a vehicle accelerating past the ceiling on
    // an unchanged 80 % slider was never clamped until the operator touched the slider again.
    // Calibration owns the servo directly, and the gear-boost PID writes µs (or the shift idle
    // hold keeps idle) — both stay clear.
    if (currentInputSource_ == InputSource::WEB && !isThrottleLockedForShift() && !throttle_.isCalibrating()) {
        float demandPct = webThrottleDemandPct_;
        if (shouldClipThrottle()) {
            demandPct = min(TRANS_UNKNOWN_GEAR_THROTTLE_MAX, demandPct);
        }
        applyOperatorThrottle(demandPct);   // park-brake drive-away / hold, then the limiter + slew
    }

    // PID-controlled RPM boost during gear changes or manual test (overrides MAVLink/web throttle).
    // A gear change starts the PID only with the boost switch on and no idle hold in progress
    // (switching it on mid-shift keeps the hold; the PID runs from the next gear change). The
    // manual test ignores the switch.
    if (gearBoostActive_ || boostManualActive_ ||
        (transmission_.needsThrottleBoost() && boostGearEnabled_ && !shiftIdleHold_)) {
        updateGearBoostPID();
    }

    // Boost switched off: the gear change holds the throttle at idle instead
    updateShiftIdleHold();

    // Update steering position control (after canController_.update(): it sees this
    // iteration's angle frames)
    steering_.update();

    // Re-derive the steering mode once more: steering_.update() may just have changed the
    // sensor health, and the MAVLink report / web telemetry built after this loop must see
    // a mode consistent with it (steer_mode is position exactly when steer_sensor_ok).
    updateSteerMode();

    // Final answer to a pending "Set sensor zero"
    if (steerZeroResponsePortal_ != nullptr &&
        steering_.getZeroState() != SteeringController::ZeroState::RUNNING) {
        if (steering_.getZeroState() == SteeringController::ZeroState::OK) {
            steerZeroResponsePortal_->sendResponse(true,
                "Sensor zero confirmed (angle 0.0°) — capture Left and Right again");
        } else {
            const char* why = steering_.getZeroError();
            String reason = why ? String(why) : String("unknown");
            if (reason == "range_latched") reason += " — restart the vehicle and retry";
            steerZeroResponsePortal_->sendResponse(false, "Sensor zero failed: " + reason);
        }
        steerZeroResponsePortal_ = nullptr;
    }

    // Update position control for transmission actuator
    transmission_.update();

    // Update brake actuator control
    updateBrakeControl();

    // Aztec parking brake on X6: reverse dwell / run timer / settle. Independent of the X7 brake.
    parkBrake_.update();
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
        EventJournal::log(JSrc::SRC, JLvl::I, "change", "from=%s to=%s",
                          inputSourceName(currentInputSource_), inputSourceName(source));
        if (source == InputSource::WEB) {
            webThrottleDemandPct_ = 0.0f;
            throttle_.idle();
        }
        prevInputSource_ = currentInputSource_;
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
    } else if (cmd.cmd == "set_gear_code" || cmd.cmd == "reset_gear_codes") {
        // Gear code calibration (bench): not restricted by input source; refused while
        // the engine is running, like throttle calibration
        processGearCodeCommand(cmd.cmd, cmd.strValue, webPortal);
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
        // Lock (aztec: Relay_4+5+6): not restricted by input source, like the light
        processWheelLockCommand(cmd.boolValue, webPortal);
        return;
    } else if (cmd.cmd == "park_brake") {
        // Aztec parking brake (X6): not restricted by input source, ignition or web control,
        // like the lock — the actuator is on permanent battery power
        processParkBrakeCommand(cmd.strValue, webPortal);
        return;
    } else if (cmd.cmd == "set_park_brake_travel_ms") {
        processSetParkBrakeTravelMsCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_park_brake_thr_pass_pct") {
        processSetParkBrakeThrPassPctCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_park_brake_invert") {
        processSetParkBrakeInvertCommand(cmd.boolValue, webPortal);
        return;
    } else if (cmd.cmd == "set_relay_test") {
        // Relay wiring test: not restricted by input source; RelayController only
        // accepts an ON while the ignition is OFF and no function relay is energized
        processRelayTestCommand(cmd.relay, cmd.boolValue, webPortal);
        return;
    } else if (cmd.cmd == "set_relay_board") {
        // Relay board selection: only changes the stored value (applies after reboot), so
        // it is accepted regardless of input source and ignition state
        processRelayBoardCommand(cmd.strValue, webPortal);
        return;
    } else if (cmd.cmd == "set_boost") {
        // Kept for old clients: no boost board on X10 (aztec) — replies "not fitted"
        processBoostCommand(cmd.boolValue, webPortal);
        return;
    } else if (cmd.cmd == "cal_bat_v") {
        // 12 V battery calibration from a multimeter reading: any input source
        processBatteryCalCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "reset_bat_v_cal") {
        processBatteryCalResetCommand(webPortal);
        return;
    } else if (cmd.cmd == "cal_tacho_ppr") {
        // Tacho ppr from a reference RPM (dash or crank sensor): any input source
        processTachoCalCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_tacho_ppr") {
        processTachoSetPprCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "reset_tacho_ppr") {
        processTachoResetPprCommand(webPortal);
        return;
    } else if (cmd.cmd == "steer_cal_center" || cmd.cmd == "steer_cal_left" || cmd.cmd == "steer_cal_right") {
        processSteerCalCommand(cmd.cmd, webPortal);
        return;
    } else if (cmd.cmd == "steer_lws_zero") {
        // Sensor zero-set: no ignition / RPM / standstill precondition, any input source
        processSteerZeroCommand(webPortal);
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
    } else if (cmd.cmd == "set_boost_enabled") {
        processSetBoostEnabledCommand(cmd.boolValue, webPortal);
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
    } else if (cmd.cmd == "set_speed_limit_floor") {
        processSetSpeedLimitFloorCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_speed_limit_slew") {
        processSetSpeedLimitSlewCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_throttle_open_slew") {
        processSetThrottleOpenSlewCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_steer_sca_base") {
        processSetSteerScaBaseCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_steer_park_enabled") {
        processSetSteerParkEnabledCommand(cmd.boolValue, webPortal);
        return;
    } else if (cmd.cmd == "set_steer_park_rung") {
        processSetSteerParkRungCommand(cmd.floatValue, cmd.strValue, webPortal);
        return;
    } else if (cmd.cmd == "set_steer_park_tol") {
        processSetSteerParkTolCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_steer_park_max") {
        processSetSteerParkMaxCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_brake_travel_ms") {
        processSetBrakeTravelMsCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_brake_hold_pwm") {
        processSetBrakeHoldPwmCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "set_test_speed") {
        processSetTestSpeedCommand(cmd.floatValue, webPortal);
        return;
    } else if (cmd.cmd == "can_probe") {
        // ECU capability probe — available regardless of input source (diagnostic)
        processCanProbeCommand(webPortal);
        return;
    } else if (cmd.cmd == "can_sniff") {
        // CAN bus sniffer — diagnostic, available regardless of input source
        processCanSniffCommand(webPortal);
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
    } else if (cmd.cmd == "steer_angle") {
        processSteerAngleCommand(cmd.floatValue, webPortal);
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
        // Physical gear unknown (ambiguous switches or a faulted input expander):
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
        // NEUTRAL, and GEAR_UNKNOWN (ambiguous switches, faulted input expander, or the
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
        default: return "?";   // GEAR_UNKNOWN: not initialised
    }
}

String VehicleController::getFromGearString() const {
    TransmissionController::Gear gear = transmission_.getFromGear();
    switch (gear) {
        case TransmissionController::Gear::GEAR_REVERSE: return "R";
        case TransmissionController::Gear::GEAR_NEUTRAL: return "N";
        case TransmissionController::Gear::GEAR_LOW: return "L";
        case TransmissionController::Gear::GEAR_HIGH: return "H";
        default: return "?";   // GEAR_UNKNOWN: not initialised
    }
}

void VehicleController::applyFailsafe() {
    if (currentInputSource_ == InputSource::FAILSAFE && !failsafeApplied_) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[FAILSAFE] Entering safe state");
        EventJournal::log(JSrc::SRC, JLvl::W, "failsafe_enter", "from=%s", inputSourceName(prevInputSource_));
        // Centres with a healthy sensor; refused without one (never centres blind). A rate
        // drive was already stopped this iteration: FAILSAFE makes the mode BLOCKED and
        // updateSteerMode() ran stopRate() before this.
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
        // RC re-sync on the way back (re-crank on recovery) is armed by update(): the RC trackers
        // are cleared in every loop whose source is not MAVLink (design D6).
        failsafeApplied_ = true;
    } else if (currentInputSource_ != InputSource::FAILSAFE && failsafeApplied_) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[FAILSAFE] Exiting safe state");
        EventJournal::log(JSrc::SRC, JLvl::I, "failsafe_exit", "to=%s", inputSourceName(currentInputSource_));
        brake_.stop();  // Ensure brake is stopped before handing control
        failsafeApplied_ = false;
    }
}

void VehicleController::updateSteerMode() {
    const bool sourceIsMavlink = (currentInputSource_ == InputSource::MAVLINK);
    const bool apKnown = mavlink_.hasAutopilotMode();
    const bool apManual = mavlink_.getAutopilotCustomMode() == ROVER_CUSTOM_MODE_MANUAL;
    const SteerMode mode = steerModeSelect(steering_.isSensorOk(), sourceIsMavlink, apKnown, apManual);
    if (mode == steerMode_) return;

    // Reason: the sensor state when it is what changed, else the source, else the autopilot mode.
    char reason[40];
    if (mode == SteerMode::POSITION) {
        snprintf(reason, sizeof(reason), "sensor OK");
    } else if (steerMode_ == SteerMode::POSITION) {
        snprintf(reason, sizeof(reason), "sensor %s",
                 SteeringController::faultToken(steering_.getSensorFault()));
    } else if (!sourceIsMavlink) {
        snprintf(reason, sizeof(reason), "source %s",
                 (currentInputSource_ == InputSource::WEB) ? INPUT_SOURCE_NAME_WEB : INPUT_SOURCE_NAME_FAILSAFE);
    } else if (!apKnown) {
        snprintf(reason, sizeof(reason), "autopilot mode unknown");
    } else if (apManual) {
        snprintf(reason, sizeof(reason), "autopilot MANUAL");
    } else {
        snprintf(reason, sizeof(reason), "autopilot mode %lu", (unsigned long)mavlink_.getAutopilotCustomMode());
    }
    Debug::printfFeature(DebugFeature::SERVO, "[STEER] mode: %s -> %s (%s)\n",
                         steerModeToken(steerMode_), steerModeToken(mode), reason);
    {
        char tok[sizeof(reason)];   // journal values carry no spaces
        size_t i = 0;
        for (; reason[i] != '\0' && i < sizeof(tok) - 1; i++) tok[i] = (reason[i] == ' ') ? '_' : reason[i];
        tok[i] = '\0';
        EventJournal::log(JSrc::STEER, JLvl::I, "mode", "from=%s to=%s reason=%s",
                          steerModeToken(steerMode_), steerModeToken(mode), tok);
    }

    // Every change stops a rate-owned drive (a web jog/nudge and a closed-loop move are
    // untouched) and every entry into RATE needs the stick centred once before motion.
    if (steerMode_ == SteerMode::RATE) {
        steering_.stopRate();
    }
    if (mode == SteerMode::RATE) {
        steering_.disarmRate();
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] rate mode: centre the stick to arm");
    }
    steerMode_ = mode;
}

void VehicleController::processMavlinkCommands() {
    // Fresh-frame detection for the rate drive: the decoded channels are re-read every loop,
    // but only a NEW SERVO_OUTPUT_RAW may refresh the rate jog (else a stale value would keep
    // the motor running past STEER_JOG_TIMEOUT_MS).
    const uint32_t cmdCount = mavlink_.getCommandCount();
    const bool freshFrame = (cmdCount != lastMavCmdCount_);
    lastMavCmdCount_ = cmdCount;

    if (!mavlink_.isSignalValid()) {
        return;  // Safety check
    }

    // Apply steering, speed-scaled. applySteeringScale() sits BETWEEN the transport's decoded
    // command and the actuator — the same place ArduPilot's own MANUAL_OPTIONS scaling sat,
    // only on the side of the link that owns the speed measurement. The web set_steering path
    // below is deliberately NOT scaled: it is a bench control with no road speed behind it.
    // In RATE mode (no sensor, MANUAL) the same scaled value is a rotation-rate command,
    // passed on once per fresh frame; POSITION and BLOCKED keep the percent path (refused
    // without a sensor).
    float steeringPct = applySteeringScale(mavlink_.getSteering());
    if (steerMode_ == SteerMode::RATE) {
        if (freshFrame) {
            steering_.setRateCommand(steeringPct);
        }
    } else {
        steering_.setSteeringPercent(steeringPct);
    }

    // Apply throttle — skipped while the gear change owns it (boost PID or shift idle hold)
    if (!isThrottleLockedForShift()) {
        float throttlePct = mavlink_.getThrottle();
        if (shouldClipThrottle()) {
            throttlePct = min(TRANS_UNKNOWN_GEAR_THROTTLE_MAX, throttlePct);
        }
        applyOperatorThrottle(throttlePct);   // park-brake drive-away / hold, then the limiter + slew
    }

    // Gear / ignition / light / lock: "last change wins" with the GCS commands (MAV_CMD_USER_2).
    // Each RC channel is applied only when its in-band decoded value CHANGES (or on the first
    // valid frame after a re-sync); an absent / out-of-band channel is not a change, so a GCS
    // command is never overwritten by an unchanged switch (design D6).

    // Gear: an RC edge becomes the pending RC gear, retried until the engine runs and setGear()
    // accepts (today's retry). An accepted GCS gear cancels it.
    rc_.stepGear(vcmdRcGear(mavlink_.getChannel(ServoChannelConfig::TRANSMISSION)));
    if (rc_.gearPending && isEngineRunning()) {
        const TransmissionController::Gear gear = vcmdGearToTrans(rc_.pendingGear);
        bool accepted = transmission_.setGear(gear);
        if (accepted || transmission_.getTargetGear() == gear) {
            rc_.gearDone();
        }
        // If not accepted (e.g. speed interlock), the RC gear stays pending → retry next loop
    }

    // Apply brake
    float brakePct = mavlink_.getBrake();
    applyBrake(brakePct);

    // Ignition, on an RC change only. Cranking is sequenced inside RelayController, which
    // enforces a pre-crank dwell: the starter (R2) does not engage until the ignition/ECU line
    // (R1) has been powered for ACC_PRECRANK_DWELL_MS (immediately if R1 was already up that long).
    const int8_t rcIgn = vcmdRcIgnition(mavlink_.getChannel(ServoChannelConfig::IGNITION));
    if (rc_.ignition.step(rcIgn)) {
        if (rcIgn == VCMD_IGN_OFF) {
            ignitionOffAndPersist();
        } else if (rcIgn == VCMD_IGN_ACC) {
            relayController_.setIgnitionState(RelayController::IgnitionState::ACC);
        } else {
            // A fresh RC ON (an edge, or the first frame after a re-sync) arms the dwell-gated
            // auto-crank; RelayController::update() suppresses it while the engine runs.
            Debug::printlnFeature(DebugFeature::VEHICLE, "[VEHICLE] IGNITION selected - arming pre-crank dwell");
            relayController_.requestCrank();
        }
    }

    // Front light (channel 6), on an RC change only
    const int8_t rcLight = vcmdRcSwitch(mavlink_.getChannel(ServoChannelConfig::FRONT_LIGHT));
    if (rc_.light.step(rcLight)) {
        relayController_.setFrontLight(rcLight == 1);
    }

    // Front-wheel / diff lock (channel 7), on an RC change only
    const int8_t rcLock = vcmdRcSwitch(mavlink_.getChannel(ServoChannelConfig::WHEEL_LOCK));
    if (rc_.lock.step(rcLock)) {
        relayController_.setWheelLock(rcLock == 1);
    }
}

void VehicleController::ignitionOffAndPersist() {
    const bool wasOff = (relayController_.getIgnitionState() == RelayController::IgnitionState::OFF);
    relayController_.setIgnitionState(RelayController::IgnitionState::OFF);
    // Flush the persisted counters (distance and engine hours) on the TRANSITION into OFF only,
    // so a normal shutdown loses nothing.
    if (!wasOff) {
        speedSensor_.persistDistance();
        engineHours_.persist();
    }
}

uint16_t VehicleController::getVehicleCapsMask() const {
    // Fixed at boot. Both transmissions know R/N/H/L; ignition and headlight are relay functions
    // on every build; the lock only where RELAY_WHEEL_LOCK_FITTED. No handbrake on this build
    // (master: X6 is the steering VESC) — bit5 is never set. Relay availability is runtime and
    // recoverable: it gives FAILED, it does not clear bits.
    return vehicleCapsMask(true, true, true, true, isWheelLockFitted(), false);
}

VehicleCmdOutcome VehicleController::executeVehicleCommand(VehicleCmdFunction fn, int32_t value) {
    const bool underMavlink = isUnderMavlinkControl();
    const bool relaysOk = relayController_.isAvailable();
    VehicleCmdOutcome o = vcmdFailed();

    switch (fn) {
        case VehicleCmdFunction::GEAR: {
            const TransmissionController::Gear gear = vcmdGearToTrans((int8_t)value);
            VcmdGearInputs in;
            in.underMavlink = underMavlink;
            in.engineRunning = isEngineRunning();
            in.transInitialised = transmission_.isInitialised();
            in.alreadyAtTarget = in.transInitialised && transmission_.getDestinationGear() == gear;
            in.speedOk = transmission_.canChangeGear(gear);
            bool doSet = false;
            o = vcmdGearDecide(in, doSet);
            if (doSet) {
                const bool accepted = transmission_.setGear(gear);
                if (!accepted && transmission_.getTargetGear() != gear) {
                    o = vcmdFailed();   // defensive: setGear() refused for a reason not checked above
                }
            }
            if (o.result == VehicleCmdResult::ACCEPTED) {
                rc_.onGcsGearAccepted();   // last change wins over a pending RC gear
            }
            Debug::printfFeature(DebugFeature::VEHICLE, "[GCS] Gear %s: result %u reason %u\n",
                                 transmission_.getGearName(gear), (unsigned)o.result, (unsigned)o.reason);
            break;
        }
        case VehicleCmdFunction::IGNITION: {
            VcmdIgnAction action = VcmdIgnAction::NONE;
            o = vcmdIgnitionDecide(value, underMavlink, relaysOk, isEngineRunning(), action);
            switch (action) {
                case VcmdIgnAction::OFF:
                    ignitionOffAndPersist();
                    break;
                case VcmdIgnAction::ACC:
                    relayController_.setIgnitionState(RelayController::IgnitionState::ACC);
                    break;
                case VcmdIgnAction::IGNITION_NO_CRANK:
                    // Engine already running: no crank. ACC and IGNITION drive the same relays.
                    if (relayController_.getIgnitionState() != RelayController::IgnitionState::CRANKING) {
                        relayController_.setIgnitionState(RelayController::IgnitionState::IGNITION);
                    }
                    break;
                case VcmdIgnAction::CRANK:
                    Debug::printlnFeature(DebugFeature::VEHICLE, "[GCS] IGNITION ON - arming pre-crank dwell");
                    relayController_.requestCrank();
                    break;
                case VcmdIgnAction::NONE:
                default:
                    break;
            }
            Debug::printfFeature(DebugFeature::VEHICLE, "[GCS] Ignition %ld: result %u reason %u\n",
                                 (long)value, (unsigned)o.result, (unsigned)o.reason);
            break;
        }
        case VehicleCmdFunction::LIGHT:
            o = vcmdLightDecide(relaysOk);
            if (o.result == VehicleCmdResult::ACCEPTED) {
                relayController_.setFrontLight(value == 1);
            }
            Debug::printfFeature(DebugFeature::VEHICLE, "[GCS] Headlight %s: result %u\n",
                                 value == 1 ? "ON" : "OFF", (unsigned)o.result);
            break;
        case VehicleCmdFunction::LOCK:
            o = vcmdLockDecide(underMavlink, relaysOk);
            if (o.result == VehicleCmdResult::ACCEPTED && !relayController_.setWheelLock(value == 1)) {
                o = vcmdFailed();
            }
            Debug::printfFeature(DebugFeature::VEHICLE, "[GCS] Lock %s: result %u reason %u\n",
                                 value == 1 ? "ON" : "OFF", (unsigned)o.result, (unsigned)o.reason);
            break;
        case VehicleCmdFunction::HANDBRAKE:   // no parking brake on this build (bit5 never set)
        case VehicleCmdFunction::QUERY:       // answered by the transport
        default:
            o = {VehicleCmdResult::UNSUPPORTED, VCMD_REASON_NONE};
            break;
    }
    return o;
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

    // Allowed with the engine off (gear calibration). An uninitialised transmission is
    // initialised first, so the shift starts from the real gear / servo position; during the
    // initialisation step into NEUTRAL setGear() queues the request like any active step.
    if (!transmission_.isInitialised()) {
        transmission_.initialise();
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
    if (steering_.isDiagLocked()) {
        webPortal.sendResponse(false, "Steering is held by the USB diagnostics console — send \"diag off\" first");
        return;
    }
    if (!steering_.isSensorOk()) {
        // Defence in depth: the web UI locks the slider while steer_mode != position.
        webPortal.sendResponse(false, "Steering rejected: no steering sensor — use Jog/Nudge");
        return;
    }
    if (!steering_.setSteeringPercent(value)) {
        webPortal.sendResponse(false, "Steering rejected (not calibrated or sensor fault)");
        return;
    }
    webPortal.sendResponse(true, "Steering set");
}

void VehicleController::processSteerAngleCommand(float deg, WebPortal& webPortal) {
    // Same gating as processSteeringCommand() (web control is checked by the caller).
    if (steering_.isDiagLocked()) {
        webPortal.sendResponse(false, "Steering is held by the USB diagnostics console — send \"diag off\" first");
        return;
    }
    if (!steering_.isSensorOk()) {
        // Defence in depth: the web UI locks the angle input while steer_mode != position.
        webPortal.sendResponse(false, "Steering rejected: no steering sensor — use Jog/Nudge");
        return;
    }
    if (!isfinite(deg)) {
        webPortal.sendResponse(false, "Invalid steering angle");
        return;
    }
    if (!steering_.setSteeringAngleDeg(deg)) {
        webPortal.sendResponse(false, "Steering rejected (not calibrated or sensor fault)");
        return;
    }
    webPortal.sendResponse(true, "Steering angle set: " + String(deg, 1) + "°");
}

bool VehicleController::shouldClipThrottle() const {
    if (transmission_.isGearChangeActive())
        return true;
    if (transmission_.getTargetGear() == TransmissionController::Gear::GEAR_NEUTRAL)
        return true;
    if (transmission_.getTargetGear() == TransmissionController::Gear::GEAR_UNKNOWN)
        return true;   // not initialised: expected gear unknown
    return false;
}

void VehicleController::processThrottleCommand(float value, WebPortal& webPortal) {
    if (isThrottleLockedForShift()) {
        webPortal.sendResponse(false, "Gear change active");
        return;
    }
    if (shouldClipThrottle()) {
        value = min(TRANS_UNKNOWN_GEAR_THROTTLE_MAX, value);
    }
    // Record the operator's standing demand; update() re-limits it every loop from here on.
    webThrottleDemandPct_ = value;
    if (applyOperatorThrottle(value)) {
        // Still a success: the demand is stored and passes once the park brake lets it.
        if (getParkBrakeDaWait() == ParkBrakeDaWait::IGN_OFF) {
            webPortal.sendResponse(true, "Throttle set — held at idle: ignition off, the park brake stays applied");
        } else {
            webPortal.sendResponse(true, "Throttle set — held at idle until the park brake is released");
        }
        return;
    }
    webPortal.sendResponse(true, "Throttle set");
}

// ============================================================================
// AZTEC PARKING BRAKE (X6) — command entry point, drive-away release, throttle hold
// ============================================================================

ParkBrakeGear VehicleController::parkBrakeGear() const {
    switch (transmission_.getTargetGear()) {
        case TransmissionController::Gear::GEAR_NEUTRAL: return ParkBrakeGear::Neutral;
        case TransmissionController::Gear::GEAR_LOW:     return ParkBrakeGear::Low;
        case TransmissionController::Gear::GEAR_HIGH:    return ParkBrakeGear::High;
        case TransmissionController::Gear::GEAR_REVERSE: return ParkBrakeGear::Reverse;
        default:                                         return ParkBrakeGear::Unknown;
    }
}

bool VehicleController::isParkBrakeHoldingThrottle() const {
    return parkBrakeHoldsThrottle(parkBrake_.getState(), parkBrakeGear(),
                                  parkBrake_.getReleaseElapsedMs(), parkBrake_.getThrPassMs());
}

ParkBrakeDaWait VehicleController::getParkBrakeDaWait() const {
    const bool ignOn = relayController_.getIgnitionState() != RelayController::IgnitionState::OFF;
    return parkBrakeDaWait(parkBrake_.getState(), parkBrakeGear(), ignOn, pbDemandPct_, pbArmed_);
}

bool VehicleController::applyOperatorThrottle(float demandPct) {
    pbDemandPct_ = demandPct;

    // Drive-away latch. "Ignition on" = the commanded ignition-relay state is not OFF (ACC,
    // IGNITION and CRANKING all count; OFF also while the relay board is unavailable), never RPM.
    const bool ignOn = relayController_.getIgnitionState() != RelayController::IgnitionState::OFF;
    const ParkBrakeGear gear = parkBrakeGear();
    const bool shiftBusy = transmission_.isGearChangeActive() || isThrottleLockedForShift();
    const ParkBrakeLatchStep st = parkBrakeLatchStep(pbArmed_, parkBrake_.getState(), ignOn, gear,
                                                     shiftBusy, demandPct);
    pbArmed_ = st.armed;
    if (st.fire) {
        Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Drive-away: throttle %.1f %% > %.0f %% in gear — releasing\n",
                             demandPct, PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT);
        const ParkBrakeController::Outcome o = parkBrake_.request(ParkBrakeRequest::RELEASE, ParkBrakeSpeedCheck::OK);
        if (o.result == ParkBrakeResult::STARTED || o.result == ParkBrakeResult::REVERSING) {
            pbArmed_ = false;
        }
        pbBlockLogged_ = ParkBrakeDaWait::NONE;
    } else {
        // One BRAKE log line per blocked press (ignition off / needs a new press).
        const ParkBrakeDaWait why = (demandPct > PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT)
                                        ? parkBrakeDaWait(parkBrake_.getState(), gear, ignOn, demandPct, pbArmed_)
                                        : ParkBrakeDaWait::NONE;
        if (why != pbBlockLogged_ && why != ParkBrakeDaWait::NONE) {
            Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Throttle %.1f %% does not release: %s\n", demandPct,
                                 why == ParkBrakeDaWait::IGN_OFF ? "ignition off"
                                                                : "drop the throttle to 10 %% or below and press again");
        }
        pbBlockLogged_ = why;
    }

    // The limiter keeps evaluating the demand under the hold, so its state never freezes.
    const float limited = applySpeedLimit(demandPct);
    if (isParkBrakeHoldingThrottle()) {
        throttle_.idle();   // bypasses the opening slew; the pass later ramps from idle
        return true;
    }
    throttle_.setThrottlePercent(limited);
    return false;
}

VehicleController::ParkBrakeCommandResult VehicleController::commandParkBrake(ParkBrakeRequest req,
                                                                              ParkBrakeSource src) {
    const char* srcName = (src == ParkBrakeSource::GCS) ? "gcs" : "web";
    ParkBrakeCommandResult r{ParkBrakeResult::STARTED, speedSensor_.getSpeedMs(), false};
    ParkBrakeSpeedCheck speed = ParkBrakeSpeedCheck::OK;
    if (req == ParkBrakeRequest::APPLY) {
        speed = parkBrakeApplySpeedOk(r.speedMs, speedSensor_.isValid(), speedSensor_.isSuspicious());
    }
    const ParkBrakeController::Outcome o = parkBrake_.request(req, speed);
    r.result = o.result;
    r.stateNotSaved = o.stateNotSaved;

    if (r.result == ParkBrakeResult::REFUSED_MOVING) {
        Debug::printfFeature(DebugFeature::BRAKE,
            "[PBRAKE] Apply refused (%s): moving at %.1f km/h (max %.1f)\n",
            srcName, r.speedMs * 3.6f, PARK_BRAKE_APPLY_MAX_SPEED_MS * 3.6f);
    } else if (r.result == ParkBrakeResult::STARTED || r.result == ParkBrakeResult::STARTED_SPEED_UNKNOWN ||
               r.result == ParkBrakeResult::REVERSING) {
        Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] %s accepted (%s)\n",
                             req == ParkBrakeRequest::APPLY ? "Apply" : "Release", srcName);
        if (req == ParkBrakeRequest::RELEASE) {
            pbArmed_ = false;   // a release always needs a new rising edge for the next drive-away
        } else if (isParkBrakeHoldingThrottle() && !isThrottleLockedForShift() && !throttle_.isCalibrating() &&
                   currentInputSource_ != InputSource::FAILSAFE) {
            // An APPLY over a passing release (or any apply in gear): the operator throttle goes back
            // to idle at once, not only at the next operator-path evaluation.
            throttle_.idle();
        }
    }
    return r;
}

void VehicleController::processParkBrakeCommand(const String& value, WebPortal& webPortal) {
    ParkBrakeRequest req;
    if (value == "apply") {
        req = ParkBrakeRequest::APPLY;
    } else if (value == "release") {
        req = ParkBrakeRequest::RELEASE;
    } else {
        webPortal.sendResponse(false, "Park brake: unknown request");
        return;
    }
    const bool apply = (req == ParkBrakeRequest::APPLY);
    const ParkBrakeCommandResult r = commandParkBrake(req, ParkBrakeSource::WEB);
    const char* notSaved = r.stateNotSaved ? " (state not saved)" : "";
    switch (r.result) {
        case ParkBrakeResult::STARTED:
            webPortal.sendResponse(true, apply ? String("Park brake applying")
                                               : String("Park brake releasing") + notSaved);
            break;
        case ParkBrakeResult::STARTED_SPEED_UNKNOWN:
            webPortal.sendResponse(true, "Park brake applying (speed unknown)");
            break;
        case ParkBrakeResult::REVERSING:
            webPortal.sendResponse(true, apply ? String("Park brake reversing to apply")
                                               : String("Park brake reversing to release") + notSaved);
            break;
        case ParkBrakeResult::ALREADY_RUNNING:
            webPortal.sendResponse(true, apply ? "Park brake already applying" : "Park brake already releasing");
            break;
        case ParkBrakeResult::REFUSED_MOVING:
            webPortal.sendResponse(false, "Park brake apply refused: moving at " + String(r.speedMs * 3.6f, 1) +
                                          " km/h (max " + String(PARK_BRAKE_APPLY_MAX_SPEED_MS * 3.6f, 1) + ")");
            break;
        case ParkBrakeResult::REFUSED_NVS:
            webPortal.sendResponse(false, "Failed to save the park brake state — apply refused");
            break;
        case ParkBrakeResult::REFUSED_DRIVER:
        default:
            webPortal.sendResponse(false, "Park brake driver not available");
            break;
    }
}

void VehicleController::processSetParkBrakeTravelMsCommand(float value, WebPortal& webPortal) {
    if (!isnan(value)) value = roundf(value);   // whole ms
    if (isnan(value) || !parkBrakeTravelMsValid((int32_t)value)) {
        webPortal.sendResponse(false, "Park brake travel time out of range (" +
                                      String(PARK_BRAKE_TRAVEL_MS_MIN) + "-" +
                                      String(PARK_BRAKE_TRAVEL_MS_MAX) + " ms)");
        return;
    }
    const uint16_t ms = (uint16_t)value;
    if (!parkBrake_.setTravelMs(ms)) {
        webPortal.sendResponse(false, "Failed to save the park brake setting");
        return;
    }
    webPortal.sendResponse(true, "Park brake travel time set to " + String(ms) + " ms");
}

void VehicleController::processSetParkBrakeThrPassPctCommand(float value, WebPortal& webPortal) {
    if (!isnan(value)) value = roundf(value);   // whole %
    if (isnan(value) || !parkBrakeThrPassPctValid((int32_t)value)) {
        webPortal.sendResponse(false, "Park brake throttle release point out of range (" +
                                      String(PARK_BRAKE_THR_PASS_PCT_MIN) + "-" +
                                      String(PARK_BRAKE_THR_PASS_PCT_MAX) + " %)");
        return;
    }
    const uint8_t pct = (uint8_t)value;
    if (!parkBrake_.setThrPassPct(pct)) {
        webPortal.sendResponse(false, "Failed to save the park brake setting");
        return;
    }
    webPortal.sendResponse(true, "Park brake throttle release point set to " + String(pct) + " %");
}

void VehicleController::processSetParkBrakeInvertCommand(bool invert, WebPortal& webPortal) {
    switch (parkBrake_.setInvert(invert)) {
        case ParkBrakeController::InvertResult::OK:
            pbArmed_ = false;   // the state became UNKNOWN: re-arm from a fresh sample
            webPortal.sendResponse(true, invert ? "Park brake direction inverted — state unknown"
                                                : "Park brake direction normal — state unknown");
            break;
        case ParkBrakeController::InvertResult::REFUSED_MOVING:
            webPortal.sendResponse(false, "Park brake direction change refused: actuator moving");
            break;
        default:
            webPortal.sendResponse(false, "Failed to save the park brake setting");
            break;
    }
}

// ============================================================================
// MAX-SPEED LIMITER — SPEED_MAX ceiling, proportional taper and integral trim
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

void VehicleController::clearSpeedLimitSaturation(uint32_t now) {
    satSinceMs_ = 0;
    if (satActive_) {
        satActive_ = false;
        satClearedMs_ = now ? now : 1;   // starts the display hold; 0 is the "never" sentinel
    }
}

float VehicleController::applySpeedLimit(float throttlePct) {
    float limitMs = getSpeedLimitMs();
    logSpeedLimitChange(limitMs);
    uint32_t now = millis();

    if (limitMs <= 0.0f) {
        // No usable SPEED_MAX => no limiting at all. Reset the law (ceiling 100 %, trim 0) so the
        // next limit that arrives never resumes from a stale ceiling or a stale trim.
        limiter_.reset();
        limiterLastMs_ = 0;
        clearSpeedLimitSaturation(now);
        return throttlePct;
    }

    // Fail OPEN on sensor loss. The limiter only ever acts NEAR AND ABOVE the maximum speed, and a
    // lost sensor reads 0 m/s — clamping throttle on a reading we do not trust, mid-manoeuvre,
    // is the more dangerous failure. Over-speed protection assumes a working sensor.
    if (!speedSensor_.isValid()) {
        if (lastSpeedLimitWarnMs_ == 0 || (now - lastSpeedLimitWarnMs_) >= SPEED_LIMIT_WARN_MS) {
            lastSpeedLimitWarnMs_ = now;
            Debug::printlnFeature(DebugFeature::VEHICLE,
                "[SPEED] WARNING: speed limit active but speed reading is invalid — not clamping (fail open)");
        }
        limiter_.reset();
        limiterLastMs_ = 0;
        clearSpeedLimitSaturation(now);
        return throttlePct;
    }
    lastSpeedLimitWarnMs_ = 0;

    // Taper + integral trim + ceiling slew (include/SpeedLimiterLaw.h). dt is capped at 1 s and
    // ignored on the first evaluation, which adopts the taper directly with a zero trim, so a
    // stalled or just-restarted loop cannot integrate an unknown interval into a step change.
    SpeedLimiterParams params;
    params.taperFraction = SPEED_LIMIT_TAPER_FRACTION;
    params.taperMinMs = SPEED_LIMIT_TAPER_MIN_MS;
    params.taperMaxMs = SPEED_LIMIT_TAPER_MAX_MS;
    params.floorPct = limiterFloorPct_;
    params.kiPctSPerMs = SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS;
    params.risePctS = SPEED_LIMIT_TRIM_RISE_PCT_S;
    params.slewPctS = limiterSlewPctS_;   // also caps the trim's fall (step ≤ slew), intended
    params.bindMarginPct = SPEED_LIMIT_BIND_MARGIN_PCT;

    float dt = 0.0f;
    if (limiterLastMs_ != 0) {
        dt = (now - limiterLastMs_) / 1000.0f;
        if (dt > 1.0f) dt = 1.0f;
    }
    limiterLastMs_ = now ? now : 1;   // 0 is the "first call" sentinel

    const float speedMs = speedSensor_.getSpeedMs();
    const float out = speedLimiterStep(limiter_, params, throttlePct, speedMs, limitMs, dt);

    // Saturation (design D6): the ceiling has run out of authority — at the floor — while the
    // vehicle stays clearly above the limit. Entry needs SPEED_LIMIT_SAT_HOLD_MS of the raw
    // condition; it clears on the first evaluation where any part fails (no release debounce).
    // Shown in the web console only: deliberately no Debug line and no MAVLink message.
    const bool raw = (limiter_.ceilPct <= limiterFloorPct_ + 0.5f) &&
                     (speedMs > limitMs + SPEED_LIMIT_SAT_MARGIN_MS);
    if (raw) {
        if (satSinceMs_ == 0) {
            satSinceMs_ = now ? now : 1;
        } else if (!satActive_ && (now - satSinceMs_) >= SPEED_LIMIT_SAT_HOLD_MS) {
            satActive_ = true;
        }
    } else {
        clearSpeedLimitSaturation(now);
    }

    return out;
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

void VehicleController::processSetSpeedLimitFloorCommand(float value, WebPortal& webPortal) {
    if (isnan(value) || value < SPEED_LIMIT_FLOOR_MIN_PCT || value > SPEED_LIMIT_FLOOR_MAX_PCT) {
        webPortal.sendResponse(false, "Speed-limiter floor out of range (" +
                                      String(SPEED_LIMIT_FLOOR_MIN_PCT, 0) + "-" +
                                      String(SPEED_LIMIT_FLOOR_MAX_PCT, 0) + " %)");
        return;
    }

    // Persist first: if the write fails, the RAM value is left as it was, so what runs is always
    // what the next boot will load. The "speed" namespace is shared with the speed-sensor
    // calibration and odometer — write the one key, never clear() it.
    Preferences prefs;
    bool ok = false;
    if (prefs.begin("speed", false)) {
        ok = prefs.putFloat(SPEED_LIMIT_FLOOR_NVS_KEY, value) == sizeof(float);
        prefs.end();
    }
    if (!ok) {
        webPortal.sendResponse(false, "Failed to save the speed-limiter floor");
        return;
    }

    limiterFloorPct_ = value;   // the law re-clamps trim and ceiling on its next evaluation
    webPortal.sendResponse(true, "Speed-limiter floor set to " + String(value, 1) + " %");
    Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] Limiter floor set to %.1f %%\n", value);
}

void VehicleController::processSetSpeedLimitSlewCommand(float value, WebPortal& webPortal) {
    if (isnan(value) || value < SPEED_LIMIT_SLEW_MIN_PCT_S || value > SPEED_LIMIT_SLEW_MAX_PCT_S) {
        webPortal.sendResponse(false, "Speed-limiter throttle rate out of range (" +
                                      String(SPEED_LIMIT_SLEW_MIN_PCT_S, 0) + "-" +
                                      String(SPEED_LIMIT_SLEW_MAX_PCT_S, 0) + " %/s)");
        return;
    }

    // Persist first, exactly like the floor: RAM only changes once NVS holds the new value.
    // One key in the shared "speed" namespace — never clear() it.
    Preferences prefs;
    bool ok = false;
    if (prefs.begin("speed", false)) {
        ok = prefs.putFloat(SPEED_LIMIT_SLEW_NVS_KEY, value) == sizeof(float);
        prefs.end();
    }
    if (!ok) {
        webPortal.sendResponse(false, "Failed to save the speed-limiter throttle rate");
        return;
    }

    limiterSlewPctS_ = value;   // used from the law's next evaluation
    webPortal.sendResponse(true, "Speed-limiter throttle rate set to " + String(value, 1) + " %/s");
    Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] Limiter slew set to %.1f %%/s\n", value);
}

void VehicleController::processSetThrottleOpenSlewCommand(float value, WebPortal& webPortal) {
    if (!isnan(value)) value = roundf(value);   // whole %/s
    if (!ThrottleController::isValidOpenSlew(value)) {
        webPortal.sendResponse(false, "Throttle open rate out of range (0 or " +
                                      String(THROTTLE_OPEN_SLEW_MIN_PCT_S, 0) + "-" +
                                      String(THROTTLE_OPEN_SLEW_MAX_PCT_S, 0) + " %/s)");
        return;
    }
    // ThrottleController persists first (NVS "throttle"/open_slew), then applies.
    if (!throttle_.setOpenSlewPctS(value)) {
        webPortal.sendResponse(false, "Failed to save the throttle open rate");
        return;
    }
    if (value == 0.0f) {
        webPortal.sendResponse(true, "Throttle open rate off");
    } else {
        webPortal.sendResponse(true, "Throttle open rate set to " + String(value, 0) + " %/s");
    }
}

bool VehicleController::setBrakeTravelMs(uint16_t ms) {
    if (!brakeTravelMsValid(ms)) return false;
    // Persist first: RAM only changes once NVS holds the new value. Own namespace, one key.
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(BRAKE_NVS_NAMESPACE, false)) {
        ok = prefs.putUShort(BRAKE_TRAVEL_NVS_KEY, ms) == sizeof(uint16_t);
        prefs.end();
    }
    if (!ok) return false;
    brakeTravelMs_ = ms;   // the estimate's rate changes from the next updateBrakeControl(); no rescale
    Debug::printfFeature(DebugFeature::BRAKE, "[BRAKE] Travel time set to %u ms\n", (unsigned)ms);
    return true;
}

bool VehicleController::setBrakeHoldPwm(uint8_t pwm) {
    if (!brakeHoldPwmValid(pwm)) return false;
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(BRAKE_NVS_NAMESPACE, false)) {
        ok = prefs.putUChar(BRAKE_HOLD_NVS_KEY, pwm) == sizeof(uint8_t);
        prefs.end();
    }
    if (!ok) return false;
    brakeHoldPwm_ = pwm;   // read every hold cycle
    Debug::printfFeature(DebugFeature::BRAKE, "[BRAKE] Hold drive set to %u\n", (unsigned)pwm);
    return true;
}

void VehicleController::processSetBrakeTravelMsCommand(float value, WebPortal& webPortal) {
    if (!isnan(value)) value = roundf(value);   // whole ms
    if (isnan(value) || !brakeTravelMsValid((int32_t)value)) {
        webPortal.sendResponse(false, "Brake travel time out of range (" +
                                      String(BRAKE_TRAVEL_MS_MIN) + "-" +
                                      String(BRAKE_TRAVEL_MS_MAX) + " ms)");
        return;
    }
    const uint16_t ms = (uint16_t)value;
    if (!setBrakeTravelMs(ms)) {
        webPortal.sendResponse(false, "Failed to save the brake setting");
        return;
    }
    webPortal.sendResponse(true, "Brake travel time set to " + String(ms) + " ms");
}

void VehicleController::processSetBrakeHoldPwmCommand(float value, WebPortal& webPortal) {
    if (!isnan(value)) value = roundf(value);   // whole PWM step
    if (isnan(value) || !brakeHoldPwmValid((int32_t)value)) {
        webPortal.sendResponse(false, "Brake hold out of range (" +
                                      String(BRAKE_HOLD_PWM_MIN) + "-" +
                                      String(BRAKE_HOLD_PWM_MAX) + ")");
        return;
    }
    const uint8_t pwm = (uint8_t)value;
    if (!setBrakeHoldPwm(pwm)) {
        webPortal.sendResponse(false, "Failed to save the brake setting");
        return;
    }
    if (pwm == 0) {
        webPortal.sendResponse(true, "Brake hold off");
    } else {
        webPortal.sendResponse(true, "Brake hold set to " + String(pwm));
    }
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

// === Steering park correction settings (SteeringController owns them, NVS "steering") ===

static const char* const PARK_SAVE_FAILED = "Failed to save the park correction setting";

void VehicleController::processSetSteerParkEnabledCommand(bool on, WebPortal& webPortal) {
    if (steering_.setParkEnabled(on) != SteeringController::ParkSetResult::OK) {
        webPortal.sendResponse(false, PARK_SAVE_FAILED);
        return;
    }
    webPortal.sendResponse(true, on ? "Park correction enabled" : "Park correction disabled");
}

// Strict non-negative integer (digits only, at most 5 of them); false otherwise
static bool parseParkInt(const String& s, int32_t& out) {
    if (s.length() == 0 || s.length() > 5) return false;
    int32_t v = 0;
    for (unsigned i = 0; i < s.length(); i++) {
        const char c = s[i];
        if (c < '0' || c > '9') return false;
        v = v * 10 + (c - '0');
    }
    out = v;
    return true;
}

void VehicleController::processSetSteerParkRungCommand(float rung, const String& strValue, WebPortal& webPortal) {
    static const char* const OUT_OF_RANGE = "Park rung out of range (duty 40-150, 40-200 ms)";
    if (isnan(rung) || rung != roundf(rung) || rung < 1.0f || rung > (float)STEER_PARK_RUNGS) {
        webPortal.sendResponse(false, "Invalid park rung (use 1-3)");
        return;
    }
    const uint8_t n = (uint8_t)rung;
    // strValue "D,T": two whole numbers, duty and ms
    String v = strValue;
    v.trim();
    const int comma = v.indexOf(',');
    int32_t duty = 0, ms = 0;
    String ds = comma > 0 ? v.substring(0, comma) : String();
    String ts = comma > 0 ? v.substring(comma + 1) : String();
    ds.trim();
    ts.trim();
    if (comma <= 0 || !parseParkInt(ds, duty) || !parseParkInt(ts, ms)) {
        webPortal.sendResponse(false, OUT_OF_RANGE);
        return;
    }
    switch (steering_.setParkRung(n, duty, ms)) {
        case SteeringController::ParkSetResult::OK:
            webPortal.sendResponse(true, "Park rung " + String(n) + " set to " + String(duty) + "×" + String(ms));
            return;
        case SteeringController::ParkSetResult::OUT_OF_RANGE:
            webPortal.sendResponse(false, OUT_OF_RANGE);
            return;
        case SteeringController::ParkSetResult::NOT_MONOTONIC:
            webPortal.sendResponse(false, "Park ladder must not decrease (rung " + String(n) + ")");
            return;
        case SteeringController::ParkSetResult::SAVE_FAILED:
        default:
            webPortal.sendResponse(false, PARK_SAVE_FAILED);
            return;
    }
}

void VehicleController::processSetSteerParkTolCommand(float deg, WebPortal& webPortal) {
    // Degrees from the page, rounded to the repo's 0.1° unit
    const int32_t tol = isfinite(deg) ? (int32_t)lroundf(deg * 10.0f) : -1;
    switch (steering_.setParkDoneTol(tol)) {
        case SteeringController::ParkSetResult::OK:
            webPortal.sendResponse(true, "Park tolerance set to " + String((float)tol / 10.0f, 1) + "°");
            return;
        case SteeringController::ParkSetResult::SAVE_FAILED:
            webPortal.sendResponse(false, PARK_SAVE_FAILED);
            return;
        default:
            webPortal.sendResponse(false, "Park tolerance out of range (0.1-0.5°)");
            return;
    }
}

void VehicleController::processSetSteerParkMaxCommand(float count, WebPortal& webPortal) {
    // Whole numbers only: 2.5 is refused, not rounded
    const bool whole = isfinite(count) && count == roundf(count) && count >= -1000.0f && count <= 1000.0f;
    const int32_t n = whole ? (int32_t)count : -1;
    switch (steering_.setParkMaxPulses(n)) {
        case SteeringController::ParkSetResult::OK:
            webPortal.sendResponse(true, "Park max pulses set to " + String(n));
            return;
        case SteeringController::ParkSetResult::SAVE_FAILED:
            webPortal.sendResponse(false, PARK_SAVE_FAILED);
            return;
        default:
            webPortal.sendResponse(false, "Park max pulses out of range (1-10)");
            return;
    }
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

void VehicleController::processGearCodeCommand(const String& which, const String& gearStr, WebPortal& webPortal) {
    const bool reset = (which == "reset_gear_codes");
    if (isEngineRunning()) {
        webPortal.sendResponse(false, reset ? "Gear code reset refused: engine is running"
                                            : "Gear code capture refused: engine is running");
        return;
    }
    if (reset) {
        if (!transmission_.resetGearCodes()) {
            webPortal.sendResponse(false, "Gear codes reset in RAM only: NVS unavailable");
            return;
        }
        webPortal.sendResponse(true, "Gear codes reset to defaults");
        return;
    }

    TransmissionController::Gear gear;
    if      (gearStr == "R") gear = TransmissionController::Gear::GEAR_REVERSE;
    else if (gearStr == "N") gear = TransmissionController::Gear::GEAR_NEUTRAL;
    else if (gearStr == "L") gear = TransmissionController::Gear::GEAR_LOW;
    else if (gearStr == "H") gear = TransmissionController::Gear::GEAR_HIGH;
    else {
        webPortal.sendResponse(false, "Gear code capture failed: invalid gear (use R/N/L/H)");
        return;
    }

    String msg;
    bool ok = transmission_.captureGearCode(gear, msg);
    Debug::printfFeature(DebugFeature::VEHICLE, "[WEB] %s\n", msg.c_str());
    webPortal.sendResponse(ok, msg);
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

void VehicleController::processSetBoostEnabledCommand(bool enabled, WebPortal& webPortal) {
    // Persist first: RAM only changes once NVS holds the new value. One key in the shared
    // "boost" namespace (beside target_rpm) — never clear() it.
    Preferences prefs;
    bool ok = false;
    if (prefs.begin("boost", false)) {
        ok = prefs.putBool(BOOST_ENABLED_NVS_KEY, enabled) == sizeof(uint8_t);
        prefs.end();
    }
    if (!ok) {
        webPortal.sendResponse(false, "Failed to save gear boost setting");
        return;
    }

    boostGearEnabled_ = enabled;

    // Switched off while the PID holds RPM for a gear change (not the manual test): release it
    // exactly like the normal end of a gear change, then let the idle hold own the rest of the
    // shift. Switching on mid-shift keeps any idle hold; update() starts no PID while it lasts.
    if (!enabled && gearBoostActive_ && !boostManualActive_) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[BOOST] Switched off, releasing PID");
        canController_.setRPMPollInterval(CAN_POLL_INTERVAL_RPM);
        throttle_.idle();
        webThrottleDemandPct_ = 0.0f;
        gearBoostActive_ = false;
        if (transmission_.needsThrottleBoost()) {
            shiftIdleHold_ = true;
            Debug::printlnFeature(DebugFeature::VEHICLE, "[BOOST] Idle hold for the rest of the gear change");
        }
    }

    Debug::printfFeature(DebugFeature::VEHICLE, "[BOOST] Gear-change boost %s\n", enabled ? "enabled" : "disabled");
    webPortal.sendResponse(true, enabled ? "Gear boost enabled" : "Gear boost disabled");
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
            webPortal.sendResponse(false, "ECU probe busy (gear change, probe or CAN sniffer in progress)");
            break;
        case CANController::ProbeStart::NO_ECU:
            Debug::printlnFeature(DebugFeature::VEHICLE, "[WEB] ECU probe: no ECU / disconnected");
            webPortal.sendResponse(false, "No ECU / CAN disconnected");
            break;
    }
}

void VehicleController::processCanSniffCommand(WebPortal& webPortal) {
    bool gearChangeActive = transmission_.isGearChangeActive() || transmission_.needsThrottleBoost();
    switch (canController_.startSniff(CAN_SNIFF_WINDOW_MS, gearChangeActive)) {
        case CANController::SniffStart::STARTED:
            Debug::printlnFeature(DebugFeature::VEHICLE, "[WEB] CAN sniffer started");
            webPortal.sendResponse(true, "CAN sniffer started");
            break;
        case CANController::SniffStart::BUSY_PROBE:
            webPortal.sendResponse(false, "CAN sniffer refused: ECU probe active");
            break;
        case CANController::SniffStart::BUSY_LWS_ZERO:
            webPortal.sendResponse(false, "CAN sniffer refused: sensor zero-set active");
            break;
        case CANController::SniffStart::NOT_INITIALIZED:
            webPortal.sendResponse(false, "CAN sniffer refused: CAN not initialized");
            break;
        case CANController::SniffStart::BUSY:
        default:
            webPortal.sendResponse(false, "CAN sniffer refused: busy (gear change or already running)");
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
        if (isEngineRunning()) {   // effective RPM (tacho when fitted, CAN otherwise)
            errorMsg = "Engine already running";
            Debug::printfFeature(DebugFeature::VEHICLE, "[IGNITION] Rejected: engine already running (RPM: %u, %s)\n",
                                 (unsigned)effRpm_.rpm, rpmSourceName(effRpm_.source));
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

bool VehicleController::setWheelLock(bool locked) {
    bool ok = relayController_.setWheelLock(locked);
    if (ok) {
        Debug::printfFeature(DebugFeature::VEHICLE, "[WHEEL_LOCK] Lock: %s\n", locked ? "LOCKED" : "UNLOCKED");
    }
    return ok;
}

void VehicleController::processWheelLockCommand(bool locked, WebPortal& webPortal) {
    if (!isWheelLockFitted()) {
        // RELAY_WHEEL_LOCK_FITTED = 0: no relay behind the function — never report a lock as applied.
        relayController_.setWheelLock(locked);   // logs the rejection once
        webPortal.sendResponse(false, "Front-wheel lock not fitted on this vehicle");
        return;
    }
    bool ok = setWheelLock(locked);
    webPortal.sendResponse(ok, ok ? String("Lock ") + (locked ? "LOCKED" : "UNLOCKED")
                                  : String("Wheel lock command rejected"));
}

void VehicleController::processRelayTestCommand(int32_t relayNumber, bool on, WebPortal& webPortal) {
    // Out-of-range numbers are folded to 0, which RelayController rejects with the
    // "must be 1-8" text (a uint8_t cast alone would wrap e.g. 257 onto Relay_1).
    uint8_t n = (relayNumber >= 1 && relayNumber <= 8) ? (uint8_t)relayNumber : 0;
    const char* reason = nullptr;
    bool ok = relayController_.setRelayTest(n, on, reason);
    if (!ok) {
        webPortal.sendResponse(false, String("Relay test: ") + (reason ? reason : "rejected"));
        return;
    }
    webPortal.sendResponse(true, String("Relay test: Relay_") + String(n) + (on ? " ON" : " OFF"));
}

void VehicleController::processRelayBoardCommand(const String& value, WebPortal& webPortal) {
    RelayBoardSetting setting = RelayBoardSetting::AUTO;
    if (!relayBoardSettingParse(value.c_str(), setting)) {
        webPortal.sendResponse(false, "Relay board must be auto, pca9557 or xl9535");
        return;
    }
    // Persist first (RelayController owns the "relay" namespace); nothing else changes now.
    if (!relayController_.saveBoardSetting(setting)) {
        webPortal.sendResponse(false, "Failed to save the relay board setting");
        return;
    }
    webPortal.sendResponse(true, String("Relay board set to ") + relayBoardSettingToken(setting) +
                                 " — takes effect after reboot");
}

void VehicleController::processBoostCommand(bool on, WebPortal& webPortal) {
    (void)on;
    // aztec has no Buck-Boost board: X10 pin 3 is the 12 V battery sense. GPIO46 stays HIGH.
    Debug::printlnFeature(DebugFeature::VEHICLE, "[BAT] set_boost ignored — boost rail not fitted");
    webPortal.sendResponse(false, "Boost rail not fitted (X10 is the 12 V battery sense)");
}

void VehicleController::processSteerCalCommand(const String& which, WebPortal& webPortal) {
    if (steering_.isDiagLocked()) {
        webPortal.sendResponse(false, "Steering is held by the USB diagnostics console — send \"diag off\" first");
        return;
    }
    if (which == "steer_cal_center") {
        // The center is the steering sensor's own stored zero: there is no center capture.
        webPortal.sendResponse(false,
            "Capture center removed: the center is the sensor zero — use \"Set sensor zero\"");
        return;
    }

    const bool left = (which == "steer_cal_left");
    const String what = left ? "left limit" : "right limit";
    const bool ok = left ? steering_.captureLeftLimit() : steering_.captureRightLimit();
    if (!ok) {
        const char* reason = steering_.getCaptureError();
        webPortal.sendResponse(false, "Capture " + what + " failed: " + String(reason ? reason : "unknown"));
        return;
    }

    // Saved immediately; reported in degrees, firmware sign (+ = right)
    String msg = "Steering " + what + " saved: " +
                 String((float)steering_.getLastCapturePos() / 10.0f, 1) + "°";
    if (steering_.isLastCaptureBeyondSafe()) {
        msg += " — WARNING: beyond the ±550° safe range, near the sensor latch at ~600°"
               " (closed-loop moves stop at 550°)";
    }
    if (steering_.isLastCaptureToggled()) {
        // The capture inferred the sensor direction; the opposite limit was cleared.
        msg += String(" — sensor direction detected: ") +
               (steering_.isSensorInverted() ? "mirrored" : "normal") +
               " — capture the " + (left ? "right" : "left") + " limit again";
    } else {
        msg += steering_.isCalibrated() ? " — calibration complete" : " — capture the other limit too";
    }
    webPortal.sendResponse(true, msg);
}

void VehicleController::processSteerZeroCommand(WebPortal& webPortal) {
    if (steering_.isDiagLocked()) {
        webPortal.sendResponse(false, "Steering is held by the USB diagnostics console — send \"diag off\" first");
        return;
    }
    if (!steering_.requestSensorZero()) {
        const char* why = steering_.getZeroRefusal();
        webPortal.sendResponse(false, "Sensor zero refused: " + String(why ? why : "unknown"));
        return;
    }
    steerZeroResponsePortal_ = &webPortal;   // final result sent from update()
    webPortal.sendResponse(true, "Sensor zero started — Left/Right limits will be erased");
}

void VehicleController::processSteerJogCommand(float value, WebPortal& webPortal) {
    if (steering_.isDiagLocked()) {
        webPortal.sendResponse(false, "Steering is held by the USB diagnostics console — send \"diag off\" first");
        return;
    }
    // value encodes direction and speed: sign = direction, |value| = PWM duty.
    // |value| <= 1 (legacy -1/0/1 form) uses the default jog duty.
    int8_t direction = (value > 0.0f) ? 1 : (value < 0.0f) ? -1 : 0;
    float mag = fabsf(value);
    uint8_t duty = (mag > 1.0f) ? (uint8_t)min(mag, 255.0f) : STEER_JOG_DUTY;
    steering_.jog(direction, duty);
    webPortal.sendResponse(true, direction == 0 ? "Jog stop" : (direction > 0 ? "Jog right" : "Jog left"));
}

void VehicleController::processSteerNudgeCommand(int8_t direction, WebPortal& webPortal) {
    if (steering_.isDiagLocked()) {
        webPortal.sendResponse(false, "Steering is held by the USB diagnostics console — send \"diag off\" first");
        return;
    }
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
        float movementRate = brakeRatePctPerMs(brakeTravelMs_) * speedFraction;  // %/ms
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
        // (configured hold drive; 0 = no push, the actuator coasts)
        if (brakeHoldPwm_ > 0) {
            brake_.setSpeed(brakeHoldPwm_);
        } else {
            brake_.stop();
        }
    }
}

void VehicleController::updateShiftIdleHold() {
    bool shifting = transmission_.needsThrottleBoost();

    if (shiftIdleHold_) {
        // A manual boost test started mid-hold owns the servo (gearBoostActive_): don't fight it.
        if (!shifting) {
            Debug::printlnFeature(DebugFeature::VEHICLE, "[BOOST] Gear change complete, releasing idle hold");
            if (!gearBoostActive_) throttle_.idle();
            webThrottleDemandPct_ = 0.0f;  // the hold ended at IDLE — don't let the old web demand undo it
            shiftIdleHold_ = false;
            return;
        }
        if (!gearBoostActive_) throttle_.idle();
        return;
    }

    // Start only with the switch off and nothing else owning the throttle (PID / manual test).
    // Idle is always safe: no CAN / engine-RPM precondition, the RPM poll rate is left alone.
    if (shifting && !boostGearEnabled_ && !gearBoostActive_ && !boostManualActive_) {
        shiftIdleHold_ = true;
        throttle_.idle();
        Debug::printlnFeature(DebugFeature::VEHICLE, "[BOOST] Boost off, holding idle during gear change");
    }
}

void VehicleController::updateGearBoostPID() {
    uint32_t now = millis();
    const EffectiveRpm rpm = effRpm_;   // tacho when fitted, CAN otherwise

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
        if (!rpm.valid || rpm.rpm < ENGINE_RUNNING_RPM_THRESHOLD) {
            return;  // Engine not running — unsafe to boost throttle
        }
        gearBoostActive_ = true;
        gearBoostStartTime_ = now;
        pidLastUpdateTime_ = now;
        lastRpmSampleMs_ = 0;
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

    // No valid RPM source (CAN stale and no fitted tacho): don't accumulate integral
    if (!rpm.valid) {
        throttle_.idle();
        return;
    }

    // No new RPM sample (CAN reply or accepted tacho edge) since last compute: hold last µs,
    // skip integral accumulation
    if (rpm.sampleMs == lastRpmSampleMs_) {
        throttle_.setThrottleUs(lastPIDThrottleUs_);
        return;
    }
    lastRpmSampleMs_ = rpm.sampleMs;

    // Compute PID
    float dt = (now - pidLastUpdateTime_) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    pidLastUpdateTime_ = now;

    float error = (float)boostTargetRpm_ - (float)rpm.rpm;

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
        Debug::printfFeature(DebugFeature::VEHICLE, "[BOOST] Us=%d Error=%.0f RPM=%u (%s) age=%lums\n", us, error,
                             (unsigned)rpm.rpm, rpmSourceName(rpm.source), (unsigned long)(now - rpm.sampleMs));
        lastProgressLog = now;
    }
}
