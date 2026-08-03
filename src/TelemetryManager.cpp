#include "TelemetryManager.h"

TelemetryManager::TelemetryManager(VehicleController& vehicleController,
                                   WebPortal& webPortal,
                                   MavlinkInterface& mavlink)
    : vehicleController_(vehicleController),
      webPortal_(webPortal),
      mavlink_(mavlink),
      lastBroadcast_(0),
      broadcastInterval_(TELEMETRY_INTERVAL) {
}

void TelemetryManager::update() {
    if (millis() - lastBroadcast_ < broadcastInterval_) {
        return;
    }
    WebPortal::Telemetry telemetry = collectTelemetry();
    broadcastToWeb(telemetry);
    lastBroadcast_ = millis();
}

InputSource TelemetryManager::determineInputSource() {
    // Latched web-control override: auto-release if all browsers have disconnected
    // (Q11a — the link is considered lost when the client count drops to zero).
    if (vehicleController_.isWebControl() && webPortal_.getClientCount() == 0) {
        vehicleController_.setWebControl(false);
    }
    // While engaged (and a browser is present), web fully overrides MAVLink.
    if (vehicleController_.isWebControl() && webPortal_.getClientCount() > 0) {
        return InputSource::WEB;
    }
    if (mavlink_.isSignalValid()) {
        return InputSource::MAVLINK;
    }
    if (webPortal_.getClientCount() > 0) {
        return InputSource::WEB;
    }
    return InputSource::FAILSAFE;
}

WebPortal::Telemetry TelemetryManager::collectTelemetry() {
    WebPortal::Telemetry telemetry;

    telemetry.timestamp = millis();

    telemetry.gear = vehicleController_.getCurrentGearString();
    telemetry.steering_pct = (int)vehicleController_.getSteeringPercent();
    const SteeringController& steering = vehicleController_.getSteering();
    telemetry.steer_center = steering.getCenter();
    telemetry.steer_left = steering.getLeftLimit();
    telemetry.steer_right = steering.getRightLimit();
    telemetry.steer_position = steering.getRawAngle();
    telemetry.steer_sensor_ok = steering.isSensorOk();
    telemetry.steer_calibrated = steering.isCalibrated();
    telemetry.steer_motor_current = steering.getMotorCurrent();
    telemetry.steer_fet_temp = steering.getFetTemp();
    telemetry.steer_vesc_fault = steering.getVescFault();
    telemetry.steer_driver_ok = steering.isDriverOk();
    telemetry.throttle_us = vehicleController_.getThrottleUs();
    telemetry.throttle_idle_us = vehicleController_.getThrottleIdleUs();
    telemetry.throttle_full_us = vehicleController_.getThrottleFullUs();
    telemetry.throttle_calibrating = vehicleController_.isThrottleCalibrating();
    telemetry.input_source = getInputSourceName(vehicleController_.getInputSource());

    // Transmission servo position (0.0–100.0 %)
    telemetry.hall_position = vehicleController_.getTransmission().getCurrentServoPct();

    telemetry.brake_pct = 0.0f;
    telemetry.mav_active = mavlink_.isSignalValid();
    telemetry.web_control = vehicleController_.isWebControl();

    CANController::VehicleData vehicleData = vehicleController_.getVehicleData();
    if (vehicleData.dataValid) {
        telemetry.engine_rpm = vehicleData.engineRPM;
        telemetry.vehicle_speed = vehicleData.vehicleSpeed;
        telemetry.coolant_temp = vehicleData.coolantTemp;
        telemetry.oil_temp = vehicleData.oilTemp;
        telemetry.throttle_position = vehicleData.throttlePosition;
        telemetry.fuel_level = vehicleData.fuelLevel;
        telemetry.map_kpa = vehicleData.mapKpa;
        telemetry.can_status = "connected";
    } else {
        telemetry.engine_rpm = 0;
        telemetry.vehicle_speed = 0;
        telemetry.coolant_temp = 0;
        telemetry.oil_temp = 0;
        telemetry.throttle_position = 0;
        telemetry.fuel_level = 0;
        telemetry.map_kpa = 0;
        telemetry.can_status = "disconnected";
    }

    // ECU capability probe: embed results only while running or freshly completed.
    telemetry.probe = vehicleController_.getProbeResults();
    telemetry.probe_present = telemetry.probe.running ||
        (telemetry.probe.complete &&
         (millis() - telemetry.probe.completedAtMs) < PROBE_RESULT_TTL);

    mavlink_.getRawChannels(telemetry.mav_channels);
    MavlinkInterface::LinkQuality linkQuality = mavlink_.getLinkQuality();
    telemetry.mav_cmd_rate = linkQuality.commandRate;
    telemetry.mav_link_age = linkQuality.signalAge;
    telemetry.mav_heartbeat_age = linkQuality.heartbeatAge;

    telemetry.gear_switching = vehicleController_.getTransmission().needsThrottleBoost();

    RelayController::IgnitionState ignitionState = vehicleController_.getIgnitionState();
    telemetry.ignition_state = getRelayIgnitionStateName(ignitionState);
    telemetry.is_cranking = (ignitionState == RelayController::IgnitionState::CRANKING);
    telemetry.front_light_on = vehicleController_.getFrontLight();
    telemetry.wheel_lock_on = vehicleController_.getWheelLock();

    telemetry.firmware_version = FIRMWARE_VERSION;

    const TransmissionController& trans = vehicleController_.getTransmission();
    telemetry.gear_default_r = trans.getGearPosition(TransmissionController::Gear::GEAR_REVERSE);
    telemetry.gear_default_n = trans.getGearPosition(TransmissionController::Gear::GEAR_NEUTRAL);
    telemetry.gear_default_l = trans.getGearPosition(TransmissionController::Gear::GEAR_LOW);
    telemetry.gear_default_h = trans.getGearPosition(TransmissionController::Gear::GEAR_HIGH);

    telemetry.boost_target_rpm = vehicleController_.getBoostTargetRpm();

    return telemetry;
}

void TelemetryManager::broadcastToWeb(const WebPortal::Telemetry& telemetry) {
    webPortal_.broadcastTelemetry(telemetry);
}
