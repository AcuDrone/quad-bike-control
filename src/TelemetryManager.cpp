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

void TelemetryManager::forceBroadcast() {
    WebPortal::Telemetry telemetry = collectTelemetry();
    broadcastToWeb(telemetry);
    lastBroadcast_ = millis();
}

InputSource TelemetryManager::determineInputSource() {
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
    telemetry.throttle_angle = (int)vehicleController_.getThrottleAngle();
    telemetry.input_source = getInputSourceName(vehicleController_.getInputSource());

    // Transmission servo position (0.0–100.0 %)
    telemetry.hall_position = vehicleController_.getTransmission().getCurrentServoPct();

    telemetry.brake_pct = 0.0f;
    telemetry.mav_active = mavlink_.isSignalValid();

    CANController::VehicleData vehicleData = vehicleController_.getVehicleData();
    if (vehicleData.dataValid) {
        telemetry.engine_rpm = vehicleData.engineRPM;
        telemetry.vehicle_speed = vehicleData.vehicleSpeed;
        telemetry.coolant_temp = vehicleData.coolantTemp;
        telemetry.oil_temp = vehicleData.oilTemp;
        telemetry.throttle_position = vehicleData.throttlePosition;
        telemetry.fuel_level = vehicleData.fuelLevel;
        telemetry.can_status = "connected";
        telemetry.can_data_age = millis() - vehicleData.lastUpdateTime;
    } else {
        telemetry.engine_rpm = 0;
        telemetry.vehicle_speed = 0;
        telemetry.coolant_temp = 0;
        telemetry.oil_temp = 0;
        telemetry.throttle_position = 0;
        telemetry.fuel_level = 0;
        telemetry.can_status = "disconnected";
        telemetry.can_data_age = (vehicleData.lastUpdateTime == 0) ? 0 : (millis() - vehicleData.lastUpdateTime);
    }

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
