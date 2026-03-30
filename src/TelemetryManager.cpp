#include "TelemetryManager.h"

TelemetryManager::TelemetryManager(VehicleController& vehicleController,
                                   EncoderCounter& transmissionEncoder,
                                   WebPortal& webPortal,
                                   SBusInput& sbusInput)
    : vehicleController_(vehicleController),
      transmissionEncoder_(transmissionEncoder),
      webPortal_(webPortal),
      sbusInput_(sbusInput),
      lastBroadcast_(0),
      broadcastInterval_(TELEMETRY_INTERVAL) {
}

void TelemetryManager::update() {
    // Rate limit broadcasts
    if (millis() - lastBroadcast_ < broadcastInterval_) {
        return;
    }

    // Collect and broadcast telemetry
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
    // Priority 1: SBUS (if signal valid)
    if (sbusInput_.isSignalValid()) {
        return InputSource::SBUS;
    }

    // Priority 2: WEB (if clients connected)
    if (webPortal_.getClientCount() > 0) {
        return InputSource::WEB;
    }

    // Priority 3: FAILSAFE
    return InputSource::FAILSAFE;
}

WebPortal::Telemetry TelemetryManager::collectTelemetry() {
    WebPortal::Telemetry telemetry;

    // Timestamp
    telemetry.timestamp = millis();

    // Vehicle state from VehicleController
    telemetry.gear = vehicleController_.getCurrentGearString();
    telemetry.steering_angle = (int)vehicleController_.getSteeringAngle();
    telemetry.throttle_angle = (int)vehicleController_.getThrottleAngle();
    telemetry.input_source = getInputSourceName(vehicleController_.getInputSource());

    // Transmission encoder position
    telemetry.hall_position = transmissionEncoder_.getPosition();

    // Brake status (TODO: implement brake position tracking)
    telemetry.brake_pct = 0.0f;

    // S-bus status
    telemetry.sbus_active = sbusInput_.isSignalValid();

    // CAN bus vehicle data
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
        // CAN data not valid - omit or set defaults
        telemetry.engine_rpm = 0;
        telemetry.vehicle_speed = 0;
        telemetry.coolant_temp = 0;
        telemetry.oil_temp = 0;
        telemetry.throttle_position = 0;
        telemetry.fuel_level = 0;
        telemetry.can_status = "disconnected";
        telemetry.can_data_age = (vehicleData.lastUpdateTime == 0) ? 0 : (millis() - vehicleData.lastUpdateTime);
    }

    // SBUS channel data
    sbusInput_.getRawChannels(telemetry.sbus_channels);
    SBusInput::SignalQuality sbusQuality = sbusInput_.getSignalQuality();
    telemetry.sbus_frame_rate = sbusQuality.frameRate;
    telemetry.sbus_error_rate = sbusQuality.errorRate;
    telemetry.sbus_signal_age = sbusQuality.signalAge;

    // Gear switching state
    telemetry.gear_switching = vehicleController_.getTransmission().isPositionControlActive();

    // Ignition and lighting state
    RelayController::IgnitionState ignitionState = vehicleController_.getIgnitionState();
    telemetry.ignition_state = getRelayIgnitionStateName(ignitionState);
    telemetry.is_cranking = (ignitionState == RelayController::IgnitionState::CRANKING);
    telemetry.front_light_on = vehicleController_.getFrontLight();

    // Firmware version
    telemetry.firmware_version = FIRMWARE_VERSION;

    return telemetry;
}

void TelemetryManager::broadcastToWeb(const WebPortal::Telemetry& telemetry) {
    webPortal_.broadcastTelemetry(telemetry);
}