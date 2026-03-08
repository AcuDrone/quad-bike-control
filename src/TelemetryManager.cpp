#include "TelemetryManager.h"

TelemetryManager::TelemetryManager(VehicleController& vehicleController,
                                   EncoderCounter& transmissionEncoder,
                                   WebPortal& webPortal)
    : vehicleController_(vehicleController),
      transmissionEncoder_(transmissionEncoder),
      webPortal_(webPortal),
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
    // S-bus has highest priority (if we had S-bus implementation)
    // For now, assume S-bus is not implemented, so check web control

    // If web clients are connected, allow web control
    // This keeps web control active even between commands
    if (webPortal_.getClientCount() > 0) {
        return InputSource::WEB;
    }

    // Default to failsafe
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

    // S-bus status (TODO: implement when S-bus is added)
    telemetry.sbus_active = false;

    return telemetry;
}

void TelemetryManager::broadcastToWeb(const WebPortal::Telemetry& telemetry) {
    webPortal_.broadcastTelemetry(telemetry);
}