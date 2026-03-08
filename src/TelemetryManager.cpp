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

    return telemetry;
}

void TelemetryManager::broadcastToWeb(const WebPortal::Telemetry& telemetry) {
    webPortal_.broadcastTelemetry(telemetry);
}