#include <Arduino.h>
#include "Constants.h"
#include "Debug.h"
#include "ServoController.h"
#include "ThrottleController.h"
#include "SteeringController.h"
#include "BTS7960Controller.h"
#include "TransmissionController.h"
#include "EncoderCounter.h"
#include "WebPortal.h"
#include "VehicleController.h"
#include "TelemetryManager.h"
#include "MavlinkInterface.h"
#include "RelayController.h"
#include "nvs_flash.h"

// ============================================================================
// ACTUATOR INSTANCES
// ============================================================================

// Steering Actuator (BTS7960 full speed + encoder)
SteeringController steeringActuator;
EncoderCounter steeringEncoder;

// Throttle Servo
ThrottleController throttle;

// Transmission servo (PWM servo-based gear selector)
TransmissionController transmissionActuator;
// Brake actuator (BTS7960)
BTS7960Controller brakeActuator;

// MAVLink interface to Pixhawk (TELEM2)
MavlinkInterface mavlinkInterface;

// Relay Controller for ignition and lights
RelayController relayController;

// Vehicle Controller (coordinates all actuators and input sources)
VehicleController vehicleController(steeringActuator, throttle, transmissionActuator, brakeActuator,
                                     mavlinkInterface, relayController);

// Web Portal for telemetry and manual control
WebPortal webPortal;

// Telemetry Manager (collects and broadcasts telemetry data)
TelemetryManager telemetryManager(vehicleController, webPortal, mavlinkInterface);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial for debugging
    Serial.begin(SERIAL_BAUD_RATE);

    Debug::setFeatureEnabled(DebugFeature::CAN, true);
    // Debug::setFeatureEnabled(DebugFeature::TRANSMISSION, true);
    // Debug::setFeatureEnabled(DebugFeature::VEHICLE, true);
    // Debug::setFeatureEnabled(DebugFeature::MAVLINK, true);  // diagnose command-stream / fail-safe flapping
    // Debug::setFeatureEnabled(DebugFeature::SERVO, true);
    

    // Initialize NVS first (required by Debug utility)
    Serial.println("[INIT] Initializing NVS...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("[INIT] NVS needs erase, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err == ESP_OK) {
        Serial.println("[INIT] NVS initialized successfully");
    } else {
        Serial.printf("[INIT] ERROR: NVS init failed with error: %d\n", err);
    }
    ESP_ERROR_CHECK(err);

    // Initialize debug utility (load state from NVS)
    Serial.println("[INIT] Initializing Debug utility...");
    Debug::begin();
    Serial.println("[INIT] Debug utility initialized");

    Debug::println("\n=== ESP32-C6 Quad Bike Control ===");

    // Initialize throttle servo (loads calibration from NVS, moves to calibrated idle)
    if (!throttle.begin(PIN_THROTTLE_PWM, LEDC_CH_THROTTLE)) {
        Debug::printlnFeature(DebugFeature::SERVO, "ERROR: Throttle servo failed");
    }

    // Initialize steering actuator (BTS7960 + encoder)
    if (!steeringEncoder.begin(PIN_STEER_ENCODER_A, PIN_STEER_ENCODER_B, PCNT_UNIT_STEER)) {
        Debug::printlnFeature(DebugFeature::SERVO, "ERROR: Steering encoder failed");
    }

    if (steeringActuator.begin(PIN_STEER_RPWM, PIN_STEER_LPWM)) {
        steeringActuator.attachEncoder(&steeringEncoder);
        steeringActuator.loadCenter();   // NVS-backed center calibration

        // Auto-home to left limit, then move to center
        if (steeringActuator.autoHome()) {
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Homed to left limit");
            Debug::printfFeature(DebugFeature::SERVO, "[STEER] Moving to center (%ld)\n", (long)steeringActuator.getCenter());
            steeringActuator.setPosition(steeringActuator.getCenter());
        } else {
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] ERROR: Auto-home failed");
        }
    } else {
        Debug::printlnFeature(DebugFeature::SERVO, "ERROR: Steering actuator failed");
    }

    // Initialize transmission servo
    if (transmissionActuator.begin()) {
        transmissionActuator.initGearSensors();
        transmissionActuator.loadDefaultPositions();
        transmissionActuator.loadGearOvershoots();
    } else {
        Debug::printlnFeature(DebugFeature::TRANSMISSION, "ERROR: Transmission servo failed");
    }

    // Initialize brake
    if (!brakeActuator.begin(PIN_BRAKE_RPWM, PIN_BRAKE_LPWM,
                             LEDC_CH_BRAKE_RPWM, LEDC_CH_BRAKE_LPWM)) {
        Debug::printlnFeature(DebugFeature::BRAKE, "ERROR: Brake actuator failed");
    }

    brakeActuator.stop();

    pinMode(PIN_BRAKE_SENSOR, INPUT);
    Debug::printfFeature(DebugFeature::BRAKE, "Brake sensor: %s\n",
        digitalRead(PIN_BRAKE_SENSOR) ? "Released (HIGH)" : "Pressed (LOW)");
 
    // Initialize MAVLink interface (Pixhawk TELEM2)
    if (!mavlinkInterface.begin()) {
        Debug::printlnFeature(DebugFeature::MAVLINK, "ERROR: MAVLink interface failed");
    }

    if (!relayController.begin()) {
        Debug::printlnFeature(DebugFeature::RELAY, "ERROR: Relay controller failed");
    }

    // Initialize CAN controller
    if (!vehicleController.initCAN()) {
        Debug::printlnFeature(DebugFeature::CAN, "WARNING: CAN controller failed (will continue without vehicle data)");
    }

    // Initialize web portal
    if (!webPortal.begin()) {
        Debug::printlnFeature(DebugFeature::WEB, "ERROR: Web portal failed");
    }

    Debug::println("\n✓ Ready");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Update MAVLink interface (parse inbound, request command stream)
    mavlinkInterface.update();

    // Update web portal (handles OTA, WebSocket cleanup, etc.)
    webPortal.update();

    // Determine current input source (MAVLINK > WEB > FAILSAFE)
    InputSource currentSource = telemetryManager.determineInputSource();
    vehicleController.setInputSource(currentSource);

    // Process web commands if web control is active
    WebPortal::WebCommand cmd = webPortal.getCommand();
    if (cmd.hasCommand) {
        vehicleController.processWebCommand(cmd, webPortal);
        webPortal.clearCommand();
    }

    // Update vehicle controller (failsafe, actuators, etc.)
    vehicleController.update();

    // Report vehicle state back to the MAVLink network (rate-limited internally)
    CANController::VehicleData vd = vehicleController.getVehicleData();
    String gearToStr   = vehicleController.getTargetGearString();  // current step / assumed gear
    String gearFromStr = vehicleController.getFromGearString();    // gear the step is leaving
    MavlinkInterface::StateReport report;
    report.canValid     = vd.dataValid;
    report.engineRpm    = vd.engineRPM;
    report.coolantTemp  = vd.coolantTemp;
    report.throttlePct  = vd.throttlePosition;
    report.gearFrom     = gearFromStr.c_str();
    report.gearTo       = gearToStr.c_str();
    report.gearMoving   = vehicleController.getTransmission().isGearChangeActive();
    report.ignition     = getRelayIgnitionStateName(vehicleController.getIgnitionState());
    report.failsafe     = (vehicleController.getInputSource() == InputSource::FAILSAFE);
    report.digitalFlags = (vehicleController.getWheelLock()  ? EFI_DIGITAL_FLAG_WHEEL_LOCK  : 0)
                        | (vehicleController.getFrontLight() ? EFI_DIGITAL_FLAG_FRONT_LIGHT : 0);
    mavlinkInterface.report(report);

    // Broadcast telemetry to web clients
    telemetryManager.update();

}