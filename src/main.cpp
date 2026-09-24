#include <Arduino.h>
#include "Constants.h"
#include "Debug.h"
#include "ServoController.h"
#include "ThrottleController.h"
#include "SteeringController.h"
#include "BTS7960Controller.h"
#include "VescMotorDriver.h"
#include "TransmissionController.h"
#include "WebPortal.h"
#include "VehicleController.h"
#include "TelemetryManager.h"
#include "MavlinkInterface.h"
#include "RelayController.h"
#include "SpeedSensor.h"
#include "nvs_flash.h"
#include <Wire.h>

// ============================================================================
// ACTUATOR INSTANCES
// ============================================================================

// Steering Actuator (Flipsky 75200 VESC over UART2 + AS5600 absolute angle sensor).
// The VESC driver MUST be constructed before the steering controller that
// references it (global init order is top-to-bottom within this file).
VescMotorDriver steeringVesc(VESC_UART_NUM);
SteeringController steeringActuator(steeringVesc);

// Throttle Servo
ThrottleController throttle;

// Transmission servo (PWM servo-based gear selector)
TransmissionController transmissionActuator;
// Brake actuator (BTS7960)
BTS7960Controller brakeActuator;

// MAVLink interface to Pixhawk (TELEM2)
MavlinkInterface mavlinkInterface;

// Relay Controller for ignition, starter, lights and wheel lock — direct GPIO
RelayController relayController;

// Driveline speed sensor (hall pickup, PCNT on PIN_SPEED_SENSOR)
SpeedSensor speedSensor;

// Vehicle Controller (coordinates all actuators and input sources)
VehicleController vehicleController(steeringActuator, throttle, transmissionActuator, brakeActuator,
                                     mavlinkInterface, relayController, speedSensor);

// Web Portal for telemetry and manual control
WebPortal webPortal;

// Telemetry Manager (collects and broadcasts telemetry data)
TelemetryManager telemetryManager(vehicleController, webPortal, mavlinkInterface);

// ============================================================================
// BRING-UP HELPERS
// ============================================================================

// Probe the 7-bit address range 0x08-0x77 on the bus and print a single summary
// line. Master-gated only (no feature flag) so a plain `debug on` shows it.
static void scanI2CBus(TwoWire& bus, const char* label) {
    String found;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        bus.beginTransmission(addr);
        if (bus.endTransmission() == 0) {
            char hex[8];
            snprintf(hex, sizeof(hex), " 0x%02X", addr);
            found += hex;
        }
    }
    Debug::println("[I2C] " + String(label) + " devices:" + (found.isEmpty() ? " none" : found));
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Console is UART0 (GPIO43 TX / GPIO44 RX) through the DevKit's on-board
    // USB-UART bridge, the "COM" port. The native USB CDC console is NOT usable
    // here: its D-/D+ pins (GPIO19/20) carry the R/N gear switches.
    // Never wait for a host — the vehicle must boot with no laptop attached.
    Serial.begin(SERIAL_BAUD_RATE);

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

    // Feature debug flags — MUST be set after Debug::begin(), which overwrites
    // the in-memory flags with the NVS-stored state. Uncomment one to trace it.
    // Debug::setFeatureEnabled(DebugFeature::CAN, true);           // MCP2515 health, unmatched RX, overflow
    // Debug::setFeatureEnabled(DebugFeature::RELAY, true);         // relay/ignition switching
    // Debug::setFeatureEnabled(DebugFeature::BRAKE, true);         // brake moves + endstop
    // Debug::setFeatureEnabled(DebugFeature::VEHICLE, true);       // speed sensor + vehicle state
    // Debug::setFeatureEnabled(DebugFeature::TRANSMISSION, true);
    // Debug::setFeatureEnabled(DebugFeature::MAVLINK, true);       // diagnose command-stream / fail-safe flapping
    Debug::setFeatureEnabled(DebugFeature::SERVO, false);

    // Ungated state dump so a disabled master switch is visible on the monitor
    Serial.printf("[INIT] Debug: master=%s SERVO=%s\n",
                  Debug::isEnabled() ? "ON" : "OFF",
                  Debug::isFeatureEnabled(DebugFeature::SERVO) ? "ON" : "OFF");

    Debug::println("\n=== ESP32-S3 Quad Bike Control (DevKitC-1) ===");

    // ── I2C bus ──────────────────────────────────────────────────────────────
    // main.cpp is the single owner of the bus: it is opened here, before any
    // consumer, so no driver depends on another driver's initialization succeeding.
    // The AS5600 steering angle sensor is its only device.
    if (!Wire.begin(PIN_STEER_SDA, PIN_STEER_SCL, I2C_BUS_FREQ_HZ)) {
        Debug::println("[INIT] ERROR: I2C (Wire) init failed");
    }

    // Enumerate the bus before any driver claims it, so a missing or mis-wired
    // sensor is obvious from the boot log alone.
    if (Debug::isEnabled()) {
        scanI2CBus(Wire, "I2C (Wire)");
    }

    // Initialize throttle servo (loads calibration from NVS, moves to calibrated idle)
    if (!throttle.begin(PIN_THROTTLE_PWM, LEDC_CH_THROTTLE)) {
        Debug::printlnFeature(DebugFeature::SERVO, "ERROR: Throttle servo failed");
    }

    // Initialize the steering VESC UART (structural success even if the VESC is
    // silent; the controller stays driver-down until the first valid reply).
    steeringVesc.begin(PIN_VESC_RX, PIN_VESC_TX, VESC_UART_BAUD);

    // Initialize steering actuator (VESC driver + AS5600 absolute angle sensor — no homing)
    if (steeringActuator.begin(PIN_STEER_SDA, PIN_STEER_SCL)) {
        steeringActuator.loadCalibration();   // NVS-backed center + left/right limits

        if (steeringActuator.isCalibrated() && steeringActuator.isSensorOk()) {
            Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Moving to center");
            steeringActuator.setSteeringPercent(0.0f);
        } else {
            Debug::printlnFeature(DebugFeature::SERVO,
                steeringActuator.isSensorOk() ? "[STEER] Not calibrated — holding position"
                                              : "[STEER] Sensor fault — holding position");
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

    // Initialize the driveline speed sensor (loads calibration from NVS, starts PCNT)
    if (!speedSensor.begin()) {
        Debug::println("[INIT] ERROR: Speed sensor PCNT init failed — speed stays invalid");
    }

    // Restore the engine hour meter from NVS (must run after NVS is up, so not in a ctor)
    vehicleController.initEngineHourMeter();

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
    // Sample the hall speed counter (rate-limited internally) before the control
    // logic and telemetry read the speed this iteration.
    speedSensor.update();

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
    // Named local, NOT a temporary: StateReport stores a const char* into this String, so it
    // must outlive the report() call below (same lifetime pattern as the two gear strings above).
    String gearPhysStr = vehicleController.getCurrentGearString();  // physically sensed ("?" = unknown)
    MavlinkInterface::StateReport report;
    report.canValid     = vd.dataValid;
    report.engineRpm    = vd.engineRPM;
    report.coolantTemp  = vd.coolantTemp;
    report.gearFrom     = gearFromStr.c_str();
    report.gearTo       = gearToStr.c_str();
    report.gearMoving   = vehicleController.getTransmission().isGearChangeActive();
    report.ignition     = getRelayIgnitionStateName(vehicleController.getIgnitionState());
    report.failsafe     = (vehicleController.getInputSource() == InputSource::FAILSAFE);
    report.digitalFlags = (vehicleController.getWheelLock()  ? EFI_DIGITAL_FLAG_WHEEL_LOCK  : 0)
                        | (vehicleController.getFrontLight() ? EFI_DIGITAL_FLAG_FRONT_LIGHT : 0);
    report.speedValid   = vehicleController.isVehicleSpeedValid();
    report.speedMs      = vehicleController.getVehicleSpeedMs();
    report.intakeTemp   = vd.intakeTemp;
    report.moduleVoltageMv = vd.moduleVoltageMv;
    report.throttlePosition = vd.throttlePosition;              // measured (ECU)
    report.throttleCmdPct   = vehicleController.getThrottlePercent();  // commanded (arbitrated)
    report.gearPhysical     = gearPhysStr.c_str();               // measured (gear switches)
    report.mapKpa           = vd.mapKpa;
    report.engineLoad       = vd.engineLoad;
    report.travelDirection  = vehicleController.getTravelDirection();  // PHYSICAL gear sign
    report.odoKm            = vehicleController.getOdoKm();       // total, never resettable
    report.tripKm           = vehicleController.getTripKm();      // resettable from the GCS
    report.engineHours      = vehicleController.getEngineHours(); // total, no reset on any interface
    report.engineTripHours  = vehicleController.getEngineTripHours(); // zeroed by the TRIP reset
    // Steering VESC telemetry (STEER_A / VESC_V / VESC_TEMP / VESC_OK named floats).
    // Gate: the DRIVER's own link health.
    const SteeringController& steering = vehicleController.getSteering();
    report.steerDriverOk      = steering.isDriverOk();
    report.steerMotorCurrentA = steering.getMotorCurrent();        // MOTOR current, not input
    report.steerFetTempC      = steering.getFetTemp();
    report.steerInputVoltageV = steering.getInputVoltage();        // supply voltage at the VESC
    // Steering POSITION from the same controller but a DIFFERENT gate: the AS5600's, not the
    // VESC's. Filled unconditionally — the gating is applied where the message is packed, so
    // this stays a plain snapshot.
    report.steerPercent     = steering.getSteeringPercent();       // -100 left .. 0 .. +100 right
    report.steerSensorOk    = steering.isSensorOk();
    report.steerCalibrated  = steering.isCalibrated();
    mavlinkInterface.report(report);

    // Broadcast telemetry to web clients
    telemetryManager.update();

}