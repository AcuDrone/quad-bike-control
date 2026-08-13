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
#include "BoardInputs.h"
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

// ── I2C TOPOLOGY: BENCH REALITY (established by live bus scans) ──────────────
// The schematic's port names suggested the opto-input expander U10 sat on I2C1;
// live scans prove otherwise. Verified map (all three devices bench-tested):
//   I2C2 (Wire1, GPIO47/48) : BOTH on-board PCA9557s — input U10 @0x1A and
//                             CD4051 mux U2 @0x1C — plus the external relay
//                             board @0x1F on header X15.
//   I2C1 (Wire,  GPIO1/2)   : external headers only — X14, reserved for the
//                             AS5600 @0x36 as designed.
// The external relay board ships strapped A2A1A0=111 → 0x1F; 0x1F collides with
// neither 0x1A nor 0x1C, so the as-shipped strap is kept.
//
// Historical note for a future board copy: X15's signal path was dead on the
// first assembled board (0x1F never appeared in an I2C2 scan, two attempts),
// traced to the FB3/FB4 ferrite area and fixed by rework. Check that area first
// if X15 is silent. Contingency if it ever is: the relay board also works on
// X14 → I2C1 (change the RelayController argument below from Wire1 to Wire,
// borrowing X14 from the AS5600).
// Opto-isolated board inputs (gear switches In1-In4, brake endstop In5) on I2C2
BoardInputs boardInputs(Wire1);

// Relay Controller for ignition, starter, lights and wheel lock.
// On I2C2 (Wire1) @0x1F via X15 — its intended home; see the note above.
RelayController relayController(Wire1);

// Driveline speed sensor (12V hall via the X2 opto input, PCNT on GPIO8)
SpeedSensor speedSensor;

// Vehicle Controller (coordinates all actuators and input sources)
VehicleController vehicleController(steeringActuator, throttle, transmissionActuator, brakeActuator,
                                     mavlinkInterface, relayController, boardInputs, speedSensor);

// Web Portal for telemetry and manual control
WebPortal webPortal;

// Telemetry Manager (collects and broadcasts telemetry data)
TelemetryManager telemetryManager(vehicleController, webPortal, mavlinkInterface);

// ============================================================================
// BRING-UP HELPERS
// ============================================================================

// Probe the 7-bit address range 0x08-0x77 on one bus and print a single summary
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
    // 24V boost rail FIRST: it feeds the steering VESC, the Jetson, Starlink and
    // the cameras, so it must be up before any subsystem that depends on it.
    // GPIO46 has a 100K pulldown (R7) so the rail is off through reset.
    pinMode(PIN_BOOST_EN, OUTPUT);
    digitalWrite(PIN_BOOST_EN, HIGH);

    // Console is USB-CDC (ARDUINO_USB_CDC_ON_BOOT=1). Never wait for a host —
    // the vehicle must boot with no laptop attached; early output may be lost.
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("[INIT] 24V boost rail enabled (GPIO46)");

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
    // the in-memory flags with the NVS-stored state
    // Debug::setFeatureEnabled(DebugFeature::CAN, true);
    // Debug::setFeatureEnabled(DebugFeature::TRANSMISSION, true);
    // Debug::setFeatureEnabled(DebugFeature::VEHICLE, true);
    // Debug::setFeatureEnabled(DebugFeature::MAVLINK, true);  // diagnose command-stream / fail-safe flapping
    Debug::setFeatureEnabled(DebugFeature::SERVO, false);
    // On for board bring-up: relay/ignition switching and brake endstop tracing.
    // TURN BOTH OFF BEFORE VEHICLE DEPLOYMENT — they are bring-up aids only and
    // chatter on every relay write and brake move.
    Debug::setFeatureEnabled(DebugFeature::RELAY, true);
    Debug::setFeatureEnabled(DebugFeature::BRAKE, true);

    // Ungated state dump so a disabled master switch is visible on the monitor
    Serial.printf("[INIT] Debug: master=%s SERVO=%s RELAY=%s BRAKE=%s\n",
                  Debug::isEnabled() ? "ON" : "OFF",
                  Debug::isFeatureEnabled(DebugFeature::SERVO) ? "ON" : "OFF",
                  Debug::isFeatureEnabled(DebugFeature::RELAY) ? "ON" : "OFF",
                  Debug::isFeatureEnabled(DebugFeature::BRAKE) ? "ON" : "OFF");

    Debug::println("\n=== ESP32-S3 Quad Bike Control (Control_v0) ===");

    // Record the boost enable and start rail monitoring (grace window + ADC setup)
    vehicleController.initBoostRail();

    // ── I2C buses ────────────────────────────────────────────────────────────
    // main.cpp is the single owner of both buses: they are opened here, before any
    // consumer, so no driver depends on another driver's initialization succeeding.
    //   I2C1 (Wire)  GPIO1/2   : external headers — AS5600 0x36 (X14)
    //   I2C2 (Wire1) GPIO48/47 : on-board expanders — opto inputs U10 0x1A
    //                            and CD4051 mux U2 0x1C (never addressed)
    //                            + relay-board PCA9557 0x1F on X15
    if (!Wire.begin(PIN_STEER_SDA, PIN_STEER_SCL, I2C_BUS_FREQ_HZ)) {
        Debug::println("[INIT] ERROR: I2C1 (Wire) init failed");
    }
    if (!Wire1.begin(PIN_RELAY_SDA, PIN_RELAY_SCL, I2C_BUS_FREQ_HZ)) {
        Debug::println("[INIT] ERROR: I2C2 (Wire1) init failed");
    }

    // Bring-up aid: enumerate both buses before any driver claims them, so a
    // missing/mis-strapped expander is obvious from the boot log alone.
    if (Debug::isEnabled()) {
        scanI2CBus(Wire,  "I2C1 (Wire)");
        scanI2CBus(Wire1, "I2C2 (Wire1)");
    }

    // Opto-isolated inputs (gear switches + brake limit sensor)
    if (!boardInputs.begin()) {
        Debug::println("[INIT] ERROR: Board input expander failed — gear will report UNKNOWN");
    }
    {
        BoardInputs::Snapshot snap = boardInputs.getSnapshot();
        Debug::printfFeature(DebugFeature::BRAKE, "Brake endstop: %s\n",
            !snap.valid ? "UNKNOWN (input expander unavailable)"
                        : (snap.brakeReleased ? "Released (endstop reached)" : "Not released"));
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
        transmissionActuator.initGearSensors(boardInputs);
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
    // Poll the opto-input expander (rate-limited internally) before any consumer
    // reads the snapshot this iteration.
    boardInputs.update();

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
    report.speedKmh     = vehicleController.getVehicleSpeedKmh();
    mavlinkInterface.report(report);

    // Broadcast telemetry to web clients
    telemetryManager.update();

}