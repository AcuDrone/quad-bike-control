#include <Arduino.h>
#include "Constants.h"
#include "Debug.h"
#include "ServoController.h"
#include "BTS7960Controller.h"
#include "TransmissionController.h"
#include "EncoderCounter.h"
#include "WebPortal.h"
#include "VehicleController.h"
#include "TelemetryManager.h"
#include "SBusInput.h"
#include "RelayController.h"
#include "nvs_flash.h"

// ============================================================================
// ACTUATOR INSTANCES
// ============================================================================

// Servo Controllers
ServoController steeringServo;
ServoController throttleServo;

// Linear Actuator Controllers (BTS7960)
TransmissionController transmissionActuator;  // Gear selector (R/N/L/H) with position control
BTS7960Controller brakeActuator;              // Brake control

// Encoder for Transmission Position Feedback
EncoderCounter transmissionEncoder;

// SBUS Input from ArduPilot Rover
SBusInput sbusInput;

// Relay Controller for ignition and lights
RelayController relayController;

// Vehicle Controller (coordinates all actuators and input sources)
VehicleController vehicleController(steeringServo, throttleServo, transmissionActuator, brakeActuator,
                                     sbusInput, relayController);

// Web Portal for telemetry and manual control
WebPortal webPortal;

// Telemetry Manager (collects and broadcasts telemetry data)
TelemetryManager telemetryManager(vehicleController, transmissionEncoder, webPortal, sbusInput);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial for debugging
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);

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

    // Initialize servos
    if (!steeringServo.begin(PIN_STEERING_PWM, LEDC_CH_STEERING, STEERING_SERVO_MIN_US, STEERING_SERVO_MAX_US)) {
        Debug::println("ERROR: Steering servo failed");
    }
    steeringServo.setAngle(STEERING_CENTER_ANGLE);

    if (!throttleServo.begin(PIN_THROTTLE_PWM, LEDC_CH_THROTTLE, THROTTLE_SERVO_MIN_US, THROTTLE_SERVO_MAX_US)) {
        Debug::println("ERROR: Throttle servo failed");
    }
    throttleServo.setAngle(THROTTLE_IDLE_ANGLE);

    // Initialize transmission system
    if (!transmissionEncoder.begin(PIN_TRANS_ENCODER_A, PIN_TRANS_ENCODER_B, PCNT_UNIT_TRANS)) {
        Debug::println("ERROR: Transmission encoder failed");
    }

    if (transmissionActuator.begin(PIN_TRANS_RPWM, PIN_TRANS_LPWM,
                                    LEDC_CH_TRANS_RPWM, LEDC_CH_TRANS_LPWM)) {
        transmissionActuator.attachEncoder(&transmissionEncoder);
        transmissionActuator.stop();

        // Initialize gear position sensors (GPIO switches)
        // GPIO 15 (strapping pin) and GPIO 18 (safe) configured with ESP-IDF GPIO API
        transmissionActuator.initGearSensors();

        // Load saved calibration from NVS (now that NVS is initialized)
        if (transmissionActuator.loadCalibration()) {
            Debug::println("[TRANS] Loaded saved calibration from storage");
        } else {
            Debug::println("[TRANS] No saved calibration found, using default positions");
        }

        // TEMPORARY: Skip calibration/homing when testing without hardware
        // Comment out this block and uncomment the full calibration below when hardware is connected
        Debug::println("[TRANS] Skipping calibration/homing (no hardware connected)");
        Debug::println("[TRANS] WARNING: Gear changes will fail without calibration");
        Debug::println("[TRANS] To calibrate: use web portal 'Calibrate' button when hardware connected");

        /* FULL CALIBRATION - Uncomment when hardware is connected:
        // Check if calibration already exists
        if (transmissionActuator.isCalibrated()) {
            // Calibration loaded from storage - just home to HIGH position
            Debug::println("[TRANS] Using saved calibration, homing to HIGH gear...");
            if (transmissionActuator.autoHome(1, TRANS_HOMING_SPEED, TRANS_HOMING_TIMEOUT)) {
                Debug::printf("[TRANS] Homed to HIGH gear at position %ld\n", transmissionEncoder.getPosition());
            } else {
                Debug::println("[TRANS] ERROR: Homing failed");
            }
        } else {
            // No saved calibration - run full calibration routine
            Debug::println("[TRANS] No saved calibration found, running full calibration...");
            if (transmissionActuator.calibrateAllGearPositions(TRANS_CALIBRATION_SPEED, TRANS_HOMING_TIMEOUT)) {
                Debug::println("[TRANS] Calibration successful!");
            } else {
                Debug::println("[TRANS] ERROR: Calibration failed");
                // Fallback to manual homing
                Debug::println("[TRANS] Attempting manual homing...");
                if (transmissionActuator.autoHome(1, TRANS_HOMING_SPEED, TRANS_HOMING_TIMEOUT)) {
                    Debug::printf("[TRANS] Position: %ld\n", transmissionEncoder.getPosition());
                } else {
                    Debug::println("[TRANS] ERROR: Homing failed");
                }
            }
        }
        */

        // To force recalibration: transmissionActuator.clearCalibration() and reboot
    } else {
        Debug::println("ERROR: Transmission actuator failed");
    }

    // Initialize brake
    if (!brakeActuator.begin(PIN_BRAKE_RPWM, PIN_BRAKE_LPWM,
                             LEDC_CH_BRAKE_RPWM, LEDC_CH_BRAKE_LPWM)) {
        Debug::println("ERROR: Brake actuator failed");
    }
    brakeActuator.stop();

    // Initialize brake sensor (HIGH = no pressure, LOW = pressure detected)
    pinMode(PIN_BRAKE_SENSOR, INPUT);
    Debug::printf("Brake sensor: %s\n", digitalRead(PIN_BRAKE_SENSOR) ? "Released (HIGH)" : "Pressed (LOW)");

    // Initialize SBUS input
    if (!sbusInput.begin()) {
        Debug::println("ERROR: SBUS input failed");
    }

    // Initialize relay controller
    if (!relayController.begin()) {
        Debug::println("ERROR: Relay controller failed");
    }

    // Initialize CAN controller
    if (!vehicleController.initCAN()) {
        Debug::println("WARNING: CAN controller failed (will continue without vehicle data)");
    }

    // Initialize web portal
    if (!webPortal.begin()) {
        Debug::println("ERROR: Web portal failed");
    }

    Debug::println("\n✓ Ready");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Update SBUS receiver
    sbusInput.update();

    // Update web portal (handles OTA, WebSocket cleanup, etc.)
    webPortal.update();

    // Determine current input source (SBUS > WEB > FAILSAFE)
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

    // Broadcast telemetry to web clients
    telemetryManager.update();

    // Small delay to prevent CPU hogging
    delay(10);
}