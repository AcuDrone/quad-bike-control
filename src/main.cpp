#include <Arduino.h>
#include "Constants.h"
#include "ServoController.h"
#include "BTS7960Controller.h"
#include "TransmissionController.h"
#include "EncoderCounter.h"
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

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial for debugging
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);
    Serial.println("\n\n=== ESP32-C6 Quad Bike Control System ===");
    Serial.println("Hardware Actuator Test Mode with Encoder Feedback\n");

    // Initialize NVS (Non-Volatile Storage)
    Serial.println("Initializing NVS...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated, erase and retry
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    Serial.println("  ✓ NVS initialized");

    // Initialize Steering Servo
    Serial.println("Initializing steering servo...");
    if (steeringServo.begin(PIN_STEERING_PWM, LEDC_CH_STEERING, SERVO_MIN_US, SERVO_MAX_US)) {
        Serial.println("  ✓ Steering servo initialized");
        steeringServo.setAngle(STEERING_CENTER_ANGLE);  // Center steering
    } else {
        Serial.println("  ✗ Steering servo FAILED");
    }

    // Initialize Throttle Servo
    Serial.println("Initializing throttle servo...");
    if (throttleServo.begin(PIN_THROTTLE_PWM, LEDC_CH_THROTTLE, SERVO_MIN_US, SERVO_MAX_US)) {
        Serial.println("  ✓ Throttle servo initialized");
        throttleServo.setAngle(THROTTLE_IDLE_ANGLE);  // Idle position
    } else {
        Serial.println("  ✗ Throttle servo FAILED");
    }

    // Initialize Transmission Encoder
    Serial.println("Initializing transmission encoder (hall sensor)...");
    if (transmissionEncoder.begin(PIN_TRANS_ENCODER_A, PIN_TRANS_ENCODER_B, PCNT_UNIT_TRANS)) {
        Serial.println("  ✓ Transmission encoder initialized");

        // Try to load saved position from NVS
        if (transmissionEncoder.loadFromNVS(NVS_KEY_ENCODER_POS)) {
            Serial.printf("  ✓ Loaded encoder position from NVS: %ld\n", transmissionEncoder.getPosition());
        } else {
            Serial.println("  ⚠ No saved position in NVS (first boot or reset)");
        }
    } else {
        Serial.println("  ✗ Transmission encoder FAILED");
    }

    // Initialize Transmission Actuator (BTS7960)
    Serial.println("Initializing transmission actuator (gear selector)...");
    if (transmissionActuator.begin(PIN_TRANS_RPWM, PIN_TRANS_LPWM,
                                    LEDC_CH_TRANS_RPWM, LEDC_CH_TRANS_LPWM)) {
        Serial.println("  ✓ Transmission actuator initialized");

        // Attach encoder for position feedback
        transmissionActuator.attachEncoder(&transmissionEncoder);
        Serial.println("  ✓ Encoder attached to transmission actuator");

        transmissionActuator.stop();  // Ensure stopped

        // Auto-home the transmission actuator
        Serial.println("\n--- Auto-Homing Transmission ---");
        Serial.println("The actuator will move to the limit to establish zero position.");
        Serial.println("Press 'h' within 5 seconds to skip auto-homing...");

        // Wait 5 seconds for user to skip homing
        uint32_t waitStart = millis();

        Serial.println("Starting auto-homing...");
        if (transmissionActuator.autoHome(1, TRANS_HOMING_SPEED, TRANS_HOMING_TIMEOUT)) {
            Serial.println("  ✓ Auto-homing complete! Zero position set.");
            transmissionEncoder.saveToNVS(NVS_KEY_ENCODER_POS);
        } else {
            Serial.println("  ✗ Auto-homing failed or timed out");
        }

        Serial.printf("Current transmission position: %ld counts\n", transmissionEncoder.getPosition());
    } else {
        Serial.println("  ✗ Transmission actuator FAILED");
    }

    // Initialize Brake Actuator (BTS7960)
    Serial.println("Initializing brake actuator...");
    if (brakeActuator.begin(PIN_BRAKE_RPWM, PIN_BRAKE_LPWM,
                            LEDC_CH_BRAKE_RPWM, LEDC_CH_BRAKE_LPWM)) {
        Serial.println("  ✓ Brake actuator initialized");
        brakeActuator.stop();  // Ensure stopped
    } else {
        Serial.println("  ✗ Brake actuator FAILED");
    }

    Serial.println("\n=== Initialization Complete ===");
    Serial.println("\nSerial Commands:");
    Serial.println("  s<angle>  - Set steering servo (0-180°), e.g., s90");
    Serial.println("  t<angle>  - Set throttle servo (0-180°), e.g., t45");
    Serial.println("  g<speed>  - Transmission actuator (-255 to +255), e.g., g100");
    Serial.println("                 +speed = extend (shift up), -speed = retract (shift down)");
    Serial.println("  b<speed>  - Brake actuator (-255 to +255), e.g., b150");
    Serial.println("                 +speed = apply brake, -speed = release brake");
    Serial.println("\nGear Selection Commands:");
    Serial.println("  R         - Set transmission to REVERSE gear");
    Serial.println("  N         - Set transmission to NEUTRAL gear");
    Serial.println("  L         - Set transmission to LOW gear");
    Serial.println("  H         - Set transmission to HIGH gear");
    Serial.println("  G         - Display current gear");
    Serial.println("\nEncoder & Position Commands:");
    Serial.println("  p         - Display current transmission encoder position");
    Serial.println("  m<pos>    - Move transmission to target position, e.g., m100");
    Serial.println("  h         - Manual home transmission (move to limit, set zero)");
    Serial.println("  z         - Set current position as zero");
    Serial.println("  v         - Save current encoder position to NVS");
    Serial.println("  l         - Load encoder position from NVS");
    Serial.println("\nDiagnostic Commands:");
    Serial.println("  d         - Display encoder diagnostics (raw count, GPIO states)");
    Serial.println("  w         - Watch encoder (continuously display for 5 seconds)");
    Serial.println("\nSafety Commands:");
    Serial.println("  x         - Emergency stop (coast all actuators)");
    Serial.println("  c         - Safe position (center, idle, stop actuators)");
    Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Check for serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
            case 's': {  // Steering
                if (Serial.available() > 0) {
                    int angle = Serial.parseInt();
                    Serial.printf("Setting steering to %d degrees\n", angle);
                    steeringServo.setAngle(angle);
                }
                break;
            }

            case 't': {  // Throttle
                if (Serial.available() > 0) {
                    int angle = Serial.parseInt();
                    Serial.printf("Setting throttle to %d degrees\n", angle);
                    throttleServo.setAngle(angle);
                }
                break;
            }

            case 'g': {  // Transmission actuator (gear selector)
                if (Serial.available() > 0) {
                    int speed = Serial.parseInt();
                    Serial.printf("Setting transmission actuator to %d\n", speed);
                    transmissionActuator.setSpeed(speed);
                }
                break;
            }

            case 'b': {  // Brake actuator
                if (Serial.available() > 0) {
                    int speed = Serial.parseInt();
                    Serial.printf("Setting brake actuator to %d\n", speed);
                    brakeActuator.setSpeed(speed);
                }
                break;
            }

            case 'R': {  // Set gear to REVERSE
                Serial.println("Setting gear to REVERSE...");
                if (transmissionActuator.setGear(TransmissionController::Gear::GEAR_REVERSE)) {
                    Serial.println("Moving to REVERSE gear position");
                } else {
                    Serial.println("Already at REVERSE gear");
                }
                break;
            }

            case 'N': {  // Set gear to NEUTRAL
                Serial.println("Setting gear to NEUTRAL...");
                if (transmissionActuator.setGear(TransmissionController::Gear::GEAR_NEUTRAL)) {
                    Serial.println("Moving to NEUTRAL gear position");
                } else {
                    Serial.println("Already at NEUTRAL gear");
                }
                break;
            }

            case 'L': {  // Set gear to LOW
                Serial.println("Setting gear to LOW...");
                if (transmissionActuator.setGear(TransmissionController::Gear::GEAR_LOW)) {
                    Serial.println("Moving to LOW gear position");
                } else {
                    Serial.println("Already at LOW gear");
                }
                break;
            }

            case 'H': {  // Set gear to HIGH
                Serial.println("Setting gear to HIGH...");
                if (transmissionActuator.setGear(TransmissionController::Gear::GEAR_HIGH)) {
                    Serial.println("Moving to HIGH gear position");
                } else {
                    Serial.println("Already at HIGH gear");
                }
                break;
            }

            case 'G': {  // Display current gear
                TransmissionController::Gear currentGear = transmissionActuator.getCurrentGear();
                int32_t currentPos = transmissionEncoder.getPosition();
                Serial.printf("Current gear: %s (position: %ld)\n",
                              transmissionActuator.getGearName(currentGear),
                              currentPos);
                break;
            }

            case 'x': {  // Emergency stop
                Serial.println("EMERGENCY STOP!");
                steeringServo.disable();
                throttleServo.disable();
                transmissionActuator.brake();  // Coast to stop
                brakeActuator.brake();         // Coast to stop
                break;
            }

            case 'c': {  // Center/Safe position
                Serial.println("Moving to safe position...");
                steeringServo.setAngle(STEERING_CENTER_ANGLE);
                throttleServo.setAngle(THROTTLE_IDLE_ANGLE);
                transmissionActuator.stop();
                brakeActuator.stop();
                break;
            }

            case 'p': {  // Display encoder position
                int32_t pos = transmissionEncoder.getPosition();
                Serial.printf("Transmission encoder position: %ld counts\n", pos);
                break;
            }

            case 'm': {  // Move to target position
                if (Serial.available() > 0) {
                    int32_t targetPos = Serial.parseInt();
                    Serial.printf("Moving transmission to position %ld...\n", targetPos);
                    if (transmissionActuator.moveToPosition(targetPos, 150)) {
                        Serial.println("Movement started (will stop automatically at target)");
                    } else {
                        Serial.println("Already at target position or no encoder");
                    }
                }
                break;
            }

            case 'h': {  // Manual homing
                Serial.println("Starting manual homing (retracting to limit)...");
                if (transmissionActuator.autoHome(-1, TRANS_HOMING_SPEED, TRANS_HOMING_TIMEOUT)) {
                    Serial.println("Homing complete! Position set to zero.");
                    transmissionEncoder.saveToNVS(NVS_KEY_ENCODER_POS);
                } else {
                    Serial.println("Homing failed or timed out");
                }
                break;
            }

            case 'z': {  // Set current position as zero
                transmissionEncoder.setPosition(0);
                Serial.println("Current position set to zero");
                transmissionEncoder.saveToNVS(NVS_KEY_ENCODER_POS);
                break;
            }

            case 'v': {  // Save position to NVS
                if (transmissionEncoder.saveToNVS(NVS_KEY_ENCODER_POS)) {
                    Serial.printf("Position %ld saved to NVS\n", transmissionEncoder.getPosition());
                } else {
                    Serial.println("Failed to save position to NVS");
                }
                break;
            }

            case 'l': {  // Load position from NVS
                if (transmissionEncoder.loadFromNVS(NVS_KEY_ENCODER_POS)) {
                    Serial.printf("Position %ld loaded from NVS\n", transmissionEncoder.getPosition());
                } else {
                    Serial.println("Failed to load position from NVS");
                }
                break;
            }

            case 'd': {  // Display encoder diagnostics
                Serial.println("\n=== Encoder Diagnostics ===");
                Serial.printf("Encoder Position: %ld\n", transmissionEncoder.getPosition());
                Serial.printf("Raw PCNT Count: %d\n", transmissionEncoder.getRawCount());

                int stateA, stateB;
                transmissionEncoder.readGPIOStates(stateA, stateB);
                Serial.printf("GPIO Pin %d (Channel A): %d\n", PIN_TRANS_ENCODER_A, stateA);
                Serial.printf("GPIO Pin %d (Channel B): %d\n", PIN_TRANS_ENCODER_B, stateB);
                Serial.println("===========================\n");
                break;
            }

            case 'w': {  // Watch encoder for 5 seconds
                Serial.println("Watching encoder for 5 seconds (rotate actuator manually)...");
                uint32_t watchStart = millis();
                int32_t lastPos = transmissionEncoder.getPosition();
                int lastRaw = transmissionEncoder.getRawCount();

                while (millis() - watchStart < 5000) {
                    int32_t currentPos = transmissionEncoder.getPosition();
                    int currentRaw = transmissionEncoder.getRawCount();
                    int stateA, stateB;
                    transmissionEncoder.readGPIOStates(stateA, stateB);

                    // Only print if something changed
                    if (currentPos != lastPos || currentRaw != lastRaw) {
                        Serial.printf("Pos: %ld, Raw: %d, A: %d, B: %d\n",
                                      currentPos, currentRaw, stateA, stateB);
                        lastPos = currentPos;
                        lastRaw = currentRaw;
                    }

                    delay(50);
                }
                Serial.println("Watch complete.");
                break;
            }

            case '\n':
            case '\r':
                // Ignore newlines
                break;

            default:
                Serial.printf("Unknown command: %c\n", cmd);
                break;
        }

        // Clear remaining buffer
        while (Serial.available() > 0) {
            Serial.read();
        }
    }

    // Update position control for transmission actuator
    transmissionActuator.update();

    // Small delay to prevent CPU hogging
    delay(10);
}