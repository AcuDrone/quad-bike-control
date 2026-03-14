#include "TransmissionController.h"
#include "Debug.h"
#include "driver/gpio.h"

TransmissionController::TransmissionController()
    : BTS7960Controller()
    , targetGear_(Gear::GEAR_NEUTRAL)
    , lastGearCheckTime_(0)
    , lastMovementLogTime_(0)
    , lastGearMismatch_(false)
    , isCalibrated_(false)
{
    // Initialize vehicle data to safe defaults
    vehicleData_.vehicleSpeed = 0;
    vehicleData_.lastUpdateTime = 0;
    vehicleData_.dataValid = false;

    // Initialize calibrated positions to defaults (will be overwritten by calibration or load)
    calibratedPositions_[(int)Gear::GEAR_HIGH] = TRANS_POSITION_HIGH;
    calibratedPositions_[(int)Gear::GEAR_LOW] = TRANS_POSITION_LOW;
    calibratedPositions_[(int)Gear::GEAR_NEUTRAL] = TRANS_POSITION_NEUTRAL;
    calibratedPositions_[(int)Gear::GEAR_REVERSE] = TRANS_POSITION_REVERSE;

    // NOTE: Do NOT load calibration here! NVS is not initialized during global construction.
    // Calibration will be loaded in main.cpp after NVS initialization.
}

TransmissionController::~TransmissionController() {
}

bool TransmissionController::setGear(TransmissionController::Gear gear, uint8_t speed) {
    // Safety check: prevent gear changes when vehicle is moving
    if (!canChangeGear(gear)) {
        return false;  // Gear change blocked
    }

    int32_t targetPosition = getGearPosition(gear);

    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] Setting gear to %s (position %ld)\n",
                  getGearName(gear), targetPosition);

    targetGear_ = gear;
    return moveToPosition(targetPosition, speed);
}

TransmissionController::Gear TransmissionController::getCurrentGear() const {
    if (!hasEncoder()) {
        return Gear::GEAR_NEUTRAL;
    }

    int32_t currentPos = getPosition();

    // Check each gear position (within tolerance)
    if (abs(currentPos - TRANS_POSITION_HIGH) <= TRANS_POSITION_TOLERANCE) {
        return Gear::GEAR_HIGH;
    }
    if (abs(currentPos - TRANS_POSITION_LOW) <= TRANS_POSITION_TOLERANCE) {
        return Gear::GEAR_LOW;
    }
    if (abs(currentPos - TRANS_POSITION_NEUTRAL) <= TRANS_POSITION_TOLERANCE) {
        return Gear::GEAR_NEUTRAL;
    }
    if (abs(currentPos - TRANS_POSITION_REVERSE) <= TRANS_POSITION_TOLERANCE) {
        return Gear::GEAR_REVERSE;
    }

    // If not at any known gear position, return NEUTRAL as safe default
    return Gear::GEAR_NEUTRAL;
}

bool TransmissionController::isAtGear(TransmissionController::Gear gear) const {
    int32_t targetPosition = getGearPosition(gear);
    return isAtPosition(targetPosition, TRANS_POSITION_TOLERANCE);
}

int32_t TransmissionController::getGearPosition(TransmissionController::Gear gear) const {
    // Use calibrated positions if available, otherwise use defaults
    if (isCalibrated_) {
        return calibratedPositions_[(int)gear];
    }

    // Fallback to default positions
    switch (gear) {
        case Gear::GEAR_HIGH:
            return TRANS_POSITION_HIGH;
        case Gear::GEAR_LOW:
            return TRANS_POSITION_LOW;
        case Gear::GEAR_NEUTRAL:
            return TRANS_POSITION_NEUTRAL;
        case Gear::GEAR_REVERSE:
            return TRANS_POSITION_REVERSE;
        default:
            return TRANS_POSITION_NEUTRAL;  // Safe default
    }
}

int32_t TransmissionController::getCalibratedPosition(TransmissionController::Gear gear) const {
    return calibratedPositions_[(int)gear];
}

const char* TransmissionController::getGearName(TransmissionController::Gear gear) const {
    switch (gear) {
        case Gear::GEAR_HIGH:
            return "HIGH";
        case Gear::GEAR_LOW:
            return "LOW";
        case Gear::GEAR_NEUTRAL:
            return "NEUTRAL";
        case Gear::GEAR_REVERSE:
            return "REVERSE";
        default:
            return "UNKNOWN";
    }
}

void TransmissionController::setVehicleData(const TransmissionVehicleData& data) {
    vehicleData_ = data;
}

bool TransmissionController::canChangeGear(TransmissionController::Gear targetGear) const {
    // Always allow changes to NEUTRAL (safety override)
    if (targetGear == Gear::GEAR_NEUTRAL) {
        return true;
    }

    // Check if CAN data is valid
    if (vehicleData_.dataValid) {
        // Block gear change if vehicle is moving above threshold
        if (vehicleData_.vehicleSpeed > TRANS_SPEED_INTERLOCK_THRESHOLD) {
            Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] Gear change blocked: vehicle moving at %d km/h\n",
                         vehicleData_.vehicleSpeed);
            return false;
        }
    } else {
        // CAN data unavailable - check timeout
        uint32_t dataAge = millis() - vehicleData_.lastUpdateTime;

        if (vehicleData_.lastUpdateTime > 0 && dataAge < TRANS_CAN_TIMEOUT) {
            // Data is recent but marked invalid - be cautious
            Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] WARNING: CAN data invalid, blocking gear change");
            return false;
        } else {
            // CAN timeout exceeded or never had data - allow gear change (fail-safe)
            Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] WARNING: CAN timeout (%lu ms), allowing gear change\n", dataAge);
        }
    }

    return true;
}

bool TransmissionController::needsThrottleBoost() const {
    // Check if we're moving to a gear (not stopped at target)
    return !isStopped() && targetGear_ != Gear::GEAR_NEUTRAL;
}

void TransmissionController::initGearSensors() {
    // Configure gear position sensor pins as inputs
    // Active-low: pin reads LOW when gear is selected
    // Hardware has external pull-up resistors - no internal pull-ups needed
    // Using ESP-IDF GPIO API for proper configuration (required for GPIO 15 strapping pin)

    gpio_set_direction(PIN_GEAR_REVERSE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_GEAR_REVERSE, GPIO_FLOATING);    // External pull-up on hardware

    gpio_set_direction(PIN_GEAR_NEUTRAL, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_GEAR_NEUTRAL, GPIO_FLOATING);    // External pull-up on hardware

    gpio_set_direction(PIN_GEAR_LOW, GPIO_MODE_INPUT);      // GPIO 15 - strapping pin
    gpio_set_pull_mode(PIN_GEAR_LOW, GPIO_FLOATING);        // External pull-up on hardware

    gpio_set_direction(PIN_GEAR_HIGH, GPIO_MODE_INPUT);     // GPIO 18 - safe
    gpio_set_pull_mode(PIN_GEAR_HIGH, GPIO_FLOATING);       // External pull-up on hardware

    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Gear position sensors initialized (active-low, external pull-ups)");
}

TransmissionController::Gear TransmissionController::getPhysicalGear() const {
    // Read all gear sensor pins (active-low: LOW = selected)
    bool reverseActive = (digitalRead(PIN_GEAR_REVERSE) == LOW);
    bool neutralActive = (digitalRead(PIN_GEAR_NEUTRAL) == LOW);
    bool lowActive = (digitalRead(PIN_GEAR_LOW) == LOW);
    bool highActive = (digitalRead(PIN_GEAR_HIGH) == LOW);

    // Count how many sensors are active (should be exactly 1)
    uint8_t activeCount = reverseActive + neutralActive + lowActive + highActive;

    if (activeCount == 0) {
        // No gear selected (all pins HIGH) - return NEUTRAL as safe default
        Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] WARNING: No gear sensor active");
        return Gear::GEAR_NEUTRAL;
    }

    if (activeCount > 1) {
        // Multiple gears active - invalid state, return NEUTRAL for safety
        Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] ERROR: Multiple gear sensors active (R:%d N:%d L:%d H:%d)\n",
                     reverseActive, neutralActive, lowActive, highActive);
        return Gear::GEAR_NEUTRAL;
    }

    // Exactly one sensor is active - return the corresponding gear
    if (reverseActive) return Gear::GEAR_REVERSE;
    if (neutralActive) return Gear::GEAR_NEUTRAL;
    if (lowActive) return Gear::GEAR_LOW;
    if (highActive) return Gear::GEAR_HIGH;

    // Should never reach here, but return NEUTRAL for safety
    return Gear::GEAR_NEUTRAL;
}

bool TransmissionController::isGearPositionValid() const {
    // Compare encoder-based gear with physical sensor gear
    Gear encoderGear = getCurrentGear();
    Gear physicalGear = getPhysicalGear();

    if (encoderGear != physicalGear) {
        Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] WARNING: Gear mismatch - Encoder: %s, Physical: %s\n",
                     getGearName(encoderGear), getGearName(physicalGear));
        return false;
    }

    return true;
}

void TransmissionController::update() {
    // Log current state if moving (throttled to reduce spam)
    uint32_t now = millis();
    if (isPositionControlActive() && (now - lastMovementLogTime_ >= 250)) {
        lastMovementLogTime_ = now;
        int32_t currentPos = getPosition();
        int32_t targetPos = getTargetPosition();
        Debug::printfFeature(DebugFeature::TRANSMISSION,
            "[TRANS] Moving: Current=%ld, Target=%ld, Error=%ld\n",
            currentPos, targetPos, abs(targetPos - currentPos));
    }

    // Check physical gear position periodically
    if (now - lastGearCheckTime_ >= TRANS_GEAR_CHECK_INTERVAL) {
        lastGearCheckTime_ = now;

        // Read physical gear position (ground truth)
        Gear physicalGear = getPhysicalGear();

        // Check if physical gear matches target gear during position control
        if (isPositionControlActive() && physicalGear == targetGear_) {
            // Physical switch confirms we've reached target gear!
            // Recalibrate encoder to match expected position for this gear
            int32_t expectedPosition = getGearPosition(targetGear_);
            int32_t currentPosition = getPosition();
            int32_t positionError = abs(currentPosition - expectedPosition);

            if (positionError > TRANS_POSITION_TOLERANCE) {
                // Encoder position differs from expected - recalibrate
                Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] Physical gear %s reached, recalibrating encoder\n",
                             getGearName(targetGear_));
                Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] Encoder: %ld -> %ld (error: %ld)\n",
                             currentPosition, expectedPosition, positionError);

                // Recalibrate encoder to expected position for this gear
                // recalibrateEncoder(expectedPosition);
            }

            // Stop position control - physical switch confirms arrival
            stopPositionControl();
            Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] Target gear %s confirmed by physical switch\n",
                         getGearName(targetGear_));

            // Clear any previous mismatch flag
            lastGearMismatch_ = false;
            return;
        }

        // Verify encoder consistency when stopped
        if (isStopped()) {
            Gear encoderGear = getCurrentGear();

            if (encoderGear != physicalGear) {
                // Mismatch detected while stopped
                if (!lastGearMismatch_) {
                    // First mismatch - log detailed warning
                    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] MISMATCH: Encoder reports %s, Physical switch shows %s\n",
                                 getGearName(encoderGear), getGearName(physicalGear));
                    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] Encoder position: %ld, Expected: %ld\n",
                                 getPosition(), getGearPosition(physicalGear));
                    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Possible causes: encoder drift, mechanical slippage, or wiring issue");
                    lastGearMismatch_ = true;
                }
                // Subsequent mismatches are silent to avoid spam
            } else {
                // Position valid
                if (lastGearMismatch_) {
                    // Mismatch resolved
                    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] Position mismatch RESOLVED: Now at %s\n", getGearName(encoderGear));
                    lastGearMismatch_ = false;
                }
            }
        }
    }

    // Call parent update() to handle encoder-based position control loop
    BTS7960Controller::update();
}

bool TransmissionController::calibrateAllGearPositions(uint8_t calibrationSpeed, uint32_t timeout) {
    if (!hasEncoder()) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ERROR: Cannot calibrate - no encoder attached");
        return false;
    }

    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ========================================");
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Starting full gear calibration...");
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ========================================");

    // First, home to REVERSE position (physical stop, position 0)
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Step 1: Homing to REVERSE position...");
    if (!autoHome(-1, TRANS_HOMING_SPEED, timeout)) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ERROR: Failed to home to REVERSE");
        return false;
    }
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Homed to REVERSE at position 0");

    uint32_t startTime = millis();

    // Structure to store gear detection data
    struct GearCalibration {
        Gear gear;
        gpio_num_t pin;
        const char* name;
        int32_t entryPosition;
        int32_t exitPosition;
        bool entryDetected;
        bool exitDetected;
    };

    // Define gears to calibrate (in order of traversal: REVERSE -> NEUTRAL -> LOW -> HIGH)
    GearCalibration gears[] = {
        {Gear::GEAR_NEUTRAL, PIN_GEAR_NEUTRAL, "NEUTRAL", 0, 0, false, false},
        {Gear::GEAR_LOW, PIN_GEAR_LOW, "LOW", 0, 0, false, false},
        {Gear::GEAR_HIGH, PIN_GEAR_HIGH, "HIGH", 0, 0, false, false}
    };
    const int numGears = 3;  // NEUTRAL, LOW, HIGH (REVERSE is at 0 by definition)

    // Step 2: Start moving forward from REVERSE (position 0)
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Step 2: Moving forward from REVERSE to detect all gears...");
    setSpeed(calibrationSpeed);

    int currentGearIndex = 0;
    bool allGearsCalibrated = false;

    while (!allGearsCalibrated) {
        // Check timeout
        if (millis() - startTime >= timeout) {
            Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ERROR: Calibration timeout!");
            stop();
            return false;
        }

        // Get current encoder position
        int32_t currentPos = getPosition();

        // Read current gear sensor
        if (currentGearIndex < numGears) {
            GearCalibration& gear = gears[currentGearIndex];
            bool sensorActive = (digitalRead(gear.pin) == LOW);

            if (!gear.entryDetected && sensorActive) {
                // Gear entry detected
                gear.entryPosition = currentPos;
                gear.entryDetected = true;
                Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] %s entry detected at position %ld\n",
                             gear.name, gear.entryPosition);
            }
            else if (gear.entryDetected && !gear.exitDetected && !sensorActive) {
                // Gear exit detected
                gear.exitPosition = currentPos;
                gear.exitDetected = true;

                // Calculate center position
                int32_t centerPosition = (gear.entryPosition + gear.exitPosition) / 2;
                calibratedPositions_[(int)gear.gear] = centerPosition;

                Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] %s exit detected at position %ld\n",
                             gear.name, gear.exitPosition);
                Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] %s calibrated to position %ld (center of %ld to %ld)\n",
                             gear.name, centerPosition, gear.entryPosition, gear.exitPosition);

                // Move to next gear
                currentGearIndex++;

                // Check if all gears calibrated
                if (currentGearIndex >= numGears) {
                    allGearsCalibrated = true;
                }
            }
        }

        delay(10);  // Small delay for stable readings
    }

    // Stop movement
    stop();
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Calibration movement complete");

    // REVERSE gear is always at position 0 (home position)
    calibratedPositions_[(int)Gear::GEAR_REVERSE] = 0;

    // Display calibration results
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ========================================");
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Calibration Results:");
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ========================================");
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] REVERSE: %6ld (home position)\n", calibratedPositions_[(int)Gear::GEAR_REVERSE]);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] NEUTRAL: %6ld\n", calibratedPositions_[(int)Gear::GEAR_NEUTRAL]);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] LOW:     %6ld\n", calibratedPositions_[(int)Gear::GEAR_LOW]);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] HIGH:    %6ld\n", calibratedPositions_[(int)Gear::GEAR_HIGH]);
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ========================================");

    // Mark as calibrated
    isCalibrated_ = true;

    // Return to target gear position
    int32_t targetPosition = getGearPosition(targetGear_);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS] Returning to target gear %s (position %ld)...\n",
                 getGearName(targetGear_), targetPosition);
    if (moveToPosition(targetPosition, calibrationSpeed)) {
        // Wait for movement to complete
        uint32_t returnStart = millis();
        while (isPositionControlActive() && (millis() - returnStart < 30000)) {
            BTS7960Controller::update();
            delay(10);
        }
    }

    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ========================================");
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Gear calibration complete!");
    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ========================================");

    // Save calibration to non-volatile storage
    if (saveCalibration()) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Calibration saved to storage");
    } else {
        Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] WARNING: Failed to save calibration");
    }

    return true;
}

bool TransmissionController::saveCalibration() {
    Preferences prefs;

    // Open preferences in read-write mode
    if (!prefs.begin("transmission", false)) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] ERROR: Failed to open preferences for write");
        return false;
    }

    // Save each gear position
    prefs.putInt("pos_high", calibratedPositions_[(int)Gear::GEAR_HIGH]);
    prefs.putInt("pos_low", calibratedPositions_[(int)Gear::GEAR_LOW]);
    prefs.putInt("pos_neutral", calibratedPositions_[(int)Gear::GEAR_NEUTRAL]);
    prefs.putInt("pos_reverse", calibratedPositions_[(int)Gear::GEAR_REVERSE]);

    // Save calibration flag
    prefs.putBool("calibrated", true);

    prefs.end();

    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Calibration saved:");
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS]   HIGH:    %ld\n", calibratedPositions_[(int)Gear::GEAR_HIGH]);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS]   LOW:     %ld\n", calibratedPositions_[(int)Gear::GEAR_LOW]);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS]   NEUTRAL: %ld\n", calibratedPositions_[(int)Gear::GEAR_NEUTRAL]);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS]   REVERSE: %ld\n", calibratedPositions_[(int)Gear::GEAR_REVERSE]);

    return true;
}

bool TransmissionController::loadCalibration() {
    Preferences prefs;

    // Open preferences in read-only mode
    if (!prefs.begin("transmission", true)) {
        Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Failed to open preferences for read");
        return false;
    }

    // Check if calibration exists
    if (!prefs.getBool("calibrated", false)) {
        prefs.end();
        return false;
    }

    // Load each gear position
    calibratedPositions_[(int)Gear::GEAR_HIGH] = prefs.getInt("pos_high", TRANS_POSITION_HIGH);
    calibratedPositions_[(int)Gear::GEAR_LOW] = prefs.getInt("pos_low", TRANS_POSITION_LOW);
    calibratedPositions_[(int)Gear::GEAR_NEUTRAL] = prefs.getInt("pos_neutral", TRANS_POSITION_NEUTRAL);
    calibratedPositions_[(int)Gear::GEAR_REVERSE] = prefs.getInt("pos_reverse", TRANS_POSITION_REVERSE);

    prefs.end();

    // Mark as calibrated
    isCalibrated_ = true;

    Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Calibration loaded:");
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS]   HIGH:    %ld\n", calibratedPositions_[(int)Gear::GEAR_HIGH]);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS]   LOW:     %ld\n", calibratedPositions_[(int)Gear::GEAR_LOW]);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS]   NEUTRAL: %ld\n", calibratedPositions_[(int)Gear::GEAR_NEUTRAL]);
    Debug::printfFeature(DebugFeature::TRANSMISSION,"[TRANS]   REVERSE: %ld\n", calibratedPositions_[(int)Gear::GEAR_REVERSE]);

    return true;
}

void TransmissionController::clearCalibration() {
    Preferences prefs;

    if (prefs.begin("transmission", false)) {
        prefs.clear();  // Clear all keys in this namespace
        prefs.end();
        Debug::printlnFeature(DebugFeature::TRANSMISSION,"[TRANS] Calibration cleared from storage");
    }

    // Reset to defaults
    calibratedPositions_[(int)Gear::GEAR_HIGH] = TRANS_POSITION_HIGH;
    calibratedPositions_[(int)Gear::GEAR_LOW] = TRANS_POSITION_LOW;
    calibratedPositions_[(int)Gear::GEAR_NEUTRAL] = TRANS_POSITION_NEUTRAL;
    calibratedPositions_[(int)Gear::GEAR_REVERSE] = TRANS_POSITION_REVERSE;
    isCalibrated_ = false;
}
