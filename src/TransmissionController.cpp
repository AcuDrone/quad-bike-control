#include "TransmissionController.h"

TransmissionController::TransmissionController()
    : BTS7960Controller()
    , targetGear_(Gear::GEAR_NEUTRAL)
{
    // Initialize vehicle data to safe defaults
    vehicleData_.vehicleSpeed = 0;
    vehicleData_.lastUpdateTime = 0;
    vehicleData_.dataValid = false;
}

TransmissionController::~TransmissionController() {
}

bool TransmissionController::setGear(TransmissionController::Gear gear, uint8_t speed) {
    // Safety check: prevent gear changes when vehicle is moving
    if (!canChangeGear(gear)) {
        return false;  // Gear change blocked
    }

    int32_t targetPosition = getGearPosition(gear);

    Serial.printf("[TRANS] Setting gear to %s (position %ld)\n",
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
            Serial.printf("[TRANS] Gear change blocked: vehicle moving at %d km/h\n",
                         vehicleData_.vehicleSpeed);
            return false;
        }
    } else {
        // CAN data unavailable - check timeout
        uint32_t dataAge = millis() - vehicleData_.lastUpdateTime;

        if (vehicleData_.lastUpdateTime > 0 && dataAge < TRANS_CAN_TIMEOUT) {
            // Data is recent but marked invalid - be cautious
            Serial.println("[TRANS] WARNING: CAN data invalid, blocking gear change");
            return false;
        } else {
            // CAN timeout exceeded or never had data - allow gear change (fail-safe)
            Serial.printf("[TRANS] WARNING: CAN timeout (%lu ms), allowing gear change\n", dataAge);
        }
    }

    return true;
}

bool TransmissionController::needsThrottleBoost() const {
    // Check if we're moving to a gear (not stopped at target)
    return !isStopped() && targetGear_ != Gear::GEAR_NEUTRAL;
}
