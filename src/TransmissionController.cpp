#include "TransmissionController.h"

TransmissionController::TransmissionController()
    : BTS7960Controller()
    , targetGear_(Gear::GEAR_NEUTRAL)
{
}

TransmissionController::~TransmissionController() {
}

bool TransmissionController::setGear(TransmissionController::Gear gear, uint8_t speed) {
    int32_t targetPosition = getGearPosition(gear);

    Serial.printf("TransmissionController: Setting gear to %s (position %ld)\n",
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
