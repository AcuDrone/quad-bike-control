#include "AS5600Sensor.h"
#include "Debug.h"
#include <Wire.h>

static constexpr uint8_t AS5600_I2C_ADDR    = 0x36;
static constexpr uint8_t AS5600_REG_STATUS  = 0x0B;   // MD bit 5 = magnet detected
static constexpr uint8_t AS5600_REG_RAW_ANGLE = 0x0C; // 0x0C hi / 0x0D lo, 12-bit

AS5600Sensor::AS5600Sensor()
    : initialized_(false) {
}

bool AS5600Sensor::begin(gpio_num_t sdaPin, gpio_num_t sclPin) {
    if (!Wire.begin(sdaPin, sclPin, 400000)) {
        Debug::printlnFeature(DebugFeature::SERVO, "[AS5600] Wire.begin failed");
        return false;
    }

    // Probe the sensor
    Wire.beginTransmission(AS5600_I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        Debug::printlnFeature(DebugFeature::SERVO, "[AS5600] Sensor not responding at 0x36");
        return false;
    }

    initialized_ = true;

    uint16_t raw;
    bool magnetOk = false;
    read(raw, magnetOk);
    Debug::printfFeature(DebugFeature::SERVO, "[AS5600] Initialized: SDA=%d, SCL=%d, magnet %s\n",
                         sdaPin, sclPin, magnetOk ? "OK" : "NOT DETECTED");
    return true;
}

bool AS5600Sensor::read(uint16_t& rawAngle, bool& magnetOk) {
    // Burst read STATUS (0x0B) + RAW ANGLE hi/lo (0x0C/0x0D) in one transaction
    uint8_t buf[3];
    if (!readRegisters(AS5600_REG_STATUS, buf, 3)) return false;
    magnetOk = (buf[0] & 0x20) != 0;   // MD bit
    rawAngle = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];
    return true;
}

bool AS5600Sensor::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (!initialized_) return false;

    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(AS5600_I2C_ADDR, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}
