#include "PCA9557Expander.h"

PCA9557Expander::PCA9557Expander(TwoWire& bus, uint8_t address)
    : bus_(bus), address_(address) {
}

bool PCA9557Expander::isPresent() {
    bus_.beginTransmission(address_);
    return bus_.endTransmission() == 0;
}

bool PCA9557Expander::readInputs(uint8_t& value) {
    return readRegister(REG_INPUT, value);
}

bool PCA9557Expander::writeOutputs(uint8_t value) {
    return writeRegister(REG_OUTPUT, value);
}

bool PCA9557Expander::readOutputRegister(uint8_t& value) {
    return readRegister(REG_OUTPUT, value);
}

bool PCA9557Expander::setDirection(uint8_t mask) {
    return writeRegister(REG_CONFIG, mask);
}

bool PCA9557Expander::setPolarityInversion(uint8_t mask) {
    return writeRegister(REG_POLARITY, mask);
}

bool PCA9557Expander::writeRegister(uint8_t reg, uint8_t value) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    bus_.write(value);
    return bus_.endTransmission() == 0;
}

bool PCA9557Expander::readRegister(uint8_t reg, uint8_t& value) {
    // Command byte, then a repeated start for the data phase. The output value
    // is only touched on success so a failed read leaves the caller's last-known
    // value intact.
    bus_.beginTransmission(address_);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) {
        return false;
    }
    if (bus_.requestFrom(address_, (uint8_t)1) != 1) {
        return false;
    }
    value = (uint8_t)bus_.read();
    return true;
}
