#include "XL9535Expander.h"

// P0 only. The P1 registers (0x01, 0x03, 0x05, 0x07) are never addressed: every write
// below is a single-byte write to a P0 register, which the PCA9535 register pointer does
// not auto-advance into P1 because exactly one data byte is sent.

XL9535Expander::XL9535Expander(TwoWire& bus, uint8_t address)
    : bus_(bus), address_(address) {
}

bool XL9535Expander::isPresent() {
    bus_.beginTransmission(address_);
    lastError_ = bus_.endTransmission();
    return lastError_ == 0;
}

bool XL9535Expander::writeOutputs(uint8_t value) {
    return writeRegister(REG_OUTPUT_P0, value);
}

bool XL9535Expander::writePolarity(uint8_t mask) {
    return writeRegister(REG_POLARITY_P0, mask);
}

bool XL9535Expander::writeConfig(uint8_t mask) {
    return writeRegister(REG_CONFIG_P0, mask);
}

bool XL9535Expander::readOutputRegister(uint8_t& value) {
    return readRegister(REG_OUTPUT_P0, value);
}

bool XL9535Expander::readPins(uint8_t& value) {
    return readRegister(REG_INPUT_P0, value);
}

bool XL9535Expander::writeRegister(uint8_t reg, uint8_t value) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    bus_.write(value);
    lastError_ = bus_.endTransmission();
    return lastError_ == 0;
}

bool XL9535Expander::readRegister(uint8_t reg, uint8_t& value) {
    // Command byte, then a repeated start for a ONE-byte data phase (P0 only). `value` is
    // touched only on success so a failed read leaves the caller's last-known value intact.
    bus_.beginTransmission(address_);
    bus_.write(reg);
    lastError_ = bus_.endTransmission(false);
    if (lastError_ != 0) {
        return false;
    }
    if (bus_.requestFrom(address_, (uint8_t)1) != 1) {
        lastError_ = ERR_READ_SHORT;
        return false;
    }
    value = (uint8_t)bus_.read();
    lastError_ = 0;
    return true;
}
