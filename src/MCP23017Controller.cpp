#include "MCP23017Controller.h"
#include "Debug.h"
#include <Wire.h>

MCP23017Controller::MCP23017Controller()
    : initialized_(false) {
}

bool MCP23017Controller::begin(uint8_t sdaPin, uint8_t sclPin, uint8_t address) {
    Wire.begin(sdaPin, sclPin, 400000);

    if (!mcp_.begin_I2C(address, &Wire)) {
        Debug::printlnFeature(DebugFeature::RELAY, "[MCP23017] ERROR: Not detected at address 0x%02X");
        initialized_ = false;
        return false;
    }

    initialized_ = true;
    Debug::printfFeature(DebugFeature::RELAY, "[MCP23017] Initialized at address 0x%02X\n", address);
    return true;
}

void MCP23017Controller::pinMode(uint8_t pin, uint8_t mode) {
    if (!initialized_) return;
    mcp_.pinMode(pin, mode);
}

void MCP23017Controller::digitalWrite(uint8_t pin, uint8_t value) {
    if (!initialized_) return;
    mcp_.digitalWrite(pin, value);
}

uint8_t MCP23017Controller::digitalRead(uint8_t pin) {
    if (!initialized_) return HIGH;  // Safe default for active-low inputs
    return mcp_.digitalRead(pin);
}

void MCP23017Controller::writePort(uint8_t port, uint8_t value) {
    if (!initialized_) return;
    // Port A = register 0x12 (GPIOA), Port B = register 0x13 (GPIOB)
    mcp_.writeGPIO(value, port);
}

uint8_t MCP23017Controller::readPort(uint8_t port) {
    if (!initialized_) return 0xFF;  // Safe default (all HIGH = no sensor active)
    return mcp_.readGPIO(port);
}
