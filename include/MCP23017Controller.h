#ifndef MCP23017_CONTROLLER_H
#define MCP23017_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

/**
 * @brief Wrapper for MCP23017 I2C GPIO expander
 *
 * Provides digital I/O via I2C for relays (Port A outputs)
 * and sensors (Port B inputs).
 */
class MCP23017Controller {
public:
    MCP23017Controller();

    /**
     * @brief Initialize MCP23017 on I2C bus
     * @param sdaPin I2C SDA GPIO pin
     * @param sclPin I2C SCL GPIO pin
     * @param address I2C address (default 0x20)
     * @return true if MCP23017 detected and configured
     */
    bool begin(uint8_t sdaPin, uint8_t sclPin, uint8_t address = 0x20);

    /**
     * @brief Set pin mode (INPUT, OUTPUT, INPUT_PULLUP)
     */
    void pinMode(uint8_t pin, uint8_t mode);

    /**
     * @brief Write digital value to output pin
     */
    void digitalWrite(uint8_t pin, uint8_t value);

    /**
     * @brief Read digital value from input pin
     */
    uint8_t digitalRead(uint8_t pin);

    /**
     * @brief Write all 8 pins of a port at once
     * @param port 0 = Port A, 1 = Port B
     * @param value 8-bit value for the port
     */
    void writePort(uint8_t port, uint8_t value);

    /**
     * @brief Read all 8 pins of a port at once
     * @param port 0 = Port A, 1 = Port B
     * @return 8-bit value representing all pin states
     */
    uint8_t readPort(uint8_t port);

    /**
     * @brief Check if MCP23017 was successfully initialized
     */
    bool isInitialized() const { return initialized_; }

private:
    Adafruit_MCP23X17 mcp_;
    bool initialized_;
};

#endif // MCP23017_CONTROLLER_H
