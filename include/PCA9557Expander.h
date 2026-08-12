#ifndef PCA9557_EXPANDER_H
#define PCA9557_EXPANDER_H

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief Minimal PCA9557 8-bit I2C port expander register driver.
 *
 * The device exposes four registers:
 *   0x00 input port, 0x01 output port, 0x02 polarity inversion,
 *   0x03 configuration (bit = 1 -> input, bit = 0 -> output).
 *
 * The instance is bound to exactly one `TwoWire` bus and one 7-bit address at
 * construction — it never scans the bus and never addresses another device.
 * Every method performs at most one I2C transaction, returns the transaction
 * result to the caller and never retries, sleeps or spins: retry and fault
 * policy belong to the caller (see `BoardInputs` and `RelayController`), which
 * keeps the cooperative main loop non-blocking.
 */
class PCA9557Expander {
public:
    static constexpr uint8_t REG_INPUT    = 0x00;
    static constexpr uint8_t REG_OUTPUT   = 0x01;
    static constexpr uint8_t REG_POLARITY = 0x02;
    static constexpr uint8_t REG_CONFIG   = 0x03;

    /**
     * @brief Bind the driver to one bus and one device address.
     * @param bus     I2C bus the device lives on (must be `begin()`-ed by the owner)
     * @param address 7-bit device address
     */
    PCA9557Expander(TwoWire& bus, uint8_t address);

    /**
     * @brief Probe the device with a zero-length write.
     * @return true if the device ACKs its address
     */
    bool isPresent();

    /**
     * @brief Read the input port register (0x00).
     * @param value Out: port state; left unmodified on failure
     * @return true on a successful transaction
     */
    bool readInputs(uint8_t& value);

    /**
     * @brief Write the output port register (0x01).
     * @return true on a successful transaction
     */
    bool writeOutputs(uint8_t value);

    /**
     * @brief Read the output port register (0x01) back for write verification.
     * @param value Out: last value latched into the output register
     * @return true on a successful transaction
     */
    bool readOutputRegister(uint8_t& value);

    /**
     * @brief Write the configuration register (0x03).
     * @param mask 1 = pin is an input, 0 = pin is an output
     * @return true on a successful transaction
     */
    bool setDirection(uint8_t mask);

    /**
     * @brief Write the polarity inversion register (0x02).
     * @param mask 1 = input bit is inverted when read
     * @return true on a successful transaction
     */
    bool setPolarityInversion(uint8_t mask);

    /** @brief 7-bit device address this instance is bound to */
    uint8_t address() const { return address_; }

    /** @brief The bus this instance is bound to */
    TwoWire& bus() const { return bus_; }

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);

    TwoWire& bus_;
    uint8_t  address_;
};

#endif // PCA9557_EXPANDER_H
