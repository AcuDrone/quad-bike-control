#ifndef PCA9557_EXPANDER_H
#define PCA9557_EXPANDER_H

#include <Arduino.h>
#include <Wire.h>
#include "IRelayPort.h"

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
 *
 * It also implements IRelayPort for the custom PCA9557 relay board (@0x1F): the relay
 * port is the whole 8-bit port, readPins() is the input register (0x00) and the
 * IRelayPort::initPort()/initAllOff() order applies (output, polarity, config, pin
 * verify). The polarity write matters here: the PCA9557 powers up with polarity 0xF0,
 * which would invert the upper nibble of the pin read-back. BoardInputs (U10 @0x1A)
 * uses the same class through its own register methods only.
 */
class PCA9557Expander : public IRelayPort {
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
    bool isPresent() override;

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
    bool writeOutputs(uint8_t value) override;

    /**
     * @brief Read the output port register (0x01) back for write verification.
     * @param value Out: last value latched into the output register
     * @return true on a successful transaction
     */
    bool readOutputRegister(uint8_t& value) override;

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

    // ---- IRelayPort ----
    /** @brief Pin levels: the input port register (0x00) */
    bool readPins(uint8_t& value) override { return readInputs(value); }
    /** @brief Polarity inversion register (0x02) — same as setPolarityInversion() */
    bool writePolarity(uint8_t mask) override { return setPolarityInversion(mask); }
    /** @brief Configuration register (0x03) — same as setDirection() */
    bool writeConfig(uint8_t mask) override { return setDirection(mask); }
    const char* name() const override { return "PCA9557"; }
    uint8_t probeRangeMin() const override { return 0x18; }   // A2A1A0 = 000
    uint8_t probeRangeMax() const override { return 0x1F; }   // A2A1A0 = 111

    /** @brief 7-bit device address this instance is bound to */
    uint8_t address() const override { return address_; }

    /** @brief The bus this instance is bound to */
    TwoWire& bus() const { return bus_; }

    /** @brief Wire error of the last transaction (see IRelayPort::lastError()) */
    uint8_t lastError() const override { return lastError_; }

    /** @brief requestFrom() returned fewer bytes than asked (its Wire error is not exposed) */
    static constexpr uint8_t ERR_READ_SHORT = 6;

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);

    TwoWire& bus_;
    uint8_t  address_;
    uint8_t  lastError_ = 0;
};

#endif // PCA9557_EXPANDER_H
