#ifndef XL9535_EXPANDER_H
#define XL9535_EXPANDER_H

#include <Arduino.h>
#include <Wire.h>
#include "IRelayPort.h"

/**
 * @brief XL9535 (PCA9535/TCA9535-compatible 16-bit I2C expander) relay-port driver for the
 * XL9535-K8V5 relay board — port P0 ONLY.
 *
 * Register map (command byte): 0x00/0x01 input P0/P1, 0x02/0x03 output P0/P1,
 * 0x04/0x05 polarity P0/P1, 0x06/0x07 configuration P0/P1 (1 = input, 0 = output).
 * Power-on defaults: output 0xFF, polarity 0x00, configuration 0xFF (all inputs).
 * The output register reads back the LATCH; the input register reads the PIN level,
 * also for pins configured as outputs — so readPins() sees a silent reset.
 * 7-bit address 0x20..0x27 (solder pads A0..A2, default 0x20). No RESET pin: the chip
 * only resets on its internal POR (e.g. a VIN brown-out), coming back as all inputs.
 *
 * P1 (0x01, 0x03, 0x05, 0x07) is NEVER written: it stays at its power-on default
 * (inputs). Every register access is a single-byte register write / read on P0.
 *
 * ⚠ ALL-ON HAZARD: the P0 output latch powers up 0xFF. Writing the configuration
 *   register (outputs enabled) before the output register would momentarily energize
 *   all eight relays, the starter included. Initialize ONLY through
 *   IRelayPort::initAllOff() / initPort(), which write output → polarity → config.
 *
 * Bound to one `TwoWire` bus and one 7-bit address at construction; the bus stays at
 * I2C_BUS_FREQ_HZ (100 kHz, the chip allows 400 kHz). Every method performs at most one
 * I2C transaction, reports the result and never retries, sleeps or spins.
 *
 * Relay mapping (identity, active-high) is in RelayBoardLaw.h (RELAY_TABLE_XL9535) and
 * is pending bench confirmation on a K8 board.
 */
class XL9535Expander : public IRelayPort {
public:
    static constexpr uint8_t REG_INPUT_P0    = 0x00;
    static constexpr uint8_t REG_OUTPUT_P0   = 0x02;
    static constexpr uint8_t REG_POLARITY_P0 = 0x04;
    static constexpr uint8_t REG_CONFIG_P0   = 0x06;
    static constexpr uint8_t ADDR_MIN        = 0x20;   // A2A1A0 = 000 (default pads)
    static constexpr uint8_t ADDR_MAX        = 0x27;   // A2A1A0 = 111

    /**
     * @param bus     I2C bus the board lives on (must be `begin()`-ed by the owner)
     * @param address 7-bit device address (0x20..0x27)
     */
    XL9535Expander(TwoWire& bus, uint8_t address);

    bool isPresent() override;
    bool writeOutputs(uint8_t value) override;          // P0 output register 0x02
    bool writePolarity(uint8_t mask) override;          // P0 polarity register 0x04
    bool writeConfig(uint8_t mask) override;            // P0 configuration register 0x06
    bool readOutputRegister(uint8_t& value) override;   // P0 output latch 0x02
    bool readPins(uint8_t& value) override;             // P0 input register 0x00 (pin levels)

    uint8_t address() const override { return address_; }
    const char* name() const override { return "XL9535"; }
    uint8_t probeRangeMin() const override { return ADDR_MIN; }
    uint8_t probeRangeMax() const override { return ADDR_MAX; }

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);

    TwoWire& bus_;
    uint8_t  address_;
};

#endif // XL9535_EXPANDER_H
