#ifndef IRELAY_PORT_H
#define IRELAY_PORT_H

#include <Arduino.h>

/**
 * @brief Abstraction of the relay board's 8-bit output port (same pattern as IMotorDriver).
 *
 * RelayController drives the relays only through this interface, so its logic (shadow
 * byte, ignition, starter escalation, web relay test, recovery) is chip-agnostic. Two
 * implementations exist: PCA9557Expander (custom PCA9557 relay board @0x1F) and
 * XL9535Expander (XL9535-K8V5, port P0 only, @0x20..0x27). RelayController owns the
 * instances (no heap allocation) and points an IRelayPort* at the detected one.
 *
 * Register primitives (writeOutputs / writePolarity / writeConfig / readOutputRegister /
 * readPins / isPresent) each perform at most one I2C transaction, report the result and
 * never retry, sleep or spin — retry and fault policy belong to RelayController.
 *
 * Initialization is implemented ONCE here, so the order is the same for both chips:
 *   1. Output   = seed  (latch the wanted state while the pins are still inputs)
 *   2. Polarity = 0x00  (truthful pin read-back; the PCA9557 powers up with 0xF0)
 *   3. Config   = 0x00  (outputs enabled — they come up at the already-latched value)
 *   4. readPins() must return the seed.
 * ⚠ NEVER write Config before Output: the XL9535 output latch powers up 0xFF, so a
 *   config-first sequence would momentarily energize all eight relays — starter included.
 * initAllOff() is that sequence with seed 0x00 (boot). The fault recovery uses
 * initPort(shadow) so a transient bus error does not blip an energized relay; the caller
 * guarantees the seed never contains a starter bit.
 */
class IRelayPort {
public:
    virtual ~IRelayPort() {}

    /** @brief Probe the device with a zero-length write; true if it ACKs */
    virtual bool isPresent() = 0;

    /** @brief Write the relay output latch (whole 8-bit port) */
    virtual bool writeOutputs(uint8_t value) = 0;

    /** @brief Write the polarity-inversion register of the relay port */
    virtual bool writePolarity(uint8_t mask) = 0;

    /** @brief Write the configuration register of the relay port (1 = input, 0 = output) */
    virtual bool writeConfig(uint8_t mask) = 0;

    /** @brief Read the output LATCH back (not the pins); `value` untouched on failure */
    virtual bool readOutputRegister(uint8_t& value) = 0;

    /**
     * @brief Read the actual PIN levels of the relay port (input-port register), also for
     * pins configured as outputs. A reset expander (config back to inputs) reads differently
     * from its latch, which is what write verification relies on. `value` untouched on failure.
     */
    virtual bool readPins(uint8_t& value) = 0;

    /** @brief 7-bit device address this instance is bound to */
    virtual uint8_t address() const = 0;

    /**
     * @brief Wire error of the last transaction (0 = ok; endTransmission() codes 2 = address NACK,
     * 3 = data NACK, 4 = other, 5 = timeout; 6 = read returned no data — requestFrom() does not
     * expose its error). For the event journal and the I2C2 bus recovery decision.
     */
    virtual uint8_t lastError() const { return 0; }

    /** @brief Chip name for logs ("PCA9557" / "XL9535") */
    virtual const char* name() const = 0;

    /** @brief Lowest 7-bit address this chip family can be strapped to (failure scan range) */
    virtual uint8_t probeRangeMin() const = 0;

    /** @brief Highest 7-bit address this chip family can be strapped to (failure scan range) */
    virtual uint8_t probeRangeMax() const = 0;

    /**
     * @brief Hardware-safe initialization with an output seed (order in the class comment).
     * A fixed, bounded sequence of four transactions; stops at the first failure.
     * @param seed       output byte latched before the outputs are enabled and expected
     *                   back from the pins
     * @param failedStep out (optional): name of the failed step ("output", "polarity",
     *                   "config", "pin read", "pin verify"); nullptr on success
     * @return true when every step succeeded and the pins read back `seed`
     */
    bool initPort(uint8_t seed, const char** failedStep = nullptr) {
        const char* step = nullptr;
        uint8_t pins = (uint8_t)~seed;
        if (!writeOutputs(seed))       step = "output";
        else if (!writePolarity(0x00)) step = "polarity";
        else if (!writeConfig(0x00))   step = "config";
        else if (!readPins(pins))      step = "pin read";
        else if (pins != seed)         step = "pin verify";
        if (failedStep) *failedStep = step;
        return step == nullptr;
    }

    /** @brief Boot initialization: initPort(0x00) — every relay off, verified on the pins */
    bool initAllOff(const char** failedStep = nullptr) { return initPort(0x00, failedStep); }
};

#endif // IRELAY_PORT_H
