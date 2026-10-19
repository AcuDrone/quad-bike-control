#ifndef AS5600_SENSOR_H
#define AS5600_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief Minimal AS5600 absolute magnetic angle sensor driver (I2C), bus-agnostic.
 *
 * Reads the 12-bit RAW ANGLE register (0-4095, ~0.088°/LSB) and the
 * magnet-detect (MD) status bit. Fixed I2C address 0x36, so two sensors must
 * sit on different buses. On the aztec buggy the steering uses two of them:
 *  - the FINE sensor on the steering shaft (I2C1 `Wire`, X14 — it has the bus
 *    to itself, so it may run its own wedged-bus recovery);
 *  - the COARSE sensor on a geared-down shaft (I2C2 `Wire1`, X15 — shared with
 *    the PCA9557 expanders, so it NEVER re-opens the bus; main.cpp coordinates
 *    recovery of that bus using consecutiveFailures()).
 * A single reading is absolute within one turn only; the steering controller
 * combines the two sensors into a multi-turn position.
 */
class AS5600Sensor {
public:
    /**
     * @param bus           The I2C bus the sensor is wired to (already opened by main.cpp)
     * @param label         Short name used in log lines ("fine" / "coarse")
     * @param mayRecoverBus true only when this sensor is the sole device on its bus;
     *                      then it may tear down and restart the bus after
     *                      AS5600_RECOVER_AFTER_FAILS failed reads in a row
     */
    AS5600Sensor(TwoWire& bus, const char* label, bool mayRecoverBus);

    /**
     * @brief Verify the sensor responds on the (already opened) bus.
     *
     * The bus is opened once by `main.cpp`, so this driver never calls
     * `begin()` on it. The pins are remembered only for the wedged-bus
     * recovery path (used only when mayRecoverBus is true).
     * A sensor that does not answer here is re-probed by read() every
     * STEER_SENSOR_REPROBE_MS, so a late-connected cable starts working
     * without a reboot.
     *
     * @param sdaPin GPIO pin for SDA (must match the bus main.cpp opened)
     * @param sclPin GPIO pin for SCL
     * @return true if the sensor ACKs on the bus
     */
    bool begin(gpio_num_t sdaPin, gpio_num_t sclPin);

    /**
     * @brief Read magnet status and raw angle in a single I2C burst.
     * STATUS (0x0B) is directly followed by RAW ANGLE (0x0C/0x0D), so one
     * 3-byte read returns both — the magnet check costs no extra transaction.
     * If the sensor never answered, this performs a rate-limited re-probe
     * instead (non-blocking beyond one address probe) and returns false.
     * @param rawAngle Out: angle in counts 0-4095
     * @param magnetOk Out: true if a magnet is detected (STATUS MD bit)
     * @return true if the I2C read succeeded (outputs valid only then)
     */
    bool read(uint16_t& rawAngle, bool& magnetOk);

    /** @brief True once the sensor has answered a probe */
    bool isPresent() const { return initialized_; }

    /**
     * @brief Failed bus transactions in a row (reads or re-probes), saturating.
     * Used by main.cpp's coordinated I2C2 recovery for the coarse sensor.
     */
    uint16_t consecutiveFailures() const { return failCount_; }

private:
    TwoWire& bus_;
    const char* label_;
    bool mayRecoverBus_;
    bool initialized_;
    gpio_num_t sdaPin_;
    gpio_num_t sclPin_;
    uint16_t failCount_;      // consecutive failed transactions (saturating)
    uint32_t lastProbeMs_;    // last re-probe attempt while not initialized

    /** @brief Address-only probe; true if the sensor ACKs */
    bool probe();

    /** @brief Count one failed transaction; run the own-bus recovery if allowed */
    void noteFailure();

    /** @brief Read consecutive registers; returns true on I2C success */
    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);

    /**
     * @brief Re-initialize the I2C bus after persistent failures (fine sensor only).
     * The ESP32 NG I2C driver can wedge into ESP_ERR_INVALID_STATE after a
     * corrupted transaction (e.g. EMI glitch) and stays broken until the bus is
     * torn down and restarted. Never called when mayRecoverBus is false.
     */
    void recoverBus();
};

#endif // AS5600_SENSOR_H
