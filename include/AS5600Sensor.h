#ifndef AS5600_SENSOR_H
#define AS5600_SENSOR_H

#include <Arduino.h>

/**
 * @brief Minimal AS5600 absolute magnetic angle sensor driver (I2C)
 *
 * Reads the 12-bit RAW ANGLE register (0-4095, ~0.088°/LSB) and the
 * magnet-detect (MD) status bit. Fixed I2C address 0x36.
 * The magnet sits on the steering shaft; travel is < 360° lock-to-lock,
 * so a single reading is fully absolute.
 */
class AS5600Sensor {
public:
    AS5600Sensor();

    /**
     * @brief Initialize the I2C bus and verify the sensor responds
     * @param sdaPin GPIO pin for SDA
     * @param sclPin GPIO pin for SCL
     * @return true if the sensor ACKs on the bus
     */
    bool begin(gpio_num_t sdaPin, gpio_num_t sclPin);

    /**
     * @brief Read magnet status and raw angle in a single I2C burst.
     * STATUS (0x0B) is directly followed by RAW ANGLE (0x0C/0x0D), so one
     * 3-byte read returns both — the magnet check costs no extra transaction.
     * @param rawAngle Out: angle in counts 0-4095
     * @param magnetOk Out: true if a magnet is detected (STATUS MD bit)
     * @return true if the I2C read succeeded (outputs valid only then)
     */
    bool read(uint16_t& rawAngle, bool& magnetOk);

private:
    bool initialized_;
    gpio_num_t sdaPin_;
    gpio_num_t sclPin_;
    uint8_t failCount_;   // consecutive failed reads (triggers bus recovery)

    /** @brief Read consecutive registers; returns true on I2C success */
    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);

    /**
     * @brief Re-initialize the I2C bus after persistent failures.
     * The ESP32 NG I2C driver can wedge into ESP_ERR_INVALID_STATE after a
     * corrupted transaction (e.g. EMI glitch) and stays broken until Wire is
     * torn down and restarted.
     */
    void recoverBus();
};

#endif // AS5600_SENSOR_H
