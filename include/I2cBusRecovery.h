#ifndef I2C_BUS_RECOVERY_H
#define I2C_BUS_RECOVERY_H

#include <Arduino.h>

/**
 * @brief The single main-owned I2C2 (Wire1, GPIO48/47) recovery routine (change
 * add-event-journal-and-link-recovery D11). It is the ONLY code that restarts Wire1 after
 * setup(); expander drivers never call Wire1.end()/begin().
 *
 * recover() acts only on bus evidence — SDA or SCL reads low, or the caller's last Wire error
 * was a timeout (5). A plain NACK on a healthy bus (device unplugged) does not restart Wire1:
 * the caller re-initializes its own device only. On this core (arduino-esp32 3.3.x, ESP-IDF 5.5
 * `i2c_master` driver) a data NACK on transmit is reported as 4 ("other"), so 4 is NOT treated
 * as bus evidence; the line levels are.
 *
 * Procedure: Wire1.end(); if SDA is low and SCL high, bit-bang up to I2C_UNLOCK_CLOCKS SCL
 * pulses (open drain, ~100 kHz) until SDA is released, then a STOP; then
 * Wire1.begin(PIN_RELAY_SDA, PIN_RELAY_SCL, I2C_BUS_FREQ_HZ) + Wire1.setTimeOut(I2C_TIMEOUT_MS).
 * Rate-limited to one run per I2C_RECOVER_MIN_MS across all callers; every run is journaled
 * (`I2C W bus_stuck`) and counted (`i2c_unlock`). Call from the main loop only.
 */
namespace I2cBusRecovery {

/** @brief Wire error codes that, by themselves, are bus evidence (timeout) */
bool isBusError(uint8_t wireErr);

/**
 * @brief Recover I2C2 if there is bus evidence.
 * @param by      caller token for the journal ("relay", "inputs")
 * @param lastErr the caller's last Wire error code (0 = none / unknown)
 * @return true if Wire1 was restarted (the caller must re-initialize its device)
 */
bool recover(const char* by, uint8_t lastErr);

/** @brief Wire1.begin() on the I2C2 pins + the transaction timeout (setup() and recover()) */
bool beginBus();

/** @brief Session total of bus recoveries (Wire1 restarts) */
uint32_t count();

}  // namespace I2cBusRecovery

#endif // I2C_BUS_RECOVERY_H
