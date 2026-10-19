#include "AS5600Sensor.h"
#include "Constants.h"
#include "Debug.h"

static constexpr uint8_t  AS5600_I2C_ADDR    = 0x36;
static constexpr uint8_t  AS5600_REG_STATUS  = 0x0B;   // MD bit 5 = magnet detected
static constexpr uint8_t  AS5600_REG_RAW_ANGLE = 0x0C; // 0x0C hi / 0x0D lo, 12-bit

AS5600Sensor::AS5600Sensor(TwoWire& bus, const char* label, bool mayRecoverBus)
    : bus_(bus),
      label_(label),
      mayRecoverBus_(mayRecoverBus),
      initialized_(false),
      sdaPin_(GPIO_NUM_NC),
      sclPin_(GPIO_NUM_NC),
      failCount_(0),
      lastProbeMs_(0) {
}

bool AS5600Sensor::begin(gpio_num_t sdaPin, gpio_num_t sclPin) {
    sdaPin_ = sdaPin;
    sclPin_ = sclPin;
    // The bus is owned by main.cpp, so this driver must NOT open or re-configure
    // it. Calling begin() here would let a steering init failure disturb a bus
    // main.cpp already owns (and, on I2C2, the expanders sharing it).
    lastProbeMs_ = millis();
    if (!probe()) {
        Debug::printfFeature(DebugFeature::SERVO,
                             "[AS5600] %s sensor not responding at 0x36 — will re-probe every %d ms\n",
                             label_, STEER_SENSOR_REPROBE_MS);
        return false;
    }

    initialized_ = true;

    uint16_t raw;
    bool magnetOk = false;
    read(raw, magnetOk);
    Debug::printfFeature(DebugFeature::SERVO, "[AS5600] %s initialized: SDA=%d, SCL=%d, magnet %s\n",
                         label_, sdaPin, sclPin, magnetOk ? "OK" : "NOT DETECTED");
    return true;
}

bool AS5600Sensor::probe() {
    bus_.beginTransmission(AS5600_I2C_ADDR);
    return bus_.endTransmission() == 0;
}

void AS5600Sensor::noteFailure() {
    if (failCount_ < UINT16_MAX) failCount_++;
    // Only the sole owner of a bus may restart it; the coarse sensor on the
    // shared I2C2 just reports, and main.cpp decides whether the bus is wedged.
    if (mayRecoverBus_ && failCount_ >= AS5600_RECOVER_AFTER_FAILS) {
        recoverBus();
        failCount_ = 0;
    }
}

bool AS5600Sensor::read(uint16_t& rawAngle, bool& magnetOk) {
    if (!initialized_) {
        // Absent at boot: re-probe at a slow cadence (one address-only
        // transaction) so a late-plugged cable starts working without a reboot.
        uint32_t now = millis();
        if (now - lastProbeMs_ < STEER_SENSOR_REPROBE_MS) return false;
        lastProbeMs_ = now;
        if (!probe()) {
            noteFailure();
            return false;
        }
        initialized_ = true;
        failCount_ = 0;
        Debug::printfFeature(DebugFeature::SERVO, "[AS5600] %s sensor now responding at 0x36\n", label_);
        // fall through to a normal read
    }

    // Burst read STATUS (0x0B) + RAW ANGLE hi/lo (0x0C/0x0D) in one transaction
    uint8_t buf[3];
    if (!readRegisters(AS5600_REG_STATUS, buf, 3)) {
        noteFailure();
        return false;
    }
    failCount_ = 0;
    magnetOk = (buf[0] & 0x20) != 0;   // MD bit
    rawAngle = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];
    return true;
}

void AS5600Sensor::recoverBus() {
    // The re-init must restore the exact parameters main.cpp opened the bus with:
    // main.cpp owns both buses, and a device added to X14 later (the contingency
    // relay-board home) must not find I2C1 running at different parameters.
    Debug::printfFeature(DebugFeature::SERVO, "[AS5600] %s: I2C wedged — re-initializing its own bus\n", label_);
    bus_.end();
    bus_.begin(sdaPin_, sclPin_, I2C_BUS_FREQ_HZ);
}

bool AS5600Sensor::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
    bus_.beginTransmission(AS5600_I2C_ADDR);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;

    if (bus_.requestFrom(AS5600_I2C_ADDR, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = bus_.read();
    }
    return true;
}
