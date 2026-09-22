#ifndef IMOTOR_DRIVER_H
#define IMOTOR_DRIVER_H

#include <Arduino.h>

/**
 * @brief Abstraction for a bidirectional motor driver.
 *
 * The steering position loop drives its actuator through this interface so it
 * is independent of the concrete driver (BTS7960 H-bridge or VESC over UART).
 * `setSpeed()` takes a signed effort in -255..+255 (positive = wheel right);
 * `stop()` coasts the motor (zero drive).
 *
 * The remaining hooks are optional telemetry/servicing methods with safe
 * no-op defaults so simple drivers (e.g. the BTS7960 brake driver) need not
 * implement them. The VESC driver overrides them to expose UART telemetry and
 * a non-blocking service pump.
 */
class IMotorDriver {
public:
    virtual ~IMotorDriver() {}

    /** @brief Set drive effort: -255 (full left/reverse) .. +255 (full right/forward), 0 = coast */
    virtual void setSpeed(int16_t speed) = 0;

    /** @brief Coast the motor (zero drive) */
    virtual void stop() = 0;

    // === Optional telemetry / servicing hooks (default no-op) ===

    /** @brief Non-blocking service: pump UART RX and issue periodic telemetry requests */
    virtual void poll() {}

    /** @brief True when the driver link is healthy (always true for drivers without a link) */
    virtual bool driverOk() const { return true; }

    /** @brief True when the driver reports a hardware fault */
    virtual bool hasFault() const { return false; }

    /** @brief Last reported motor current in amps (0 if unavailable) */
    virtual float motorCurrentA() const { return 0.0f; }

    /** @brief Last reported power-stage (FET) temperature in °C (0 if unavailable) */
    virtual float fetTempC() const { return 0.0f; }

    /** @brief Last reported input voltage in volts (0 if unavailable) */
    virtual float inputVoltageV() const { return 0.0f; }

    /** @brief Last reported driver fault code (0 = no fault) */
    virtual uint8_t faultCode() const { return 0; }
};

#endif // IMOTOR_DRIVER_H
