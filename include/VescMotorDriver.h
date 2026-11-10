#ifndef VESC_MOTOR_DRIVER_H
#define VESC_MOTOR_DRIVER_H

#include <Arduino.h>
#include "IMotorDriver.h"
#include "VescProtocol.h"
#include "VescRxParser.h"
#include "Constants.h"

/**
 * @brief IMotorDriver backed by a Flipsky 75200 VESC over a dedicated UART.
 *
 * `setSpeed(-255..+255)` maps to COMM_SET_DUTY (duty = speed/255, clamped
 * -1..+1); `stop()` sends zero duty (coast). `poll()` issues COMM_GET_VALUES at
 * ~STEER_VESC_TELEM_MS (setTelemetryPeriodMs() changes it) and pumps the UART receiver.
 *
 * The receiver is fully NON-BLOCKING: `poll()` only consumes bytes already
 * buffered by the UART (VescRxParser, a byte-pump state machine with a VESC_RX_GAP_MS
 * inter-byte resync) and never busy-waits on serial, so the main loop can never block on
 * the VESC. On the driverOk() true -> false transition the UART RX buffer is discarded and
 * the parser reset, so the next request starts from a clean receiver.
 *
 * `driverOk()` is true only while a valid COMM_GET_VALUES reply has been seen
 * within STEER_VESC_COMM_TIMEOUT_MS, so an absent/silent VESC reads as "down"
 * without blocking boot.
 */
class VescMotorDriver : public IMotorDriver {
public:
    explicit VescMotorDriver(uint8_t uartNum);

    /** @brief Open the UART (8N1) on the given pins/baud. Always structurally succeeds. */
    bool begin(gpio_num_t rxPin, gpio_num_t txPin, uint32_t baud);

    // IMotorDriver
    void setSpeed(int16_t speed) override;
    void stop() override;
    void poll() override;
    bool driverOk() const override;
    bool hasFault() const override { return values_.faultCode != 0; }
    float motorCurrentA() const override { return values_.motorCurrentA; }
    float fetTempC() const override { return values_.fetTempC; }
    float inputVoltageV() const override { return values_.inputVoltageV; }
    uint8_t faultCode() const override { return values_.faultCode; }
    int16_t lastSpeed() const override { return lastSpeed_; }
    /** @brief COMM_GET_VALUES period: STEER_VESC_TELEM_MS normally, STEER_DIAG_VESC_TELEM_MS in USB diag mode */
    void setTelemetryPeriodMs(uint32_t ms) override { telemPeriodMs_ = ms ? ms : STEER_VESC_TELEM_MS; }
    LinkStats linkStats() const override;

private:
    void sendGetValues();
    void pumpRx();
    void handlePacket(const uint8_t* payload, uint16_t len);

    HardwareSerial serial_;
    bool initialized_;

    // Command state
    int16_t lastSpeed_;
    uint32_t lastRequestTime_;
    uint32_t telemPeriodMs_;

    // Telemetry state
    VescProtocol::Values values_;
    bool haveReply_;
    uint32_t lastValidReplyTime_;

    // Non-blocking receiver (include/VescRxParser.h) + link-loss bookkeeping
    VescRxParser parser_;
    bool linkWasOk_;
    const char* stateAtLoss_;
};

#endif // VESC_MOTOR_DRIVER_H
