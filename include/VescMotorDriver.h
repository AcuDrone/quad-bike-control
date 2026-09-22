#ifndef VESC_MOTOR_DRIVER_H
#define VESC_MOTOR_DRIVER_H

#include <Arduino.h>
#include "IMotorDriver.h"
#include "VescProtocol.h"

/**
 * @brief IMotorDriver backed by a Flipsky 75200 VESC over a dedicated UART.
 *
 * `setSpeed(-255..+255)` maps to COMM_SET_DUTY (duty = speed/255, clamped
 * -1..+1); `stop()` sends zero duty (coast). `poll()` issues COMM_GET_VALUES at
 * ~STEER_VESC_TELEM_MS and pumps the UART receiver.
 *
 * The receiver is fully NON-BLOCKING: `poll()` only consumes bytes already
 * buffered by the UART (a byte-pump state machine) and never busy-waits on
 * serial, so the main loop can never block on the VESC.
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

private:
    void sendGetValues();
    void pumpRx();
    void handlePacket(const uint8_t* payload, uint16_t len);

    HardwareSerial serial_;
    bool initialized_;

    // Command state
    int16_t lastSpeed_;
    uint32_t lastRequestTime_;

    // Telemetry state
    VescProtocol::Values values_;
    bool haveReply_;
    uint32_t lastValidReplyTime_;

    // Non-blocking receive state machine
    enum RxState : uint8_t {
        RX_START,      // waiting for 0x02 (short) or 0x03 (long) start byte
        RX_LEN_SHORT,  // reading 1-byte length
        RX_LEN_HI,     // reading high byte of 2-byte length
        RX_LEN_LO,     // reading low byte of 2-byte length
        RX_PAYLOAD,    // reading payload bytes
        RX_CRC_HI,
        RX_CRC_LO,
        RX_END,        // expecting 0x03 stop byte
    };
    RxState rxState_;
    static constexpr uint16_t RX_BUF_SIZE = 256;
    uint8_t rxPayload_[RX_BUF_SIZE];
    uint16_t rxLen_;        // declared payload length
    uint16_t rxIdx_;        // bytes of payload read so far
    uint16_t rxCrc_;        // received CRC
};

#endif // VESC_MOTOR_DRIVER_H
