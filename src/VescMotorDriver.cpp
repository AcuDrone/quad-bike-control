#include "VescMotorDriver.h"
#include "Constants.h"
#include "Debug.h"

VescMotorDriver::VescMotorDriver(uint8_t uartNum)
    : serial_(uartNum),
      initialized_(false),
      lastSpeed_(0),
      lastRequestTime_(0),
      haveReply_(false),
      lastValidReplyTime_(0),
      rxState_(RX_START),
      rxLen_(0),
      rxIdx_(0),
      rxCrc_(0) {
}

bool VescMotorDriver::begin(gpio_num_t rxPin, gpio_num_t txPin, uint32_t baud) {
    serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
    initialized_ = true;
    lastRequestTime_ = millis();
    Debug::printfFeature(DebugFeature::SERVO,
                         "[VESC] UART%d started: RX=%d TX=%d @ %lu baud\n",
                         VESC_UART_NUM, (int)rxPin, (int)txPin, (unsigned long)baud);
    return true;   // structural success even if the VESC is silent (see driverOk())
}

void VescMotorDriver::setSpeed(int16_t speed) {
    lastSpeed_ = speed;
    if (!initialized_) return;

    float duty = (float)speed / 255.0f;   // clamped inside encodeSetDuty()
    uint8_t payload[5];
    uint8_t plen = VescProtocol::encodeSetDuty(duty, payload);

    uint8_t frame[16];
    uint16_t flen = VescProtocol::frameShort(payload, plen, frame);
    serial_.write(frame, flen);
}

void VescMotorDriver::stop() {
    setSpeed(0);   // zero duty = coast
}

void VescMotorDriver::sendGetValues() {
    if (!initialized_) return;
    uint8_t payload[1] = { VescProtocol::COMM_GET_VALUES };
    uint8_t frame[8];
    uint16_t flen = VescProtocol::frameShort(payload, 1, frame);
    serial_.write(frame, flen);
}

void VescMotorDriver::poll() {
    if (!initialized_) return;

    uint32_t now = millis();
    if (now - lastRequestTime_ >= STEER_VESC_TELEM_MS) {
        lastRequestTime_ = now;
        sendGetValues();
    }

    pumpRx();   // consume only what is already buffered — never busy-waits
}

void VescMotorDriver::pumpRx() {
    // Process every byte the UART has already received. Bounded by the hardware
    // FIFO + driver RX buffer; this reads what is present and returns — it does
    // not wait for more data, so the main loop cannot block here.
    while (serial_.available() > 0) {
        uint8_t b = (uint8_t)serial_.read();

        switch (rxState_) {
            case RX_START:
                if (b == 0x02) {
                    rxState_ = RX_LEN_SHORT;
                } else if (b == 0x03) {
                    rxState_ = RX_LEN_HI;
                }
                // any other byte: stay hunting for a start byte (resync)
                break;

            case RX_LEN_SHORT:
                rxLen_ = b;
                rxIdx_ = 0;
                if (rxLen_ == 0 || rxLen_ > RX_BUF_SIZE) {
                    rxState_ = RX_START;   // implausible length: resync
                } else {
                    rxState_ = RX_PAYLOAD;
                }
                break;

            case RX_LEN_HI:
                rxLen_ = (uint16_t)b << 8;
                rxState_ = RX_LEN_LO;
                break;

            case RX_LEN_LO:
                rxLen_ |= b;
                rxIdx_ = 0;
                if (rxLen_ == 0 || rxLen_ > RX_BUF_SIZE) {
                    rxState_ = RX_START;
                } else {
                    rxState_ = RX_PAYLOAD;
                }
                break;

            case RX_PAYLOAD:
                rxPayload_[rxIdx_++] = b;
                if (rxIdx_ >= rxLen_) rxState_ = RX_CRC_HI;
                break;

            case RX_CRC_HI:
                rxCrc_ = (uint16_t)b << 8;
                rxState_ = RX_CRC_LO;
                break;

            case RX_CRC_LO:
                rxCrc_ |= b;
                rxState_ = RX_END;
                break;

            case RX_END:
                if (b == 0x03 &&
                    VescProtocol::crc16(rxPayload_, rxLen_) == rxCrc_) {
                    handlePacket(rxPayload_, rxLen_);
                }
                rxState_ = RX_START;   // packet complete (or dropped) — resync
                break;
        }
    }
}

void VescMotorDriver::handlePacket(const uint8_t* payload, uint16_t len) {
    VescProtocol::Values v;
    if (VescProtocol::decodeGetValues(payload, len, v)) {
        values_ = v;
        haveReply_ = true;
        lastValidReplyTime_ = millis();
    }
}

bool VescMotorDriver::driverOk() const {
    if (!haveReply_) return false;   // never blocks boot: silent VESC = not ok
    return (millis() - lastValidReplyTime_) < STEER_VESC_COMM_TIMEOUT_MS;
}
