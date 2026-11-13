#include "VescMotorDriver.h"
#include "Constants.h"
#include "Debug.h"
#include "EventJournal.h"
#include <driver/gpio.h>

VescMotorDriver::VescMotorDriver(uint8_t uartNum)
    : serial_(uartNum),
      initialized_(false),
      lastSpeed_(0),
      lastRequestTime_(0),
      telemPeriodMs_(STEER_VESC_TELEM_MS),
      haveReply_(false),
      lastValidReplyTime_(0),
      parser_(VESC_RX_GAP_MS),
      linkWasOk_(false),
      stateAtLoss_("-") {
}

bool VescMotorDriver::begin(gpio_num_t rxPin, gpio_num_t txPin, uint32_t baud) {
    serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
    // Weak internal pull-up on the RX pad so the line idles HIGH (UART idle level) instead of
    // floating between bytes or while the VESC is unpowered / unplugged. On aztec X1 (GPIO44) has
    // only the 510 R series resistor — no pull-up, capacitor or ESD on the board (change
    // gate-obd-watchdog-and-pull-up-vesc-rx). Applied AFTER begin(), never via pinMode() before
    // it: Arduino-ESP32 3.3.x begin() -> uartSetPins() hands the pad to the UART through the
    // peripheral manager and IDF 5.5 uart_set_pin(), whose RX path (IOMUX for UART0/GPIO44,
    // GPIO matrix otherwise) only enables the input and enables NO pull-up (checked in the
    // prebuilt libesp_driver_uart.a: its only gpio_pullup_en() is on the CTS pin), so after
    // begin() the pad's pull is whatever reset / earlier code left. The pull bits live in the
    // IO_MUX pad register, independent of the pad's function, so setting them here keeps the
    // UART routing intact. Nothing re-opens this UART at runtime (link recovery only flushes the
    // RX buffer and resets the parser in poll()), so begin() is the only place that needs it.
    gpio_set_pull_mode(rxPin, GPIO_PULLUP_ONLY);
    gpio_io_config_t io = {};
    const bool pullUp = (gpio_get_io_config(rxPin, &io) == ESP_OK) && io.pu && !io.pd;
    if (!pullUp) {
        EventJournal::log(JSrc::VESC, JLvl::W, "rx_pullup", "pin=%d ok=0", (int)rxPin);
    }
    initialized_ = true;
    lastRequestTime_ = millis();
    Debug::printfFeature(DebugFeature::SERVO,
                         "[VESC] UART%d started: RX=%d (pull-up %s) TX=%d @ %lu baud\n",
                         VESC_UART_NUM, (int)rxPin, pullUp ? "on" : "FAILED", (int)txPin,
                         (unsigned long)baud);
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
    if (now - lastRequestTime_ >= telemPeriodMs_) {
        lastRequestTime_ = now;
        sendGetValues();
    }

    pumpRx();   // consume only what is already buffered — never busy-waits

    // Link-lost transition: discard whatever sits in the UART RX buffer and restart the
    // parser, so the next GET_VALUES reply is received from a clean start.
    const bool ok = driverOk();
    if (linkWasOk_ && !ok) {
        stateAtLoss_ = VescRxParser::stateToken(parser_.state());
        while (serial_.available() > 0) {
            (void)serial_.read();
        }
        parser_.reset();
    }
    linkWasOk_ = ok;
}

IMotorDriver::LinkStats VescMotorDriver::linkStats() const {
    const VescRxParser::Stats& st = parser_.stats();
    LinkStats ls;
    ls.bytes = st.bytes;
    ls.good = st.good;
    ls.dropCrc = st.dropCrc;
    ls.dropStop = st.dropStop;
    ls.resync = st.resync;
    ls.bytesSinceGood = st.bytesSinceGood;
    ls.state = VescRxParser::stateToken(parser_.state());
    ls.stateAtLoss = stateAtLoss_;
    return ls;
}

void VescMotorDriver::pumpRx() {
    // Process every byte the UART has already received. Bounded by the hardware
    // FIFO + driver RX buffer; this reads what is present and returns — it does
    // not wait for more data, so the main loop cannot block here. The bytes are
    // timestamped when the loop reads them (VescRxParser gap rule, design D9).
    const uint32_t now = millis();
    while (serial_.available() > 0) {
        const uint8_t b = (uint8_t)serial_.read();
        if (parser_.feed(b, now)) {
            handlePacket(parser_.payload(), parser_.length());
        }
    }

    VescRxParser::State st;
    uint32_t idleMs = 0;
    if (parser_.takeResync(st, idleMs)) {
        const VescRxParser::Stats& s = parser_.stats();
        EventJournal::log(JSrc::VESC, JLvl::W, "resync", "state=%s idle=%lu bytes_since_good=%lu drop=%lu",
                          VescRxParser::stateToken(st), (unsigned long)idleMs,
                          (unsigned long)s.bytesSinceGood, (unsigned long)(s.dropCrc + s.dropStop));
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
