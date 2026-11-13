#ifndef VESC_RX_PARSER_H
#define VESC_RX_PARSER_H

/**
 * @file VescRxParser.h
 * @brief Header-only, Arduino-free VESC UART packet receiver (byte-pump state machine) with an
 * inter-byte gap resync and link counters. Used by VescMotorDriver; native-tested in
 * test/test_vesc_rx.
 *
 * Framing (VESC serial protocol):
 *   short packet (payload <= 255): 0x02, len, payload..., crc_hi, crc_lo, 0x03
 *   long  packet (payload <= 65535): 0x03, len_hi, len_lo, payload..., crc_hi, crc_lo, 0x03
 * CRC is CRC16-CCITT/XMODEM (poly 0x1021, init 0) over the payload only. Payloads longer than
 * RX_BUF_SIZE are rejected (resync).
 *
 * Gap resync (change add-event-journal-and-link-recovery D9): when a byte arrives more than
 * `gapMs` after the previous one while a frame is incomplete, the parser first returns to the
 * start-byte search. Without it one lost byte makes every following reply's header be swallowed
 * by the previous reply's 0x03 stop byte (which is also the long-frame start byte), and the
 * misalignment can repeat reply after reply for minutes. `gapMs = 0` disables the rule (tests).
 */

#include <stdint.h>
#include <stddef.h>

class VescRxParser {
public:
    static constexpr uint16_t RX_BUF_SIZE = 256;

    enum State : uint8_t {
        RX_START,      // waiting for 0x02 (short) or 0x03 (long) start byte
        RX_LEN_SHORT,  // reading 1-byte length
        RX_LEN_HI,     // reading high byte of 2-byte length
        RX_LEN_LO,     // reading low byte of 2-byte length
        RX_PAYLOAD,    // reading payload bytes
        RX_CRC_HI,
        RX_CRC_LO,
        RX_END,        // expecting 0x03 stop byte
    };

    /** @brief Link counters (session totals; bytesSinceGood resets on every good packet) */
    struct Stats {
        uint32_t bytes = 0;           // bytes fed
        uint32_t good = 0;            // packets with a valid CRC and stop byte
        uint32_t dropCrc = 0;         // packets dropped for a CRC mismatch
        uint32_t dropStop = 0;        // packets dropped for a wrong stop byte
        uint32_t resync = 0;          // gap resyncs (incomplete frame abandoned)
        uint32_t bytesSinceGood = 0;  // bytes fed since the last good packet
    };

    explicit VescRxParser(uint32_t gapMs = 5) : gapMs_(gapMs) {}

    /** @brief CRC16-CCITT/XMODEM (matches VescProtocol::crc16 / VESC bldc crc16()) */
    static uint16_t crc16(const uint8_t* buf, uint16_t len) {
        uint16_t c = 0;
        for (uint16_t i = 0; i < len; i++) {
            c ^= (uint16_t)buf[i] << 8;
            for (uint8_t j = 0; j < 8; j++) {
                c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
            }
        }
        return c;
    }

    /**
     * @brief Feed one received byte.
     * @param nowMs time the byte was read (ms)
     * @return true when this byte completed a good packet (payload()/length() valid until the
     *         next feed())
     */
    bool feed(uint8_t b, uint32_t nowMs) {
        stats_.bytes++;
        stats_.bytesSinceGood++;

        if (gapMs_ != 0 && state_ != RX_START && haveByte_ &&
            (uint32_t)(nowMs - lastByteMs_) > gapMs_) {
            lastResyncState_ = state_;
            lastResyncIdleMs_ = nowMs - lastByteMs_;
            stats_.resync++;
            resyncPending_ = true;
            state_ = RX_START;
        }
        lastByteMs_ = nowMs;
        haveByte_ = true;

        switch (state_) {
            case RX_START:
                if (b == 0x02) {
                    state_ = RX_LEN_SHORT;
                } else if (b == 0x03) {
                    state_ = RX_LEN_HI;
                }
                // any other byte: stay hunting for a start byte
                break;

            case RX_LEN_SHORT:
                len_ = b;
                idx_ = 0;
                state_ = (len_ == 0 || len_ > RX_BUF_SIZE) ? RX_START : RX_PAYLOAD;
                break;

            case RX_LEN_HI:
                len_ = (uint16_t)b << 8;
                state_ = RX_LEN_LO;
                break;

            case RX_LEN_LO:
                len_ |= b;
                idx_ = 0;
                state_ = (len_ == 0 || len_ > RX_BUF_SIZE) ? RX_START : RX_PAYLOAD;
                break;

            case RX_PAYLOAD:
                buf_[idx_++] = b;
                if (idx_ >= len_) state_ = RX_CRC_HI;
                break;

            case RX_CRC_HI:
                crc_ = (uint16_t)b << 8;
                state_ = RX_CRC_LO;
                break;

            case RX_CRC_LO:
                crc_ |= b;
                state_ = RX_END;
                break;

            case RX_END:
                state_ = RX_START;   // packet complete (or dropped) — hunt for the next start
                if (b != 0x03) {
                    stats_.dropStop++;
                    return false;
                }
                if (crc16(buf_, len_) != crc_) {
                    stats_.dropCrc++;
                    return false;
                }
                stats_.good++;
                stats_.bytesSinceGood = 0;
                return true;
        }
        return false;
    }

    /** @brief Back to the start-byte search (counters kept) */
    void reset() {
        state_ = RX_START;
        idx_ = 0;
        len_ = 0;
        haveByte_ = false;
    }

    const uint8_t* payload() const { return buf_; }
    uint16_t length() const { return len_; }
    State state() const { return state_; }
    const Stats& stats() const { return stats_; }
    uint32_t lastByteMs() const { return lastByteMs_; }

    /** @brief A resync happened since the last call; fills its state and idle time */
    bool takeResync(State& stateOut, uint32_t& idleMsOut) {
        if (!resyncPending_) return false;
        resyncPending_ = false;
        stateOut = lastResyncState_;
        idleMsOut = lastResyncIdleMs_;
        return true;
    }

    static const char* stateToken(State s) {
        switch (s) {
            case RX_START:     return "START";
            case RX_LEN_SHORT: return "LEN";
            case RX_LEN_HI:    return "LEN_HI";
            case RX_LEN_LO:    return "LEN_LO";
            case RX_PAYLOAD:   return "PAYLOAD";
            case RX_CRC_HI:    return "CRC_HI";
            case RX_CRC_LO:    return "CRC_LO";
            case RX_END:       return "END";
        }
        return "?";
    }

private:
    uint32_t gapMs_;
    State state_ = RX_START;
    uint8_t buf_[RX_BUF_SIZE] = {0};
    uint16_t len_ = 0;
    uint16_t idx_ = 0;
    uint16_t crc_ = 0;
    uint32_t lastByteMs_ = 0;
    bool haveByte_ = false;
    Stats stats_;
    bool resyncPending_ = false;
    State lastResyncState_ = RX_START;
    uint32_t lastResyncIdleMs_ = 0;
};

#endif // VESC_RX_PARSER_H
