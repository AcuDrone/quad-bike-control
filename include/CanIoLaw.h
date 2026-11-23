#ifndef CAN_IO_LAW_H
#define CAN_IO_LAW_H

/**
 * @file CanIoLaw.h
 * @brief Pure logic of the MCP2515 CAN task (header-only, no Arduino includes; native-tested in
 *        test/test_can_io). Change move-can-rx-to-interrupt, design D3, D4, D6, D9.
 *
 * - CanRing / CanFrameRing: the fixed-size rings between the `can_io` task and loop()
 *   (receive: drop the oldest when full; transmit / events: reject the newest when full).
 *   No locking inside: the firmware guards every ring with its own portMUX.
 * - canTxHeader(): TXBnSIDH..TXBnDLC bytes, the same bits as mcp2515_write_id() in the pinned
 *   mcp_can library (mcp_can.cpp:690-716), so the task can load TXB0 with one SPI WRITE.
 * - READ STATUS decoding, the TX timeout decision, the post-reset liveness gate and the INT
 *   wake-up state.
 * All time arithmetic is millis()-wrap safe (unsigned differences only).
 */

#include <stdint.h>
#include <string.h>

/** One CAN frame. `id` keeps the readMsgBuf() encoding: bit 31 = extended, bit 30 = remote. */
struct CanFrame {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
};

/**
 * Fixed ring of N elements (N <= 255). Not thread-safe by itself.
 * pushDropOldest() never fails (the receive ring: the newest frames are the useful ones);
 * pushReject() refuses when full (the transmit and event rings).
 */
template <typename T, uint8_t N>
struct CanRing {
    static_assert(N > 0, "CanRing needs at least one slot");

    T buf[N];
    uint8_t head = 0;      // next write
    uint8_t tail = 0;      // next read
    uint8_t n = 0;         // elements stored
    uint8_t hwm_ = 0;      // highest n seen
    uint32_t drops_ = 0;   // elements discarded by pushDropOldest()

    /** @return true when the oldest element was discarded to make room */
    bool pushDropOldest(const T& v) {
        bool dropped = false;
        if (n == N) {
            tail = (uint8_t)((tail + 1) % N);
            n--;
            drops_++;
            dropped = true;
        }
        put(v);
        return dropped;
    }

    /** @return false (content unchanged) when the ring is full */
    bool pushReject(const T& v) {
        if (n == N) return false;
        put(v);
        return true;
    }

    bool pop(T& out) {
        if (n == 0) return false;
        out = buf[tail];
        tail = (uint8_t)((tail + 1) % N);
        n--;
        return true;
    }

    uint8_t count() const { return n; }
    uint8_t hwm() const { return hwm_; }
    uint32_t drops() const { return drops_; }
    static constexpr uint8_t capacity() { return N; }

private:
    void put(const T& v) {
        buf[head] = v;
        head = (uint8_t)((head + 1) % N);
        n++;
        if (n > hwm_) hwm_ = n;
    }
};

template <uint8_t N>
using CanFrameRing = CanRing<CanFrame, N>;

/**
 * Single-entry hand-off slot (task -> loop) for results that must never be lost: one command
 * result at a time, one recovery report. post() overwrites (the newest result wins; the protocol
 * never has two outstanding), take() empties it. No locking inside (the firmware holds a portMUX).
 */
template <typename T>
struct CanSlot {
    T value;
    bool full = false;
    void post(const T& v) { value = v; full = true; }
    bool take(T& out) {
        if (!full) return false;
        out = value;
        full = false;
        return true;
    }
};

namespace CanIoLaw {

// TXBnSIDL.EXIDE (mcp_can_dfs.h MCP_TXB_EXIDE_M)
constexpr uint8_t TXB_EXIDE = 0x08;

/**
 * TXBnSIDH, TXBnSIDL, TXBnEID8, TXBnEID0, TXBnDLC for one data frame (no RTR).
 * Same encoding as the library's mcp2515_write_id() (ext: SIDL = id[17:16] | id[20:18] << 5 |
 * EXIDE); the ID is masked to 11 / 29 bits first, which changes nothing for an in-range ID.
 * @param dlc clamped to 8
 */
inline void canTxHeader(uint32_t id, bool ext, uint8_t dlc, uint8_t out[5]) {
    if (ext) {
        id &= 0x1FFFFFFFUL;
        const uint16_t lo = (uint16_t)(id & 0xFFFF);
        const uint16_t hi = (uint16_t)(id >> 16);
        out[0] = (uint8_t)(hi >> 5);
        out[1] = (uint8_t)(((hi & 0x1C) << 3) | (hi & 0x03) | TXB_EXIDE);
        out[2] = (uint8_t)(lo >> 8);
        out[3] = (uint8_t)(lo & 0xFF);
    } else {
        id &= 0x7FFUL;
        out[0] = (uint8_t)(id >> 3);
        out[1] = (uint8_t)((id & 0x07) << 5);
        out[2] = 0;
        out[3] = 0;
    }
    out[4] = dlc > 8 ? 8 : dlc;
}

// READ STATUS (0xA0): bit 0 RX0IF, 1 RX1IF, 2 TX0REQ, 3 TX0IF, 4 TX1REQ, 5 TX1IF, 6 TX2REQ, 7 TX2IF
inline bool rxPending(uint8_t st) { return (st & 0x03) != 0; }
/** @param buf transmit buffer 0..2 */
inline bool txPending(uint8_t st, uint8_t buf) { return buf < 3 && (st & (uint8_t)(0x04 << (2 * buf))) != 0; }

/** A frame loaded at loadedMs has not left the buffer within timeoutMs (wrap-safe, >=). */
inline bool txTimedOut(uint32_t nowMs, uint32_t loadedMs, uint32_t timeoutMs) {
    return (uint32_t)(nowMs - loadedMs) >= timeoutMs;
}

/**
 * Liveness gate after an SPI RESET: CANSTAT.OPMOD (bits 7:5) must show CONFIG (0x80) and CANCTRL
 * its reset value 0x87. A floating or stuck MISO (0x00 / 0xFF) fails here — with 0x00 alone
 * CANSTAT would decode as NORMAL mode (design audit item g).
 */
inline bool mcp2515AliveAfterReset(uint8_t canstat, uint8_t canctrl) {
    return (canstat & 0xE0) == 0x80 && canctrl == 0x87;
}

// TXBnCTRL bits (mcp_can_dfs.h MCP_TXB_ABTF_M / MCP_TXB_TXREQ_M)
constexpr uint8_t TXB_ABTF = 0x40;
constexpr uint8_t TXB_TXREQ = 0x08;

enum class TxOutcome : uint8_t { PENDING, SENT, ABORTED };

/**
 * Fate of the frame in a transmit buffer (design D4). TXREQ still set: pending. TXREQ clear
 * without an abort request: sent. After the task cleared TXREQ to abort, the chip still finishes
 * an attempt already on the wire, so TXREQ clear alone does not mean "not sent": ABTF decides
 * (set = aborted; clear = the attempt on the wire succeeded). ABTF is cleared when TXREQ is set
 * again, so it is never stale here. ABAT is never used.
 * @param ctrl TXBnCTRL read after TXREQ was seen clear (or with it)
 */
inline TxOutcome txOutcome(uint8_t ctrl, bool abortRequested) {
    if (ctrl & TXB_TXREQ) return TxOutcome::PENDING;
    if (abortRequested && (ctrl & TXB_ABTF)) return TxOutcome::ABORTED;
    return TxOutcome::SENT;
}

/** A frame whose requester gave up (cancel sequence) is not loaded (design D4). seq 0 is never used. */
inline bool txCancelled(bool wantResult, uint8_t seq, uint8_t cancelSeq) {
    return wantResult && cancelSeq != 0 && seq == cancelSeq;
}

/** Next TX sequence number: 1..255, never 0 (0 = "no cancel"). */
inline uint8_t nextTxSeq(uint8_t seq) { return (uint8_t)(seq == 255 ? 1 : seq + 1); }

/**
 * The task's own init retry (design D7, extended): the chip is down after any init / re-init
 * attempt (boot, or a failed re-init sequence), no loop command is waiting, and CAN_INIT_RETRY_MS
 * have passed since the last attempt of any kind — so it never interleaves with the loop's
 * re-init attempts, which are CAN_REINIT_RETRY_MS apart.
 */
inline bool initRetryDue(bool chipUp, bool attempted, bool cmdPending, uint32_t nowMs,
                         uint32_t lastTryMs, uint32_t periodMs) {
    return !chipUp && attempted && !cmdPending && (uint32_t)(nowMs - lastTryMs) >= periodMs;
}

enum class IntState : uint8_t { NONE, OK, DEAD };

/**
 * INT wake-up state at the end of a CAN_INT_WINDOW_MS window (design D9).
 * @param prev       state of the previous window (OK at start: unproven)
 * @param hasIntPin  an interrupt is attached (PIN_CAN_INT defined and no CAN_TEST_NO_INT)
 * @param intWakes   task wake-ups caused by the INT line in the window
 * @param frames     frames read in the window
 */
inline IntState canIntState(IntState prev, bool hasIntPin, uint32_t intWakes, uint32_t frames) {
    if (!hasIntPin) return IntState::NONE;
    if (frames == 0) return prev;           // no evidence either way
    return intWakes > 0 ? IntState::OK : IntState::DEAD;
}

/** Telemetry / journal token: "ok", "dead", "none" */
inline const char* intStateToken(IntState s) {
    switch (s) {
        case IntState::OK:   return "ok";
        case IntState::DEAD: return "dead";
        case IntState::NONE:
        default:             return "none";
    }
}

}  // namespace CanIoLaw

#endif  // CAN_IO_LAW_H
