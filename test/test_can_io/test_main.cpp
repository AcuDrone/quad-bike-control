// Native unit tests for the CAN task's pure logic (include/CanIoLaw.h):
// frame ring, MCP2515 TX header encoding, READ STATUS decoding, TX timeout, liveness gate, INT state.
// Run: pio test -e native
#include <unity.h>
#include "CanIoLaw.h"

using namespace CanIoLaw;

void setUp() {}
void tearDown() {}

static CanFrame frame(uint32_t id, uint8_t b0 = 0) {
    CanFrame f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.len = 1;
    f.data[0] = b0;
    return f;
}

// ---- ring -------------------------------------------------------------------------------------

static void test_ring_fifo_order() {
    CanFrameRing<8> r;
    for (uint32_t i = 0; i < 5; i++) TEST_ASSERT_FALSE(r.pushDropOldest(frame(0x100 + i)));
    TEST_ASSERT_EQUAL_UINT8(5, r.count());
    CanFrame f;
    for (uint32_t i = 0; i < 5; i++) {
        TEST_ASSERT_TRUE(r.pop(f));
        TEST_ASSERT_EQUAL_UINT32(0x100 + i, f.id);
    }
    TEST_ASSERT_FALSE(r.pop(f));
    TEST_ASSERT_EQUAL_UINT8(0, r.count());
}

static void test_ring_drop_oldest_keeps_newest_64() {
    CanFrameRing<64> r;
    for (uint32_t i = 0; i < 64; i++) TEST_ASSERT_FALSE(r.pushDropOldest(frame(i)));
    TEST_ASSERT_EQUAL_UINT32(0, r.drops());
    for (uint32_t i = 64; i < 74; i++) TEST_ASSERT_TRUE(r.pushDropOldest(frame(i)));   // 10 dropped
    TEST_ASSERT_EQUAL_UINT32(10, r.drops());
    TEST_ASSERT_EQUAL_UINT8(64, r.count());
    CanFrame f;
    for (uint32_t i = 10; i < 74; i++) {   // the newest 64, oldest first
        TEST_ASSERT_TRUE(r.pop(f));
        TEST_ASSERT_EQUAL_UINT32(i, f.id);
    }
    TEST_ASSERT_FALSE(r.pop(f));
}

static void test_ring_reject_when_full() {
    CanFrameRing<8> r;
    for (uint32_t i = 0; i < 8; i++) TEST_ASSERT_TRUE(r.pushReject(frame(i)));
    TEST_ASSERT_FALSE(r.pushReject(frame(99)));
    TEST_ASSERT_EQUAL_UINT8(8, r.count());
    TEST_ASSERT_EQUAL_UINT32(0, r.drops());
    CanFrame f;
    for (uint32_t i = 0; i < 8; i++) {
        TEST_ASSERT_TRUE(r.pop(f));
        TEST_ASSERT_EQUAL_UINT32(i, f.id);   // content unchanged, 99 never stored
    }
}

static void test_ring_hwm_tracks_maximum() {
    CanFrameRing<64> r;
    CanFrame f;
    for (int i = 0; i < 3; i++) r.pushDropOldest(frame(i));
    TEST_ASSERT_EQUAL_UINT8(3, r.hwm());
    r.pop(f);
    r.pop(f);
    r.pushDropOldest(frame(7));
    TEST_ASSERT_EQUAL_UINT8(3, r.hwm());   // max, not current
    for (int i = 0; i < 100; i++) r.pushDropOldest(frame(i));
    TEST_ASSERT_EQUAL_UINT8(64, r.hwm());
}

static void test_ring_wraparound_1000_pushes() {
    CanFrameRing<64> r;
    CanFrame f;
    uint32_t next = 0;
    for (uint32_t i = 0; i < 1000; i++) {
        r.pushDropOldest(frame(i, (uint8_t)i));
        if (i % 3 == 2) {   // the reader keeps up only partly: 2 pops per 3 pushes
            for (int k = 0; k < 2; k++) {
                TEST_ASSERT_TRUE(r.pop(f));
                TEST_ASSERT_TRUE(f.id >= next);   // monotonic even across drops
                TEST_ASSERT_EQUAL_UINT8((uint8_t)f.id, f.data[0]);
                next = f.id + 1;
            }
        }
    }
    while (r.pop(f)) {
        TEST_ASSERT_TRUE(f.id >= next);
        next = f.id + 1;
    }
    TEST_ASSERT_EQUAL_UINT32(1000, next);   // the very last frame was delivered
    TEST_ASSERT_TRUE(r.drops() > 0);
    TEST_ASSERT_EQUAL_UINT8(0, r.count());
}

// ---- TX header (expected bytes = mcp2515_write_id() of mcp_can 34807287, worked by hand) -------

static void assertHeader(uint32_t id, bool ext, uint8_t dlc, const uint8_t exp[5]) {
    uint8_t h[5];
    canTxHeader(id, ext, dlc, h);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp, h, 5);
}

static void test_tx_header_standard() {
    const uint8_t obd[5] = {0xFB, 0xE0, 0x00, 0x00, 0x08};   // 0x7DF, DLC 8
    assertHeader(0x7DF, false, 8, obd);
    const uint8_t lwsCfg[5] = {0xF8, 0x00, 0x00, 0x00, 0x02};   // 0x7C0, DLC 2
    assertHeader(0x7C0, false, 2, lwsCfg);
    const uint8_t lws[5] = {0x60, 0xA0, 0x00, 0x00, 0x08};   // 0x305: 0x305>>3 = 0x60, (5)<<5 = 0xA0
    assertHeader(0x305, false, 8, lws);
}

static void test_tx_header_extended() {
    // 0x00000901 (VESC STATUS, controller ID 1): EID = 0x0901, SID bits = 0 -> SIDL = EXIDE
    const uint8_t vesc[5] = {0x00, 0x08, 0x09, 0x01, 0x08};
    assertHeader(0x00000901UL, true, 8, vesc);
    // 0x1FFFFFFF: hi = 0x1FFF -> SIDH = 0xFF, SIDL = 0xE0 | 0x03 | 0x08 = 0xEB; EID = 0xFFFF
    const uint8_t all[5] = {0xFF, 0xEB, 0xFF, 0xFF, 0x04};
    assertHeader(0x1FFFFFFFUL, true, 4, all);
    // flag bits from the readMsgBuf() encoding are ignored
    assertHeader(0x80000000UL | 0x00000901UL, true, 8, vesc);
}

static void test_tx_header_dlc_clamped() {
    uint8_t h[5];
    canTxHeader(0x7DF, false, 15, h);
    TEST_ASSERT_EQUAL_HEX8(0x08, h[4]);
    canTxHeader(0x1FFFFFFFUL, true, 9, h);
    TEST_ASSERT_EQUAL_HEX8(0x08, h[4]);
}

// ---- READ STATUS --------------------------------------------------------------------------------

static void test_read_status_bits() {
    TEST_ASSERT_FALSE(rxPending(0x00));
    TEST_ASSERT_TRUE(rxPending(0x01));    // RX0IF
    TEST_ASSERT_TRUE(rxPending(0x02));    // RX1IF
    TEST_ASSERT_TRUE(rxPending(0x03));
    TEST_ASSERT_FALSE(rxPending(0xFC));   // every TX bit, no RX
    TEST_ASSERT_TRUE(txPending(0x04, 0));
    TEST_ASSERT_FALSE(txPending(0x04, 1));
    TEST_ASSERT_TRUE(txPending(0x10, 1));
    TEST_ASSERT_TRUE(txPending(0x40, 2));
    TEST_ASSERT_FALSE(txPending(0x08, 0));   // TX0IF is not TX0REQ
    TEST_ASSERT_FALSE(txPending(0xAB, 0));   // 1010 1011: RX0IF RX1IF TX0IF TX1IF TX2IF
    TEST_ASSERT_TRUE(txPending(0x57, 0));    // 0101 0111: all three TXREQ + RX
    TEST_ASSERT_TRUE(txPending(0x57, 1));
    TEST_ASSERT_TRUE(txPending(0x57, 2));
    TEST_ASSERT_TRUE(rxPending(0x57));
    TEST_ASSERT_FALSE(txPending(0xFF, 3));   // no buffer 3
}

// ---- TX timeout ---------------------------------------------------------------------------------

static void test_tx_timeout() {
    TEST_ASSERT_FALSE(txTimedOut(1019, 1000, 20));
    TEST_ASSERT_TRUE(txTimedOut(1020, 1000, 20));
    TEST_ASSERT_TRUE(txTimedOut(1021, 1000, 20));
    const uint32_t loaded = 0xFFFFFFF6u;   // 10 ms before the millis() wrap
    TEST_ASSERT_FALSE(txTimedOut(0x00000009u, loaded, 20));   // 19 ms
    TEST_ASSERT_TRUE(txTimedOut(0x0000000Au, loaded, 20));    // 20 ms
    TEST_ASSERT_TRUE(txTimedOut(0x0000000Bu, loaded, 20));
}

// ---- liveness gate ------------------------------------------------------------------------------

static void test_liveness_gate() {
    TEST_ASSERT_TRUE(mcp2515AliveAfterReset(0x80, 0x87));
    TEST_ASSERT_TRUE(mcp2515AliveAfterReset(0x81, 0x87));    // ICOD bits do not matter
    TEST_ASSERT_FALSE(mcp2515AliveAfterReset(0x00, 0x00));   // MISO stuck low
    TEST_ASSERT_FALSE(mcp2515AliveAfterReset(0xFF, 0xFF));   // MISO floating high
    TEST_ASSERT_FALSE(mcp2515AliveAfterReset(0x80, 0xFF));
    TEST_ASSERT_FALSE(mcp2515AliveAfterReset(0x00, 0x87));
    TEST_ASSERT_FALSE(mcp2515AliveAfterReset(0x40, 0x87));   // LOOPBACK, not CONFIG
}

// ---- INT state ----------------------------------------------------------------------------------

static void test_int_state() {
    TEST_ASSERT_EQUAL_INT((int)IntState::NONE, (int)canIntState(IntState::OK, false, 100, 100));
    TEST_ASSERT_EQUAL_INT((int)IntState::NONE, (int)canIntState(IntState::NONE, false, 0, 0));
    TEST_ASSERT_EQUAL_INT((int)IntState::DEAD, (int)canIntState(IntState::OK, true, 0, 500));
    TEST_ASSERT_EQUAL_INT((int)IntState::OK, (int)canIntState(IntState::DEAD, true, 1, 500));
    TEST_ASSERT_EQUAL_INT((int)IntState::OK, (int)canIntState(IntState::OK, true, 490, 500));
    TEST_ASSERT_EQUAL_INT((int)IntState::DEAD, (int)canIntState(IntState::DEAD, true, 0, 0));   // no evidence
    TEST_ASSERT_EQUAL_INT((int)IntState::OK, (int)canIntState(IntState::OK, true, 0, 0));
    TEST_ASSERT_EQUAL_STRING("ok", intStateToken(IntState::OK));
    TEST_ASSERT_EQUAL_STRING("dead", intStateToken(IntState::DEAD));
    TEST_ASSERT_EQUAL_STRING("none", intStateToken(IntState::NONE));
}

static void test_ring_pop_empty_after_wrap() {
    CanFrameRing<4> r;
    CanFrame f;
    for (uint32_t i = 0; i < 6; i++) {           // indices wrap past the end once
        TEST_ASSERT_TRUE(r.pushReject(frame(i)));
        TEST_ASSERT_TRUE(r.pop(f));
        TEST_ASSERT_EQUAL_UINT32(i, f.id);
    }
    TEST_ASSERT_FALSE(r.pop(f));                 // empty right after a wrap
    TEST_ASSERT_EQUAL_UINT8(0, r.count());
    TEST_ASSERT_EQUAL_UINT32(5, f.id);           // untouched by the failed pop
}

static void test_tx_ring_reject_at_wrap_boundary() {
    CanFrameRing<8> r;                           // the TX ring size
    CanFrame f;
    for (uint32_t i = 0; i < 5; i++) { r.pushReject(frame(i)); r.pop(f); }   // head/tail at 5
    for (uint32_t i = 0; i < 8; i++) TEST_ASSERT_TRUE(r.pushReject(frame(100 + i)));   // wraps 7 -> 0
    TEST_ASSERT_FALSE(r.pushReject(frame(999)));
    for (uint32_t i = 0; i < 8; i++) {
        TEST_ASSERT_TRUE(r.pop(f));
        TEST_ASSERT_EQUAL_UINT32(100 + i, f.id);
    }
    TEST_ASSERT_FALSE(r.pop(f));
}

// ---- TX outcome: TXREQ + ABTF ------------------------------------------------------------------

static void test_tx_outcome() {
    using CanIoLaw::TxOutcome;
    TEST_ASSERT_EQUAL_INT((int)TxOutcome::PENDING, (int)txOutcome(0x0B, false));   // TXREQ | TXP
    TEST_ASSERT_EQUAL_INT((int)TxOutcome::PENDING, (int)txOutcome(0x4B, true));    // still on the wire
    TEST_ASSERT_EQUAL_INT((int)TxOutcome::SENT, (int)txOutcome(0x03, false));
    TEST_ASSERT_EQUAL_INT((int)TxOutcome::SENT, (int)txOutcome(0x33, false));      // MLOA/TXERR history
    TEST_ASSERT_EQUAL_INT((int)TxOutcome::SENT, (int)txOutcome(0x01, true));       // abort came too late
    TEST_ASSERT_EQUAL_INT((int)TxOutcome::ABORTED, (int)txOutcome(0x41, true));
    TEST_ASSERT_EQUAL_INT((int)TxOutcome::SENT, (int)txOutcome(0x41, false));      // no abort asked: ABTF ignored
}

static void test_tx_cancel_and_seq() {
    TEST_ASSERT_TRUE(txCancelled(true, 7, 7));
    TEST_ASSERT_FALSE(txCancelled(false, 7, 7));   // OBD frames are never cancelled
    TEST_ASSERT_FALSE(txCancelled(true, 7, 8));
    TEST_ASSERT_FALSE(txCancelled(true, 0, 0));    // 0 = no cancel
    TEST_ASSERT_EQUAL_UINT8(1, nextTxSeq(0));
    TEST_ASSERT_EQUAL_UINT8(2, nextTxSeq(1));
    TEST_ASSERT_EQUAL_UINT8(1, nextTxSeq(255));    // never 0
}

// ---- command result hand-off ------------------------------------------------------------------

static void test_slot_handoff() {
    CanSlot<uint32_t> slot;
    uint32_t v = 0;
    TEST_ASSERT_FALSE(slot.take(v));
    slot.post(11);
    TEST_ASSERT_TRUE(slot.take(v));
    TEST_ASSERT_EQUAL_UINT32(11, v);
    TEST_ASSERT_FALSE(slot.take(v));               // taken once
    slot.post(12);
    slot.post(13);                                 // newest wins, never "full"
    TEST_ASSERT_TRUE(slot.take(v));
    TEST_ASSERT_EQUAL_UINT32(13, v);
}

// ---- init retry gating ------------------------------------------------------------------------

static void test_init_retry_due() {
    // boot init failed at 1000
    TEST_ASSERT_FALSE(initRetryDue(false, true, false, 5999, 1000, 5000));
    TEST_ASSERT_TRUE(initRetryDue(false, true, false, 6000, 1000, 5000));
    // chip up, or never attempted: no retry
    TEST_ASSERT_FALSE(initRetryDue(true, true, false, 60000, 1000, 5000));
    TEST_ASSERT_FALSE(initRetryDue(false, false, false, 60000, 0, 5000));
    // a loop command waiting has priority
    TEST_ASSERT_FALSE(initRetryDue(false, true, true, 60000, 1000, 5000));
    // after a failed re-init sequence (last attempt at 20200, 100 ms apart): only 5 s later
    TEST_ASSERT_FALSE(initRetryDue(false, true, false, 20300, 20200, 5000));
    TEST_ASSERT_TRUE(initRetryDue(false, true, false, 25200, 20200, 5000));
    // millis() wrap
    TEST_ASSERT_FALSE(initRetryDue(false, true, false, 0x00000387u, 0xFFFFF000u, 5000));   // 4999 ms
    TEST_ASSERT_TRUE(initRetryDue(false, true, false, 0x00000388u, 0xFFFFF000u, 5000));    // 5000 ms
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_ring_fifo_order);
    RUN_TEST(test_ring_drop_oldest_keeps_newest_64);
    RUN_TEST(test_ring_reject_when_full);
    RUN_TEST(test_ring_hwm_tracks_maximum);
    RUN_TEST(test_ring_wraparound_1000_pushes);
    RUN_TEST(test_tx_header_standard);
    RUN_TEST(test_tx_header_extended);
    RUN_TEST(test_tx_header_dlc_clamped);
    RUN_TEST(test_read_status_bits);
    RUN_TEST(test_tx_timeout);
    RUN_TEST(test_liveness_gate);
    RUN_TEST(test_int_state);
    RUN_TEST(test_ring_pop_empty_after_wrap);
    RUN_TEST(test_tx_ring_reject_at_wrap_boundary);
    RUN_TEST(test_tx_outcome);
    RUN_TEST(test_tx_cancel_and_seq);
    RUN_TEST(test_slot_handoff);
    RUN_TEST(test_init_retry_due);
    return UNITY_END();
}
