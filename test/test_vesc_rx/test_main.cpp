// Native unit tests for the VESC UART receive parser (include/VescRxParser.h): the "phase lock"
// after one lost byte without the inter-byte gap rule, its recovery with the rule, a clean
// stream, CRC error, bad stop byte and a loop stall in the middle of a reply.
// Run: pio test -e native
#include <unity.h>
#include <stdio.h>
#include <vector>
#include "VescRxParser.h"

void setUp() {}
void tearDown() {}

typedef std::vector<uint8_t> Bytes;

// A COMM_GET_VALUES-like reply (id 0x04 + 73 data bytes) whose data, length and CRC bytes
// contain no 0x02 / 0x03 — the standstill case where the misalignment never breaks by itself.
static Bytes makeReply(uint8_t seed) {
    Bytes payload;
    payload.push_back(0x04);
    for (int i = 0; i < 73; i++) payload.push_back((uint8_t)(0x10 + ((i + seed) % 0x60)));
    for (int tweak = 0; tweak < 256; tweak++) {
        payload[1] = (uint8_t)(0x10 + tweak % 0xE0);
        uint16_t c = VescRxParser::crc16(payload.data(), (uint16_t)payload.size());
        uint8_t hi = (uint8_t)(c >> 8), lo = (uint8_t)(c & 0xFF);
        if (hi != 0x02 && hi != 0x03 && lo != 0x02 && lo != 0x03) break;
    }
    uint16_t c = VescRxParser::crc16(payload.data(), (uint16_t)payload.size());
    Bytes f;
    f.push_back(0x02);
    f.push_back((uint8_t)payload.size());
    f.insert(f.end(), payload.begin(), payload.end());
    f.push_back((uint8_t)(c >> 8));
    f.push_back((uint8_t)(c & 0xFF));
    f.push_back(0x03);
    return f;
}

static bool noStartBytesInside(const Bytes& f) {
    for (size_t i = 1; i + 1 < f.size(); i++) if (f[i] == 0x02 || f[i] == 0x03) return false;
    return true;
}

// Feed a burst: 1 ms per ~11 bytes (115200 baud ≈ 87 µs/byte). Returns packets decoded.
static int feedBurst(VescRxParser& p, const Bytes& f, uint32_t t0) {
    int ok = 0;
    for (size_t i = 0; i < f.size(); i++) {
        if (p.feed(f[i], t0 + (uint32_t)(i / 11))) ok++;
    }
    return ok;
}

// One reply with a dropped data byte, then N identical replies 300 ms apart.
static int runLostByteScenario(uint32_t gapMs, int n) {
    VescRxParser p(gapMs);
    Bytes r = makeReply(0);
    TEST_ASSERT_TRUE(noStartBytesInside(r));
    Bytes broken = r;
    broken.erase(broken.begin() + 6);   // d4 lost on the wire
    feedBurst(p, broken, 0);
    int decoded = 0;
    for (int k = 1; k <= n; k++) decoded += feedBurst(p, r, (uint32_t)k * 300);
    return decoded;
}

static void test_stuck_without_gap_rule() {
    const int decoded = runLostByteScenario(0, 10);
    printf("[vesc_rx] without gap rule: %d of 10 replies decoded after one lost byte\n", decoded);
    TEST_ASSERT_EQUAL_INT(0, decoded);
}

static void test_recovers_with_gap_rule() {
    const int decoded = runLostByteScenario(5, 10);
    printf("[vesc_rx] with 5 ms gap rule: %d of 10 replies decoded after one lost byte\n", decoded);
    TEST_ASSERT_EQUAL_INT(10, decoded);
}

static void test_resync_counted() {
    VescRxParser p(5);
    Bytes r = makeReply(0);
    Bytes broken = r;
    broken.erase(broken.begin() + 6);
    feedBurst(p, broken, 0);
    TEST_ASSERT_EQUAL_UINT32(0, p.stats().resync);
    TEST_ASSERT_EQUAL_INT(1, feedBurst(p, r, 300));
    TEST_ASSERT_EQUAL_UINT32(1, p.stats().resync);
    VescRxParser::State st;
    uint32_t idle = 0;
    TEST_ASSERT_TRUE(p.takeResync(st, idle));
    TEST_ASSERT_EQUAL_STRING("END", VescRxParser::stateToken(st));
    TEST_ASSERT_TRUE(idle >= 290);
    TEST_ASSERT_FALSE(p.takeResync(st, idle));
    TEST_ASSERT_EQUAL_UINT32(0, p.stats().bytesSinceGood);
}

static void test_clean_stream() {
    VescRxParser p(5);
    int ok = 0;
    for (int k = 0; k < 20; k++) ok += feedBurst(p, makeReply((uint8_t)k), (uint32_t)k * 50);
    TEST_ASSERT_EQUAL_INT(20, ok);
    TEST_ASSERT_EQUAL_UINT32(20, p.stats().good);
    TEST_ASSERT_EQUAL_UINT32(0, p.stats().resync);
    TEST_ASSERT_EQUAL_UINT32(0, p.stats().dropCrc);
    TEST_ASSERT_EQUAL_UINT32(0, p.stats().dropStop);
    TEST_ASSERT_EQUAL_UINT32(0x04, p.payload()[0]);
    TEST_ASSERT_EQUAL_UINT32(74, p.length());
}

static void test_crc_error() {
    VescRxParser p(5);
    Bytes r = makeReply(1);
    Bytes bad = r;
    bad[10] ^= 0x40;
    TEST_ASSERT_EQUAL_INT(0, feedBurst(p, bad, 0));
    TEST_ASSERT_EQUAL_UINT32(1, p.stats().dropCrc);
    TEST_ASSERT_EQUAL_INT(1, feedBurst(p, r, 300));
}

static void test_bad_stop_byte() {
    VescRxParser p(5);
    Bytes r = makeReply(2);
    Bytes bad = r;
    bad.back() = 0x00;
    TEST_ASSERT_EQUAL_INT(0, feedBurst(p, bad, 0));
    TEST_ASSERT_EQUAL_UINT32(1, p.stats().dropStop);
    TEST_ASSERT_EQUAL_INT(1, feedBurst(p, r, 300));
}

static void test_mid_burst_stall_loses_at_most_one() {
    VescRxParser p(5);
    Bytes r = makeReply(3);
    // First half read, then the loop stalls 20 ms before reading the rest
    int ok = 0;
    size_t half = r.size() / 2;
    for (size_t i = 0; i < half; i++) ok += p.feed(r[i], 0) ? 1 : 0;
    for (size_t i = half; i < r.size(); i++) ok += p.feed(r[i], 20) ? 1 : 0;
    TEST_ASSERT_EQUAL_INT(0, ok);
    TEST_ASSERT_EQUAL_UINT32(1, p.stats().resync);
    // The next reply decodes
    TEST_ASSERT_EQUAL_INT(1, feedBurst(p, r, 300));
    TEST_ASSERT_EQUAL_INT(1, feedBurst(p, r, 600));
}

static void test_oversize_length_rejected() {
    VescRxParser p(5);
    const uint8_t longHdr[] = {0x03, 0x02, 0x4A};   // 586 > RX_BUF_SIZE
    for (uint8_t b : longHdr) p.feed(b, 0);
    TEST_ASSERT_EQUAL_INT(VescRxParser::RX_START, p.state());
    TEST_ASSERT_EQUAL_INT(1, feedBurst(p, makeReply(4), 1));
}

static void test_reset() {
    VescRxParser p(5);
    Bytes r = makeReply(5);
    for (size_t i = 0; i < 10; i++) p.feed(r[i], 0);
    TEST_ASSERT_EQUAL_INT(VescRxParser::RX_PAYLOAD, p.state());
    p.reset();
    TEST_ASSERT_EQUAL_INT(VescRxParser::RX_START, p.state());
    TEST_ASSERT_EQUAL_INT(1, feedBurst(p, r, 1));
    TEST_ASSERT_EQUAL_UINT32(0, p.stats().resync);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_stuck_without_gap_rule);
    RUN_TEST(test_recovers_with_gap_rule);
    RUN_TEST(test_resync_counted);
    RUN_TEST(test_clean_stream);
    RUN_TEST(test_crc_error);
    RUN_TEST(test_bad_stop_byte);
    RUN_TEST(test_mid_burst_stall_loses_at_most_one);
    RUN_TEST(test_oversize_length_rejected);
    RUN_TEST(test_reset);
    return UNITY_END();
}
