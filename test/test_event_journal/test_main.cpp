// Native unit tests for the event journal law (include/EventJournalLaw.h): line format and
// truncation, dedupe window / sup= count / exempt lines / pending drain, rotation decision,
// tail finder and the RTC replay ring.
// Run: pio test -e native
#include <unity.h>
#include <string>
#include <vector>
#include "EventJournalLaw.h"

void setUp() {}
void tearDown() {}

static void test_format_basic() {
    char out[JOURNAL_LINE_MAX + 1];
    size_t n = jFormatLine(out, 47, 612447, JSrc::VESC, JLvl::W, "resync",
                           "state=PAYLOAD idle=287 bytes_since_good=1412 drop=1", 0);
    TEST_ASSERT_EQUAL_STRING("47 612.447 VESC W resync state=PAYLOAD idle=287 bytes_since_good=1412 drop=1\n", out);
    TEST_ASSERT_EQUAL_UINT32(strlen(out), n);
}

static void test_format_no_fields_and_sup() {
    char out[JOURNAL_LINE_MAX + 1];
    jFormatLine(out, 3, 5, JSrc::I2C, JLvl::W, "input_fail", "addr=0x1A err=2", 9);
    TEST_ASSERT_EQUAL_STRING("3 0.005 I2C W input_fail addr=0x1A err=2 sup=9\n", out);
    jFormatLine(out, 3, 60001, JSrc::SNAP, JLvl::I, "", "heap=1", 0);
    TEST_ASSERT_EQUAL_STRING("3 60.001 SNAP I heap=1\n", out);
    jFormatLine(out, 1, 0, JSrc::SYS, JLvl::E, "x", nullptr, 0);
    TEST_ASSERT_EQUAL_STRING("1 0.000 SYS E x\n", out);
}

static void test_format_truncation() {
    char out[JOURNAL_LINE_MAX + 1];
    std::string longf(300, 'a');
    size_t n = jFormatLine(out, 1, 1000, JSrc::CAN, JLvl::I, "reinit", longf.c_str(), 0);
    TEST_ASSERT_EQUAL_UINT32(JOURNAL_LINE_MAX, n);
    TEST_ASSERT_EQUAL_UINT32(JOURNAL_LINE_MAX, strlen(out));
    TEST_ASSERT_EQUAL_CHAR('\n', out[n - 1]);
    TEST_ASSERT_EQUAL_CHAR('~', out[n - 2]);
    // Exactly at the limit: not cut
    char body[JOURNAL_LINE_MAX];
    const char* prefix = "1 1.000 CAN I e ";
    size_t room = JOURNAL_LINE_MAX - 1 - strlen(prefix);
    memset(body, 'b', room);
    body[room] = '\0';
    n = jFormatLine(out, 1, 1000, JSrc::CAN, JLvl::I, "e", body, 0);
    TEST_ASSERT_EQUAL_UINT32(JOURNAL_LINE_MAX, n);
    TEST_ASSERT_EQUAL_CHAR('b', out[n - 2]);
}

static void test_format_newline_sanitised() {
    char out[JOURNAL_LINE_MAX + 1];
    jFormatLine(out, 1, 0, JSrc::SYS, JLvl::I, "ev", "a=1\nb=2", 0);
    TEST_ASSERT_EQUAL_STRING("1 0.000 SYS I ev a=1_b=2\n", out);
}

static void test_dedupe_burst() {
    JDedupe<24> d(2000);
    uint32_t sup = 0;
    TEST_ASSERT_TRUE(d.admit(JSrc::I2C, "input_fail", 1000, sup));
    TEST_ASSERT_EQUAL_UINT32(0, sup);
    for (int i = 1; i < 10; i++) {
        TEST_ASSERT_FALSE(d.admit(JSrc::I2C, "input_fail", 1000 + i * 100, sup));
    }
    // Different event of the same source is independent
    TEST_ASSERT_TRUE(d.admit(JSrc::I2C, "input_ok", 1500, sup));
    // After the window: written with sup=9
    TEST_ASSERT_TRUE(d.admit(JSrc::I2C, "input_fail", 3000, sup));
    TEST_ASSERT_EQUAL_UINT32(9, sup);
    TEST_ASSERT_FALSE(d.admit(JSrc::I2C, "input_fail", 3100, sup));
    TEST_ASSERT_TRUE(d.admit(JSrc::I2C, "input_fail", 5000, sup));
    TEST_ASSERT_EQUAL_UINT32(1, sup);
}

static void test_dedupe_exempt() {
    JDedupe<24> d(2000);
    uint32_t sup = 0;
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_TRUE(d.admit(JSrc::SYS, "boot", 10, sup));
        TEST_ASSERT_TRUE(d.admit(JSrc::SYS, "reboot", 10, sup));
        TEST_ASSERT_TRUE(d.admit(JSrc::SYS, "journal_drop", 10, sup));
        TEST_ASSERT_TRUE(d.admit(JSrc::SYS, "journal_cleared", 10, sup));
        TEST_ASSERT_TRUE(d.admit(JSrc::SNAP, "", 10, sup));
        TEST_ASSERT_TRUE(d.admit(JSrc::RTC, "replay", 10, sup));
    }
    TEST_ASSERT_TRUE(d.admit(JSrc::SYS, "loop_overrun", 10, sup));
    TEST_ASSERT_FALSE(d.admit(JSrc::SYS, "loop_overrun", 20, sup));
}

static void test_dedupe_pending_drain() {
    JDedupe<24> d(2000);
    uint32_t sup = 0;
    d.admit(JSrc::CAN, "lws_silent", 0, sup);
    d.admit(JSrc::CAN, "lws_silent", 500, sup);
    d.admit(JSrc::CAN, "lws_silent", 900, sup);
    int calls = 0;
    uint32_t got = 0;
    // Not yet 60 s old
    d.drainPending(30000, 60000, [&](JSrc, const char*, uint32_t n) { calls++; got = n; });
    TEST_ASSERT_EQUAL_INT(0, calls);
    d.drainPending(60000, 60000, [&](JSrc s, const char* ev, uint32_t n) {
        calls++; got = n;
        TEST_ASSERT_TRUE(s == JSrc::CAN);
        TEST_ASSERT_EQUAL_STRING("lws_silent", ev);
    });
    TEST_ASSERT_EQUAL_INT(1, calls);
    TEST_ASSERT_EQUAL_UINT32(2, got);
    // Cleared: next write carries no sup
    TEST_ASSERT_TRUE(d.admit(JSrc::CAN, "lws_silent", 70000, sup));
    TEST_ASSERT_EQUAL_UINT32(0, sup);
}

static void test_dedupe_lru() {
    JDedupe<4> d(2000);
    uint32_t sup = 0;
    const char* ev[5] = {"a", "b", "c", "d", "e"};
    for (int i = 0; i < 4; i++) d.admit(JSrc::SYS, ev[i], (uint32_t)i, sup);
    d.admit(JSrc::SYS, "a", 10, sup);           // suppressed, refreshes "a"
    d.admit(JSrc::SYS, "e", 11, sup);           // evicts the LRU ("b")
    TEST_ASSERT_TRUE(d.admit(JSrc::SYS, "b", 12, sup));   // "b" is new again → written
    TEST_ASSERT_FALSE(d.admit(JSrc::SYS, "e", 13, sup));  // "e" still in the table
}

static void test_rotation() {
    TEST_ASSERT_FALSE(jShouldRotate(0, 200000, 131072));        // empty active: never rotate
    TEST_ASSERT_FALSE(jShouldRotate(131000, 72, 131072));
    TEST_ASSERT_TRUE(jShouldRotate(131000, 73, 131072));
    TEST_ASSERT_FALSE(jShouldRotate(1000, 1000, 131072));
}

static void test_tail() {
    const char* b = "l1\nl2\nl3\nl4\n";
    size_t len = strlen(b);
    TEST_ASSERT_EQUAL_UINT32(len, jTailStart(b, len, 0));
    TEST_ASSERT_EQUAL_STRING("l4\n", b + jTailStart(b, len, 1));
    TEST_ASSERT_EQUAL_STRING("l3\nl4\n", b + jTailStart(b, len, 2));
    TEST_ASSERT_EQUAL_STRING(b, b + jTailStart(b, len, 4));
    TEST_ASSERT_EQUAL_STRING(b, b + jTailStart(b, len, 50));
    const char* p = "l1\nl2\npart";
    TEST_ASSERT_EQUAL_STRING("part", p + jTailStart(p, strlen(p), 1));
    TEST_ASSERT_EQUAL_STRING("l2\npart", p + jTailStart(p, strlen(p), 2));
    TEST_ASSERT_EQUAL_UINT32(4, jCountLines(b, len));
    TEST_ASSERT_EQUAL_UINT32(3, jCountLines(p, strlen(p)));
}

static JRtcRing ring;

static void test_ring_replay_three() {
    jRingReset(ring, 46);
    TEST_ASSERT_TRUE(jRingValid(ring));
    jRingPush(ring, "46 1.000 SYS I a\n");
    jRingPush(ring, "46 2.000 SYS I b\n");
    jRingMarkFlushed(ring);
    jRingPush(ring, "46 3.000 SYS I c\n");
    jRingPush(ring, "46 4.000 SYS I d\n");
    jRingPush(ring, "46 5.000 SYS I e\n");
    TEST_ASSERT_TRUE(jRingValid(ring));
    std::vector<std::string> got;
    uint32_t lost = 99;
    size_t n = jRingReplay(ring, lost, [&](const char* l) { got.push_back(l); });
    TEST_ASSERT_EQUAL_UINT32(3, n);
    TEST_ASSERT_EQUAL_UINT32(0, lost);
    TEST_ASSERT_EQUAL_STRING("46 3.000 SYS I c\n", got[0].c_str());
    TEST_ASSERT_EQUAL_STRING("46 5.000 SYS I e\n", got[2].c_str());
}

static void test_ring_overflow_lost() {
    jRingReset(ring, 7);
    char line[64];
    for (int i = 0; i < 20; i++) {
        snprintf(line, sizeof(line), "7 %d.000 SYS I n%d\n", i, i);
        jRingPush(ring, line);
    }
    std::vector<std::string> got;
    uint32_t lost = 0;
    size_t n = jRingReplay(ring, lost, [&](const char* l) { got.push_back(l); });
    TEST_ASSERT_EQUAL_UINT32(JOURNAL_RTC_RING, n);
    TEST_ASSERT_EQUAL_UINT32(4, lost);
    TEST_ASSERT_EQUAL_STRING("7 4.000 SYS I n4\n", got[0].c_str());     // newest 16, oldest first
    TEST_ASSERT_EQUAL_STRING("7 19.000 SYS I n19\n", got[15].c_str());
}

static void test_ring_partial_consume() {
    jRingReset(ring, 9);
    jRingPush(ring, "9 1.000 SYS I a\n");
    jRingPush(ring, "9 2.000 SYS I b\n");
    jRingPush(ring, "9 3.000 SYS I c\n");
    jRingConsume(ring, 2);                  // a, b flushed; c still pending
    jRingPush(ring, "9 4.000 SYS I d\n");   // must not overwrite c
    std::vector<std::string> got;
    uint32_t lost = 0;
    size_t n = jRingReplay(ring, lost, [&](const char* l) { got.push_back(l); });
    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_EQUAL_STRING("9 3.000 SYS I c\n", got[0].c_str());
    TEST_ASSERT_EQUAL_STRING("9 4.000 SYS I d\n", got[1].c_str());
    jRingConsume(ring, 10);
    TEST_ASSERT_EQUAL_UINT32(0, ring.pending);
}

static void test_ring_corruption() {
    jRingReset(ring, 5);
    jRingPush(ring, "5 1.000 SYS I a\n");
    ring.pending = 1000;            // header altered without re-sealing
    TEST_ASSERT_FALSE(jRingValid(ring));
    jRingReset(ring, 5);
    ring.magic = 0;
    TEST_ASSERT_FALSE(jRingValid(ring));
    // Garbage in a line slot is skipped and counted as lost
    jRingReset(ring, 5);
    jRingPush(ring, "5 1.000 SYS I a\n");
    jRingPush(ring, "5 2.000 SYS I b\n");
    ring.lines[0][3] = (char)0x01;   // slot of "a"
    uint32_t lost = 0;
    size_t n = jRingReplay(ring, lost, [&](const char*) {});
    TEST_ASSERT_EQUAL_UINT32(1, n);
    TEST_ASSERT_EQUAL_UINT32(1, lost);
    static_assert(sizeof(JRtcRing) <= 2304, "RTC ring budget");
}

static void test_crc32_known() {
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, jCrc32("123456789", 9));
}

// ─── RAM line buffer / RAM tail (jLineBufAppend) ─────────────────────────────

static void test_linebuf_fits() {
    char buf[32];
    size_t len = 0;
    JBufCut c = jLineBufAppend(buf, sizeof(buf), len, "aaaa\n", 5);
    TEST_ASSERT_EQUAL_UINT32(0, c.bytes);
    TEST_ASSERT_EQUAL_UINT32(0, c.lines);
    c = jLineBufAppend(buf, sizeof(buf), len, "bbbbbbbbb\n", 10);
    TEST_ASSERT_EQUAL_UINT32(0, c.lines);
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_STRING_LEN("aaaa\nbbbbbbbbb\n", buf, 15);
}

static void test_linebuf_drops_oldest_whole_lines() {
    char buf[20];
    size_t len = 0;
    jLineBufAppend(buf, sizeof(buf), len, "11111\n", 6);   // 6
    jLineBufAppend(buf, sizeof(buf), len, "22222\n", 6);   // 12
    jLineBufAppend(buf, sizeof(buf), len, "33333\n", 6);   // 18
    // 18 + 6 = 24 > 20: drop "11111\n" only (need 4 → one whole line of 6)
    JBufCut c = jLineBufAppend(buf, sizeof(buf), len, "44444\n", 6);
    TEST_ASSERT_EQUAL_UINT32(6, c.bytes);
    TEST_ASSERT_EQUAL_UINT32(1, c.lines);
    TEST_ASSERT_EQUAL_UINT32(18, len);
    TEST_ASSERT_EQUAL_STRING_LEN("22222\n33333\n44444\n", buf, 18);
    // A 15-byte line needs 13 bytes freed: three whole lines go (never a partial line)
    c = jLineBufAppend(buf, sizeof(buf), len, "xxxxxxxxxxxxxx\n", 15);
    TEST_ASSERT_EQUAL_UINT32(18, c.bytes);
    TEST_ASSERT_EQUAL_UINT32(3, c.lines);
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_STRING_LEN("xxxxxxxxxxxxxx\n", buf, 15);
}

static void test_linebuf_rejects_oversize_and_empty() {
    char buf[8];
    size_t len = 0;
    jLineBufAppend(buf, sizeof(buf), len, "abc\n", 4);
    JBufCut c = jLineBufAppend(buf, sizeof(buf), len, "123456789\n", 10);
    TEST_ASSERT_EQUAL_UINT32(0, c.lines);
    TEST_ASSERT_EQUAL_UINT32(4, len);
    c = jLineBufAppend(buf, sizeof(buf), len, "", 0);
    TEST_ASSERT_EQUAL_UINT32(4, len);
    // Exactly the capacity: everything older goes, the line fits
    c = jLineBufAppend(buf, sizeof(buf), len, "1234567\n", 8);
    TEST_ASSERT_EQUAL_UINT32(1, c.lines);
    TEST_ASSERT_EQUAL_UINT32(8, len);
}

static void test_linebuf_tail_accounting() {
    // RAM tail of 4 KB fed with 1000 formatted lines: always the newest lines, whole lines only,
    // and appended bytes == kept bytes + dropped bytes (the journal's byte accounting).
    static char buf[4096];
    size_t len = 0;
    size_t appended = 0, dropped = 0;
    uint32_t droppedLines = 0;
    char line[JOURNAL_LINE_MAX + 1];
    for (uint32_t i = 0; i < 1000; i++) {
        size_t n = jFormatLine(line, 3, i * 1000u, JSrc::SYS, JLvl::I, "tick", (i % 3) ? "k=v" : "a_longer_field=12345", 0);
        JBufCut c = jLineBufAppend(buf, sizeof(buf), len, line, n);
        appended += n;
        dropped += c.bytes;
        droppedLines += c.lines;
        TEST_ASSERT_TRUE(len <= sizeof(buf));
    }
    TEST_ASSERT_EQUAL_UINT32(appended, dropped + len);
    TEST_ASSERT_EQUAL_UINT32(1000, droppedLines + jCountLines(buf, len));
    TEST_ASSERT_EQUAL_CHAR('\n', buf[len - 1]);
    // Starts at a line boundary and ends with the newest line
    TEST_ASSERT_EQUAL_STRING_LEN("3 ", buf, 2);
    const size_t last = jTailStart(buf, len, 1);
    TEST_ASSERT_EQUAL_STRING_LEN("3 999.000 SYS I tick a_longer_field=12345\n", buf + last, len - last);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_format_basic);
    RUN_TEST(test_format_no_fields_and_sup);
    RUN_TEST(test_format_truncation);
    RUN_TEST(test_format_newline_sanitised);
    RUN_TEST(test_dedupe_burst);
    RUN_TEST(test_dedupe_exempt);
    RUN_TEST(test_dedupe_pending_drain);
    RUN_TEST(test_dedupe_lru);
    RUN_TEST(test_rotation);
    RUN_TEST(test_tail);
    RUN_TEST(test_ring_replay_three);
    RUN_TEST(test_ring_overflow_lost);
    RUN_TEST(test_ring_partial_consume);
    RUN_TEST(test_ring_corruption);
    RUN_TEST(test_crc32_known);
    RUN_TEST(test_linebuf_fits);
    RUN_TEST(test_linebuf_drops_oldest_whole_lines);
    RUN_TEST(test_linebuf_rejects_oversize_and_empty);
    RUN_TEST(test_linebuf_tail_accounting);
    return UNITY_END();
}
