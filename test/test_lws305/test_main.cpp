// Native unit tests for the Bosch LWS 0x305 decoder (include/Lws305.h) and its
// validation / health logic (src/LwsSensor.cpp). Run: pio test -e native
//
// Vectors come from the bench session in CAN-TEST/README.md ("Формат кадру 0x305",
// "Вихід за діапазон (±600°)"); the check byte is computed for the chosen counter.
#include <unity.h>
#include <string.h>
#include "Lws305.h"
#include "LwsSensor.h"

// Mirror of include/Constants.h (which needs Arduino.h and cannot be built natively).
static const uint32_t STEER_LWS_TIMEOUT_MS = 100;
static const uint8_t STEER_LWS_RECOVER_FRAMES = 5;

static const uint8_t F_REST[6]    = {0x0B, 0x32, 0x00, 0x07, 0x49, 0x00};  // +286.6°, at rest, adj+cal+OK, cnt 9
static const uint8_t F_TURN[6]    = {0x0C, 0x77, 0x01, 0x87, 0x13, 0x00};  // +319.1°, 4 °/s CW, OK, cnt 3
static const uint8_t F_LATCHED[6] = {0x7F, 0xFF, 0xFF, 0x4C, 0x55, 0x00};  // latched range fault, cnt 5

// Build a frame with a valid check byte for any counter
static void makeFrame(uint8_t* f, int16_t angle, uint8_t speed, uint8_t status, uint8_t counter) {
    f[0] = (uint8_t)((uint16_t)angle >> 8);
    f[1] = (uint8_t)angle;
    f[2] = speed;
    f[3] = status;
    uint8_t x = counter & 0x0F;
    for (int i = 0; i < 4; i++) x ^= (f[i] >> 4) ^ (f[i] & 0x0F);
    f[4] = (uint8_t)((x << 4) | (counter & 0x0F));
    f[5] = 0x00;
}

static LwsSensor makeSensor() { return LwsSensor(STEER_LWS_TIMEOUT_MS, STEER_LWS_RECOVER_FRAMES); }

void setUp() {}
void tearDown() {}

static void test_rest_frame_decodes() {
    TEST_ASSERT_TRUE(lwsCrcOk(F_REST));
    TEST_ASSERT_EQUAL_INT16(2866, lwsAngleRaw(F_REST));
    TEST_ASSERT_TRUE(lwsAngleValid(F_REST));
    TEST_ASSERT_TRUE(lwsSpeedValid(F_REST));
    TEST_ASSERT_EQUAL_UINT16(0, lwsSpeedDegS(F_REST));
    TEST_ASSERT_TRUE(lwsOk(F_REST));
    TEST_ASSERT_TRUE(lwsCal(F_REST));
    TEST_ASSERT_TRUE(lwsAdj(F_REST));
    TEST_ASSERT_EQUAL_UINT8(0, lwsFault(F_REST));
    TEST_ASSERT_EQUAL_UINT8(9, lwsCounter(F_REST));

    LwsSensor s = makeSensor();
    TEST_ASSERT_TRUE(s.onFrame(F_REST, 6, 1000));
    TEST_ASSERT_EQUAL_INT16(2866, s.angleRaw());
    TEST_ASSERT_EQUAL_INT32(-2866, s.position());   // firmware sign: + = right
}

static void test_turn_frame_decodes() {
    TEST_ASSERT_TRUE(lwsCrcOk(F_TURN));
    TEST_ASSERT_EQUAL_INT16(3191, lwsAngleRaw(F_TURN));
    TEST_ASSERT_EQUAL_UINT16(4, lwsSpeedDegS(F_TURN));
    TEST_ASSERT_TRUE(lwsDirCW(F_TURN));
    TEST_ASSERT_TRUE(lwsOk(F_TURN));
    TEST_ASSERT_EQUAL_UINT8(0, lwsFault(F_TURN));
    TEST_ASSERT_EQUAL_UINT8(3, lwsCounter(F_TURN));

    LwsSensor s = makeSensor();
    TEST_ASSERT_TRUE(s.onFrame(F_TURN, 6, 1000));
    TEST_ASSERT_TRUE(s.speedValid());
    TEST_ASSERT_EQUAL_INT32(4, s.rateDegS());      // clockwise = turning right = positive
    TEST_ASSERT_EQUAL_INT32(-3191, s.position());
}

static void test_bit_flip_rejected_by_checksum() {
    LwsSensor s = makeSensor();
    uint8_t a[6], b[6];
    memcpy(a, F_REST, 6);
    memcpy(b, F_TURN, 6);
    a[0] ^= 0x01;
    b[0] ^= 0x10;
    TEST_ASSERT_FALSE(lwsCrcOk(a));
    TEST_ASSERT_FALSE(lwsCrcOk(b));
    TEST_ASSERT_FALSE(s.onFrame(a, 6, 1000));
    TEST_ASSERT_EQUAL_UINT32(1, s.crcErrors());
    TEST_ASSERT_FALSE(s.onFrame(b, 6, 1010));
    TEST_ASSERT_EQUAL_UINT32(2, s.crcErrors());
    TEST_ASSERT_FALSE(s.hasFrame());
    TEST_ASSERT_EQUAL_UINT32(0, s.acceptedFrames());
}

static void test_counter_repeat_and_miss() {
    LwsSensor s = makeSensor();
    uint8_t f[6];
    makeFrame(f, 100, 0, 0x07, 3);
    TEST_ASSERT_TRUE(s.onFrame(f, 6, 1000));
    // Same counter again: duplicate / frozen sender
    TEST_ASSERT_FALSE(s.onFrame(f, 6, 1010));
    TEST_ASSERT_EQUAL_UINT32(1, s.counterErrors());
    TEST_ASSERT_EQUAL_UINT32(1, s.acceptedFrames());
    // 3 -> 6: two frames lost, frame still accepted
    makeFrame(f, 110, 0, 0x07, 6);
    TEST_ASSERT_TRUE(s.onFrame(f, 6, 1020));
    TEST_ASSERT_EQUAL_UINT32(2, s.missedFrames());
    TEST_ASSERT_EQUAL_INT32(-110, s.position());
    // 15 -> 0 wrap is a step of one
    makeFrame(f, 120, 0, 0x07, 15);
    TEST_ASSERT_TRUE(s.onFrame(f, 6, 1030));
    TEST_ASSERT_EQUAL_UINT32(10, s.missedFrames());   // 6 -> 15 = 8 lost
    makeFrame(f, 121, 0, 0x07, 0);
    TEST_ASSERT_TRUE(s.onFrame(f, 6, 1040));
    TEST_ASSERT_EQUAL_UINT32(10, s.missedFrames());
}

static void test_latched_range_fault() {
    TEST_ASSERT_TRUE(lwsCrcOk(F_LATCHED));
    TEST_ASSERT_FALSE(lwsAngleValid(F_LATCHED));
    TEST_ASSERT_FALSE(lwsSpeedValid(F_LATCHED));
    TEST_ASSERT_FALSE(lwsOk(F_LATCHED));
    TEST_ASSERT_FALSE(lwsCal(F_LATCHED));
    TEST_ASSERT_TRUE(lwsAdj(F_LATCHED));
    TEST_ASSERT_EQUAL_UINT8(9, lwsFault(F_LATCHED));
    TEST_ASSERT_EQUAL_UINT8(5, lwsCounter(F_LATCHED));

    LwsSensor s = makeSensor();
    TEST_ASSERT_TRUE(s.onFrame(F_LATCHED, 6, 1000));
    TEST_ASSERT_EQUAL(LwsFault::RANGE_LATCHED, s.evaluate(1001, false, true));
    TEST_ASSERT_EQUAL_STRING("range_latched", lwsFaultToken(s.fault()));
    TEST_ASSERT_FALSE(s.isHealthy());
    TEST_ASSERT_FALSE(s.speedValid());
    TEST_ASSERT_FALSE(s.angleValid());
}

static void test_status_tokens_and_short_dlc() {
    uint8_t f[6];
    {
        LwsSensor s = makeSensor();
        makeFrame(f, (int16_t)LWS_ANGLE_INV, 0, 0x01, 1);   // OK=1 cal=0, angle invalid
        TEST_ASSERT_TRUE(s.onFrame(f, 6, 1000));
        TEST_ASSERT_EQUAL(LwsFault::NOT_CALIBRATED, s.evaluate(1001, false, true));
    }
    {
        LwsSensor s = makeSensor();
        makeFrame(f, 0, 0, (uint8_t)(0x03 | (2 << 3)), 1);  // OK=1 cal=1 fault=2
        TEST_ASSERT_TRUE(s.onFrame(f, 6, 1000));
        TEST_ASSERT_EQUAL(LwsFault::SENSOR_FAULT, s.evaluate(1001, false, true));
    }
    {
        LwsSensor s = makeSensor();
        makeFrame(f, 0, 0, 0x02, 1);                        // OK=0 cal=1 fault=0
        TEST_ASSERT_TRUE(s.onFrame(f, 6, 1000));
        TEST_ASSERT_EQUAL(LwsFault::SENSOR_FAULT, s.evaluate(1001, false, true));
    }
    {
        LwsSensor s = makeSensor();
        makeFrame(f, (int16_t)LWS_ANGLE_INV, 0, 0x03, 1);   // OK=1 cal=1, angle invalid
        TEST_ASSERT_TRUE(s.onFrame(f, 6, 1000));
        TEST_ASSERT_EQUAL(LwsFault::ANGLE_INVALID, s.evaluate(1001, false, true));
    }
    {
        LwsSensor s = makeSensor();
        TEST_ASSERT_FALSE(s.onFrame(F_REST, 5, 1000));      // DLC 5: ignored, not an error
        TEST_ASSERT_FALSE(s.hasFrame());
        TEST_ASSERT_EQUAL_UINT32(0, s.crcErrors());
        TEST_ASSERT_EQUAL_UINT32(0, s.counterErrors());
        TEST_ASSERT_TRUE(s.onFrame(F_REST, 7, 1000));       // DLC 7 (PSA-RE) accepted
    }
}

static void test_timeout_and_recovery_streak() {
    LwsSensor s = makeSensor();
    // Never received
    TEST_ASSERT_EQUAL(LwsFault::TIMEOUT, s.evaluate(0, false, true));

    uint8_t f[6];
    uint32_t t = 1000;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < STEER_LWS_RECOVER_FRAMES - 1; i++) {
        makeFrame(f, 0, 0, 0x07, cnt++);
        TEST_ASSERT_TRUE(s.onFrame(f, 6, t));
        s.evaluate(t, false, true);
        TEST_ASSERT_FALSE(s.isHealthy());
        TEST_ASSERT_NOT_EQUAL(LwsFault::NONE, s.fault());   // never "none" while unhealthy
        t += 10;
    }
    makeFrame(f, 0, 0, 0x07, cnt++);
    TEST_ASSERT_TRUE(s.onFrame(f, 6, t));
    TEST_ASSERT_EQUAL(LwsFault::NONE, s.evaluate(t, false, true));
    TEST_ASSERT_TRUE(s.isHealthy());

    // No frame for more than the timeout
    TEST_ASSERT_EQUAL(LwsFault::NONE, s.evaluate(t + STEER_LWS_TIMEOUT_MS, false, true));
    TEST_ASSERT_EQUAL(LwsFault::TIMEOUT, s.evaluate(t + STEER_LWS_TIMEOUT_MS + 1, false, true));
    TEST_ASSERT_FALSE(s.isHealthy());

    // Frames return: healthy only after 5 consecutive good frames, token stays "timeout"
    t += 500;
    for (uint8_t i = 0; i < STEER_LWS_RECOVER_FRAMES - 1; i++) {
        makeFrame(f, 0, 0, 0x07, cnt++);
        TEST_ASSERT_TRUE(s.onFrame(f, 6, t));
        TEST_ASSERT_EQUAL(LwsFault::TIMEOUT, s.evaluate(t, false, true));
        TEST_ASSERT_FALSE(s.isHealthy());
        t += 10;
    }
    makeFrame(f, 0, 0, 0x07, cnt++);
    TEST_ASSERT_TRUE(s.onFrame(f, 6, t));
    TEST_ASSERT_EQUAL(LwsFault::NONE, s.evaluate(t, false, true));
    TEST_ASSERT_TRUE(s.isHealthy());

    // A fault frame takes effect immediately and restarts the streak
    makeFrame(f, 0, 0, (uint8_t)(0x07 | (1 << 3)), cnt++);
    TEST_ASSERT_TRUE(s.onFrame(f, 6, t + 10));
    TEST_ASSERT_EQUAL(LwsFault::SENSOR_FAULT, s.evaluate(t + 10, false, true));
    TEST_ASSERT_FALSE(s.isHealthy());
    makeFrame(f, 0, 0, 0x07, cnt++);
    TEST_ASSERT_TRUE(s.onFrame(f, 6, t + 20));
    TEST_ASSERT_EQUAL(LwsFault::SENSOR_FAULT, s.evaluate(t + 20, false, true));
    TEST_ASSERT_FALSE(s.isHealthy());
}

static void test_priority_zeroing_and_can_down() {
    LwsSensor s = makeSensor();
    TEST_ASSERT_TRUE(s.onFrame(F_LATCHED, 6, 1000));
    TEST_ASSERT_EQUAL(LwsFault::ZEROING, s.evaluate(1001, true, false));
    TEST_ASSERT_EQUAL(LwsFault::CAN_DOWN, s.evaluate(1001, false, false));
    TEST_ASSERT_EQUAL(LwsFault::TIMEOUT, s.evaluate(1200, false, true));
}

static void test_fps_window() {
    LwsSensor s = makeSensor();
    uint8_t f[6];
    uint32_t t = 0;
    s.evaluate(t, false, true);
    for (uint8_t i = 0; i < 100; i++) {
        makeFrame(f, 0, 0, 0x07, (uint8_t)(i & 0x0F));
        s.onFrame(f, 6, t);
        t += 10;
    }
    s.evaluate(t, false, true);
    TEST_ASSERT_EQUAL_UINT16(100, s.fps());
}

static void test_config_payloads_guard_id_trap() {
    TEST_ASSERT_EQUAL_HEX16(0x305, ((0x30 << 8) | 0x53) >> 4);
    TEST_ASSERT_EQUAL_HEX16(0x305, lwsCfgOutputId(LWS_CFG_ZERO_ID305));
    TEST_ASSERT_EQUAL_HEX8(0x53, LWS_CFG_ZERO_ID305[0]);
    TEST_ASSERT_EQUAL_HEX8(0x30, LWS_CFG_ZERO_ID305[1]);
    TEST_ASSERT_EQUAL_HEX8(0x05, LWS_CFG_RESET[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, LWS_CFG_RESET[1]);
    // 03 00 would zero the sensor AND move its output to ID 0x000
    TEST_ASSERT_FALSE(LWS_CFG_ZERO_ID305[0] == 0x03 && LWS_CFG_ZERO_ID305[1] == 0x00);
    const uint8_t trap[2] = {0x03, 0x00};
    TEST_ASSERT_EQUAL_HEX16(0x000, lwsCfgOutputId(trap));
    TEST_ASSERT_EQUAL_HEX8(3, LWS_CFG_ZERO_ID305[0] & 0x07);   // CCW 3 = zero here
    TEST_ASSERT_EQUAL_HEX8(5, LWS_CFG_RESET[0] & 0x07);        // CCW 5 = reset calibration
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_rest_frame_decodes);
    RUN_TEST(test_turn_frame_decodes);
    RUN_TEST(test_bit_flip_rejected_by_checksum);
    RUN_TEST(test_counter_repeat_and_miss);
    RUN_TEST(test_latched_range_fault);
    RUN_TEST(test_status_tokens_and_short_dlc);
    RUN_TEST(test_timeout_and_recovery_streak);
    RUN_TEST(test_priority_zeroing_and_can_down);
    RUN_TEST(test_fps_window);
    RUN_TEST(test_config_payloads_guard_id_trap);
    return UNITY_END();
}
