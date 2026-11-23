// Native tests of include/IboosterProto.h (change replace-aztec-brake-with-ibooster, task 2.1).
// Reference vectors reused from the bench tool Arduino/iBooster-TEST/test/test_proto:
//  - the 48 frames of the reference trace (ThatguyCplus/ibooster-can-research-kit,
//    "ibooster EXTERNAL_BRAKE_REQUEST.csv": 16 groups 0x38B / 0x38C / 0x38D, counter 0..15,
//    external_request = 1, flow 0x8300, speed 0);
//  - real 0x39D / 0x38E / 0x38F frames of one unit (bench 22.11.2026).
#include <unity.h>
#include <string.h>
#include "IboosterProto.h"

// Trace CRC byte for counter 0..15
static const uint8_t crc38B[16] = {0xF8, 0x77, 0xFB, 0x74, 0xFE, 0x71, 0xFD, 0x72,
                                   0xF4, 0x7B, 0xF7, 0x78, 0xF2, 0x7D, 0xF1, 0x7E};
static const uint8_t crc38C[16] = {0x90, 0x1F, 0x93, 0x1C, 0x96, 0x19, 0x95, 0x1A,
                                   0x9C, 0x13, 0x9F, 0x10, 0x9A, 0x15, 0x99, 0x16};
static const uint8_t crc38D[16] = {0xE8, 0xAE, 0x64, 0x22, 0xED, 0xAB, 0x61, 0x27,
                                   0xE2, 0xA4, 0x6E, 0x28, 0xE7, 0xA1, 0x6B, 0x2D};

// Real 0x39D frames (DLC 5), bench 22.11.2026
static const uint8_t real39D[][5] = {
    {0x00, 0x58, 0xE1, 0x27, 0x00}, {0x01, 0x59, 0xE1, 0x27, 0x00}, {0x07, 0x5F, 0xE1, 0x27, 0x00},
    {0x1C, 0x60, 0xF5, 0x27, 0x00}, {0x27, 0x6B, 0xF5, 0x27, 0x00}, {0x5D, 0x60, 0x35, 0x28, 0x00},
    {0x68, 0x6B, 0x35, 0x28, 0x00}, {0xDF, 0x13, 0x00, 0x2C, 0x00}, {0xF8, 0x50, 0xE1, 0x27, 0x00},
    {0xFF, 0x57, 0xE1, 0x27, 0x00},
};

void setUp() {}
void tearDown() {}

static void test_crc_trace_frames() {
    for (uint8_t n = 0; n < 16; n++) {
        const uint8_t b[4] = {crc38B[n], (uint8_t)(0x40 | n), 0x00, 0x05};
        const uint8_t c[4] = {crc38C[n], (uint8_t)(0x40 | n), 0x00, 0x83};
        const uint8_t d[7] = {crc38D[n], n, 0x00, 0x00, 0x00, 0x50, 0x47};
        TEST_ASSERT_TRUE(ibCrcOk(b, 4));
        TEST_ASSERT_TRUE(ibCrcOk(c, 4));
        TEST_ASSERT_TRUE(ibCrcOk(d, 7));
        TEST_ASSERT_TRUE(ibChecksumOk(IB_ID_VEHICLE_STATUS, b, 4));
    }
}

// ibBuild(n, true, 0x8300, 0) reproduces the 48 trace frames byte for byte
static void test_build_matches_trace() {
    for (uint8_t n = 0; n < 16; n++) {
        uint8_t b[4], c[4], d[7];
        ibBuild(n, true, 0x8300, 0, b, c, d);
        const uint8_t eb[4] = {crc38B[n], (uint8_t)(0x40 | n), 0x00, 0x05};
        const uint8_t ec[4] = {crc38C[n], (uint8_t)(0x40 | n), 0x00, 0x83};
        const uint8_t ed[7] = {crc38D[n], n, 0x00, 0x00, 0x00, 0x50, 0x47};
        TEST_ASSERT_EQUAL_HEX8_ARRAY(eb, b, 4);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(ec, c, 4);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(ed, d, 7);
    }
}

static void test_idle_frames_have_no_request_bits() {
    uint8_t b[4], c[4], d[7];
    ibBuild(5, false, IB_FLOW_ZERO, 0, b, c, d);
    TEST_ASSERT_EQUAL_HEX8(0x05, b[1]);
    TEST_ASSERT_EQUAL_HEX8(0x05, c[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, c[2]);
    TEST_ASSERT_EQUAL_HEX8(0x7E, c[3]);
    TEST_ASSERT_TRUE(ibCrcOk(b, 4));
    TEST_ASSERT_TRUE(ibCrcOk(c, 4));
    TEST_ASSERT_TRUE(ibCrcOk(d, 7));
    // Without a request the flow is 0x7E00 whatever is passed in
    ibBuild(5, false, 0x9200, 0, b, c, d);
    TEST_ASSERT_EQUAL_HEX16(IB_FLOW_ZERO, (uint16_t)(c[2] | (c[3] << 8)));
}

static void test_flow_clamped_and_no_0x80() {
    uint8_t b[4], c[4], d[7];
    ibBuild(0, true, 0xFFFF, 0, b, c, d);
    TEST_ASSERT_EQUAL_HEX16(IB_FLOW_MAX, (uint16_t)(c[2] | (c[3] << 8)));
    ibBuild(0, true, 0x0000, 0, b, c, d);
    TEST_ASSERT_EQUAL_HEX16(IB_FLOW_MIN, (uint16_t)(c[2] | (c[3] << 8)));
    TEST_ASSERT_EQUAL_HEX16(IB_FLOW_MIN, ibClampFlow(-100000));
    TEST_ASSERT_EQUAL_HEX16(IB_FLOW_MAX, ibClampFlow(100000));
    // Exhaustive over counter, request, a flow sweep and every speed byte: byte 1 of 0x38B / 0x38C
    // never carries 0x80, the pMC1 word is always 117 / qualifier 1, and every CRC checks.
    for (uint16_t cnt = 0; cnt < 32; cnt++) {
        for (int req = 0; req < 2; req++) {
            for (uint32_t flow = 0; flow <= 0xFFFF; flow += 0x0FFF) {
                for (uint16_t spd = 0; spd < 256; spd += 17) {
                    ibBuild((uint8_t)cnt, req != 0, (uint16_t)flow, (uint8_t)spd, b, c, d);
                    TEST_ASSERT_EQUAL_HEX8(0, b[1] & 0x80);
                    TEST_ASSERT_EQUAL_HEX8(0, c[1] & 0x80);
                    TEST_ASSERT_EQUAL_HEX8(req ? 0x40 : 0x00, b[1] & 0xF0);
                    TEST_ASSERT_EQUAL_HEX8(req ? 0x40 : 0x00, c[1] & 0xF0);
                    TEST_ASSERT_EQUAL_HEX8(cnt & 0x0F, d[1]);
                    TEST_ASSERT_EQUAL_HEX8(0x50, d[5]);
                    TEST_ASSERT_EQUAL_HEX8(0x47, d[6]);
                    TEST_ASSERT_EQUAL_HEX8(1, d[6] >> 6);   // qualifier bits 14..15
                    const uint16_t w = (uint16_t)(c[2] | (c[3] << 8));
                    TEST_ASSERT_TRUE(w >= IB_FLOW_MIN && w <= IB_FLOW_MAX);
                    TEST_ASSERT_EQUAL_HEX8((uint8_t)spd, b[2]);
                    TEST_ASSERT_EQUAL_HEX8(0x05, b[3]);
                    TEST_ASSERT_TRUE(ibCrcOk(b, 4) && ibCrcOk(c, 4) && ibCrcOk(d, 7));
                }
            }
        }
    }
}

// The speed byte lands in 0x38B byte 2 with a valid CRC (bench: 5A 40 19 05 for 25 at counter 0)
static void test_speed_byte_in_38b() {
    uint8_t b[4], c[4], d[7];
    ibBuild(0, true, IB_FLOW_ZERO, 25, b, c, d);
    const uint8_t e[4] = {0x5A, 0x40, 0x19, 0x05};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(e, b, 4);
    TEST_ASSERT_TRUE(ibCrcOk(b, 4));
}

static void test_39d_sum_checksum_real_frames() {
    for (uint8_t i = 0; i < sizeof(real39D) / sizeof(real39D[0]); i++) {
        TEST_ASSERT_TRUE(ibChecksumOk(IB_ID_STATUS, real39D[i], 5));
        TEST_ASSERT_FALSE(ibCrcOk(real39D[i], 5));   // J1850 does not hold on 0x39D
    }
    const uint8_t bad[5] = {0x01, 0x58, 0xE1, 0x27, 0x00};
    TEST_ASSERT_FALSE(ibChecksumOk(IB_ID_STATUS, bad, 5));
    // Any single-bit corruption of a data byte fails the sum
    for (uint8_t byte = 1; byte < 5; byte++) {
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint8_t f[5];
            memcpy(f, real39D[8], 5);
            f[byte] ^= (uint8_t)(1u << bit);
            TEST_ASSERT_FALSE(ibChecksumOk(IB_ID_STATUS, f, 5));
        }
    }
    TEST_ASSERT_FALSE(ibChecksumOk(IB_ID_STATUS, real39D[0], 1));
}

static void test_39d_decode_real_frames() {
    IbStatus s;
    TEST_ASSERT_TRUE(ibDecode39D(real39D[8], 5, s));        // F8 50 E1 27 00
    TEST_ASSERT_EQUAL(5, s.status);
    TEST_ASSERT_EQUAL(0, s.counter);
    TEST_ASSERT_EQUAL(1, s.drv);
    TEST_ASSERT_EQUAL(0, s.istate);
    TEST_ASSERT_EQUAL(319, s.rodRaw);
    TEST_ASSERT_EQUAL(-2, ibRodCentiMm(s.rodRaw));          // -0.02 mm
    TEST_ASSERT_TRUE(ibDecode39D(real39D[6], 5, s));        // 68 6B 35 28 00
    TEST_ASSERT_EQUAL(6, s.status);
    TEST_ASSERT_EQUAL(5, s.istate);
    TEST_ASSERT_EQUAL(1, s.drv);
    TEST_ASSERT_EQUAL(321, s.rodRaw);
    TEST_ASSERT_FALSE(ibDecode39D(real39D[6], 4, s));
}

// Synthetic 0x39D built from the DBC layout (bench test_decode_39d)
static void test_decode_39d_layout() {
    const uint16_t raw = 0xA5B;
    uint8_t f[8] = {0};
    f[1] = (uint8_t)((5 << 4) | 9);
    f[2] = (uint8_t)(1 | (3 << 2) | ((raw & 0x07) << 5));
    f[3] = (uint8_t)((raw >> 3) & 0xFF);
    f[4] = (uint8_t)((raw >> 11) & 0x01);
    IbStatus s;
    TEST_ASSERT_TRUE(ibDecode39D(f, 8, s));
    TEST_ASSERT_EQUAL(9, s.counter);
    TEST_ASSERT_EQUAL(5, s.status);
    TEST_ASSERT_EQUAL(1, s.drv);
    TEST_ASSERT_EQUAL(3, s.istate);
    TEST_ASSERT_EQUAL_HEX16(raw, s.rodRaw);
    TEST_ASSERT_EQUAL(0, ibRodCentiMm(IB_ROD_RAW_OFFSET));
    TEST_ASSERT_EQUAL(800, ibRodCentiMm(IB_ROD_RAW_OFFSET + 8 * 64));
    TEST_ASSERT_EQUAL(1600, ibRodCentiMm(IB_ROD_RAW_OFFSET + 16 * 64));
}

static void test_38e_38f_real_frames() {
    const uint8_t e1[6] = {0xF5, 0x29, 0x40, 0x11, 0x40, 0x11};
    const uint8_t e2[6] = {0x94, 0x25, 0x40, 0x11, 0x4F, 0x11};
    const uint8_t f1[6] = {0x90, 0x76, 0x42, 0x32, 0x80, 0x00};
    const uint8_t f2[6] = {0xA4, 0x70, 0x42, 0x32, 0x40, 0x1F};
    TEST_ASSERT_TRUE(ibChecksumOk(IB_ID_EXT_STATUS, e1, 6));
    TEST_ASSERT_TRUE(ibChecksumOk(IB_ID_EXT_STATUS, e2, 6));
    TEST_ASSERT_TRUE(ibChecksumOk(IB_ID_RUNOUT, f1, 6));
    TEST_ASSERT_TRUE(ibChecksumOk(IB_ID_RUNOUT, f2, 6));
    uint8_t bad[6];
    memcpy(bad, f2, 6);
    bad[3] ^= 0x01;
    TEST_ASSERT_FALSE(ibChecksumOk(IB_ID_RUNOUT, bad, 6));

    IbRunout r;
    TEST_ASSERT_TRUE(ibDecode38F(f1, 6, r));
    TEST_ASSERT_EQUAL(0, r.partner);
    TEST_ASSERT_EQUAL(0x32, r.runoutBar);   // 50 bar standing
    TEST_ASSERT_TRUE(ibDecode38F(f2, 6, r));
    TEST_ASSERT_EQUAL(1, r.partner);
    TEST_ASSERT_TRUE(ibDecode38F(e2, 6, r));
    TEST_ASSERT_EQUAL(-1, r.partner);        // byte 4 0x4F: neither flag
    TEST_ASSERT_FALSE(ibDecode38F(f2, 5, r));

    IbExtStatus e;
    TEST_ASSERT_TRUE(ibDecode38E(e1, 6, e));
    TEST_ASSERT_EQUAL(2, e.extStatus);
    TEST_ASSERT_EQUAL(0, e.aCmm);             // field A 0x140 = 0.00 mm
    TEST_ASSERT_EQUAL(0, e.bCmm);
    TEST_ASSERT_EQUAL(1, e.qualA);
    TEST_ASSERT_EQUAL(1, e.qualB);
    TEST_ASSERT_TRUE(ibDecode38E(e2, 6, e));
    TEST_ASSERT_EQUAL(2, e.extStatus);
    TEST_ASSERT_EQUAL(23, e.bCmm);            // 0x14F = 335 -> 0.23 mm
    TEST_ASSERT_FALSE(ibDecode38E(e1, 5, e));
}

static void test_pmc_packing() {
    uint8_t b5, b6;
    ibPmcBytes(117, 1, b5, b6);
    TEST_ASSERT_EQUAL_HEX8(0x50, b5);
    TEST_ASSERT_EQUAL_HEX8(0x47, b6);
    ibPmcBytes(100, 1, b5, b6);
    TEST_ASSERT_EQUAL_HEX8(0x40, b5);
    TEST_ASSERT_EQUAL_HEX8(0x46, b6);
    ibPmcBytes(117, 2, b5, b6);
    TEST_ASSERT_EQUAL_HEX8(0x50, b5);
    TEST_ASSERT_EQUAL_HEX8(0x87, b6);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_crc_trace_frames);
    RUN_TEST(test_build_matches_trace);
    RUN_TEST(test_idle_frames_have_no_request_bits);
    RUN_TEST(test_flow_clamped_and_no_0x80);
    RUN_TEST(test_speed_byte_in_38b);
    RUN_TEST(test_39d_sum_checksum_real_frames);
    RUN_TEST(test_39d_decode_real_frames);
    RUN_TEST(test_decode_39d_layout);
    RUN_TEST(test_38e_38f_real_frames);
    RUN_TEST(test_pmc_packing);
    return UNITY_END();
}
