// Native unit tests for the per-chip SSID / OTA hostname suffix (include/DeviceId.h).
// Run: pio test -e native
#include <unity.h>
#include <string.h>
#include "DeviceId.h"

using namespace DeviceId;

void setUp() {}
void tearDown() {}

// The quad board's base MAC 3C:DC:75:FC:5E:CC → "5ECC" (octets 4 and 5).
static void test_suffix_from_array() {
    const uint8_t mac[6] = {0x3C, 0xDC, 0x75, 0xFC, 0x5E, 0xCC};
    char s[5];
    formatChipSuffix(mac, s);
    TEST_ASSERT_EQUAL_STRING("5ECC", s);
}

// ESP.getEfuseMac() packs the same MAC little-endian: 0x0000CC5EFC75DC3C → "5ECC".
static void test_suffix_from_efuse_uint64() {
    char s[5];
    formatChipSuffixFromEfuse(0x0000CC5EFC75DC3CULL, s);
    TEST_ASSERT_EQUAL_STRING("5ECC", s);
}

// Leading zeros are kept: ..:0A:05 → "0A05".
static void test_suffix_leading_zeros() {
    const uint8_t mac[6] = {0x11, 0x22, 0x33, 0x44, 0x0A, 0x05};
    char s[5];
    formatChipSuffix(mac, s);
    TEST_ASSERT_EQUAL_STRING("0A05", s);
    formatChipSuffixFromEfuse(0x0000050A44332211ULL, s);
    TEST_ASSERT_EQUAL_STRING("0A05", s);
}

static void test_ssid_name() {
    char out[NAME_CAP];
    buildSuffixedName("QuadBike-Control", "5ECC", false, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("QuadBike-Control-5ECC", out);
}

static void test_hostname_lowercase() {
    char out[NAME_CAP];
    buildSuffixedName("quadbike-control", "5ECC", true, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("quadbike-control-5ecc", out);
}

// A 40-char base is cut so the SSID is exactly 32 chars and still ends with "-5ECC".
static void test_long_base_truncated() {
    const char* base = "ABCDEFGHIJABCDEFGHIJABCDEFGHIJABCDEFGHIJ";   // 40 chars
    char out[NAME_CAP];
    buildSuffixedName(base, "5ECC", false, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(32, strlen(out));
    TEST_ASSERT_EQUAL_STRING("-5ECC", out + 27);
    TEST_ASSERT_EQUAL_INT(0, strncmp(out, base, 27));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_suffix_from_array);
    RUN_TEST(test_suffix_from_efuse_uint64);
    RUN_TEST(test_suffix_leading_zeros);
    RUN_TEST(test_ssid_name);
    RUN_TEST(test_hostname_lowercase);
    RUN_TEST(test_long_base_truncated);
    return UNITY_END();
}
