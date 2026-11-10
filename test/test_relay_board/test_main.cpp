// Native unit tests for the relay board law (include/RelayBoardLaw.h): Relay_N -> bit
// tables, function-mask construction and the relay-board detection decision.
// Run: pio test -e native
#include <unity.h>
#include "RelayBoardLaw.h"

void setUp() {}
void tearDown() {}

// Mirror of the per-vehicle relay-number sets (include/Constants.h needs Arduino.h).
// aztec buggy: ignition Relay_1, starter Relay_2, light Relay_3, lock Relay_4+5+6
// (switched together), reserved Relay_7..8.
static const uint8_t AZ_IGN   = RELAY_N(1);
static const uint8_t AZ_START = RELAY_N(2);
static const uint8_t AZ_LIGHT = RELAY_N(3);
static const uint8_t AZ_LOCK  = RELAY_N(4) | RELAY_N(5) | RELAY_N(6);
static const uint8_t AZ_RES   = RELAY_N(7) | RELAY_N(8);
// quad bike (master): ignition {1,2}, starter {3}, light {4}, wheel lock {5,6}, reserved {7,8}.
static const uint8_t Q_IGN   = RELAY_N(1) | RELAY_N(2);
static const uint8_t Q_START = RELAY_N(3);
static const uint8_t Q_LIGHT = RELAY_N(4);
static const uint8_t Q_LOCK  = RELAY_N(5) | RELAY_N(6);
static const uint8_t Q_RES   = RELAY_N(7) | RELAY_N(8);

static void test_relay_n_encoding() {
    TEST_ASSERT_EQUAL_HEX8(0x01, RELAY_N(1));
    TEST_ASSERT_EQUAL_HEX8(0x80, RELAY_N(8));
}

static void test_tables_are_permutations() {
    TEST_ASSERT_TRUE(relayTableIsPermutation(RELAY_TABLE_PCA9557));
    TEST_ASSERT_TRUE(relayTableIsPermutation(RELAY_TABLE_XL9535));
    const uint8_t dup[8] = {0x01, 0x01, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
    const uint8_t twoBits[8] = {0x03, 0x00, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
    const uint8_t zero[8] = {0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
    TEST_ASSERT_FALSE(relayTableIsPermutation(dup));
    TEST_ASSERT_FALSE(relayTableIsPermutation(twoBits));
    TEST_ASSERT_FALSE(relayTableIsPermutation(zero));
}

static void test_pca9557_routing() {
    // Relay.PrjPcb: Relay_1=IO4 Relay_2=IO5 Relay_3=IO6 Relay_4=IO7 Relay_5=IO3 Relay_6=IO2 Relay_7=IO0 Relay_8=IO1
    const uint8_t expect[8] = {0x10, 0x20, 0x40, 0x80, 0x08, 0x04, 0x01, 0x02};
    for (uint8_t n = 1; n <= 8; n++) {
        TEST_ASSERT_EQUAL_HEX8(expect[n - 1], relayTableMask(RELAY_TABLE_PCA9557, n));
        TEST_ASSERT_EQUAL_HEX8((uint8_t)(1u << (n - 1)), relayTableMask(RELAY_TABLE_XL9535, n));
    }
    TEST_ASSERT_EQUAL_HEX8(0, relayTableMask(RELAY_TABLE_PCA9557, 0));
    TEST_ASSERT_EQUAL_HEX8(0, relayTableMask(RELAY_TABLE_PCA9557, 9));
    TEST_ASSERT_EQUAL_HEX8(0, relayTableMask(RELAY_TABLE_XL9535, 0));
    TEST_ASSERT_EQUAL_HEX8(0, relayTableMask(RELAY_TABLE_XL9535, 9));
}

static void test_aztec_masks_pca9557() {
    RelayPortMasks m = relayBuildMasks(RELAY_TABLE_PCA9557, AZ_IGN, AZ_START, AZ_LIGHT, AZ_LOCK, AZ_RES);
    TEST_ASSERT_EQUAL_HEX8(0x10, m.ignition);
    TEST_ASSERT_EQUAL_HEX8(0x20, m.starter);
    TEST_ASSERT_EQUAL_HEX8(0x40, m.frontLight);
    // ⚠ non-1:1: Relay_4=IO7, Relay_5=IO3, Relay_6=IO2 (X7/X8/X9) — NOT 0x38
    TEST_ASSERT_EQUAL_HEX8(0x8C, m.wheelLock);
    TEST_ASSERT_EQUAL_HEX8(0x03, m.reserved);
    TEST_ASSERT_TRUE(relayMasksValid(m));
    // CRANKING pins: ignition + starter
    TEST_ASSERT_EQUAL_HEX8(0x30, (uint8_t)(m.ignition | m.starter));
}

static void test_aztec_masks_xl9535() {
    RelayPortMasks m = relayBuildMasks(RELAY_TABLE_XL9535, AZ_IGN, AZ_START, AZ_LIGHT, AZ_LOCK, AZ_RES);
    TEST_ASSERT_EQUAL_HEX8(0x01, m.ignition);
    TEST_ASSERT_EQUAL_HEX8(0x02, m.starter);
    TEST_ASSERT_EQUAL_HEX8(0x04, m.frontLight);
    TEST_ASSERT_EQUAL_HEX8(0x38, m.wheelLock);   // P0.3..P0.5
    TEST_ASSERT_EQUAL_HEX8(0xC0, m.reserved);
    TEST_ASSERT_TRUE(relayMasksValid(m));
    TEST_ASSERT_EQUAL_HEX8(0x03, (uint8_t)(m.ignition | m.starter));
}

// Lock engaged together with the other functions: on either board the energized pins are
// exactly the OR of the function masks, and the spare relays stay off.
static void test_aztec_lock_with_functions() {
    const RelayPortMasks p = relayBuildMasks(RELAY_TABLE_PCA9557, AZ_IGN, AZ_START, AZ_LIGHT, AZ_LOCK, AZ_RES);
    const RelayPortMasks x = relayBuildMasks(RELAY_TABLE_XL9535, AZ_IGN, AZ_START, AZ_LIGHT, AZ_LOCK, AZ_RES);
    TEST_ASSERT_EQUAL_HEX8(0xFC, (uint8_t)(p.ignition | p.starter | p.frontLight | p.wheelLock));
    TEST_ASSERT_EQUAL_HEX8(0x3F, (uint8_t)(x.ignition | x.starter | x.frontLight | x.wheelLock));
    TEST_ASSERT_EQUAL_HEX8(0x00, (uint8_t)(p.wheelLock & p.reserved));
    TEST_ASSERT_EQUAL_HEX8(0x00, (uint8_t)(x.wheelLock & x.reserved));
    // each lock relay individually (relay test / relayIoMask) lies inside the lock mask
    for (uint8_t n = 4; n <= 6; n++) {
        TEST_ASSERT_TRUE((relayTableMask(RELAY_TABLE_PCA9557, n) & p.wheelLock) != 0);
        TEST_ASSERT_TRUE((relayTableMask(RELAY_TABLE_XL9535, n) & x.wheelLock) != 0);
    }
}

static void test_quad_masks_both_tables() {
    RelayPortMasks p = relayBuildMasks(RELAY_TABLE_PCA9557, Q_IGN, Q_START, Q_LIGHT, Q_LOCK, Q_RES);
    TEST_ASSERT_EQUAL_HEX8(0x30, p.ignition);
    TEST_ASSERT_EQUAL_HEX8(0x40, p.starter);
    TEST_ASSERT_EQUAL_HEX8(0x80, p.frontLight);
    TEST_ASSERT_EQUAL_HEX8(0x0C, p.wheelLock);
    TEST_ASSERT_EQUAL_HEX8(0x03, p.reserved);
    TEST_ASSERT_TRUE(relayMasksValid(p));

    RelayPortMasks x = relayBuildMasks(RELAY_TABLE_XL9535, Q_IGN, Q_START, Q_LIGHT, Q_LOCK, Q_RES);
    TEST_ASSERT_EQUAL_HEX8(0x03, x.ignition);
    TEST_ASSERT_EQUAL_HEX8(0x04, x.starter);
    TEST_ASSERT_EQUAL_HEX8(0x08, x.frontLight);
    TEST_ASSERT_EQUAL_HEX8(0x30, x.wheelLock);
    TEST_ASSERT_EQUAL_HEX8(0xC0, x.reserved);
    TEST_ASSERT_TRUE(relayMasksValid(x));
}

static void test_set_invariants() {
    TEST_ASSERT_TRUE(relaySetsValid(AZ_IGN, AZ_START, AZ_LIGHT, AZ_LOCK, AZ_RES));
    TEST_ASSERT_TRUE(relaySetsValid(Q_IGN, Q_START, Q_LIGHT, Q_LOCK, Q_RES));
    // overlap (lock relay also reserved / also the light)
    TEST_ASSERT_FALSE(relaySetsValid(AZ_IGN | AZ_START, AZ_START, AZ_LIGHT, AZ_LOCK, AZ_RES));
    TEST_ASSERT_FALSE(relaySetsValid(AZ_IGN, AZ_START, AZ_LIGHT, AZ_LOCK, (uint8_t)(AZ_RES | RELAY_N(4))));
    TEST_ASSERT_FALSE(relaySetsValid(AZ_IGN, AZ_START, AZ_LIGHT, (uint8_t)(AZ_LOCK | AZ_LIGHT), AZ_RES));
    // gap (Relay_8 uncovered; lock missing Relay_6)
    TEST_ASSERT_FALSE(relaySetsValid(AZ_IGN, AZ_START, AZ_LIGHT, AZ_LOCK, (uint8_t)(AZ_RES & ~RELAY_N(8))));
    TEST_ASSERT_FALSE(relaySetsValid(AZ_IGN, AZ_START, AZ_LIGHT, (uint8_t)(AZ_LOCK & ~RELAY_N(6)), AZ_RES));
    // the lock left out (the pre-1.5.25 "not fitted" shape) no longer covers the port
    TEST_ASSERT_FALSE(relaySetsValid(AZ_IGN, AZ_START, AZ_LIGHT, 0, AZ_RES));
    // empty function
    TEST_ASSERT_FALSE(relaySetsValid(0, AZ_START, AZ_LIGHT, AZ_IGN, AZ_RES));
}

static void test_port_to_set_roundtrip() {
    for (uint16_t set = 0; set <= 0xFF; set++) {
        TEST_ASSERT_EQUAL_HEX8((uint8_t)set,
            relaySetFromPort(RELAY_TABLE_PCA9557, relayPortMask(RELAY_TABLE_PCA9557, (uint8_t)set)));
        TEST_ASSERT_EQUAL_HEX8((uint8_t)set,
            relaySetFromPort(RELAY_TABLE_XL9535, relayPortMask(RELAY_TABLE_XL9535, (uint8_t)set)));
    }
    // relay_test telemetry: Relay_5 test-energized -> 16 on either board
    TEST_ASSERT_EQUAL_UINT8(16, relaySetFromPort(RELAY_TABLE_PCA9557, 0x08));   // IO3
    TEST_ASSERT_EQUAL_UINT8(16, relaySetFromPort(RELAY_TABLE_XL9535, 0x10));    // P0.4
    // aztec lock mask maps back to exactly Relay_4+5+6 on either board
    TEST_ASSERT_EQUAL_HEX8(AZ_LOCK, relaySetFromPort(RELAY_TABLE_PCA9557, 0x8C));
    TEST_ASSERT_EQUAL_HEX8(AZ_LOCK, relaySetFromPort(RELAY_TABLE_XL9535, 0x38));
}

// ---- detection -------------------------------------------------------------

static const uint16_t ACK_NONE = 0;
static const uint16_t ACK_PCA  = RELAY_ACK_PCA9557;
static const uint16_t ACK_XL20 = relayAckXl9535(0x20);
static const uint16_t ACK_XL25 = relayAckXl9535(0x25);
static const uint16_t ACK_BOTH = (uint16_t)(RELAY_ACK_PCA9557 | relayAckXl9535(0x20));
static const uint16_t ACK_TWO  = (uint16_t)(relayAckXl9535(0x20) | relayAckXl9535(0x27));

static void assertDecision(RelayBoardKind kind, uint8_t addr, RelayBoardError err,
                           const RelayBoardDecision& d) {
    TEST_ASSERT_EQUAL_UINT8((uint8_t)kind, (uint8_t)d.kind);
    TEST_ASSERT_EQUAL_HEX8(addr, d.address);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)err, (uint8_t)d.error);
}

static void test_ack_bits() {
    TEST_ASSERT_EQUAL_HEX16(0x0001, relayAckXl9535(0x20));
    TEST_ASSERT_EQUAL_HEX16(0x0080, relayAckXl9535(0x27));
    TEST_ASSERT_EQUAL_HEX16(0x0000, relayAckXl9535(0x1F));
    TEST_ASSERT_EQUAL_HEX16(0x0000, relayAckXl9535(0x28));
    TEST_ASSERT_EQUAL_HEX16(0x0000, relayAckXl9535(0x1A));
    TEST_ASSERT_EQUAL_HEX16(0x0000, relayAckXl9535(0x1C));
}

static void test_decide_auto() {
    const RelayBoardSetting s = RelayBoardSetting::AUTO;
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::NONE_FOUND, relayBoardDecide(ACK_NONE, s));
    assertDecision(RelayBoardKind::PCA9557, 0x1F, RelayBoardError::NONE, relayBoardDecide(ACK_PCA, s));
    assertDecision(RelayBoardKind::XL9535, 0x20, RelayBoardError::NONE, relayBoardDecide(ACK_XL20, s));
    assertDecision(RelayBoardKind::XL9535, 0x25, RelayBoardError::NONE, relayBoardDecide(ACK_XL25, s));
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::BOTH_FOUND, relayBoardDecide(ACK_BOTH, s));
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::MULTIPLE_XL9535, relayBoardDecide(ACK_TWO, s));
    // 0x1F plus two XL9535 addresses: both_found wins (checked first)
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::BOTH_FOUND,
                   relayBoardDecide((uint16_t)(ACK_PCA | ACK_TWO), s));
}

static void test_decide_pca9557() {
    const RelayBoardSetting s = RelayBoardSetting::PCA9557;
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::NONE_FOUND, relayBoardDecide(ACK_NONE, s));
    assertDecision(RelayBoardKind::PCA9557, 0x1F, RelayBoardError::NONE, relayBoardDecide(ACK_PCA, s));
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::CFG_MISMATCH, relayBoardDecide(ACK_XL20, s));
    assertDecision(RelayBoardKind::PCA9557, 0x1F, RelayBoardError::NONE, relayBoardDecide(ACK_BOTH, s));
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::CFG_MISMATCH, relayBoardDecide(ACK_TWO, s));
    assertDecision(RelayBoardKind::PCA9557, 0x1F, RelayBoardError::NONE,
                   relayBoardDecide((uint16_t)(ACK_PCA | ACK_TWO), s));
}

static void test_decide_xl9535() {
    const RelayBoardSetting s = RelayBoardSetting::XL9535;
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::NONE_FOUND, relayBoardDecide(ACK_NONE, s));
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::CFG_MISMATCH, relayBoardDecide(ACK_PCA, s));
    assertDecision(RelayBoardKind::XL9535, 0x20, RelayBoardError::NONE, relayBoardDecide(ACK_XL20, s));
    assertDecision(RelayBoardKind::XL9535, 0x20, RelayBoardError::NONE, relayBoardDecide(ACK_BOTH, s));
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::MULTIPLE_XL9535, relayBoardDecide(ACK_TWO, s));
    assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::MULTIPLE_XL9535,
                   relayBoardDecide((uint16_t)(ACK_PCA | ACK_TWO), s));
}

static void test_out_of_range_setting() {
    bool valid = true;
    TEST_ASSERT_EQUAL_UINT8((uint8_t)RelayBoardSetting::AUTO, (uint8_t)relayBoardSettingFromRaw(0, valid));
    TEST_ASSERT_TRUE(valid);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)RelayBoardSetting::PCA9557, (uint8_t)relayBoardSettingFromRaw(1, valid));
    TEST_ASSERT_TRUE(valid);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)RelayBoardSetting::XL9535, (uint8_t)relayBoardSettingFromRaw(2, valid));
    TEST_ASSERT_TRUE(valid);
    const uint8_t bad[] = {3, 7, 0x80, 0xFF};
    for (uint8_t raw : bad) {
        valid = true;
        RelayBoardSetting s = relayBoardSettingFromRaw(raw, valid);
        TEST_ASSERT_FALSE(valid);
        TEST_ASSERT_EQUAL_UINT8((uint8_t)RelayBoardSetting::AUTO, (uint8_t)s);
        // ... and decides exactly like auto
        assertDecision(RelayBoardKind::NONE, 0, RelayBoardError::BOTH_FOUND, relayBoardDecide(ACK_BOTH, s));
        assertDecision(RelayBoardKind::XL9535, 0x20, RelayBoardError::NONE, relayBoardDecide(ACK_XL20, s));
    }
    // An out-of-range enum value passed straight in also behaves like auto.
    assertDecision(RelayBoardKind::PCA9557, 0x1F, RelayBoardError::NONE,
                   relayBoardDecide(ACK_PCA, (RelayBoardSetting)9));
}

static void test_tokens() {
    RelayBoardSetting s = RelayBoardSetting::AUTO;
    TEST_ASSERT_TRUE(relayBoardSettingParse("xl9535", s));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)RelayBoardSetting::XL9535, (uint8_t)s);
    TEST_ASSERT_TRUE(relayBoardSettingParse("pca9557", s));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)RelayBoardSetting::PCA9557, (uint8_t)s);
    TEST_ASSERT_TRUE(relayBoardSettingParse("auto", s));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)RelayBoardSetting::AUTO, (uint8_t)s);
    TEST_ASSERT_FALSE(relayBoardSettingParse("", s));
    TEST_ASSERT_FALSE(relayBoardSettingParse("XL9535", s));
    TEST_ASSERT_FALSE(relayBoardSettingParse(nullptr, s));

    TEST_ASSERT_EQUAL_STRING("auto", relayBoardSettingToken(RelayBoardSetting::AUTO));
    TEST_ASSERT_EQUAL_STRING("none", relayBoardKindToken(RelayBoardKind::NONE));
    TEST_ASSERT_EQUAL_STRING("xl9535", relayBoardKindToken(RelayBoardKind::XL9535));
    TEST_ASSERT_EQUAL_STRING("pca9557", relayBoardKindToken(RelayBoardKind::PCA9557));
    TEST_ASSERT_NULL(relayBoardErrorToken(RelayBoardError::NONE));
    TEST_ASSERT_EQUAL_STRING("both_found", relayBoardErrorToken(RelayBoardError::BOTH_FOUND));
    TEST_ASSERT_EQUAL_STRING("none_found", relayBoardErrorToken(RelayBoardError::NONE_FOUND));
    TEST_ASSERT_EQUAL_STRING("multiple_xl9535", relayBoardErrorToken(RelayBoardError::MULTIPLE_XL9535));
    TEST_ASSERT_EQUAL_STRING("cfg_mismatch", relayBoardErrorToken(RelayBoardError::CFG_MISMATCH));
    TEST_ASSERT_EQUAL_STRING("init_failed", relayBoardErrorToken(RelayBoardError::INIT_FAILED));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_relay_n_encoding);
    RUN_TEST(test_tables_are_permutations);
    RUN_TEST(test_pca9557_routing);
    RUN_TEST(test_aztec_masks_pca9557);
    RUN_TEST(test_aztec_masks_xl9535);
    RUN_TEST(test_aztec_lock_with_functions);
    RUN_TEST(test_quad_masks_both_tables);
    RUN_TEST(test_set_invariants);
    RUN_TEST(test_port_to_set_roundtrip);
    RUN_TEST(test_ack_bits);
    RUN_TEST(test_decide_auto);
    RUN_TEST(test_decide_pca9557);
    RUN_TEST(test_decide_xl9535);
    RUN_TEST(test_out_of_range_setting);
    RUN_TEST(test_tokens);
    return UNITY_END();
}
