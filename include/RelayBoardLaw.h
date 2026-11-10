#ifndef RELAY_BOARD_LAW_H
#define RELAY_BOARD_LAW_H

// Relay board law: Relay_N -> port-bit tables, function-mask construction and the
// relay-board detection decision. Pure, header-only and Arduino-free so the native
// suite test/test_relay_board can exercise it on the host (`pio test -e native`).
//
// Two relay boards can sit on I2C2 / header X15 (one at a time):
//   - the custom PCA9557 relay board at 0x1F (Relay.PrjPcb) — IO routing NOT 1:1;
//   - the XL9535-K8V5 (PCA9535-compatible, P0 only) at 0x20..0x27 — identity routing.
// The per-vehicle function assignment lives in Constants.h as RELAY-NUMBER sets
// (bit0 = Relay_1 … bit7 = Relay_8, the same encoding as the `relay_test` telemetry);
// RelayController::begin() maps them through the detected board's table.

#include <stdint.h>
#include <string.h>

/** @brief Relay-number set bit for Relay_n (n = 1..8): bit0 = Relay_1 … bit7 = Relay_8 */
#define RELAY_N(n) ((uint8_t)(1u << ((n) - 1)))

// Probed addresses. Only these nine are ever used for the board decision; the on-board
// PCA9557s at 0x1A (opto inputs) and 0x1C (mux, never addressed) are never a relay board.
constexpr uint8_t RELAY_LAW_PCA9557_ADDR     = 0x1F;
constexpr uint8_t RELAY_LAW_XL9535_ADDR_MIN  = 0x20;
constexpr uint8_t RELAY_LAW_XL9535_ADDR_MAX  = 0x27;
// Diagnostic scan range on a detection failure (the PCA9557 block + the XL9535 block).
constexpr uint8_t RELAY_LAW_SCAN_ADDR_MIN    = 0x18;
constexpr uint8_t RELAY_LAW_SCAN_ADDR_MAX    = 0x27;

// ---------------------------------------------------------------------------
// Relay_N -> port bit tables (index = relay number - 1)
// ---------------------------------------------------------------------------

// ⚠ PCA9557 relay board (Relay.PrjPcb netlist). NOT 1:1 — do NOT "correct" it to
//   Relay_N = bit N-1: that would command the starter when the light is requested.
//   Relay_1=IO4 Relay_2=IO5 Relay_3=IO6 Relay_4=IO7 Relay_5=IO3 Relay_6=IO2 Relay_7=IO0 Relay_8=IO1
constexpr uint8_t RELAY_TABLE_PCA9557[8] = {
    (uint8_t)(1u << 4), (uint8_t)(1u << 5), (uint8_t)(1u << 6), (uint8_t)(1u << 7),
    (uint8_t)(1u << 3), (uint8_t)(1u << 2), (uint8_t)(1u << 0), (uint8_t)(1u << 1)
};

// XL9535-K8V5: Relay_N = P0.(N-1), active-high (bit 1 = energized).
// Bench-confirmed 2026-11-07 on an XL9535-K8V5 @0x20 (firmware 1.5.24, tasks.md 1.4).
// Should another board revision prove permuted or active-low, only this table (and, for
// active-low, the driver) changes.
constexpr uint8_t RELAY_TABLE_XL9535[8] = {
    (uint8_t)(1u << 0), (uint8_t)(1u << 1), (uint8_t)(1u << 2), (uint8_t)(1u << 3),
    (uint8_t)(1u << 4), (uint8_t)(1u << 5), (uint8_t)(1u << 6), (uint8_t)(1u << 7)
};

/** @brief Port bit of relay `relayNumber` (1..8) in `table`; 0 for any other number */
constexpr uint8_t relayTableMask(const uint8_t (&table)[8], uint8_t relayNumber) {
    return (relayNumber >= 1 && relayNumber <= 8) ? table[relayNumber - 1] : (uint8_t)0;
}

/** @brief Whole-port mask of a relay-number set (bit0 = Relay_1 …) mapped through `table` */
constexpr uint8_t relayPortMask(const uint8_t (&table)[8], uint8_t relaySet) {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (relaySet & (uint8_t)(1u << i)) {
            mask = (uint8_t)(mask | table[i]);
        }
    }
    return mask;
}

/** @brief Inverse of relayPortMask(): port bits -> relay-number set */
constexpr uint8_t relaySetFromPort(const uint8_t (&table)[8], uint8_t portBits) {
    uint8_t set = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (portBits & table[i]) {
            set = (uint8_t)(set | (uint8_t)(1u << i));
        }
    }
    return set;
}

/** @brief True if `table` maps the eight relays onto eight distinct single port bits */
constexpr bool relayTableIsPermutation(const uint8_t (&table)[8]) {
    uint8_t seen = 0;
    for (uint8_t i = 0; i < 8; i++) {
        const uint8_t b = table[i];
        if (b == 0 || (b & (uint8_t)(b - 1)) != 0) return false;   // exactly one bit
        if (seen & b) return false;                                 // distinct
        seen = (uint8_t)(seen | b);
    }
    return seen == 0xFF;
}

/**
 * @brief Function sets (or masks) are valid: ignition, starter and front light non-empty,
 * every set pairwise disjoint, and together with the reserved set covering all eight bits.
 * `wheelLock` is 0 where the lock is not fitted (RELAY_WHEEL_LOCK_FITTED = 0).
 */
constexpr bool relaySetsValid(uint8_t ignition, uint8_t starter, uint8_t frontLight,
                              uint8_t wheelLock, uint8_t reserved) {
    return ignition != 0 && starter != 0 && frontLight != 0 &&
           (ignition & starter) == 0 && (ignition & frontLight) == 0 &&
           (ignition & wheelLock) == 0 && (ignition & reserved) == 0 &&
           (starter & frontLight) == 0 && (starter & wheelLock) == 0 &&
           (starter & reserved) == 0 && (frontLight & wheelLock) == 0 &&
           (frontLight & reserved) == 0 && (wheelLock & reserved) == 0 &&
           (uint8_t)(ignition | starter | frontLight | wheelLock | reserved) == 0xFF;
}

/** @brief Function port masks of one board (wheelLock = 0 where not fitted) */
struct RelayPortMasks {
    uint8_t ignition;
    uint8_t starter;
    uint8_t frontLight;
    uint8_t wheelLock;
    uint8_t reserved;
};

/** @brief Map the relay-number sets through `table` */
constexpr RelayPortMasks relayBuildMasks(const uint8_t (&table)[8], uint8_t ignitionSet,
                                         uint8_t starterSet, uint8_t frontLightSet,
                                         uint8_t wheelLockSet, uint8_t reservedSet) {
    return RelayPortMasks{
        relayPortMask(table, ignitionSet),
        relayPortMask(table, starterSet),
        relayPortMask(table, frontLightSet),
        relayPortMask(table, wheelLockSet),
        relayPortMask(table, reservedSet)
    };
}

/** @brief Port masks disjoint and OR to 0xFF (same rule as relaySetsValid) */
constexpr bool relayMasksValid(const RelayPortMasks& m) {
    return relaySetsValid(m.ignition, m.starter, m.frontLight, m.wheelLock, m.reserved);
}

static_assert(relayTableIsPermutation(RELAY_TABLE_PCA9557), "PCA9557 relay table is not a permutation");
static_assert(relayTableIsPermutation(RELAY_TABLE_XL9535), "XL9535 relay table is not a permutation");

// ---------------------------------------------------------------------------
// Board selection and detection
// ---------------------------------------------------------------------------

/** @brief Persisted setting (NVS `relay`/`board`, uint8). Out-of-range raw values read as AUTO. */
enum class RelayBoardSetting : uint8_t { AUTO = 0, PCA9557 = 1, XL9535 = 2 };

/** @brief Board in use (latched at boot) */
enum class RelayBoardKind : uint8_t { NONE = 0, PCA9557 = 1, XL9535 = 2 };

/** @brief Why no relay board is in use (telemetry `relay_board_error`) */
enum class RelayBoardError : uint8_t {
    NONE = 0,
    BOTH_FOUND,        // auto: 0x1F and an XL9535 address both answer
    NONE_FOUND,        // none of 0x1F, 0x20..0x27 answers
    MULTIPLE_XL9535,   // auto / xl9535: two or more of 0x20..0x27 answer
    CFG_MISMATCH,      // manual selection contradicts the bus
    INIT_FAILED        // detected, but the all-off initialization did not verify
};

// Probe result bitmask: bit i (0..7) = 0x20 + i ACKed; bit 8 = 0x1F ACKed.
constexpr uint16_t RELAY_ACK_PCA9557 = (uint16_t)(1u << 8);
constexpr uint16_t relayAckXl9535(uint8_t address) {
    return (address >= RELAY_LAW_XL9535_ADDR_MIN && address <= RELAY_LAW_XL9535_ADDR_MAX)
               ? (uint16_t)(1u << (address - RELAY_LAW_XL9535_ADDR_MIN)) : (uint16_t)0;
}

/** @brief Decision: kind NONE iff error != NONE; address 0 when NONE */
struct RelayBoardDecision {
    RelayBoardKind kind;
    uint8_t address;
    RelayBoardError error;
};

/** @brief Raw NVS byte -> setting; `valid` false (and AUTO returned) for an out-of-range value */
constexpr RelayBoardSetting relayBoardSettingFromRaw(uint8_t raw, bool& valid) {
    valid = raw <= (uint8_t)RelayBoardSetting::XL9535;
    return valid ? (RelayBoardSetting)raw : RelayBoardSetting::AUTO;
}

/**
 * @brief The detection decision (design.md table d).
 *
 * auto    : 0x1F only -> PCA9557; exactly one XL9535 address and no 0x1F -> XL9535;
 *           0x1F plus any XL9535 address -> both_found (checked before multiple_xl9535);
 *           two or more XL9535 addresses -> multiple_xl9535.
 * pca9557 : 0x1F -> PCA9557 (XL9535 addresses ignored); no 0x1F -> cfg_mismatch.
 * xl9535  : exactly one XL9535 address -> XL9535 (0x1F ignored); two or more ->
 *           multiple_xl9535; none (0x1F present) -> cfg_mismatch.
 * Any setting with nothing answering -> none_found. Never a guess.
 */
constexpr RelayBoardDecision relayBoardDecide(uint16_t ackMask, RelayBoardSetting setting) {
    const bool pca = (ackMask & RELAY_ACK_PCA9557) != 0;
    uint8_t xlCount = 0;
    uint8_t xlAddr = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (ackMask & (uint16_t)(1u << i)) {
            xlCount++;
            if (xlAddr == 0) xlAddr = (uint8_t)(RELAY_LAW_XL9535_ADDR_MIN + i);
        }
    }
    const RelayBoardDecision none = {RelayBoardKind::NONE, 0, RelayBoardError::NONE_FOUND};
    if (!pca && xlCount == 0) return none;

    switch (setting) {
        case RelayBoardSetting::PCA9557:
            if (pca) return {RelayBoardKind::PCA9557, RELAY_LAW_PCA9557_ADDR, RelayBoardError::NONE};
            return {RelayBoardKind::NONE, 0, RelayBoardError::CFG_MISMATCH};
        case RelayBoardSetting::XL9535:
            if (xlCount == 0) return {RelayBoardKind::NONE, 0, RelayBoardError::CFG_MISMATCH};
            if (xlCount > 1)  return {RelayBoardKind::NONE, 0, RelayBoardError::MULTIPLE_XL9535};
            return {RelayBoardKind::XL9535, xlAddr, RelayBoardError::NONE};
        case RelayBoardSetting::AUTO:
        default:
            if (pca && xlCount > 0) return {RelayBoardKind::NONE, 0, RelayBoardError::BOTH_FOUND};
            if (pca) return {RelayBoardKind::PCA9557, RELAY_LAW_PCA9557_ADDR, RelayBoardError::NONE};
            if (xlCount > 1) return {RelayBoardKind::NONE, 0, RelayBoardError::MULTIPLE_XL9535};
            return {RelayBoardKind::XL9535, xlAddr, RelayBoardError::NONE};
    }
}

// ---------------------------------------------------------------------------
// Tokens (telemetry / web command) and log names. All static strings.
// ---------------------------------------------------------------------------

inline const char* relayBoardSettingToken(RelayBoardSetting s) {
    switch (s) {
        case RelayBoardSetting::PCA9557: return "pca9557";
        case RelayBoardSetting::XL9535:  return "xl9535";
        case RelayBoardSetting::AUTO:
        default:                         return "auto";
    }
}

inline const char* relayBoardKindToken(RelayBoardKind k) {
    switch (k) {
        case RelayBoardKind::PCA9557: return "pca9557";
        case RelayBoardKind::XL9535:  return "xl9535";
        case RelayBoardKind::NONE:
        default:                      return "none";
    }
}

/** @brief Log name of a board ("PCA9557" / "XL9535" / "none") */
inline const char* relayBoardKindName(RelayBoardKind k) {
    switch (k) {
        case RelayBoardKind::PCA9557: return "PCA9557";
        case RelayBoardKind::XL9535:  return "XL9535";
        case RelayBoardKind::NONE:
        default:                      return "none";
    }
}

/** @brief Telemetry token of an error; nullptr for NONE (JSON null) */
inline const char* relayBoardErrorToken(RelayBoardError e) {
    switch (e) {
        case RelayBoardError::BOTH_FOUND:      return "both_found";
        case RelayBoardError::NONE_FOUND:      return "none_found";
        case RelayBoardError::MULTIPLE_XL9535: return "multiple_xl9535";
        case RelayBoardError::CFG_MISMATCH:    return "cfg_mismatch";
        case RelayBoardError::INIT_FAILED:     return "init_failed";
        case RelayBoardError::NONE:
        default:                               return nullptr;
    }
}

/** @brief Parse a web token ("auto" / "pca9557" / "xl9535"); false for anything else */
inline bool relayBoardSettingParse(const char* token, RelayBoardSetting& out) {
    if (token == nullptr) return false;
    if (strcmp(token, "auto") == 0)    { out = RelayBoardSetting::AUTO;    return true; }
    if (strcmp(token, "pca9557") == 0) { out = RelayBoardSetting::PCA9557; return true; }
    if (strcmp(token, "xl9535") == 0)  { out = RelayBoardSetting::XL9535;  return true; }
    return false;
}

#endif // RELAY_BOARD_LAW_H
