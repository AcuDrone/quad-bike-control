#ifndef IBOOSTER_PROTO_H
#define IBOOSTER_PROTO_H

/**
 * @file IboosterProto.h
 * @brief Bosch iBooster Gen2 (Tesla Model 3/Y, P/N 1044671-00-x) frame builder / decoder for the
 *        control bus (X150 pins 18 / 10, 500 kbit/s). Header-only, <stdint.h> only (no Arduino,
 *        no Constants.h): native-tested in test/test_ibooster_proto with the bench vectors.
 *        Change replace-aztec-brake-with-ibooster, design D3 / D5.
 *
 * Ported from the bench tool Arduino/iBooster-TEST/src/ibooster_proto.h (validated against the
 * 48 frames of the reference trace and real frames of one unit, README «Що виміряно на стенді»,
 * 22–23.11.2026). Bench-only helpers (ibBuildEx*, the dynamic master-cylinder pressure) are not
 * ported: the firmware sends the bench content (constant pMC1 117 / qualifier 1, byte 3 of 0x38B =
 * 0x05); the only variable fields are the counter, the request bits, the flow and the 0x38B
 * vehicle-speed byte.
 *
 * Our frames, every 10 ms (byte 0 = CRC-8 SAE J1850 over bytes 1..end, low nibble of byte 1 = one
 * 4-bit counter shared by the three frames of a slot):
 *   0x38B DLC 4: [CRC] [Q|cnt] [speed] 05
 *   0x38C DLC 4: [CRC] [Q|cnt] flow_lo flow_hi       (flow little-endian, 0x7E00 = 0)
 *   0x38D DLC 7: [CRC] [cnt]   00 00 00 50 47        (pMC1 117 = 5.1 bar, qualifier 1)
 * Q = 0x40 (external request) while requesting, 0x00 otherwise. 0x80 in 0x38B ("speed faulty")
 * and a pMC1 qualifier other than 1 make the unit drop the request at once (bench): this builder
 * can produce neither. In 0x38B, byte 1 bits 6-7 are also the vehicle-speed qualifier
 * (VehicleSpeed_Q, bench field table: 0 not initialised, 1 normal, 2 faulty): Q = 0x40 is
 * qualifier 1, so frames without a request carry qualifier 0 (whether the unit uses their speed
 * byte is not known). The speed byte (0.4 km/h per count) is never below 5 counts = 2 km/h:
 * ibSpeedByte() in IboosterLaw.h applies the floor (change add-ibooster-speed-floor, firmware
 * 1.6.1); this builder sends the byte it is given.
 *
 * Unit frames:
 *   0x38E DLC 6 / 10 ms, J1850: byte 1 high nibble = ext-request status (2 idle, 4->5 request);
 *         field A = b2 | (b3 & 0x0F) << 8, field B = b4 | (b5 & 0x0F) << 8, mm = raw / 64 - 5;
 *         high nibbles of b3 / b5 = quality (1 normal, 2 after a failure).
 *   0x38F DLC 6 / 20 ms, J1850: byte 3 = pRunout bar (0 at the stroke stop), byte 4 = 0x40 the
 *         unit sees our frames / 0x80 it does not.
 *   0x39D DLC 5 / 40 ms, byte 0 = (0x03 + 0x9D + b1..b4) & 0xFF (a byte SUM, not J1850):
 *         status (b1 bits 4-6), counter (b1 low nibble), driver_brake_apply (b2 bits 0-1),
 *         internal state (b2 bits 2-4), rod 12 bit (b2 >> 5 | b3 << 3 | (b4 & 1) << 11),
 *         mm = raw / 64 - 5.
 */

#include <stdint.h>

#define IB_ID_VEHICLE_STATUS 0x38B   // ours: request + vehicle speed
#define IB_ID_BRAKE_REQUEST  0x38C   // ours: request + flow
#define IB_ID_VEHICLE_ALIVE  0x38D   // ours: alive + master-cylinder pressure word
#define IB_ID_EXT_STATUS     0x38E   // unit: ext-request status, rod fields A / B
#define IB_ID_RUNOUT         0x38F   // unit: pRunout, partner flag
#define IB_ID_STATUS         0x39D   // unit: status, internal state, driver_brake_apply, rod

#define IB_EXT_REQUEST  0x40     // byte 1 of 0x38B and 0x38C while requesting
#define IB_FLOW_ZERO    0x7E00   // flow 0 (hold)
#define IB_FLOW_MIN     0x6A00   // -40 ml/s (fastest release)
#define IB_FLOW_MAX     0x9200   // +40 ml/s (fastest apply)
#define IB_FLOW_UNITS_PER_MLPS 128   // 0.0078125 ml/s per unit
#define IB_FLOW_RANGE   0x1400   // 5120 units = 40 ml/s

#define IB_ROD_RAW_OFFSET 320    // raw for 0 mm (5 mm / 0.015625)

#define IB_PMC_STD_VALUE  117    // 0x38D pMC1: 117 * 0.3 - 30 = 5.1 bar (bench constant)
#define IB_PMC_STD_QUAL   1      // 0x38D pMC1 qualifier: only 1 is accepted (0 / 2 drop the request)
#define IB_38B_BYTE3      0x05   // 0x38B byte 3 as on the bench / trace

/** CRC-8 SAE J1850: polynomial 0x1D, init 0xFF, xorout 0xFF, not reflected. */
static inline uint8_t ibCrc8(const uint8_t* d, uint8_t n) {
  uint8_t c = 0xFF;
  while (n--) {
    c ^= *d++;
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x1D) : (uint8_t)(c << 1);
  }
  return (uint8_t)(c ^ 0xFF);
}

/** Byte 0 equals the J1850 CRC of bytes 1..dlc-1. */
static inline bool ibCrcOk(const uint8_t* d, uint8_t dlc) {
  return dlc >= 2 && dlc <= 8 && ibCrc8(d + 1, (uint8_t)(dlc - 1)) == d[0];
}

/** 0x39D checksum (bench, 25 / 25 frames): (0x03 + 0x9D + byte1 + ... + byte[dlc-1]) & 0xFF. */
static inline uint8_t ibSum39D(const uint8_t* d, uint8_t dlc) {
  uint8_t c = (uint8_t)(0x03 + 0x9D);
  for (uint8_t i = 1; i < dlc && i < 8; i++) c = (uint8_t)(c + d[i]);
  return c;
}

/** Byte 0 check for a unit frame of this ID: 0x39D = sum, the others = J1850. */
static inline bool ibChecksumOk(uint16_t id, const uint8_t* d, uint8_t dlc) {
  if (id == IB_ID_STATUS) return dlc >= 2 && dlc <= 8 && ibSum39D(d, dlc) == d[0];
  return ibCrcOk(d, dlc);
}

/** Any flow word into the unit's valid range 0x6A00..0x9200. */
static inline uint16_t ibClampFlow(int32_t f) {
  if (f < IB_FLOW_MIN) return IB_FLOW_MIN;
  if (f > IB_FLOW_MAX) return IB_FLOW_MAX;
  return (uint16_t)f;
}

/**
 * 0x38D bytes 5-6 (Intel word w = b5 | b6 << 8): bits 4..13 = pMC1 (0.3 bar/bit - 30), bits
 * 14..15 = qualifier, bits 0..3 = 0. Standard 50 47 = value 117, qualifier 1.
 */
static inline void ibPmcBytes(uint16_t val, uint8_t q, uint8_t& b5, uint8_t& b6) {
  const uint16_t w = (uint16_t)(((val & 0x3FF) << 4) | ((uint16_t)(q & 3) << 14));
  b5 = (uint8_t)(w & 0xFF);
  b6 = (uint8_t)(w >> 8);
}

/**
 * Fill the three frames of one slot. b / c: 4 bytes (0x38B / 0x38C), d: 7 bytes (0x38D).
 * Request bits 0x40 only when `req` (never 0x80); flow forced to 0x7E00 without a request and
 * always clamped to 0x6A00..0x9200; speed byte in 0x38B byte 2; pMC1 fixed at 117 / qualifier 1.
 */
static inline void ibBuild(uint8_t cnt, bool req, uint16_t flow, uint8_t speedByte,
                           uint8_t* b, uint8_t* c, uint8_t* d) {
  const uint8_t q = req ? IB_EXT_REQUEST : 0x00;
  const uint16_t f = req ? ibClampFlow(flow) : (uint16_t)IB_FLOW_ZERO;
  cnt &= 0x0F;
  b[1] = (uint8_t)(q | cnt);
  b[2] = speedByte;
  b[3] = IB_38B_BYTE3;
  b[0] = ibCrc8(b + 1, 3);
  c[1] = (uint8_t)(q | cnt);
  c[2] = (uint8_t)(f & 0xFF);
  c[3] = (uint8_t)(f >> 8);
  c[0] = ibCrc8(c + 1, 3);
  d[1] = cnt;
  d[2] = 0x00;
  d[3] = 0x00;
  d[4] = 0x00;
  ibPmcBytes(IB_PMC_STD_VALUE, IB_PMC_STD_QUAL, d[5], d[6]);
  d[0] = ibCrc8(d + 1, 6);
}

/** Decoded 0x39D. */
struct IbStatus {
  uint8_t counter;  // 0..15
  uint8_t status;   // 0 off, 1 init, 2 failure, 3 diagnostic, 4 active_good_check, 5 ready, 6 actuation
  uint8_t drv;      // driver_brake_apply: 0 not_init_or_off, 1 not_applied, 2 driver_applying, 3 fault
  uint8_t istate;   // internal_state: 3 = external_brake_request, 5 = to_idle
  uint16_t rodRaw;  // 12 bit; mm = raw / 64 - 5
};

/** Decode 0x39D (at least 5 bytes). Does not check the checksum. */
static inline bool ibDecode39D(const uint8_t* d, uint8_t dlc, IbStatus& s) {
  if (dlc < 5) return false;
  s.counter = d[1] & 0x0F;
  s.status = (d[1] >> 4) & 0x07;
  s.drv = d[2] & 0x03;
  s.istate = (d[2] >> 2) & 0x07;
  s.rodRaw = (uint16_t)((d[2] >> 5) | ((uint16_t)d[3] << 3) | ((uint16_t)(d[4] & 0x01) << 11));
  return true;
}

/** Rod (or 0x38E field) position in 0.01 mm from the 12-bit raw value (raw / 64 - 5 mm). */
static inline int16_t ibRodCentiMm(uint16_t raw) {
  return (int16_t)(((int32_t)(raw & 0x0FFF) * 100) / 64 - 500);
}

/** Decoded 0x38E. */
struct IbExtStatus {
  uint8_t extStatus;  // byte 1 high nibble: 2 idle, 4 -> 5 under an external request
  int16_t aCmm;       // field A, 0.01 mm (follows the rod, offset -0.17..+0.27 mm)
  int16_t bCmm;       // field B, 0.01 mm (drive travel, ~ rod + 2 mm)
  uint8_t qualA;      // byte 3 high nibble (1 normal, 2 after a failure)
  uint8_t qualB;      // byte 5 high nibble
};

/** Decode 0x38E (at least 6 bytes). Does not check the checksum. */
static inline bool ibDecode38E(const uint8_t* d, uint8_t dlc, IbExtStatus& e) {
  if (dlc < 6) return false;
  e.extStatus = (uint8_t)(d[1] >> 4);
  e.aCmm = ibRodCentiMm((uint16_t)(d[2] | ((uint16_t)(d[3] & 0x0F) << 8)));
  e.bCmm = ibRodCentiMm((uint16_t)(d[4] | ((uint16_t)(d[5] & 0x0F) << 8)));
  e.qualA = (uint8_t)(d[3] >> 4);
  e.qualB = (uint8_t)(d[5] >> 4);
  return true;
}

/** Decoded 0x38F. */
struct IbRunout {
  uint8_t runoutBar;  // byte 3: available pressure, 0 at the stroke stop
  int8_t partner;     // byte 4: 1 = 0x40 (sees our frames), 0 = 0x80 (does not), -1 = other
};

/** Decode 0x38F (at least 6 bytes). Does not check the checksum. */
static inline bool ibDecode38F(const uint8_t* d, uint8_t dlc, IbRunout& r) {
  if (dlc < 6) return false;
  r.runoutBar = d[3];
  r.partner = (d[4] == 0x40) ? (int8_t)1 : (d[4] == 0x80) ? (int8_t)0 : (int8_t)-1;
  return true;
}

#endif  // IBOOSTER_PROTO_H
