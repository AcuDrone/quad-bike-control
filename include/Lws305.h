// Lws305.h — decoder for the Bosch LWS steering-wheel-angle sensor 0 265 005 486
// (PSA 9650236180): standard CAN ID 0x305, DLC 6, 100 Hz, 500 kbit/s.
//
// Ported unchanged from the bench-verified CAN-TEST/src/lws305.h (2026-10-24), plus
// the rotation-speed decode and the two fixed 0x7C0 configuration payloads.
// NO Arduino dependency: this header is compiled by the native unit tests.
//
// Frame layout (PSA-RE buses/AEE2004.full/HS.IS/305.yml, bytes numbered from 0):
//   bytes 0-1: steering angle, int16 big-endian, 0.1 deg/bit; 0x7FFF = invalid;
//              positive = counter-clockwise (LEFT), negative = clockwise (RIGHT).
//              The firmware position is the NEGATED angle (positive = right).
//   byte 2:    rotation speed, uint8, 4 deg/s per bit; 0xFF = invalid
//   byte 3:    bit7 direction (1 = clockwise), bits 6..3 fault code (0 = none),
//              bit2 adjusted, bit1 calibrated, bit0 sensor OK
//   byte 4:    high nibble = XOR of the 8 nibbles of bytes 0..3 XOR the counter,
//              low nibble = rolling counter 0..15
//   byte 5:    "optimised" rotation speed, uint8, 4 deg/s per bit; 0xFF = invalid
//
// Configuration frame 0x7C0 (DLC 2): ONLY the two constants below are ever sent.
//   05 00 = reset calibration (cal=0, angle 0x7FFF)
//   53 30 = zero at the current position, output ID ((0x30 << 8) | 0x53) >> 4 = 0x305
// ⚠ 03 00 would also zero the sensor but move its output ID to 0x000.
#pragma once

#include <stdint.h>

#define LWS_DLC_MIN     6        // our sensor sends 6 bytes, PSA-RE documents 7 — both accepted
#define LWS_ANGLE_INV   0x7FFF
#define LWS_SPEED_INV   0xFF
#define LWS_SPEED_LSB_DEG_S   4  // deg/s per bit of bytes 2 and 5
#define LWS_FAULT_RANGE_LATCHED 9  // out-of-range latch (~±600°), cleared only by a 12 V power cycle

// Checksum: high nibble of byte 4 = XOR of all nibbles of bytes 0..3 and the counter
static inline bool lwsCrcOk(const uint8_t *d) {
  uint8_t x = d[4] & 0x0F;
  for (uint8_t i = 0; i < 4; i++) x ^= (d[i] >> 4) ^ (d[i] & 0x0F);
  return (d[4] >> 4) == x;
}

static inline int16_t lwsAngleRaw(const uint8_t *d) { return (int16_t)(((uint16_t)d[0] << 8) | d[1]); }  // 0.1 deg
static inline bool    lwsAngleValid(const uint8_t *d) { return lwsAngleRaw(d) != LWS_ANGLE_INV; }
static inline bool    lwsOk(const uint8_t *d)      { return d[3] & 0x01; }
static inline bool    lwsCal(const uint8_t *d)     { return (d[3] >> 1) & 1; }
static inline bool    lwsAdj(const uint8_t *d)     { return (d[3] >> 2) & 1; }
static inline uint8_t lwsFault(const uint8_t *d)   { return (d[3] >> 3) & 0x0F; }
static inline bool    lwsDirCW(const uint8_t *d)   { return d[3] & 0x80; }
static inline uint8_t lwsCounter(const uint8_t *d) { return d[4] & 0x0F; }

// Rotation speed (byte 2) and optimised speed (byte 5), deg/s (unsigned; direction = lwsDirCW)
static inline bool     lwsSpeedValid(const uint8_t *d)    { return d[2] != LWS_SPEED_INV; }
static inline uint16_t lwsSpeedDegS(const uint8_t *d)     { return (uint16_t)d[2] * LWS_SPEED_LSB_DEG_S; }
static inline bool     lwsSpeedOptValid(const uint8_t *d) { return d[5] != LWS_SPEED_INV; }
static inline uint16_t lwsSpeedOptDegS(const uint8_t *d)  { return (uint16_t)d[5] * LWS_SPEED_LSB_DEG_S; }

// The ONLY two 0x7C0 payloads the firmware may transmit (zero-set sequence, CANController).
static const uint8_t LWS_CFG_RESET[2]      = {0x05, 0x00};  // reset calibration
static const uint8_t LWS_CFG_ZERO_ID305[2] = {0x53, 0x30};  // zero here, keep output ID 0x305

// Output ID encoded in a 0x7C0 payload: ((byte1 << 8) | byte0) >> 4
static inline uint16_t lwsCfgOutputId(const uint8_t *cfg) {
  return (uint16_t)((((uint16_t)cfg[1] << 8) | cfg[0]) >> 4);
}
