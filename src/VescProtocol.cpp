#include "VescProtocol.h"

namespace VescProtocol {

uint16_t crc16(const uint8_t* buf, uint16_t len) {
    // CRC16-CCITT/XMODEM: poly 0x1021, init 0x0000 (matches VESC bldc crc16()).
    uint16_t cksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        cksum ^= (uint16_t)buf[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (cksum & 0x8000) {
                cksum = (uint16_t)((cksum << 1) ^ 0x1021);
            } else {
                cksum <<= 1;
            }
        }
    }
    return cksum;
}

// --- big-endian buffer helpers (VESC uses big-endian on the wire) ---

static int16_t getInt16(const uint8_t* b, uint16_t i) {
    return (int16_t)(((uint16_t)b[i] << 8) | (uint16_t)b[i + 1]);
}

static int32_t getInt32(const uint8_t* b, uint16_t i) {
    return (int32_t)(((uint32_t)b[i] << 24) | ((uint32_t)b[i + 1] << 16) |
                     ((uint32_t)b[i + 2] << 8) | (uint32_t)b[i + 3]);
}

uint16_t frameShort(const uint8_t* payload, uint8_t payloadLen, uint8_t* out) {
    uint16_t idx = 0;
    out[idx++] = 0x02;              // short-packet start
    out[idx++] = payloadLen;        // 1-byte length
    for (uint8_t i = 0; i < payloadLen; i++) out[idx++] = payload[i];
    uint16_t crc = crc16(payload, payloadLen);
    out[idx++] = (uint8_t)(crc >> 8);
    out[idx++] = (uint8_t)(crc & 0xFF);
    out[idx++] = 0x03;              // stop byte
    return idx;
}

uint8_t encodeSetDuty(float duty, uint8_t* out) {
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    int32_t d = (int32_t)(duty * 100000.0f);   // VESC scales duty by 1e5
    out[0] = COMM_SET_DUTY;
    out[1] = (uint8_t)((d >> 24) & 0xFF);
    out[2] = (uint8_t)((d >> 16) & 0xFF);
    out[3] = (uint8_t)((d >> 8) & 0xFF);
    out[4] = (uint8_t)(d & 0xFF);
    return 5;
}

bool decodeGetValues(const uint8_t* payload, uint16_t payloadLen, Values& out) {
    if (payloadLen < 1 || payload[0] != COMM_GET_VALUES) return false;

    // Canonical bldc send_values() layout (payload[0] = id). Offsets are
    // FW-version-dependent — bench-verify against VESC Tool (tasks.md 8.3):
    //   offset  1 : temp_fet            int16  / 10
    //   offset  3 : temp_motor          int16  / 10
    //   offset  5 : avg_motor_current   int32  / 100
    //   offset  9 : avg_input_current   int32  / 100
    //   offset 13 : avg_id / avg_iq     int32 x2
    //   offset 21 : duty_now            int16  / 1000
    //   offset 23 : rpm                 int32
    //   offset 27 : v_in                int16  / 10
    //   ...
    //   offset 53 : fault_code          uint8
    // Need the fault byte at offset 53, so require at least 54 bytes.
    if (payloadLen < 54) return false;

    out.fetTempC      = getInt16(payload, 1) / 10.0f;
    out.motorCurrentA = getInt32(payload, 5) / 100.0f;
    out.inputVoltageV = getInt16(payload, 27) / 10.0f;
    out.faultCode     = payload[53];
    return true;
}

} // namespace VescProtocol
