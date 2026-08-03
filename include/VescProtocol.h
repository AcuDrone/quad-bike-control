#ifndef VESC_PROTOCOL_H
#define VESC_PROTOCOL_H

#include <Arduino.h>

/**
 * @brief Minimal in-repo VESC UART packet layer.
 *
 * Implements only what this project needs to drive a Flipsky 75200 VESC in
 * brushed-DC mode: the VESC short/long framing, the CRC16-XMODEM check, the
 * `COMM_SET_DUTY` encoder, and a `COMM_GET_VALUES` decoder for the four fields
 * the firmware monitors (motor current, FET temperature, input voltage, fault
 * code). A general VESC library is intentionally avoided (project convention:
 * minimal dependencies); only two commands are supported.
 *
 * Framing (VESC serial protocol):
 *   short packet (payload <= 255): 0x02, len, payload..., crc_hi, crc_lo, 0x03
 *   long  packet (payload <= 65535): 0x03, len_hi, len_lo, payload..., crc_hi, crc_lo, 0x03
 * CRC is CRC16-CCITT/XMODEM (poly 0x1021, init 0x0000) over the payload only.
 */
namespace VescProtocol {

// COMM_PACKET_ID values (VESC datatypes.h). Verified against VESC bldc firmware
// and the SolidGeek/VescUart sources: GET_VALUES = 4, SET_DUTY = 5.
enum CommId : uint8_t {
    COMM_GET_VALUES = 4,
    COMM_SET_DUTY   = 5,
};

// Decoded subset of a COMM_GET_VALUES reply.
struct Values {
    float   fetTempC     = 0.0f;   // power-stage temperature (°C)
    float   motorCurrentA = 0.0f;  // average motor current (A)
    float   inputVoltageV = 0.0f;  // pack voltage (V)
    uint8_t faultCode    = 0;      // 0 = no fault
};

/** @brief CRC16-CCITT/XMODEM over `len` bytes of `buf`. */
uint16_t crc16(const uint8_t* buf, uint16_t len);

/**
 * @brief Frame a payload into a short VESC packet.
 * @param payload  command payload (payload[0] = command id)
 * @param payloadLen  length of the payload (must be <= 255 for a short packet)
 * @param out  destination buffer; must hold at least payloadLen + 5 bytes
 * @return total framed length written to `out`, or 0 if payloadLen > 255
 */
uint16_t frameShort(const uint8_t* payload, uint8_t payloadLen, uint8_t* out);

/**
 * @brief Encode a COMM_SET_DUTY payload.
 * @param duty  duty cycle, clamped to -1.0 .. +1.0
 * @param out  destination for the 5-byte payload (id + int32 duty*1e5, big-endian)
 * @return payload length (5)
 */
uint8_t encodeSetDuty(float duty, uint8_t* out);

/**
 * @brief Decode the fields the firmware needs from a COMM_GET_VALUES payload.
 * @param payload  packet payload (payload[0] must be COMM_GET_VALUES)
 * @param payloadLen  payload length
 * @param out  decoded values (populated only on success)
 * @return true if the payload is a well-formed GET_VALUES reply of sufficient length
 *
 * FIELD OFFSETS ARE VESC-FIRMWARE-VERSION DEPENDENT. These match the canonical
 * bldc `send_values` layout (stable across FW 3.x–6.x for these early fields);
 * newer firmware only appends fields AFTER the fault code, so the offsets read
 * here do not shift. They MUST still be bench-verified against VESC Tool for the
 * flashed firmware (see change tasks.md task 8.3).
 */
bool decodeGetValues(const uint8_t* payload, uint16_t payloadLen, Values& out);

} // namespace VescProtocol

#endif // VESC_PROTOCOL_H
