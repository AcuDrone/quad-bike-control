#ifndef DEVICE_ID_H
#define DEVICE_ID_H

/**
 * @file DeviceId.h
 * @brief Per-chip name suffix for the WiFi AP SSID and the OTA hostname (header-only,
 *        no Arduino includes; native-tested in test/test_device_id).
 *
 * The suffix is the last two octets of the factory eFuse base MAC (not the SoftAP MAC,
 * which ESP-IDF derives as base + 1), printed as 4 uppercase hex characters:
 * MAC 3C:DC:75:FC:5E:CC → "5ECC". Computed at every boot, never stored.
 *
 * Byte order: esp_efuse_mac_get_default(mac) fills mac[0..5] in transmission order
 * (mac[0] = 0x3C ... mac[5] = 0xCC for the example). Arduino's ESP.getEfuseMac() copies
 * the same 6 octets into a little-endian uint64_t, so octet 0 is bits 0–7 and octet 5 is
 * bits 40–47: the example reads 0x0000CC5EFC75DC3C. formatChipSuffixFromEfuse() therefore
 * takes octet 4 from bits 32–39 and octet 5 from bits 40–47.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace DeviceId {

// 802.11 SSID limit is 32 octets; buffers hold that plus the NUL.
static constexpr size_t NAME_CAP = 33;

inline void formatSuffixOctets(uint8_t o4, uint8_t o5, char out[5]) {
    static const char HEX_DIGITS[] = "0123456789ABCDEF";
    out[0] = HEX_DIGITS[(o4 >> 4) & 0x0F];
    out[1] = HEX_DIGITS[o4 & 0x0F];
    out[2] = HEX_DIGITS[(o5 >> 4) & 0x0F];
    out[3] = HEX_DIGITS[o5 & 0x0F];
    out[4] = '\0';
}

// Primary form: mac[] as filled by esp_efuse_mac_get_default() → octets 4 and 5.
inline void formatChipSuffix(const uint8_t mac[6], char out[5]) {
    formatSuffixOctets(mac[4], mac[5], out);
}

// ESP.getEfuseMac() form (little-endian uint64, see the byte-order note above).
inline void formatChipSuffixFromEfuse(uint64_t efuse, char out[5]) {
    formatSuffixOctets(static_cast<uint8_t>((efuse >> 32) & 0xFF),
                       static_cast<uint8_t>((efuse >> 40) & 0xFF), out);
}

// out = base + "-" + suffix (suffix lower-cased when lowercase is true). The base is
// truncated so the result fits cap - 1 characters; the "-suffix" part is always kept.
inline void buildSuffixedName(const char* base, const char* suffix, bool lowercase,
                              char* out, size_t cap) {
    if (out == nullptr || cap == 0) return;
    out[0] = '\0';
    if (base == nullptr) base = "";
    if (suffix == nullptr) suffix = "";
    const size_t sufLen = strlen(suffix);
    const size_t tailLen = sufLen + 1;            // "-" + suffix
    if (tailLen > cap - 1) return;                // cannot fit even the tail
    size_t baseLen = strlen(base);
    if (baseLen > cap - 1 - tailLen) baseLen = cap - 1 - tailLen;
    memcpy(out, base, baseLen);
    out[baseLen] = '-';
    for (size_t i = 0; i < sufLen; ++i) {
        char c = suffix[i];
        if (lowercase && c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
        out[baseLen + 1 + i] = c;
    }
    out[baseLen + tailLen] = '\0';
}

}  // namespace DeviceId

#endif  // DEVICE_ID_H
