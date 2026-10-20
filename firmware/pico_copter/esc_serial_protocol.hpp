#ifndef PICO_COPTER_ESC_SERIAL_PROTOCOL_HPP
#define PICO_COPTER_ESC_SERIAL_PROTOCOL_HPP

#include <stdint.h>

namespace esc_serial_protocol {
// BLHeli_32 / KISS dedicated TX-pad telemetry: 115200 baud, 8N1.
// One 10-byte response, big-endian fields, CRC8 polynomial 0x07 (initial 0).
constexpr uint8_t kFrameSize = 10;

struct Reading {
    uint8_t temperature_c = 0;
    uint16_t voltage_centivolt = 0;
    uint16_t current_centiamp = 0;
    uint16_t consumption_mah = 0;
    uint32_t electrical_rpm = 0;
};

inline uint8_t crc8(const uint8_t* bytes, unsigned count) {
    uint8_t crc = 0;
    for (unsigned i = 0; i < count; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = static_cast<uint8_t>((crc & 0x80u)
                ? ((crc << 1) ^ 0x07u) : (crc << 1));
    }
    return crc;
}

inline bool decode(const uint8_t* bytes, Reading& output) {
    if (crc8(bytes, 9) != bytes[9]) return false;
    output.temperature_c = bytes[0];
    output.voltage_centivolt = static_cast<uint16_t>((bytes[1] << 8) | bytes[2]);
    output.current_centiamp = static_cast<uint16_t>((bytes[3] << 8) | bytes[4]);
    output.consumption_mah = static_cast<uint16_t>((bytes[5] << 8) | bytes[6]);
    output.electrical_rpm = static_cast<uint32_t>(
        (static_cast<uint16_t>((bytes[7] << 8) | bytes[8])) * 100u);
    return true;
}
}  // namespace esc_serial_protocol
#endif
