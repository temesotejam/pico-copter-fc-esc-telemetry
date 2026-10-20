#ifndef PICO_COPTER_DSHOT_BIDIR_DECODE_HPP
#define PICO_COPTER_DSHOT_BIDIR_DECODE_HPP
#include <stdint.h>

// Decoding follows the open-source pio-dshot eRPM telemetry representation
// (https://github.com/simonwunderlich/pio-dshot, MIT license).
// No dynamic allocation; this is run once per motor per 400-Hz cycle.
namespace dshot {
enum class RxResult : uint8_t { Erpm, OtherTelemetry, NoResponse, BadGcr, BadChecksum };
struct ErpmResult {
    RxResult result;
    uint32_t erpm;
};
inline int8_t decode_gcr_nibble(uint32_t bits) {
    // Standard 4b/5b GCR: entries correspond to data nibbles 0..15.
    constexpr uint8_t table[16] = {
        0x19, 0x1b, 0x12, 0x13, 0x1d, 0x15, 0x16, 0x17,
        0x1a, 0x09, 0x0a, 0x0b, 0x1e, 0x0d, 0x0e, 0x0f
    };
    for (uint8_t i = 0; i < 16; ++i) {
        if (bits == table[i]) return static_cast<int8_t>(i);
    }
    return -1;
}
inline ErpmResult decode_erpm(uint32_t rx_word) {
    if (rx_word == 0) return {RxResult::NoResponse, 0};
    // PIO receives 21 NRZI bits. Convert into 20 GCR bits.
    const uint32_t gcr = rx_word ^ (rx_word >> 1);
    uint16_t telemetry = 0;
    for (int i = 3; i >= 0; --i) {
        const int8_t nibble = decode_gcr_nibble((gcr >> (5 * i)) & 0x1f);
        if (nibble < 0) return {RxResult::BadGcr, 0};
        telemetry = static_cast<uint16_t>((telemetry << 4) | nibble);
    }
    const uint8_t crc = static_cast<uint8_t>(
        ~(telemetry >> 4 ^ telemetry >> 8 ^ telemetry >> 12) & 0x0f);
    if (crc != (telemetry & 0x0f)) return {RxResult::BadChecksum, 0};

    // Extended telemetry may report voltage/current/temperature instead.
    const uint8_t high = static_cast<uint8_t>(telemetry >> 12);
    if ((high & 1u) == 0u && high != 0u) {
        return {RxResult::OtherTelemetry, 0};
    }
    const uint32_t exponent = (telemetry >> 13) & 7u;
    const uint32_t mantissa = (telemetry >> 4) & 0x1ffu;
    const uint32_t period_us = mantissa << exponent;
    if (period_us == 0xff80u) return {RxResult::Erpm, 0}; // ESC stopped
    if (!period_us) return {RxResult::NoResponse, 0};
    return {RxResult::Erpm, 60000000u / period_us};
}
} // namespace dshot
#endif
