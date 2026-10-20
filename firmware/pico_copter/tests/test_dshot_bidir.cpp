#include "dshot_protocol.hpp"
#include "dshot_bidir_decode.hpp"
#include <cassert>
#include <cstdint>
#include <cstdio>

constexpr uint8_t gcr_encode[16] = {
    0x19, 0x1b, 0x12, 0x13, 0x1d, 0x15, 0x16, 0x17,
    0x1a, 0x09, 0x0a, 0x0b, 0x1e, 0x0d, 0x0e, 0x0f
};

// Encode an eRPM response (inverted CRC + GCR + NRZI) to exercise decoder.
uint32_t telemetry_word(uint16_t payload12) {
    const uint8_t crc = static_cast<uint8_t>(
        ~(payload12 ^ (payload12 >> 4) ^ (payload12 >> 8)) & 0xf);
    const uint16_t frame = static_cast<uint16_t>((payload12 << 4) | crc);
    uint32_t gcr = 0;
    for (int i = 3; i >= 0; --i)
        gcr = (gcr << 5) | gcr_encode[(frame >> (i * 4)) & 0xf];
    // Invert gray mapping: gcr = NRZI ^ (NRZI >> 1), low 20 bits.
    uint32_t raw = 0;
    for (int i = 19; i >= 0; --i) {
        const uint32_t higher = (raw >> (i + 1)) & 1u;
        const uint32_t bit = ((gcr >> i) & 1u) ^ higher;
        raw |= bit << i;
    }
    return raw;
}
int main() {
    using namespace dshot;
    assert(bidirectional_frame(0) == 0x000f);
    assert(bidirectional_pio_word(0) == 0xfff00000u);
    assert(bidirectional_frame(48) == (frame(48) ^ 0x000fu));
    // Real logs captured these packet values for STOP and MAX while
    // testing the controller. Guard their bit order and inverse CRC.
    assert(bidirectional_frame(2047) == 0xffe1u);
    assert(bidirectional_pio_word(2047) == 0x001e0000u);
    assert(bidirectional_frame(0xffffu) == bidirectional_frame(2047));
    assert(bidirectional_pio_word(48) ==
           (static_cast<uint32_t>(~bidirectional_frame(48) & 0xffffu) << 16));
    for (uint16_t t = 0; t < 2048; ++t) {
        const uint16_t out = bidirectional_frame(t);
        const uint16_t payload = out >> 4;
        const uint16_t expected_crc = static_cast<uint16_t>(
            ~(payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0fu);
        assert((out & 15) == expected_crc);
        assert((out >> 5) == t);
        assert((out & 16) == 0); // No separate telemetry request bit.
    }

    // Exponent=2, mantissa=300 -> period 1200 us, 50,000 eRPM.
    uint16_t payload = static_cast<uint16_t>((2u << 9) | 300u);
    auto good = decode_erpm(telemetry_word(payload));
    assert(good.result == RxResult::Erpm && good.erpm == 50000u);

    // Upper nibble odd (0x9..0xf) can also be a valid eRPM exponent;
    // a broad "high bit means extended telemetry" check is incorrect.
    payload = static_cast<uint16_t>((7u << 9) | 300u);
    good = decode_erpm(telemetry_word(payload));
    assert(good.result == RxResult::Erpm);
    assert(good.erpm == 60000000u / (300u << 7));

    // Special no-rotation eRPM period marker.
    payload = static_cast<uint16_t>((7u << 9) | 511u);
    good = decode_erpm(telemetry_word(payload));
    assert(good.result == RxResult::Erpm && good.erpm == 0);

    assert(decode_erpm(0).result == RxResult::NoResponse);
    uint32_t corrupt = telemetry_word(static_cast<uint16_t>((2u << 9) | 300u));
    const RxResult bad = decode_erpm(corrupt ^ 4u).result;
    assert(bad == RxResult::BadGcr || bad == RxResult::BadChecksum);

    // Type=2 (extended temperature report); must never overwrite motor eRPM.
    auto other = decode_erpm(telemetry_word(0x200u));
    assert(other.result == RxResult::OtherTelemetry);
    std::printf("Bidirectional DShot%u CRC, NRZI/GCR and eRPM tests passed\n",
                static_cast<unsigned int>(dshot::kRateKbps));
}
