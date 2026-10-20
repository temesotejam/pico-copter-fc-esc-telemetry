#include "sbus_parser.hpp"
#include <cassert>
#include <cstdint>
#include <cstdio>

void encode(const uint16_t in[16], uint8_t flags, uint8_t end, uint8_t bytes[25]) {
    for (int i = 0; i < 25; ++i) bytes[i] = 0;
    bytes[0] = 0x0f;
    for (uint32_t ch = 0; ch < 16; ++ch) {
        uint32_t offset = ch * 11;
        for (uint32_t bit = 0; bit < 11; ++bit) {
            if ((in[ch] >> bit) & 1u) {
                bytes[1 + (offset + bit) / 8] |= static_cast<uint8_t>(1u << ((offset + bit) % 8));
            }
        }
    }
    bytes[23] = flags;
    bytes[24] = end;
}
bool feed(sbus::Decoder& decoder, const uint8_t bytes[25], uint32_t us, sbus::Frame& out) {
    bool result = false;
    for (uint32_t i = 0; i < 25; ++i) result = decoder.push(bytes[i], us + i * 120, out) || result;
    return result;
}

int main() {
    uint16_t values[16];
    for (uint32_t i = 0; i < 16; ++i) values[i] = static_cast<uint16_t>((353 + 137 * i) & 0x7ff);
    uint8_t bytes[25];
    sbus::Decoder decoder;
    sbus::Frame frame;
    encode(values, 0, 0x00, bytes);
    assert(feed(decoder, bytes, 10000, frame));
    for (int i = 0; i < 16; ++i) assert(frame.channels[i] == values[i]);
    assert(sbus::healthy_frame(frame));
    assert(sbus::link_fresh(110000, 10000, true));
    assert(!sbus::link_fresh(110001, 10000, true));
    assert(!sbus::link_fresh(10000, 10000, false));
    assert(sbus::link_fresh(250, UINT32_MAX - 100, true));

    encode(values, sbus::kFailsafeFlag, 0x14, bytes);
    assert(feed(decoder, bytes, 20000, frame));
    assert(!sbus::healthy_frame(frame));
    encode(values, sbus::kFrameLostFlag, 0x00, bytes);
    assert(feed(decoder, bytes, 30000, frame));
    assert(!sbus::healthy_frame(frame));

    encode(values, 0, 0xff, bytes);
    assert(!feed(decoder, bytes, 40000, frame));
    encode(values, 0, 0x00, bytes);
    for (uint32_t i = 0; i < 7; ++i) assert(!decoder.push(bytes[i], 50000+i*120, frame));
    assert(feed(decoder, bytes, 60000, frame));
    assert(sbus::healthy_frame(frame));

    sbus::ReArmLatch latch;
    for (int i = 0; i < 90; ++i) assert(!latch.update(false, false, true, true));
    for (int i = 0; i < 90; ++i) assert(!latch.update(true, false, true, false));
    for (int i = 0; i < 79; ++i) assert(!latch.update(true, false, true, true));
    assert(latch.update(true, false, true, true));
    // An explicit failsafe event must latch even if fresh good data has already arrived.
    assert(!latch.update(true, true, true, true));
    for (int i = 0; i < 79; ++i) assert(!latch.update(true, false, true, true));
    assert(latch.update(true, false, true, true));
    assert(!latch.update(false, false, true, true));
    for (int i = 0; i < 90; ++i) assert(!latch.update(true, false, false, true));
    for (int i = 0; i < 79; ++i) assert(!latch.update(true, false, true, true));
    assert(latch.update(true, false, true, true));
    std::puts("SBUS safety parser and re-arm latch tests passed");
}
