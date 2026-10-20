#include "esc_serial_stream.hpp"
#include <cassert>
#include <cstdio>

using namespace esc_serial_protocol;
using namespace esc_serial_stream;

void make_packet(uint8_t b[10], uint8_t temp, uint16_t voltage, uint16_t current,
                 uint16_t mah, uint16_t erpm_div100) {
    b[0] = temp;
    b[1] = voltage >> 8; b[2] = voltage & 255;
    b[3] = current >> 8; b[4] = current & 255;
    b[5] = mah >> 8; b[6] = mah & 255;
    b[7] = erpm_div100 >> 8; b[8] = erpm_div100 & 255;
    b[9] = crc8(b, 9);
}
int main() {
    uint8_t good[10];
    make_packet(good, 29, 780, 42, 1, 260);
    ByteScan parser{};
    Gate gate{};
    Reading r{};
    uint32_t t = 10000;
    // Noise and incomplete bytes do not establish a packet.
    for (unsigned i = 0; i < 7; ++i)
        assert(parser.push(0xfa, t += 87, r) == ScanResult::Partial);
    // Gap drops an old partial and permits autonomous 32 ms packets.
    t += 32000;
    for (unsigned i = 0; i < 10; ++i) {
        const auto result = parser.push(good[i], t += 87, r);
        if (i == 9) assert(result == ScanResult::Packet);
    }
    assert(!gate.accept(r, t)); // One CRC hit is NOT enough.
    t += 32000;
    for (auto b : good) {
        const auto result = parser.push(b, t += 87, r);
        if (b == good[9]) assert(result == ScanResult::Packet);
    }
    assert(gate.accept(r, t));
    assert(r.temperature_c == 29 && r.voltage_centivolt == 780
           && r.current_centiamp == 42 && r.electrical_rpm == 26000);

    // A bit error can lose a whole frame; next good 32ms packet resyncs.
    uint8_t corrupt[10];
    for (unsigned i = 0; i < 10; ++i) corrupt[i] = good[i];
    corrupt[4] ^= 0x20;
    t += 32000;
    ScanResult last = ScanResult::Partial;
    for (auto b : corrupt) last = parser.push(b, t += 87, r);
    assert(last == ScanResult::BadCrc);
    t += 32000;
    for (auto b : good) last = parser.push(b, t += 87, r);
    assert(last == ScanResult::Packet && gate.accept(r, t));

    // Inserted leading junk within the same burst requires sliding.
    parser = ByteScan{};
    t += 32000;
    parser.push(0x55, t += 87, r);
    for (auto b : good) last = parser.push(b, t += 87, r);
    assert(last == ScanResult::Packet);

    // Wrong-but-CRC-valid value from the real log: 250 C / 74.27 V.
    uint8_t bad[10];
    make_packet(bad, 250, 7427, 2304, 10240, 257);
    assert(decode(bad, r) && !plausible(r));
    t += 32000;
    for (auto b : bad) last = parser.push(b, t += 87, r);
    assert(last == ScanResult::BadRange);

    // Plausible but severely inconsistent voltage is not published.
    Reading jump{};
    jump.temperature_c = 28;
    jump.voltage_centivolt = 3000;
    jump.current_centiamp = 20;
    jump.electrical_rpm = 10000;
    assert(!gate.accept(jump, t));
    assert(!gate.accept(jump, t + 400000)); // timed out
    assert(gate.accept(jump, t + 430000));
    std::puts("Stream resync, auto-packet, corruption and plausibility tests passed");
}
