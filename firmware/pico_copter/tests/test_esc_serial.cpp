#include "esc_serial_protocol.hpp"
#include "dshot_protocol.hpp"
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

int main() {
    using namespace esc_serial_protocol;
    // 42 degC, 24.12V, 12.34A, 123mAh, 34500 electrical RPM.
    uint8_t frame_bytes[kFrameSize] = {
        42, 0x09, 0x6c, 0x04, 0xd2, 0x00, 0x7b, 0x01, 0x59, 0
    };
    frame_bytes[9] = crc8(frame_bytes, 9);
    Reading result{};
    assert(decode(frame_bytes, result));
    assert(result.temperature_c == 42);
    assert(result.voltage_centivolt == 2412);
    assert(result.current_centiamp == 1234);
    assert(result.consumption_mah == 123);
    assert(result.electrical_rpm == 34500);
    frame_bytes[4] ^= 1;
    assert(!decode(frame_bytes, result));
    frame_bytes[4] ^= 1;

    // Requesting UART telemetry changes only this ESC's telemetry bit + CRC,
    // never the commanded throttle or the three other DShot frames.
    using namespace dshot;
    for (uint16_t cmd : {uint16_t{0}, uint16_t{48}, uint16_t{1046}, uint16_t{2047}}) {
        assert((frame(cmd, true) >> 5) == cmd);
        assert((frame(cmd, true) & 0x10u) != 0);
        assert((frame(cmd) & 0x10u) == 0);
        assert(pio_word(cmd, true) == (static_cast<uint32_t>(frame(cmd, true)) << 16));
        assert((bidirectional_frame(cmd, true) >> 5) == cmd);
        assert((bidirectional_frame(cmd, true) & 0x10u) != 0);
        assert(bidirectional_pio_word(cmd, true) ==
               (static_cast<uint32_t>(~bidirectional_frame(cmd, true) & 0xffffu) << 16));
    }
    assert(frame(48) == 0x0606);
    assert(bidirectional_frame(48) == 0x0609);
    std::puts("ESC UART telemetry CRC8/units and DShot request-bit tests passed");
}
