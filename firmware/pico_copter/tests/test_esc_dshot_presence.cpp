#include "esc_dshot_presence.hpp"
#include "dshot_bidir_decode.hpp"
#include "dshot_protocol.hpp"
#include <cassert>
#include <cstdint>
#include <cstdio>

int main() {
    using esc_dshot_presence::connected;
    constexpr uint32_t at = 500000;
    // No reply -> never connected, even if motor commands were sent.
    assert(!connected(0, 0, at));
    // A single checksum-valid eRPM response, including a stopped motor,
    // establishes presence. Electrical speed itself does not gate presence.
    assert(connected(1, at, at));
    assert(connected(1, at, at + 249999u));
    assert(connected(1, at, at + 250000u));
    assert(!connected(1, at, at + 250001u));
    // A later validated reply restores detection.
    assert(connected(2, at + 300000u, at + 310000u));
    // Unsigned age calculation remains valid across time_us_32 wraparound.
    assert(connected(1, 0xfffffff0u, 0x00000020u));
    assert(!connected(1, 0xfffffff0u, 0x00040000u));

    // Corrupt/no-response DShot frames are not valid eRPM replies.
    assert(dshot::decode_erpm(0).result == dshot::RxResult::NoResponse);
    // Request flag can be added without changing commanded throttle.
    assert((dshot::bidirectional_frame(48, true) >> 5) == 48);
    assert((dshot::bidirectional_frame(48, true) & 0x10u) != 0);
    assert((dshot::bidirectional_frame(48, false) & 0x10u) == 0);
    std::puts("ESC presence timeout, wraparound and DShot request tests passed");
}
