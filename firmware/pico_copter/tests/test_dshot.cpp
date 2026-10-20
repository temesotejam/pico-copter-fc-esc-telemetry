#include "dshot_protocol.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

int main() {
    using namespace dshot;
    static_assert(kPioClockHz == kBitRate * 40u, "Bidir PIO clock scales with bitrate");
    static_assert(kPioCyclesPerBit == 40u, "40 PIO cycles per DShot bit");
    static_assert(kTurnaroundCycles == kRateKbps, "25us turnaround in PIO cycles");
    static_assert(kRxWaitOuterY == (8u * kRateKbps / 300u) - 1u,
                  "RX wait counter scales with DShot rate");
    static_assert((kRateKbps != 300u || kRxWaitOuterY == 7u) &&
                  (kRateKbps != 600u || kRxWaitOuterY == 15u) &&
                  (kRateKbps != 150u || kRxWaitOuterY == 3u) &&
                  (kRateKbps != 1200u || kRxWaitOuterY == 31u),
                  "Unexpected RX wait timing");
    static_assert(kPioClockHz / kBitRate == 40u,
                  "PIO TX/RX clock must follow bitrate");
    static_assert(kRateKbps == 150u || kRateKbps == 300u ||
                  kRateKbps == 600u || kRateKbps == 1200u,
                  "Only supported DShot speeds");
    static_assert(kOneWayPioCyclesPerBit == 8u, "One-way DShot 8 PIO cycles/bit");
    static_assert(kOneWayPioClockHz == kBitRate * 8u,
                  "One-way PIO clock follows selected rate");
    static_assert(DSHOT_BIDIRECTIONAL == 0 || DSHOT_BIDIRECTIONAL == 1,
                  "Protocol mode is strictly selected at compile time");
    assert(frame(0) == 0);
    assert(frame(48) == 0x0606);
    assert(frame(2047) == 0xFFEE);
    assert(frame(2047, true) == 0xFFFF); // Verify CRC includes telemetry bit.
    for (unsigned throttle = 0; throttle <= 2047; ++throttle) {
        uint16_t f = frame(static_cast<uint16_t>(throttle));
        uint16_t payload = f >> 4;
        assert(((payload ^ (payload >> 4) ^ (payload >> 8)) & 15) == (f & 15));
        assert((f >> 5) == throttle);
        assert((f & 16) == 0); // Telemetry bit remains zero in normal operation.
    }
    assert(throttle_from_legacy_duty(0.0f) == kStop);
    assert(throttle_from_legacy_duty(-1.0f) == kStop);
    assert(throttle_from_legacy_duty(NAN) == kStop);
    assert(throttle_from_legacy_duty(INFINITY) == kStop);
    // Every running DShot value 48..2047 must be reachable from the
    // existing motor mixer's 0.10..0.95 duty interval.
    assert(throttle_from_legacy_duty(kLegacyMinimumRunningDuty) == kMinimumThrottle);
    assert(throttle_from_legacy_duty(kLegacyMaximumRunningDuty) == kMaximumThrottle);
    assert(throttle_from_legacy_duty(0.001f) == kMinimumThrottle);
    assert(throttle_from_legacy_duty(1.0f) == kMaximumThrottle);
    assert(throttle_from_legacy_duty(10.0f) == kMaximumThrottle);
    assert(throttle_from_legacy_duty(0.525f) >= 1047);
    assert(throttle_from_legacy_duty(0.525f) <= 1048);

    uint16_t previous = 0;
    for (int i = 0; i <= 10000; ++i) {
        const float duty = kLegacyMinimumRunningDuty +
            (kLegacyMaximumRunningDuty - kLegacyMinimumRunningDuty) * i / 10000.0f;
        const uint16_t command = throttle_from_legacy_duty(duty);
        assert(command >= kMinimumThrottle && command <= kMaximumThrottle);
        assert(command >= previous);
        previous = command;
    }
    // Test all 2000 running values, including boundaries, for gaps.
    for (uint16_t cmd = kMinimumThrottle; cmd <= kMaximumThrottle; ++cmd) {
        const float position = static_cast<float>(cmd - kMinimumThrottle) /
            static_cast<float>(kMaximumThrottle - kMinimumThrottle);
        const float duty = kLegacyMinimumRunningDuty +
            (kLegacyMaximumRunningDuty - kLegacyMinimumRunningDuty) * position;
        assert(throttle_from_legacy_duty(duty) == cmd);
    }
    assert(pio_word(kStop) == 0);
    assert(pio_word(48) == 0x06060000u);
    std::printf("DShot%u %s encoding/CRC, stop, throttle and PIO rate passed\n",
                static_cast<unsigned int>(kRateKbps),
                kBidirectional ? "bidir" : "oneway");
}
