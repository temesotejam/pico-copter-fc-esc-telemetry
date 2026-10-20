#ifndef PICO_COPTER_ESC_SERIAL_STREAM_HPP
#define PICO_COPTER_ESC_SERIAL_STREAM_HPP

#include "esc_serial_protocol.hpp"
#include <stdint.h>

// Stream-oriented, fixed-memory UART parser. Each incoming byte is handled
// independently: no dependence on the DShot request bit, packet phase or USB.
// A gap resets framing; sliding windows recover after an inserted/dropped byte.
namespace esc_serial_stream {

constexpr uint32_t kGapUs = 2500u;
constexpr uint32_t kCorroborationUs = 300000u;

inline bool plausible(const esc_serial_protocol::Reading& r) {
    // ESC is an 8S-class 70A unit. These deliberately broad diagnostic bounds
    // exclude clearly corrupt readings but DO NOT certify measurement accuracy.
    return r.temperature_c <= 150u
        && r.voltage_centivolt >= 300u
        && r.voltage_centivolt <= 5000u
        && r.current_centiamp <= 30000u
        && r.electrical_rpm <= 1500000u;
}

inline uint32_t absolute_diff(uint32_t a, uint32_t b) {
    return a > b ? a - b : b - a;
}

inline bool consistent(const esc_serial_protocol::Reading& a,
                       const esc_serial_protocol::Reading& b) {
    // Frames on the same electrical bus within a short time should agree
    // on battery voltage and temperature. Do not gate current/RPM changes.
    return absolute_diff(a.voltage_centivolt, b.voltage_centivolt) <= 350u
        && absolute_diff(a.temperature_c, b.temperature_c) <= 20u;
}

enum class ScanResult : uint8_t { Partial, BadCrc, BadRange, Packet };

struct ByteScan {
    uint8_t window[esc_serial_protocol::kFrameSize]{};
    uint8_t count = 0;
    uint32_t last_byte_us = 0;
    bool has_byte = false;
    uint32_t gaps = 0;

    ScanResult push(uint8_t value, uint32_t now_us,
                    esc_serial_protocol::Reading& reading) {
        if (has_byte && static_cast<uint32_t>(now_us - last_byte_us) > kGapUs) {
            if (count) ++gaps;
            count = 0;
        }
        has_byte = true;
        last_byte_us = now_us;
        if (count < esc_serial_protocol::kFrameSize) {
            window[count++] = value;
        } else {
            for (unsigned i = 1; i < esc_serial_protocol::kFrameSize; ++i)
                window[i - 1] = window[i];
            window[esc_serial_protocol::kFrameSize - 1] = value;
        }
        if (count < esc_serial_protocol::kFrameSize) return ScanResult::Partial;
        if (!esc_serial_protocol::decode(window, reading)) return ScanResult::BadCrc;
        if (!plausible(reading)) return ScanResult::BadRange;
        // A complete valid frame consumes all ten bytes.
        count = 0;
        return ScanResult::Packet;
    }
};

struct Gate {
    bool tentative = false;
    bool accepted = false;
    esc_serial_protocol::Reading previous{};
    uint32_t previous_us = 0;
    uint32_t last_accepted_us = 0;

    // First candidate is corroborated by a second frame. After lock,
    // every new frame must remain consistent with the preceding accepted
    // one; failures do not replace the published measurement.
    bool accept(const esc_serial_protocol::Reading& next, uint32_t now_us) {
        if (!plausible(next)) return false;
        const bool recent = tentative
            && static_cast<uint32_t>(now_us - previous_us) <= kCorroborationUs;
        const bool stable = recent && consistent(previous, next);
        if (accepted && stable) {
            previous = next;
            previous_us = now_us;
            last_accepted_us = now_us;
            return true;
        }
        if (!accepted && stable) {
            accepted = true;
            previous = next;
            previous_us = now_us;
            last_accepted_us = now_us;
            return true;
        }
        // New reference frame: wait for another consistent frame.
        tentative = true;
        accepted = false;
        previous = next;
        previous_us = now_us;
        return false;
    }

    void clear() {
        tentative = false;
        accepted = false;
    }
};
}  // namespace esc_serial_stream
#endif
