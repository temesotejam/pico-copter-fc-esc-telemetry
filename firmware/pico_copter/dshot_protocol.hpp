#ifndef PICO_COPTER_DSHOT_PROTOCOL_HPP
#define PICO_COPTER_DSHOT_PROTOCOL_HPP

#include <stdint.h>
#include <cmath>
#include "dshot_config.hpp"

namespace dshot {
// Rate and bidirectional PIO timing are configured in dshot_config.hpp.
// This file handles frame/CRC encoding only. The old one-way frame helpers
// below are also used as the standard packet format in one-way builds.
constexpr uint16_t kStop = 0;
constexpr uint16_t kMinimumThrottle = 48;
constexpr uint16_t kMaximumThrottle = 2047;

// One-way DShot: optionally request a dedicated TX-pad UART response.
// 0: motor stop; 1..47: reserved ESC commands; 48..2047: throttle.
inline uint16_t frame(uint16_t throttle, bool telemetry = false) {
    if (throttle > kMaximumThrottle) throttle = kMaximumThrottle;
    const uint16_t payload = static_cast<uint16_t>((throttle << 1) | (telemetry ? 1u : 0u));
    const uint16_t checksum = static_cast<uint16_t>((payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0fu);
    return static_cast<uint16_t>((payload << 4) | checksum);
}

inline uint16_t bidirectional_frame(uint16_t throttle, bool telemetry = false) {
    if (throttle > kMaximumThrottle) throttle = kMaximumThrottle;
    const uint16_t payload = static_cast<uint16_t>((throttle << 1) | (telemetry ? 1u : 0u));
    const uint16_t checksum = static_cast<uint16_t>(
        ~(payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0fu);
    return static_cast<uint16_t>((payload << 4) | checksum);
}

// Active bidirectional PIO: invert the complete 16-bit logical packet,
// place it in bits 31..16, then shift out MSB-first with MANUAL pull.
inline uint32_t bidirectional_pio_word(uint16_t throttle, bool telemetry = false) {
    return static_cast<uint32_t>(~bidirectional_frame(throttle, telemetry) & 0xffffu) << 16;
}

// In bidirectional DShot, telemetry arrives on the motor signal wire.
// The outgoing frame uses the INVERTED CRC nibble (unlike one-way DShot).

// Keep the confirmed-working duty->DShot mapping unchanged. control.cpp
// clamps running motor duty to 0.10..0.95; map this to DShot 48..2047.
// The PWM-era duty naming remains for compatibility with the control API.
// Zero/negative/nonfinite duty is a motor STOP (DShot 0), never a command 1..47.
// This deliberately changes motor speed response relative to prior PWM/DShot
// versions; check behavior WITHOUT propellers before attempting a flight.
constexpr float kLegacyMinimumRunningDuty = 0.10f;
constexpr float kLegacyMaximumRunningDuty = 0.95f;
inline uint16_t throttle_from_legacy_duty(float duty) {
    if (!std::isfinite(duty) || duty <= 0.0f) return kStop;
    if (duty <= kLegacyMinimumRunningDuty) return kMinimumThrottle;
    if (duty >= kLegacyMaximumRunningDuty) return kMaximumThrottle;

    const float position = (duty - kLegacyMinimumRunningDuty) /
        (kLegacyMaximumRunningDuty - kLegacyMinimumRunningDuty);
    const float value = static_cast<float>(kMinimumThrottle) +
        static_cast<float>(kMaximumThrottle - kMinimumThrottle) * position;
    return static_cast<uint16_t>(value + 0.5f);
}

// This is the active frame-word helper in ONE-WAY builds.
// Bidirectional builds use bidirectional_pio_word() instead.
inline uint32_t pio_word(uint16_t command, bool telemetry = false) {
    return static_cast<uint32_t>(frame(command, telemetry)) << 16;
}
} // namespace dshot

#endif
