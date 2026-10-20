#ifndef PICO_COPTER_ESC_DSHOT_PRESENCE_HPP
#define PICO_COPTER_ESC_DSHOT_PRESENCE_HPP

#include <stdint.h>

// Detection uses only checksum-valid bidirectional DShot eRPM responses.
// CRC/GCR errors, raw UART bytes, and send counters do NOT imply connection.
// Does NOT influence motor output, failsafe, arming, PID, or EKF.
namespace esc_dshot_presence {
constexpr uint32_t kValidReplyUs = 250000u;
// Unsigned wrap-safe age check. 0 eRPM after a valid stopped-motor response
// is still a presence indication.
inline bool connected(uint32_t valid_replies, uint32_t last_valid_us,
                      uint32_t now_us) {
    return valid_replies != 0u &&
           static_cast<uint32_t>(now_us - last_valid_us) <= kValidReplyUs;
}
}
#endif
