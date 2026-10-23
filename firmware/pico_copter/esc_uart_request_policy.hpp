#ifndef PICO_COPTER_ESC_UART_REQUEST_POLICY_HPP
#define PICO_COPTER_ESC_UART_REQUEST_POLICY_HPP
#include <stdint.h>

// RP2350 request-triggered ESC TX telemetry. KISS frames carry no ESC ID.
// Accept a frame for motor N only inside the one outstanding DShot request
// window. Never derive the source from a passive UART stream.
namespace esc_uart_request_policy {
inline bool in_window(bool pending, uint32_t request_started_us,
                      uint32_t now_us, uint32_t timeout_us) {
    return pending &&
        static_cast<uint32_t>(now_us - request_started_us) < timeout_us;
}
inline bool fresh(uint32_t valid_requested_frames, uint32_t last_good_us,
                  uint32_t now_us, uint32_t fresh_us) {
    return valid_requested_frames != 0 &&
        static_cast<uint32_t>(now_us - last_good_us) <= fresh_us;
}
}
#endif
