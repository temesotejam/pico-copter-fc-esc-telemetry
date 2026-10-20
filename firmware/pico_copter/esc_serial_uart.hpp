#ifndef PICO_COPTER_ESC_SERIAL_UART_HPP
#define PICO_COPTER_ESC_SERIAL_UART_HPP

#include <stdint.h>
#include "esc_serial_protocol.hpp"

// ESC TX/GPIO5 is read-only diagnostic data, never flight-control feedback.
// The single detected ESC can be identified via bidirectional DShot, so
// unsolicited/autonomous KISS serial packets may also be safely attributed.
namespace esc_serial {
constexpr bool kEnabled = true;
constexpr uint32_t kBaud = 115200;
constexpr uint32_t kRequestSpacingUs = 40000u;
constexpr uint32_t kResponseTimeoutUs = 8000u;
constexpr uint32_t kPassiveHoldUs = 250000u;
constexpr uint32_t kFreshUs = 500000u;
constexpr unsigned kRxGpio = 5;

struct MotorSnapshot {
    esc_serial_protocol::Reading reading{};
    uint32_t requests = 0;
    uint32_t received = 0;            // Confirmed, physically plausible frames
    uint32_t passive_received = 0;    // No outstanding request
    uint32_t requested_received = 0;  // Outstanding request
    uint32_t corroborating = 0;       // First frames pending stable confirmation
    uint32_t rejects = 0;
    uint32_t timeouts = 0;
    uint32_t age_ms = UINT32_MAX;
    bool fresh = false;
    bool last_was_passive = false;
};

struct Snapshot {
    MotorSnapshot motors[4]{};
    uint32_t stray_bytes = 0; // UART bytes while no DShot-detected ESC
    uint32_t bytes_total = 0;
    uint32_t bad_crc_windows = 0; // Sliding 10-byte windows, NOT failed replies
    uint32_t bad_range_frames = 0;
    uint32_t plausible_frames = 0;
    uint32_t unassigned_frames = 0;
    uint32_t gaps = 0;
    uint8_t connected_mask = 0;
    bool passive_active = false;
    bool enabled = false;
};

void init();
// Called on core0 PWM IRQ only, following bidirectional eRPM decoding.
void service_400hz(uint32_t now_us, uint8_t connected_mask);
// Called after verifying DShot FIFO capacity. Returns request bit to send.
bool request_dshot_telemetry(uint8_t motor_index, uint16_t throttle,
                             bool dshot_connected, uint32_t now_us);
// Called from core0 USB diagnostics only, not from flight control.
Snapshot snapshot();
}  // namespace esc_serial
#endif
