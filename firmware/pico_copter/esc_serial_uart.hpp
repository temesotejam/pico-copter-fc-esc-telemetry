#ifndef PICO_COPTER_ESC_SERIAL_UART_HPP
#define PICO_COPTER_ESC_SERIAL_UART_HPP

#include <stdint.h>
#include "esc_serial_protocol.hpp"

// Both Pico/RP2040 and Pico 2/RP2350 share GP17 dedicated PIO UART.
// Dedicated TX-pad packets are accepted only after a DShot telemetry request;
// source identification is independent of bidirectional-DShot eRPM.
// FC_SHARED_ESC_UART uses the same request-driven path for both chips.
#define FC_SHARED_ESC_UART 1
namespace esc_serial {
constexpr bool kEnabled = true;
constexpr uint32_t kBaud = 115200;
constexpr uint32_t kRequestSpacingUs = 40000u;
constexpr uint32_t kResponseTimeoutUs = 8000u;
constexpr uint32_t kPassiveHoldUs = 250000u;
constexpr uint32_t kFreshUs = 500000u;
constexpr unsigned kRxGpio = 17; // independent GP17 PIO RX on either chip
constexpr bool kProbeWithoutBidir = true;
constexpr int kFixedUartSourceMotor = -1;

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
    uint32_t stray_bytes = 0; // bytes outside request window on both boards
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
