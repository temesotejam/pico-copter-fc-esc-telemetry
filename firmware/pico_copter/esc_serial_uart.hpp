#ifndef PICO_COPTER_ESC_SERIAL_UART_HPP
#define PICO_COPTER_ESC_SERIAL_UART_HPP

#include <stdint.h>
#include "esc_serial_protocol.hpp"

// Dedicated ESC TX telemetry is separate from bidirectional DShot/eRPM.
// Read only: it must never control motors, arming, PID, EKF or the failsafe.
namespace esc_serial {
constexpr bool kEnabled = true;  // Set false to disable ESC TX/UART telemetry.
constexpr uint32_t kBaud = 115200;
constexpr uint32_t kRequestSpacingUs = 40000;  // 25 requests/s total = 6.25/motor.
constexpr uint32_t kResponseTimeoutUs = 8000;
constexpr uint32_t kFreshUs = 500000;
// GPIO5 = UART1 RX. UART0 GPIO1 remains reserved for SBUS.
constexpr unsigned kRxGpio = 5;

struct MotorSnapshot {
    esc_serial_protocol::Reading reading;
    uint32_t requests = 0;
    uint32_t received = 0;
    uint32_t crc_errors = 0;
    uint32_t timeouts = 0;
    uint32_t age_ms = UINT32_MAX;
    bool fresh = false;
};

struct Snapshot {
    MotorSnapshot motors[4];
    uint32_t stray_bytes = 0;
    bool enabled = false;
};

void init();
// Called from the 400Hz core0 PWM IRQ, never blocks or waits for the ESC.
void service_400hz(uint32_t now_us);
// True exactly once per scheduled serial telemetry request. Call only after
// the matching DShot TX has capacity; never request on a stopped motor.
bool request_dshot_telemetry(uint8_t motor_index, uint16_t throttle, uint32_t now_us);
// Called only from core0's background USB diagnostic thread.
Snapshot snapshot();
}  // namespace esc_serial
#endif
