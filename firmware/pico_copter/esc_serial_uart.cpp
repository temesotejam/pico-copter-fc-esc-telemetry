/*
 * Non-blocking BLHeli_32/KISS ESC TX-pad telemetry on UART1 RX / GPIO5.
 * Each 10-byte 115200/8N1 packet follows a telemetry-bit DShot command.
 * ESCs share the UART RX line; requests are issued to only one ESC at a time.
 *
 * UART1 RX IRQ is higher priority than the 400 Hz PWM tick to prevent
 * 115200-baud receive overruns during IMU/flight-control calculations.
 * ISR stores a decoded snapshot only. All USB printing is elsewhere.
 */
#include "esc_serial_uart.hpp"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

namespace esc_serial {
namespace {
struct MotorState {
    esc_serial_protocol::Reading reading;
    uint32_t last_good_us = 0;
    uint32_t requests = 0;
    uint32_t received = 0;
    uint32_t crc_errors = 0;
    uint32_t timeouts = 0;
};
MotorState motors[4]{};
uint8_t rx_buffer[esc_serial_protocol::kFrameSize]{};
volatile uint8_t rx_count = 0;
volatile uint8_t pending_motor = 0;
volatile bool pending = false;
uint8_t next_motor = 0;
uint32_t request_started_us = 0;
uint32_t last_request_us = 0;
uint32_t last_byte_us = 0;
uint32_t stray_bytes = 0;
bool initialized = false;

void uart1_rx_handler() {
    while (uart_is_readable(uart1)) {
        const uint8_t byte = static_cast<uint8_t>(uart_getc(uart1));
        const uint32_t now_us = time_us_32();
        if (!pending) {
            ++stray_bytes;
            continue;
        }
        // A long inter-byte gap indicates a partial or unrelated packet.
        // Never merge two incomplete packets into one valid reading.
        if (rx_count != 0 &&
            static_cast<uint32_t>(now_us - last_byte_us) > 2000u) {
            rx_count = 0;
        }
        last_byte_us = now_us;
        rx_buffer[rx_count++] = byte;
        if (rx_count == esc_serial_protocol::kFrameSize) {
            MotorState& state = motors[pending_motor];
            esc_serial_protocol::Reading reading{};
            if (esc_serial_protocol::decode(rx_buffer, reading)) {
                state.reading = reading;
                state.last_good_us = now_us;
                ++state.received;
            } else {
                ++state.crc_errors;
            }
            rx_count = 0;
            pending = false;
        }
    }
}
}  // namespace

void init() {
    if (!kEnabled) return;
    // GPIO5 = UART1 RX. UART0 and its existing SBUS IRQ are untouched.
    uart_init(uart1, kBaud);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, false); // IRQ once per byte.
    gpio_set_function(kRxGpio, GPIO_FUNC_UART);
    irq_set_exclusive_handler(UART1_IRQ, uart1_rx_handler);
    // Higher priority than the flight-control PWM IRQ (default 0x80).
    irq_set_priority(UART1_IRQ, 0x40);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);
    initialized = true;
}

void service_400hz(uint32_t now_us) {
    if (!kEnabled || !initialized) return;
    if (pending &&
        static_cast<uint32_t>(now_us - request_started_us) >= kResponseTimeoutUs) {
        ++motors[pending_motor].timeouts;
        rx_count = 0;
        pending = false;
    }
}

bool request_dshot_telemetry(uint8_t motor_index, uint16_t throttle,
                             bool dshot_connected, uint32_t now_us) {
    if (!kEnabled || !initialized || motor_index >= 4 || pending ||
        motor_index != next_motor ||
        static_cast<uint32_t>(now_us - last_request_us) < kRequestSpacingUs)
        return false;
    // Rotate past absent ESCs, or stopped motors, without scheduling a UART
    // response. An ESC becomes eligible only after a fresh, CRC-valid eRPM
    // response has been received on its bidirectional DShot GPIO.
    if (!dshot_connected || throttle == 0) {
        next_motor = static_cast<uint8_t>((next_motor + 1) % 4);
        return false;
    }
    // The caller has verified DShot FIFO capacity. UART1 IRQ may preempt
    // this lower-priority ISR; publish pending=true only after state reset.
    rx_count = 0;
    pending_motor = motor_index;
    request_started_us = now_us;
    last_request_us = now_us;
    next_motor = static_cast<uint8_t>((next_motor + 1) % 4);
    ++motors[motor_index].requests;
    pending = true;
    return true;
}

Snapshot snapshot() {
    Snapshot out{};
    out.enabled = kEnabled && initialized;
    if (!out.enabled) return out;
    // UART1 and PWM ISR both run on core0; no core1 access to these fields.
    const uint32_t irq_state = save_and_disable_interrupts();
    const uint32_t now_us = time_us_32();
    out.stray_bytes = stray_bytes;
    for (unsigned i = 0; i < 4; ++i) {
        out.motors[i].reading = motors[i].reading;
        out.motors[i].requests = motors[i].requests;
        out.motors[i].received = motors[i].received;
        out.motors[i].crc_errors = motors[i].crc_errors;
        out.motors[i].timeouts = motors[i].timeouts;
        if (motors[i].received) {
            out.motors[i].age_ms =
                static_cast<uint32_t>(now_us - motors[i].last_good_us) / 1000u;
            out.motors[i].fresh =
                static_cast<uint32_t>(now_us - motors[i].last_good_us) <= kFreshUs;
        }
    }
    restore_interrupts(irq_state);
    return out;
}
} // namespace esc_serial
