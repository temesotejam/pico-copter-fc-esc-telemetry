/*
 * Continuous, nonblocking BLHeli_32 TX-pad UART telemetry at 115200/8N1.
 * Some ESC firmware autonomously transmits ~10-byte frames every ~32ms.
 * Do NOT reset packet phase when requesting DShot data; scan every byte.
 *
 * Source attribution: only one bidir-DShot-detected ESC -> passive UART
 * frames can safely be assigned. Multiple detected ESCs -> unsolicited
 * packets cannot be assigned to a motor (no address in KISS frame).
 * Never change flight-control outputs/arming/failsafe based on UART data.
 */
#include "esc_serial_uart.hpp"
#include "esc_serial_stream.hpp"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#if PICO_RP2350
#include "pico2_uart_rx.hpp"
#endif

namespace esc_serial {
namespace {
struct MotorState {
    esc_serial_protocol::Reading reading{};
    esc_serial_stream::Gate gate{};
    uint32_t last_good_us = 0;
    uint32_t requests = 0;
    uint32_t received = 0;
    uint32_t passive_received = 0;
    uint32_t requested_received = 0;
    uint32_t rejects = 0;
    uint32_t corroborating = 0;
    uint32_t timeouts = 0;
    bool last_was_passive = false;
};
MotorState motors[4]{};
esc_serial_stream::ByteScan scanner{};
volatile uint8_t connected_mask = 0; // refreshed in 400Hz DShot ISR
volatile uint8_t pending_motor = 0;
volatile bool pending = false;
uint8_t next_motor = 0;
uint32_t request_started_us = 0;
uint32_t last_request_us = 0;
uint32_t last_unsolicited_us = 0;
bool seen_unsolicited = false;
uint32_t bytes_total = 0;
uint32_t bad_crc_windows = 0;
uint32_t bad_range_frames = 0;
uint32_t plausible_frames = 0;
uint32_t unassigned_frames = 0;
uint32_t stray_bytes = 0;
bool initialized = false;

int sole_motor(uint8_t mask) {
    if (mask == 0 || (mask & (mask - 1u))) return -1;
    for (int i = 0; i < 4; ++i)
        if (mask & (1u << i)) return i;
    return -1;
}

void on_uart_byte(uint8_t byte, uint32_t now_us) {
    ++bytes_total;
        if (!connected_mask) ++stray_bytes;

        esc_serial_protocol::Reading reading{};
        const auto result = scanner.push(byte, now_us, reading);
        if (result == esc_serial_stream::ScanResult::Partial) continue;
        if (result == esc_serial_stream::ScanResult::BadCrc) {
            ++bad_crc_windows;
            continue;
        }
        if (result == esc_serial_stream::ScanResult::BadRange) {
            ++bad_range_frames;
            continue;
        }

        ++plausible_frames;
        const bool requested = pending;
        // A valid unsolicited frame indicates that the TX output can be
        // streaming without DShot telemetry-request bits.
        if (!requested) {
            seen_unsolicited = true;
            last_unsolicited_us = now_us;
        }

        const int single = kFixedUartSourceMotor >= 0
            ? kFixedUartSourceMotor : sole_motor(connected_mask);
        int owner = -1;
        if (single >= 0) {
            // The only detected ESC is the only attributable source.
            owner = single;
        } else if (requested && ((connected_mask & (1u << pending_motor)) ||
                                  kProbeWithoutBidir)) {
            // For multiple detected ESCs, only a pending, individually
            // requested reply is attributable. Autonomous packets are not.
            // If autonomous traffic is active, request scheduling is paused.
            owner = pending_motor;
        }
        if (requested) pending = false;

        if (owner < 0) {
            ++unassigned_frames;
            continue;
        }
        MotorState& state = motors[owner];
        if (!state.gate.accept(reading, now_us)) {
            ++state.corroborating;
            continue;
        }
        state.reading = reading;
        state.last_good_us = now_us;
        state.last_was_passive = !requested;
        ++state.received;
        if (requested) ++state.requested_received;
        else ++state.passive_received;

}
#if !PICO_RP2350
void uart1_rx_handler() {
    while (uart_is_readable(uart1)) {
        on_uart_byte(static_cast<uint8_t>(uart_getc(uart1)), time_us_32());
    }
}
#endif
}  // namespace

void init() {
    if (!kEnabled) return;
#if PICO_RP2350
    // GP17 is UART0 RX but UART0 already carries SBUS on GP1.
    // Dedicated PIO2 UART RX avoids that peripheral conflict.
    pico2_uart_rx::init(on_uart_byte);
#else
    uart_init(uart1, kBaud);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, false);
    gpio_set_function(kRxGpio, GPIO_FUNC_UART);
    irq_set_exclusive_handler(UART1_IRQ, uart1_rx_handler);
    irq_set_priority(UART1_IRQ, 0x40);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);
#endif
    initialized = true;
}

void service_400hz(uint32_t now_us, uint8_t connected) {
    if (!kEnabled || !initialized) return;
    const uint8_t new_mask = connected & 0x0fu;
    if (new_mask != connected_mask) {
        // A change of connected motor(s) may change who is driving TX.
        // Never reuse the preceding source's corroboration reference.
        for (MotorState& motor : motors) motor.gate.clear();
        connected_mask = new_mask;
    }
    if (pending &&
        static_cast<uint32_t>(now_us - request_started_us) >= kResponseTimeoutUs) {
        ++motors[pending_motor].timeouts;
        pending = false;
    }
}

bool request_dshot_telemetry(uint8_t motor_index, uint16_t throttle,
                             bool dshot_connected, uint32_t now_us) {
    if (!kEnabled || !initialized || motor_index >= 4 || pending ||
        motor_index != next_motor ||
        static_cast<uint32_t>(now_us - last_request_us) < kRequestSpacingUs)
        return false;
    next_motor = static_cast<uint8_t>((next_motor + 1) % 4);
    if ((!dshot_connected && !kProbeWithoutBidir) || throttle == 0) return false;

    // If valid autonomous frames are already arriving, further request
    // bits are redundant and can collide with the auto-transmission.
    // A 250ms absence allows request-driven fallback for Auto=Off ESCs.
    if (seen_unsolicited &&
        static_cast<uint32_t>(now_us - last_unsolicited_us) <= kPassiveHoldUs)
        return false;

    pending_motor = motor_index;
    request_started_us = now_us;
    last_request_us = now_us;
    ++motors[motor_index].requests;
    pending = true;
    return true;
}

Snapshot snapshot() {
    Snapshot out{};
    out.enabled = kEnabled && initialized;
    if (!out.enabled) return out;
    const uint32_t irq_state = save_and_disable_interrupts();
    const uint32_t now_us = time_us_32();
    out.stray_bytes = stray_bytes;
    out.bytes_total = bytes_total;
    out.bad_crc_windows = bad_crc_windows;
    out.bad_range_frames = bad_range_frames;
    out.plausible_frames = plausible_frames;
    out.unassigned_frames = unassigned_frames;
    out.gaps = scanner.gaps;
    out.connected_mask = connected_mask;
    out.passive_active = seen_unsolicited &&
        static_cast<uint32_t>(now_us - last_unsolicited_us) <= kPassiveHoldUs;
    for (unsigned i = 0; i < 4; ++i) {
        const MotorState& m = motors[i];
        auto& dst = out.motors[i];
        dst.reading = m.reading;
        dst.requests = m.requests;
        dst.received = m.received;
        dst.passive_received = m.passive_received;
        dst.requested_received = m.requested_received;
        dst.rejects = m.rejects;
        dst.corroborating = m.corroborating;
        dst.timeouts = m.timeouts;
        dst.last_was_passive = m.last_was_passive;
        if (m.received) {
            dst.age_ms = static_cast<uint32_t>(now_us - m.last_good_us) / 1000u;
            dst.fresh = static_cast<uint32_t>(now_us - m.last_good_us) <= kFreshUs
                && ((connected_mask & (1u << i)) ||
                    (kProbeWithoutBidir && m.requested_received != 0) ||
                    (kFixedUartSourceMotor == static_cast<int>(i)));
        }
    }
    restore_interrupts(irq_state);
    return out;
}
}  // namespace esc_serial
