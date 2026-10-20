/*
 * radio.cpp - SBUS receiver handling and link freshness.
 */
#include "radio.hpp"
#include "sbus_parser.hpp"
#include "hardware/sync.h"

uint16_t Chdata[18] = {};

namespace {
// Updated exclusively in UART IRQ on core 0. The PWM IRQ snapshots this with
// interrupts disabled, avoiding half-written SBUS control frames.
sbus::Decoder decoder;
volatile uint16_t latest_channels[16] = {};
volatile uint8_t latest_flags = 0;
volatile uint8_t latest_end = 0;
volatile uint32_t last_healthy_us = 0;
volatile uint32_t failsafe_event_count = 0;
volatile uint32_t good_frame_count = 0;
volatile bool have_healthy_frame = false;
}

void on_uart_rx(void);

void radio_init(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, true);
    const int uart_irq = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, on_uart_rx);
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(UART_ID, true, false);
}

RadioStatus radio_update(void) {
    const uint32_t irq_state = save_and_disable_interrupts();
    // Read the timer inside the critical section. A packet arriving between
    // an earlier clock read and this snapshot must not look "71 min old".
    const uint32_t now_us = time_us_32();
    for (uint32_t i = 0; i < 16; ++i) Chdata[i] = latest_channels[i];
    Chdata[16] = latest_flags;
    Chdata[17] = latest_end;
    const bool fresh = sbus::link_fresh(now_us, last_healthy_us, have_healthy_frame);
    const bool explicit_failsafe = (latest_flags & sbus::kFailsafeFlag) != 0;
    const uint32_t events = failsafe_event_count;
    restore_interrupts(irq_state);
    return {fresh && !explicit_failsafe, events};
}

void on_uart_rx(void) {
    while (uart_is_readable(UART_ID)) {
        sbus::Frame frame;
        if (!decoder.push(uart_getc(UART_ID), time_us_32(), frame)) continue;
        // Loss-flagged frames do not refresh the watchdog. Explicit SBUS
        // failsafe (bit 3) is recorded even if good frames arrive later.
        latest_flags = frame.flags;
        latest_end = frame.end_byte;
        if (frame.flags & sbus::kFailsafeFlag) ++failsafe_event_count;
        if (!sbus::healthy_frame(frame)) continue;
        for (uint32_t i = 0; i < 16; ++i) latest_channels[i] = frame.channels[i];
        last_healthy_us = time_us_32();
        have_healthy_frame = true;
        ++good_frame_count;
    }
}

RadioDebugSnapshot radio_debug_snapshot(void) {
    RadioDebugSnapshot snapshot{};
    // UART RX interrupt and PWM control loop both run on core 0.
    const uint32_t saved = save_and_disable_interrupts();
    const uint32_t now_us = time_us_32();
    for (uint32_t i = 0; i < 6; ++i) snapshot.channels[i] = latest_channels[i];
    snapshot.received = have_healthy_frame;
    snapshot.age_ms = have_healthy_frame
        ? static_cast<uint32_t>(now_us - last_healthy_us) / 1000u
        : UINT32_MAX;
    snapshot.flags = latest_flags;
    snapshot.good_frames = good_frame_count;
    snapshot.failsafe_events = failsafe_event_count;
    snapshot.healthy = sbus::link_fresh(now_us, last_healthy_us, have_healthy_frame)
        && (latest_flags & sbus::kFailsafeFlag) == 0;
    restore_interrupts(saved);
    return snapshot;
}
