#include "pico2_uart_rx.hpp"
#if PICO_RP2350 || PICO_RP2040
#include "pico2_uart_rx.pio.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

namespace pico2_uart_rx {
namespace {
constexpr uint kRxPin = 17;
constexpr uint32_t kBaud = 115200;
#if PICO_RP2350
PIO port = pio2;
#else
PIO port = pio1;  // RP2040 has no PIO2
#endif
uint sm = 0;
Receiver consumer = nullptr;
void on_rx_irq() {
    while (!pio_sm_is_rx_fifo_empty(port, sm)) {
        const uint8_t byte = static_cast<uint8_t>(pio_sm_get(port, sm) >> 24);
        if (consumer) consumer(byte, time_us_32());
    }
}
}
void init(Receiver receiver) {
    consumer = receiver;
    const uint offset = pio_add_program(port, &pico2_uart_rx_program);
    sm = pio_claim_unused_sm(port, true);
    pio_gpio_init(port, kRxPin);
    gpio_pull_up(kRxPin);
    pio_sm_config cfg = pico2_uart_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&cfg, kRxPin);
    sm_config_set_jmp_pin(&cfg, kRxPin);
    sm_config_set_in_shift(&cfg, true, false, 32);
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&cfg, static_cast<float>(clock_get_hz(clk_sys)) /
                                    (8.0f * kBaud));
    pio_sm_init(port, sm, offset, &cfg);
    pio_sm_set_consecutive_pindirs(port, sm, kRxPin, 1, false);
    irq_set_exclusive_handler(pio_get_irq_num(port, 0), on_rx_irq);
    irq_set_priority(pio_get_irq_num(port, 0), 0x40);
    pio_set_irq0_source_enabled(port, pio_get_rx_fifo_not_empty_interrupt_source(sm), true);
    irq_set_enabled(pio_get_irq_num(port, 0), true);
    pio_sm_set_enabled(port, sm, true);
}
}
#endif
