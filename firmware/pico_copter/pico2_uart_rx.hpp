#ifndef PICO2_ESC_UART_RX_HPP
#define PICO2_ESC_UART_RX_HPP
#include <stdint.h>
namespace pico2_uart_rx {
using Receiver = void (*)(uint8_t byte, uint32_t timestamp_us);
void init(Receiver receiver);
}
#endif
