#include "esc_uart_request_policy.hpp"
#include "dshot_protocol.hpp"
#include <cassert>
#include <cstdio>

int main() {
    using namespace esc_uart_request_policy;
    const uint32_t t = 100000;
    assert(!in_window(false, t, t, 8000));
    assert(in_window(true, t, t, 8000));
    assert(in_window(true, t, t + 7999, 8000));
    assert(!in_window(true, t, t + 8000, 8000));
    // A UART-only ESC can be requested using ordinary noninverted DShot.
    const auto normal_request = dshot::frame(500, true);
    const auto normal_command = dshot::frame(500, false);
    assert(normal_request != normal_command);
    assert((normal_request >> 5) == 500);
    assert((normal_request & 0x10u) == 0x10u);
    assert((normal_command & 0x10u) == 0u);
    // Until two corroborated UART frames are accepted, do not mark present.
    assert(!fresh(0, t, t, 500000));
    assert(fresh(1, t, t + 500000, 500000));
    assert(!fresh(1, t, t + 500001, 500000));
    // Wrap-safe age tests.
    assert(in_window(true, 0xfffffff0u, 0x00000020u, 8000));
    assert(fresh(2, 0xfffffff0u, 0x00000020u, 500000));
    std::puts("Pico2 request-only ESC UART attribution policy passed");
}
