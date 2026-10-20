
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART_ID uart0
#define BAUD_RATE 100000
#define DATA_BITS 8
#define STOP_BITS 2
#define PARITY UART_PARITY_EVEN
#define CH1MAX 1680
#define CH1MIN 368
#define CH2MAX 1680
#define CH2MIN 368
#define CH3MAX 1680
#define CH3MIN 368
#define CH4MAX 1680
#define CH4MIN 368
#define CH5MAX 1904
#define CH5MIN 144
#define CH6MAX 1904
#define CH6MIN 144


//０番と1番ピンに接続
#define UART_TX_PIN 0
#define UART_RX_PIN 1

//グローバル変数の宣言
extern uint16_t Chdata[18];
struct RadioStatus {
    bool healthy;
    uint32_t failsafe_count; // Records brief failsafe events across good frames
};
RadioStatus radio_update(void);

// Read-only, low-rate USB diagnostics. Channels hold the last accepted
// healthy frame and therefore can be stale after the RC link is lost.
struct RadioDebugSnapshot {
    uint16_t channels[6];
    uint32_t age_ms;        // UINT32_MAX before first healthy SBUS frame
    uint32_t good_frames;
    uint32_t failsafe_events;
    uint8_t flags;          // flags from the most recently decoded frame
    bool healthy;
    bool received;
};
RadioDebugSnapshot radio_debug_snapshot(void);

//グローバル関数の宣言
void radio_init(void);
