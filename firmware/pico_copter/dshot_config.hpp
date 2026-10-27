#ifndef PICO_COPTER_DSHOT_CONFIG_HPP
#define PICO_COPTER_DSHOT_CONFIG_HPP

#include <stdint.h>

// ============================================================================
// DShot communication rate: change JUST THIS VALUE and rebuild the Pico UF2.
//   150 = DShot150; 300 = DShot300 (RP2040 hardware-tested);
//   600 = DShot600; 1200 = DShot1200 (higher rates need ESC/oscilloscope test).
// DShot direction is set below: 1=bidirectional or 0=one-way. Neither
// setting changes the 400 Hz motor update period or PID/EKF control.
//
// A build can optionally override this with -DDSHOT_RATE_KBPS=600.
// ============================================================================
#ifndef DSHOT_RATE_KBPS
#define DSHOT_RATE_KBPS 300
#endif

// Manually selected protocol. NO runtime auto-switching.
//   1 = bidirectional DShot (inverted TX/CRC, ESC eRPM reply; selected)
//   0 = standard one-way DShot (normal TX/CRC, NO eRPM reply)
// To change: edit just this value, rebuild, and flash the resulting UF2.
// Alternatively pass -DDSHOT_BIDIRECTIONAL=0/1 to CMake.
#ifndef DSHOT_BIDIRECTIONAL
#if PICO_RP2350
#define DSHOT_BIDIRECTIONAL 1  // Pico2: experimental bidirectional DShot300, shifter GP16/GP18
#else
#define DSHOT_BIDIRECTIONAL 1  // Pico/RP2040: preserve original mode
#endif
#endif

static_assert(DSHOT_BIDIRECTIONAL == 0 || DSHOT_BIDIRECTIONAL == 1,
              "DSHOT_BIDIRECTIONAL must be 0 (one-way) or 1 (bidirectional).");

static_assert(DSHOT_RATE_KBPS == 150 || DSHOT_RATE_KBPS == 300 ||
              DSHOT_RATE_KBPS == 600 || DSHOT_RATE_KBPS == 1200,
              "Unsupported DShot speed: choose 150, 300, 600, or 1200.");

namespace dshot {
constexpr bool kBidirectional = (DSHOT_BIDIRECTIONAL == 1);
constexpr uint32_t kRateKbps = DSHOT_RATE_KBPS;
constexpr uint32_t kBitRate = kRateKbps * 1000u;

// The proven PIO TX loop takes 40 PIO instructions/cycles per 1 DShot bit.
// Changing this value alone would change the waveform and RX sampling.
// Keep it at 40: PIO CLKDIV, TX, and RX all scale with kBitRate.
constexpr uint32_t kPioCyclesPerBit = 40;
constexpr uint32_t kPioClockHz = kBitRate * kPioCyclesPerBit;
// Proven conventional DShot PIO: 8 cycles/bit, non-inverted TX only.
constexpr uint32_t kOneWayPioCyclesPerBit = 8;
constexpr uint32_t kOneWayPioClockHz = kBitRate * kOneWayPioCyclesPerBit;

// Host sends this loop count into the second PIO TX word. PIO executes
// roughly one iteration per clock; integer values for all four speeds.
// At DShot300 the count remains EXACTLY 300 (previous verified firmware).
constexpr uint32_t kTurnaroundUs = 25;
constexpr uint32_t kTurnaroundCycles =
    kTurnaroundUs * kPioClockHz / 1000000u;

static_assert(kPioClockHz >= 6000000u && kPioClockHz <= 48000000u,
              "PIO clock outside tested DShot configuration limits.");
static_assert(kTurnaroundCycles == kRateKbps,
              "Unexpected PIO turnaround calculation.");

// Scale the PIO reply-start search to roughly 43 us at every DShot speed.
// The existing PIO program contains SET Y,7. At the proven DShot300
// setting, kRxWaitOuterY is 7 and its instruction is left unchanged.
// At other speeds, patch only that SET instruction in PIO memory BEFORE
// starting state machines (150:3, 300:7, 600:15, 1200:31).
constexpr uint32_t kRxWaitOuterY = (8u * kRateKbps / 300u) - 1u;
static_assert(kRxWaitOuterY <= 31u, "RX wait loop must fit 5-bit PIO SET");


} // namespace dshot
#endif
