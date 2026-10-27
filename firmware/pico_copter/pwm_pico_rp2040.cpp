/*
 * RP2040 motor driver for the SAME GP16/17/18 level-shifter circuit as Pico 2.
 * Four motor DShot channels are controlled by PIO0. PIO1 hosts a timing
 * controller and a separate GP17 UART receiver. Only hardware/bench tests
 * can validate cross-PIO timing; this is not a flight-certified port.
 * Four physical DShot channels: GP15/14/3/2. GP20/21 are INPUT with weak
 * pull-downs and do NOT emit DShot STOP on RP2040. Do not use those ESCs.
 * SN74LVC245A A-side=Pico2, B-side=ESC, DIR=GP16 (pin 1), /OE=GP18 (pin 19).
 */
#include "pwm.hpp"
#include "dshot_protocol.hpp"
#include "dshot_bidir_decode.hpp"
#include "esc_dshot_presence.hpp"
#include "esc_serial_uart.hpp"
#include "dshot_tx_pico2.pio.h"
#include "dshot_oneway.pio.h"
#include "pico_shifter_rp2040.pio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

static_assert(!PICO_RP2350, "pwm_pico_rp2040.cpp is only for RP2040.");
#if DSHOT_BIDIRECTIONAL
static_assert(DSHOT_RATE_KBPS == 300,
    "Pico2 level-shifter bidirectional synchronizer currently supports DShot300 only.");
#endif

uint8_t ESC_calib = 0;
namespace {
constexpr uint16_t kWrap = 3124;
constexpr uint32_t kControlHz = 400;
constexpr unsigned kChannels = 4;
constexpr unsigned kControlled = 4;
constexpr uint kDirGpio = 16; // IC pin 1; 1 => Pico A to ESC B
constexpr uint kOeGpio = 18;  // IC pin 19; 1 => Hi-Z
constexpr uint kPins[kChannels] = {15, 14, 3, 2};
PIO motor_port(unsigned) { return pio0; }
uint sm[kChannels]{};
uint offset0 = 0;
uint ctrl_sm = 0, ctrl_offset = 0;
uint mask0 = 0;
float duties[kControlled]{};
bool sent[kChannels]{};
bool ready = false;
struct State {
    uint32_t erpm=0, good_at=0, tx=0, rx_ok=0, rx_timeout=0;
    uint32_t bad_gcr=0, bad_crc=0, other=0, stray=0, busy=0;
    uint16_t command=0, frame=0;
} stats[kChannels]{};

void shifter_pins_init() {
    // NOTE: /OE requires a hardware 3.3V pullup so boot and reset stay Hi-Z.
    gpio_init(kDirGpio);
    gpio_put(kDirGpio, true);
    gpio_set_dir(kDirGpio, GPIO_OUT);
    gpio_init(kOeGpio);
    gpio_put(kOeGpio, true);
    gpio_set_dir(kOeGpio, GPIO_OUT);
}

void initialize_motor_channels() {
    const pio_program *program;
#if DSHOT_BIDIRECTIONAL
    program = &dshot_tx_pico2_program;
#else
    program = &dshot_oneway_program;
#endif
    offset0 = pio_add_program(pio0, program);
    const float clockdiv = static_cast<float>(clock_get_hz(clk_sys)) /
#if DSHOT_BIDIRECTIONAL
        static_cast<float>(dshot::kPioClockHz);
#else
        static_cast<float>(dshot::kOneWayPioClockHz);
#endif
    for (unsigned i = 0; i < kChannels; ++i) {
        PIO p = motor_port(i);
        sm[i] = pio_claim_unused_sm(p, true);
        const uint pin = kPins[i];
        pio_sm_config cfg =
#if DSHOT_BIDIRECTIONAL
            dshot_tx_pico2_program_get_default_config(offset0);
#else
            dshot_oneway_program_get_default_config(offset0);
#endif
#if DSHOT_BIDIRECTIONAL
        sm_config_set_out_pins(&cfg, pin, 1);
        sm_config_set_set_pins(&cfg, pin, 1);
        sm_config_set_in_pins(&cfg, pin);
        sm_config_set_jmp_pin(&cfg, pin);
        sm_config_set_out_shift(&cfg, false, false, 32);
        sm_config_set_in_shift(&cfg, false, false, 32);
#else
        sm_config_set_sideset_pins(&cfg, pin);
        sm_config_set_out_shift(&cfg, false, true, 16);
        sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_TX);
#endif
        sm_config_set_clkdiv(&cfg, clockdiv);
        pio_gpio_init(p, pin);
#if DSHOT_BIDIRECTIONAL
        gpio_set_pulls(pin, true, false); // idle high on Pico side
#endif
        pio_sm_init(p, sm[i], offset0, &cfg);
#if DSHOT_BIDIRECTIONAL
        pio_sm_set_pins(p, sm[i], 1);
        pio_sm_set_consecutive_pindirs(p, sm[i], pin, 1, false);
#else
        pio_sm_set_consecutive_pindirs(p, sm[i], pin, 1, true);
#endif
        mask0 |= 1u << sm[i];
    }
}

#if DSHOT_BIDIRECTIONAL
void initialize_direction_sm() {
    // The GP17 PIO UART was already allocated in PIO1. The shifter PIO
    // shares PIO1's 32-instruction memory (24 + 8 instructions).
    ctrl_offset = pio_add_program(pio1, &pico_shifter_rp2040_program);
    ctrl_sm = pio_claim_unused_sm(pio1, true);
    pio_sm_config cfg = pico_shifter_rp2040_program_get_default_config(ctrl_offset);
    // Nonadjacent pins: SET drives GP16/DIR, side-set drives GP18,/OE.
    sm_config_set_set_pins(&cfg, kDirGpio, 1);
    sm_config_set_sideset_pins(&cfg, kOeGpio);
    sm_config_set_clkdiv(&cfg, static_cast<float>(clock_get_hz(clk_sys)) /
                                    static_cast<float>(dshot::kPioClockHz));
    pio_gpio_init(pio1, kDirGpio);
    pio_gpio_init(pio1, kOeGpio);
    pio_sm_init(pio1, ctrl_sm, ctrl_offset, &cfg);
    // Initialize both nonadjacent pins HIGH while the SM is disabled.
    pio_sm_set_pins_with_mask(pio1, ctrl_sm,
        (1u << kDirGpio) | (1u << kOeGpio),
        (1u << kDirGpio) | (1u << kOeGpio));
    pio_sm_set_consecutive_pindirs(pio1, ctrl_sm, kDirGpio, 1, true);
    pio_sm_set_consecutive_pindirs(pio1, ctrl_sm, kOeGpio, 1, true);
}
void collect() {
    const uint32_t now = time_us_32();
    for (unsigned i=0; i<kChannels; ++i) {
        PIO p=motor_port(i);
        bool received=false;
        while (!pio_sm_is_rx_fifo_empty(p, sm[i])) {
            const uint32_t word=pio_sm_get(p,sm[i]);
            if (!sent[i] || received) { ++stats[i].stray; continue; }
            received=true;
            auto decoded=dshot::decode_erpm(word);
            switch (decoded.result) {
            case dshot::RxResult::Erpm:
                stats[i].erpm=decoded.erpm; stats[i].good_at=now; ++stats[i].rx_ok; break;
            case dshot::RxResult::NoResponse: ++stats[i].rx_timeout; break;
            case dshot::RxResult::BadGcr: ++stats[i].bad_gcr; break;
            case dshot::RxResult::BadChecksum: ++stats[i].bad_crc; break;
            case dshot::RxResult::OtherTelemetry: ++stats[i].other; break;
            }
        }
        if (sent[i] && !received) ++stats[i].rx_timeout;
        sent[i]=false;
    }
}
#endif

void send_cycle() {
    if (!ready) return;
    const uint32_t now=time_us_32();
#if DSHOT_BIDIRECTIONAL
    // Preceding PIO cycle has ended and /OE is disabled. Reset *only*
    // communication SMs; never reset continuously running UART RX on PIO2.
    pio_set_sm_mask_enabled(pio0, mask0, false);
    pio_set_sm_mask_enabled(pio1, 1u << ctrl_sm, false);
#endif
    for (unsigned i=0; i<kChannels; ++i) {
        PIO p=motor_port(i);
        const uint16_t throttle = i < kControlled
            ? dshot::throttle_from_legacy_duty(duties[i]) : 0;
        const bool detected = i < kControlled &&
            esc_dshot_presence::connected(stats[i].rx_ok,stats[i].good_at,now);
        const bool serial_request = i < kControlled &&
            esc_serial::request_dshot_telemetry(i, throttle, detected, now);
#if DSHOT_BIDIRECTIONAL
        pio_sm_clear_fifos(p,sm[i]);
        pio_sm_restart(p,sm[i]);
        pio_sm_exec(p,sm[i],pio_encode_jmp(offset0));
        pio_sm_put(p,sm[i],dshot::bidirectional_pio_word(throttle,serial_request));
        pio_sm_put(p,sm[i],dshot::kTurnaroundCycles);
        stats[i].frame=dshot::bidirectional_frame(throttle,serial_request);
        sent[i]=true;
#else
        if (!pio_sm_is_tx_fifo_empty(p,sm[i])) {
            pio_sm_clear_fifos(p,sm[i]); ++stats[i].busy;
        }
        pio_sm_put(p,sm[i],dshot::pio_word(throttle,serial_request));
        stats[i].frame=dshot::frame(throttle,serial_request);
        sent[i]=false;
#endif
        stats[i].command=throttle;
        ++stats[i].tx;
    }
#if DSHOT_BIDIRECTIONAL
    pio_sm_restart(pio1,ctrl_sm);
    pio_sm_exec(pio1,ctrl_sm,pio_encode_jmp(ctrl_offset));
    // RP2040 has no cross-PIO simultaneous-start instruction. Motor PIO0
    // is started first, immediately followed by GP16/18 shifter PIO1.
    // The shifter program is adjusted for this skew. Scope verification
    // is mandatory before connecting an ESC, unlike RP2350's hardware sync.
    pio_enable_sm_mask_in_sync(pio0, mask0);
    pio_enable_sm_mask_in_sync(pio1, 1u << ctrl_sm);
#endif
}
void wrap_irq() {
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN));
    MAINLOOP(); // Existing four-motor PID, arm, failsafe untouched.
#if DSHOT_BIDIRECTIONAL
    collect();
    uint8_t mask=0;
    const uint32_t now=time_us_32();
    for (unsigned i=0; i<kControlled; ++i)
        if (esc_dshot_presence::connected(stats[i].rx_ok,stats[i].good_at,now))
            mask |= 1u<<i;
    esc_serial::service_400hz(now,mask);
#else
    esc_serial::service_400hz(time_us_32(),0);
#endif
    send_cycle();
}
}

void pwm_init() {
    shifter_pins_init();
    // On the six-signal circuit the two unused lines must not float;
    // they remain inputs (not DShot-enabled) with weak pull-downs.
    gpio_init(20); gpio_pull_down(20);
    gpio_init(21); gpio_pull_down(21);
    initialize_motor_channels();
#if DSHOT_BIDIRECTIONAL
    initialize_direction_sm();
#else
    // With output configured, select permanently forward (A->B) on Pico2.
    gpio_put(kDirGpio,true);
    gpio_put(kOeGpio,false);
    pio_set_sm_mask_enabled(pio0,mask0,true);

#endif
    const uint pwm_slice=pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN);
    const uint servo_slice=pwm_gpio_to_slice_num(PWM_PIN_SERVO);
    if (servo_slice==pwm_slice) panic("Pico2 servo/timebase PWM collision");
    const float divider=static_cast<float>(clock_get_hz(clk_sys)) /
                        (static_cast<float>(kWrap+1u)*kControlHz);
    gpio_set_function(PWM_PIN_SERVO,GPIO_FUNC_PWM);
    pwm_set_wrap(servo_slice,kWrap);
    pwm_set_clkdiv(servo_slice,divider);
    pwm_set_gpio_level(PWM_PIN_SERVO,625);
    pwm_set_wrap(pwm_slice,kWrap);
    pwm_set_clkdiv(pwm_slice,divider);
    pwm_clear_irq(pwm_slice);
    pwm_set_irq_enabled(pwm_slice,true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP,wrap_irq);
    irq_set_enabled(PWM_IRQ_WRAP,true);
    pwm_set_enabled(servo_slice,true);
    pwm_set_enabled(pwm_slice,true);
    ready=true;
    sleep_ms(3000);
    sleep_ms(1000);
}
void set_duty_fr(float duty) { duties[0]=duty; }
void set_duty_fl(float duty) { duties[1]=duty; }
void set_duty_rr(float duty) { duties[2]=duty; }
void set_duty_rl(float duty) { duties[3]=duty; }

DShotTelemetrySnapshot dshot_telemetry_snapshot() {
    DShotTelemetrySnapshot out{};
    const uint32_t s=save_and_disable_interrupts();
    const uint32_t now=time_us_32();
    for (unsigned i=0; i<kControlled; ++i) {
        const State& st=stats[i];
        auto& m=out.motors[i];
        m.erpm=st.erpm; m.tx=st.tx; m.rx_ok=st.rx_ok;
        m.rx_timeout=st.rx_timeout; m.rx_bad_gcr=st.bad_gcr;
        m.rx_bad_crc=st.bad_crc; m.rx_other=st.other; m.rx_discarded=st.stray;
        m.tx_busy=st.busy;
        m.requested_duty=duties[i]; m.last_command=st.command; m.last_frame=st.frame;
#if DSHOT_BIDIRECTIONAL
        m.fresh=esc_dshot_presence::connected(st.rx_ok,st.good_at,now);
#else
        m.fresh=false;
#endif
    }
    restore_interrupts(s);
    return out;
}
