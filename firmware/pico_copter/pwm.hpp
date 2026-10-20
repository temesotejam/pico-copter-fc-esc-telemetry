/*
 * Motor DShot GPIO mapping and optional bidirectional telemetry diagnostics.
 * Servo output and the 400 Hz scheduling timer still use hardware PWM.
 */
#ifndef PWM_HPP
#define PWM_HPP
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico_copter.hpp"
#include "control.hpp"

//#define DUTYMIN 1250
//#define DUTYMAX 2500
#define DUTYMIN 1375
#define DUTYMAX 2375

// Motor signals use selected PIO DShot150/300/600/1200; servo remains PWM.
// PWM_MAINLOOP_PIN identifies the internal PWM slice 7 timebase only.
// Keep these GPIO numbers identical to the current I-26 control board.
constexpr uint PWM_PIN_RL    = 2;
constexpr uint PWM_PIN_RR    = 3;
constexpr uint PWM_PIN_FL    = 14;
constexpr uint PWM_PIN_FR    = 15;
constexpr uint PWM_PIN_SERVO = 6;

// The main 400 Hz control loop is driven by the PWM wrap IRQ of this output.
constexpr uint PWM_MAINLOOP_PIN = PWM_PIN_FL;

//グローバル変数
extern uint8_t ESC_calib;

//関数の宣言
void pwm_init();
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);

// In one-way builds eRPM is always unavailable (fresh=false).
// In bidirectional builds eRPM is diagnostic only: not used by PID/EKF/mixing.
// Motor array order: FR, FL, RR, RL. fresh=1 with erpm=0 means a valid
// stopped-motor response; fresh=0 means telemetry is unavailable/stale.
struct DShotMotorTelemetry {
    uint32_t erpm;
    uint32_t tx;
    uint32_t rx_ok;
    uint32_t rx_timeout;
    uint32_t rx_bad_gcr;
    uint32_t rx_bad_crc;
    uint32_t rx_other;
    uint32_t rx_discarded; // Late/orphan telemetry words, not used in control.
    uint32_t tx_busy;
    bool fresh;
    // Diagnostics: value requested by the controller and the last packet
    // actually queued into this channel's PIO TX FIFO.
    float requested_duty;
    uint16_t last_command; // 0=STOP; 48..2047=running
    uint16_t last_frame;   // logical 16-bit selected-mode DShot packet
};
struct DShotTelemetrySnapshot {
    DShotMotorTelemetry motors[4];
};
DShotTelemetrySnapshot dshot_telemetry_snapshot(void);


#endif
