/*
 * Centralized, optional USB CDC status/debug output.
 *
 * Keep diagnostic printf calls here, never inside the 400 Hz timer ISR,
 * PIO TX/RX, SBUS parsing IRQ, or core 1 angle controller. The CH5 flight
 * recorder's exact export format is intentionally owned by control.cpp.
 */
#include "debug_usb.hpp"
#include "pico_copter.hpp"
#include "pico/stdio_usb.h"
#include "dshot_config.hpp"
#include "esc_serial_uart.hpp"

namespace debug_usb {

void startup_countdown(int seconds_remaining) {
    printf("#Please wait %d[s]\r", seconds_remaining);
}

void startup_complete() {
    printf("\n");
    printf("#DShot CONFIG rate=%lu kbps mode=%s control=400Hz\n",
           static_cast<unsigned long>(dshot::kRateKbps),
           dshot::kBidirectional ? "bidir" : "oneway");
}

void sensor_not_found() {
    printf("Device not found !\n");
}

void poll() {
    // The CH5 RAM flight recorder emits its own data stream. Never mix
    // ordinary status output into a running export, even RC transitions.
    if (Arm_flag == 4 || Logoutputflag != 0 || !stdio_usb_connected())
        return;

    static bool last_reported_failsafe = true;
    static uint32_t last_diagnostic_ms =
        to_ms_since_boot(get_absolute_time());

    const bool fs = rc_failsafe_latched;
    if (kEnableRcStateMessages && fs != last_reported_failsafe) {
        last_reported_failsafe = fs;
        printf("#RC %s (motors %s)\n", fs ? "FAILSAFE" : "GOOD",
               fs ? "INHIBITED" : "DISARMED UNTIL RE-ARM");
    }

    if (!kEnablePeriodicDiagnostics)
        return;

    // Preserve the original 500 ms disarmed / 1000 ms armed cadence.
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    const uint32_t period_ms = Arm_flag == 2 ? 1000u : 500u;
    if (static_cast<uint32_t>(now_ms - last_diagnostic_ms) < period_ms)
        return;
    last_diagnostic_ms = now_ms;

    // Read only the enabled categories: no extra sensor transactions.
    if (kEnableState || kEnablePidMixer) {
        const ControlDebugSnapshot ctrl = control_debug_snapshot();
        if (kEnableState) {
            printf("#DBG STATE t_ms=%lu arm=%u lock=%u rc_latch=%u "
                   "record=%u overg=%u\n",
                   static_cast<unsigned long>(now_ms),
                   static_cast<unsigned int>(ctrl.arm),
                   static_cast<unsigned int>(ctrl.lock_mode),
                   ctrl.failsafe_latched ? 1u : 0u,
                   static_cast<unsigned int>(ctrl.logflag),
                   static_cast<unsigned int>(ctrl.over_g));
        }
        if (kEnablePidMixer) {
            // Existing motor mix/PID output format is unchanged.
            printf("#DBG MIX active=%u T=%.3f P=%.3f Q=%.3f R=%.3f "
                   "FR=%.3f FL=%.3f RR=%.3f RL=%.3f\n",
                   (ctrl.arm == 2 && ctrl.lock_mode == 2
                       && !ctrl.failsafe_latched) ? 1u : 0u,
                   static_cast<double>(ctrl.t_ref),
                   static_cast<double>(ctrl.p_com),
                   static_cast<double>(ctrl.q_com),
                   static_cast<double>(ctrl.r_com),
                   static_cast<double>(ctrl.fr_duty),
                   static_cast<double>(ctrl.fl_duty),
                   static_cast<double>(ctrl.rr_duty),
                   static_cast<double>(ctrl.rl_duty));
        }
    }

    if (kEnableSbus) {
        const RadioDebugSnapshot radio = radio_debug_snapshot();
        // CH1..CH6 are the last accepted frame; check age and healthy.
        printf("#DBG SBUS ch1=%u ch2=%u ch3=%u ch4=%u ch5=%u ch6=%u "
               "healthy=%u age_ms=%ld flags=0x%02x good=%lu fs_events=%lu\n",
               static_cast<unsigned int>(radio.channels[0]),
               static_cast<unsigned int>(radio.channels[1]),
               static_cast<unsigned int>(radio.channels[2]),
               static_cast<unsigned int>(radio.channels[3]),
               static_cast<unsigned int>(radio.channels[4]),
               static_cast<unsigned int>(radio.channels[5]),
               radio.healthy ? 1u : 0u,
               radio.received ? static_cast<long>(radio.age_ms) : -1L,
               static_cast<unsigned int>(radio.flags),
               static_cast<unsigned long>(radio.good_frames),
               static_cast<unsigned long>(radio.failsafe_events));
    }

    if (kEnableImu || kEnableMag) {
        const ImuDebugSnapshot imu = imu_debug_snapshot();
        if (!imu.valid) {
            printf("#DBG IMU valid=0 (waiting for first sensor sample)\n");
        } else {
            const unsigned long age_ms = static_cast<unsigned long>(
                (time_us_32() - imu.sample_time_us) / 1000u);
            if (kEnableImu) {
                // Flight-controller axes; accel m/s^2 and gyro rad/s.
                printf("#DBG IMU ax=%.4f ay=%.4f az=%.4f unit=m/s2 age_ms=%lu\n",
                       static_cast<double>(imu.ax), static_cast<double>(imu.ay),
                       static_cast<double>(imu.az), age_ms);
                printf("#DBG GYRO p=%.5f q=%.5f r=%.5f unit=rad/s age_ms=%lu\n",
                       static_cast<double>(imu.wp), static_cast<double>(imu.wq),
                       static_cast<double>(imu.wr), age_ms);
            }
            if (kEnableMag) {
                // Mx/My/Mz in control.cpp are calibrated AND normalized.
                // These are NOT raw mG field measurements.
                printf("#DBG MAG mx=%.5f my=%.5f mz=%.5f unit=normalized age_ms=%lu\n",
                       static_cast<double>(imu.mx), static_cast<double>(imu.my),
                       static_cast<double>(imu.mz), age_ms);
            }
        }
    }

    if (kEnableAttitude || kEnableEkf) {
        const AttitudeDebugSnapshot att = attitude_debug_snapshot();
        if (!att.valid) {
            printf("#DBG ATT valid=0 (estimator not started)\n");
        } else {
            const unsigned long age_ms = static_cast<unsigned long>(
                (time_us_32() - att.sample_time_us) / 1000u);
            if (kEnableAttitude) {
                // Bias-subtracted angles match CH5 log rows 20..22.
                // Raw angles are also provided to distinguish the reference.
                constexpr double kRadToDeg = 57.29577951308232;
                printf("#DBG ATT roll_deg=%.2f pitch_deg=%.2f yaw_deg=%.2f "
                       "raw_roll_deg=%.2f raw_pitch_deg=%.2f raw_yaw_deg=%.2f age_ms=%lu\n",
                       static_cast<double>(att.roll - att.roll_bias) * kRadToDeg,
                       static_cast<double>(att.pitch - att.pitch_bias) * kRadToDeg,
                       static_cast<double>(att.yaw - att.yaw_bias) * kRadToDeg,
                       static_cast<double>(att.roll) * kRadToDeg,
                       static_cast<double>(att.pitch) * kRadToDeg,
                       static_cast<double>(att.yaw) * kRadToDeg,
                       age_ms);
            }
            if (kEnableEkf) {
                // EKF quaternion uses scalar-first [q0,q1,q2,q3].
                printf("#DBG EKF q0=%.6f q1=%.6f q2=%.6f q3=%.6f "
                       "bias_p=%.6f bias_q=%.6f bias_r=%.6f age_ms=%lu\n",
                       static_cast<double>(att.quaternion[0]),
                       static_cast<double>(att.quaternion[1]),
                       static_cast<double>(att.quaternion[2]),
                       static_cast<double>(att.quaternion[3]),
                       static_cast<double>(att.gyro_bias[0]),
                       static_cast<double>(att.gyro_bias[1]),
                       static_cast<double>(att.gyro_bias[2]),
                       age_ms);
            }
        }
    }

    if (kEnableDShot) {
        const DShotTelemetrySnapshot dshot = dshot_telemetry_snapshot();
        const char* names[4] = {"FR", "FL", "RR", "RL"};
        for (int i = 0; i < 4; ++i) {
            const DShotMotorTelemetry& m = dshot.motors[i];
            const uint32_t success_percent = m.tx
                ? static_cast<uint32_t>((static_cast<uint64_t>(m.rx_ok) * 100u) / m.tx)
                : 0u;
            printf("#DShot%lu %s mode=%s duty=%.3f cmd=%u pkt=0x%04x "
                   "eRPM=%ld valid=%u connected=%u rx=%lu/%lu (%lu%%) "
                   "timeout=%lu gcr=%lu crc=%lu other=%lu stray=%lu busy=%lu\n",
                   static_cast<unsigned long>(dshot::kRateKbps),
                   names[i], dshot::kBidirectional ? "bidir" : "oneway",
                   static_cast<double>(m.requested_duty),
                   static_cast<unsigned int>(m.last_command),
                   static_cast<unsigned int>(m.last_frame),
                   m.fresh ? static_cast<long>(m.erpm) : -1L,
                   m.fresh ? 1u : 0u,
                   m.fresh ? 1u : 0u,
                   static_cast<unsigned long>(m.rx_ok),
                   static_cast<unsigned long>(m.tx),
                   static_cast<unsigned long>(success_percent),
                   static_cast<unsigned long>(m.rx_timeout),
                   static_cast<unsigned long>(m.rx_bad_gcr),
                   static_cast<unsigned long>(m.rx_bad_crc),
                   static_cast<unsigned long>(m.rx_other),
                   static_cast<unsigned long>(m.rx_discarded),
                   static_cast<unsigned long>(m.tx_busy));
        }
    }

    if (kEnableEscSerial) {
        const esc_serial::Snapshot esc = esc_serial::snapshot();
#if !PICO_RP2350
        const DShotTelemetrySnapshot link = dshot_telemetry_snapshot();
#endif
        const char* names[4] = {"FR", "FL", "RR", "RL"};
        double total_current_a = 0.0;
        unsigned fresh_count = 0;
        for (unsigned i = 0; i < 4; ++i) {
            const esc_serial::MotorSnapshot& m = esc.motors[i];
            if (m.fresh) {
                total_current_a += m.reading.current_centiamp / 100.0;
                ++fresh_count;
            }
            // Pico2's ESC presence is validated by requested UART packets,
            // not by the DShot signal wire's optional eRPM reply.
#if PICO_RP2350
            const bool detected = m.fresh;
#else
            const bool detected = link.motors[i].fresh;
#endif
            // Never present stale measurements as valid.
            printf("#ESC_UART %s detected=%u valid=%u temp_c=%ld voltage_v=%.2f "
                   "current_a=%.2f consumed_mah=%ld erpm=%ld "
                   "age_ms=%ld requests=%lu rx=%lu auto_rx=%lu req_rx=%lu "
                   "confirm=%lu timeout=%lu src=%s\n",
                   names[i], detected ? 1u : 0u,
                   m.fresh ? 1u : 0u,
                   m.fresh ? static_cast<long>(m.reading.temperature_c) : -1L,
                   m.fresh ? m.reading.voltage_centivolt / 100.0 : -1.0,
                   m.fresh ? m.reading.current_centiamp / 100.0 : -1.0,
                   m.fresh ? static_cast<long>(m.reading.consumption_mah) : -1L,
                   m.fresh ? static_cast<long>(m.reading.electrical_rpm) : -1L,
                   m.received ? static_cast<long>(m.age_ms) : -1L,
                   static_cast<unsigned long>(m.requests),
                   static_cast<unsigned long>(m.received),
                   static_cast<unsigned long>(m.passive_received),
                   static_cast<unsigned long>(m.requested_received),
                   static_cast<unsigned long>(m.corroborating),
                   static_cast<unsigned long>(m.timeouts),
                   m.received ? (m.last_was_passive ? "auto" : "request")
                              : "none");
        }
        printf("#ESC_UART SUM current_a=%.2f fresh=%u/4 uart_enabled=%u "
               "mode=%s mask=0x%02x bytes=%lu plausible=%lu crc_windows=%lu "
               "range_reject=%lu unassigned=%lu gaps=%lu no_esc_bytes=%lu\n",
               total_current_a, fresh_count, esc.enabled ? 1u : 0u,
#if PICO_RP2350
               "request",
#else
               esc.passive_active ? "stream" : "poll_or_wait",
#endif
               static_cast<unsigned int>(esc.connected_mask),
               static_cast<unsigned long>(esc.bytes_total),
               static_cast<unsigned long>(esc.plausible_frames),
               static_cast<unsigned long>(esc.bad_crc_windows),
               static_cast<unsigned long>(esc.bad_range_frames),
               static_cast<unsigned long>(esc.unassigned_frames),
               static_cast<unsigned long>(esc.gaps),
               static_cast<unsigned long>(esc.stray_bytes));
    }
}

} // namespace debug_usb
