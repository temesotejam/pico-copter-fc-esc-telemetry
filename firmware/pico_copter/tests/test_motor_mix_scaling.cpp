#include "motor_mix_scaling.hpp"
#include "dshot_protocol.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

// Full signal path for motor baseline: CH3 -> T_ref -> mixer -> duty clamp
// -> DShot (same stop threshold as control.cpp).
constexpr float kVoltage = 11.1f;
constexpr float kDisable = 0.10f;
constexpr float kMinDuty = 0.10f;
constexpr float kMaxDuty = 0.95f;
constexpr int kCh3Min = 368;
constexpr int kCh3Max = 1680;

float clamp_duty(float value) {
    if (value < kMinDuty) value = kMinDuty;
    if (value > kMaxDuty) value = kMaxDuty;
    return value;
}

uint16_t command_for_ch3(int ch3) {
    const float t_ref = motor_output::kThrottleReferenceMax * kVoltage *
        static_cast<float>(ch3 - kCh3Min) / (kCh3Max - kCh3Min);
    if (t_ref / kVoltage < kDisable) return dshot::kStop;
    const auto duty = motor_output::mix(
        t_ref, 0.0f, 0.0f, 0.0f, kVoltage, kDisable, kMinDuty, kMaxDuty);
    assert(std::fabs(duty.fr - duty.fl) < 0.00001f);
    assert(std::fabs(duty.fr - duty.rr) < 0.00001f);
    assert(std::fabs(duty.fr - duty.rl) < 0.00001f);
    return dshot::throttle_from_legacy_duty(clamp_duty(duty.fr));
}

int main() {
    assert(command_for_ch3(368) == 0);
    assert(command_for_ch3(500) == 0);
    assert(command_for_ch3(586) == 0);
    const uint16_t just_enabled = command_for_ch3(587);
    assert(just_enabled >= 48 && just_enabled < 60);
    assert(command_for_ch3(614) > just_enabled);
    assert(command_for_ch3(614) < 2047);
    assert(command_for_ch3(913) > command_for_ch3(614));
    assert(command_for_ch3(1024) > command_for_ch3(913));
    assert(command_for_ch3(1024) < 2047);
    assert(command_for_ch3(1395) > command_for_ch3(1024));
    assert(command_for_ch3(1680) == 2047);

    // This catches the originally observed STOP -> MAX-only regression.
    uint16_t previous = 0;
    unsigned distinct_running = 0;
    for (int ch = kCh3Min; ch <= kCh3Max; ++ch) {
        const uint16_t command = command_for_ch3(ch);
        assert(command >= previous);
        if (command >= dshot::kMinimumThrottle && command != previous) {
            ++distinct_running;
        }
        previous = command;
    }
    assert(distinct_running > 600);

    // Opposing motor signs are the original I-26 mixer convention,
    // with Pico Copter's specified yaw sign. No PID gains are changed.
    const auto mixed = motor_output::mix(
        0.3f * kVoltage, 0.3f, -0.15f, 0.05f,
        kVoltage, kDisable, kMinDuty, kMaxDuty);
    const float correction = motor_output::kCorrectionMixerGain / kVoltage;
    const float base = kMinDuty +
        (kMaxDuty - kMinDuty) * ((0.3f - kDisable) /
        (motor_output::kThrottleReferenceMax - kDisable));
    assert(std::fabs(mixed.fr - (base + correction * (-0.3f - 0.15f - 0.05f))) < 0.00001f);
    assert(std::fabs(mixed.fl - (base + correction * (0.3f - 0.15f + 0.05f))) < 0.00001f);
    assert(std::fabs(mixed.rr - (base + correction * (-0.3f + 0.15f + 0.05f))) < 0.00001f);
    assert(std::fabs(mixed.rl - (base + correction * (0.3f + 0.15f - 0.05f))) < 0.00001f);

    std::puts("CH3 usable travel -> smooth full-range DShot and mixer signs passed");
    return 0;
}
