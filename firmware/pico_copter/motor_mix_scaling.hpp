#ifndef PICO_COPTER_MOTOR_MIX_SCALING_HPP
#define PICO_COPTER_MOTOR_MIX_SCALING_HPP

// Float-only flight-control math. Can be tested on a PC without Pico SDK.
// Does not change the existing PID outputs or roll/pitch/yaw mixer signs.
namespace motor_output {
constexpr float kThrottleReferenceMax = 0.60f; // T_ref/BATTERY_VOLTAGE at full CH3.
constexpr float kCorrectionMixerGain = 0.90f;   // i-26's existing final gain.

struct MotorDuties {
    float fr;
    float fl;
    float rr;
    float rl;
};

// The old code scaled T_ref in volts directly into a 0..1 motor duty,
// causing all motors to saturate before the existing stop threshold.
// Normalize the throttle AFTER the existing stop threshold, so it covers
// the complete 0.10..0.95 duty window over the *usable* CH3 stick travel.
// PID correction voltages use the existing 0.9 gain with a V->duty scale.
inline MotorDuties mix(float t_ref, float p, float q, float r,
                       float battery_voltage, float disable_duty,
                       float min_duty, float max_duty) {
    const float normalized_throttle = t_ref / battery_voltage;
    float travel = (normalized_throttle - disable_duty)
                   / (kThrottleReferenceMax - disable_duty);
    if (travel < 0.0f) travel = 0.0f;
    if (travel > 1.0f) travel = 1.0f;

    const float base = min_duty + (max_duty - min_duty) * travel;
    const float correction_scale = kCorrectionMixerGain / battery_voltage;
    return {
        base + correction_scale * (-p + q - r), // Front Right
        base + correction_scale * ( p + q + r), // Front Left
        base + correction_scale * (-p - q + r), // Rear Right
        base + correction_scale * ( p - q - r)  // Rear Left
    };
}
} // namespace motor_output
#endif
