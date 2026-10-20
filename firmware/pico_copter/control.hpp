#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <stdio.h>
#include "pico_copter.hpp"
#include "pico/stdlib.h"
#include "sensor.hpp"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <Eigen/Dense>
#include "ekf.hpp"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Matrix;
using Eigen::PartialPivLU;
using namespace Eigen;

#define BATTERY_VOLTAGE (11.1)


//グローバル関数の宣言
void loop_400Hz(void);
void control_init();
void rate_control(void);
void angle_control(void);
void gyro_calibration(void);
void variable_init(void);
void log_output(void);
void flight_log_init(void);

// Read-only snapshot for USB diagnostics. These are most recently computed
// mixer values, not guaranteed active when the motors are disarmed/stopped.
struct ControlDebugSnapshot {
    float t_ref, p_com, q_com, r_com;
    float fr_duty, fl_duty, rr_duty, rl_duty;
    uint8_t arm, lock_mode, logflag, over_g;
    bool failsafe_latched;
};
ControlDebugSnapshot control_debug_snapshot(void);

// Debug snapshots contain COPIES only: no direct reading of the live
// IMU globals or Eigen matrices from the USB printing thread.
// IMU values are expressed in the flight controller's body axes:
// acceleration in m/s^2, gyro rates in rad/s, magnetic field normalized.
struct ImuDebugSnapshot {
    float ax, ay, az;
    float wp, wq, wr;
    float mx, my, mz;
    uint32_t sample_time_us;
    bool valid;
};
ImuDebugSnapshot imu_debug_snapshot(void);

// Published as one coherent snapshot by the 100 Hz core-1 estimator.
// Angles and biases in radians, quaternion scalar-first, EKF gyro bias
// in estimator-native units. USB converts angles to degrees for display.
struct AttitudeDebugSnapshot {
    float roll, pitch, yaw;
    float roll_bias, pitch_bias, yaw_bias;
    float quaternion[4];
    float gyro_bias[3];
    uint32_t sample_time_us;
    bool valid;
};
AttitudeDebugSnapshot attitude_debug_snapshot(void);

//グローバル変数
extern volatile uint8_t LockMode;
extern volatile bool rc_failsafe_latched;
extern volatile uint8_t Logoutputflag;

class PID
{
  private:
    float m_kp;
    float m_ti;
    float m_td;
    float m_filter_time_constant;
    float m_err,m_err2,m_err3;
    float m_h;
  public:
    float m_filter_output;
    float m_integral;
    PID();
    void set_parameter(
        float kp, 
        float ti, 
        float td,
        float filter_time_constant, 
        float h);
    void reset(void);
    void i_reset(void);
    void printGain(void);
    float filter(float x);
    float update(float err);
};

class Filter
{
  private:
    float m_state;
    float m_T;
    float m_h;
  public:
    float m_out;
    Filter();
    void set_parameter(
        float T,
        float h);
    void reset(void);
    float update(float u);
};


#endif
