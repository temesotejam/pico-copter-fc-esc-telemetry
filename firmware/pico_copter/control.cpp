//control.cpp
//
//pull request


#include "control.hpp"
#include "sbus_parser.hpp"
#include "flight_log_ring.hpp"
#include "log_capture_gate.hpp"
#include "motor_mix_scaling.hpp"
#include "pico/sync.h"
#include "pico/mutex.h"

//Sensor data
float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
float Acc_norm=0.0;

//Times
float Elapsed_time=0.0;
uint32_t S_time=0,E_time=0,D_time=0,S_time2=0,E_time2=0,D_time2=0;

//Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t BiasCounter=0;
uint16_t LedBlinkCounter=0;

//Control 
float FR_duty, FL_duty, RR_duty, RL_duty;
float P_com, Q_com, R_com;
float T_ref;
float Pbias=0.0,Qbias=0.0,Rbias=0.0;
float Phi_bias=0.0,Theta_bias=0.0,Psi_bias=0.0;  
float Phi,Theta,Psi;
float Phi_ref=0.0,Theta_ref=0.0,Psi_ref=0.0;
float Elevator_center=0.0, Aileron_center=0.0, Rudder_center=0.0;
float Pref=0.0,Qref=0.0,Rref=0.0;
const float Phi_trim   = 0.01;
const float Theta_trim = 0.02;
const float Psi_trim   = 0.0;

//Extended Kalman filter 
Matrix<float, 7 ,1> Xp = MatrixXf::Zero(7,1);
Matrix<float, 7 ,1> Xe = MatrixXf::Zero(7,1);
Matrix<float, 6 ,1> Z = MatrixXf::Zero(6,1);
Matrix<float, 3, 1> Omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 3, 1> Oomega;
Matrix<float, 7, 7> P;
Matrix<float, 6, 6> Q;// = MatrixXf::Identity(3, 3)*0.1;
Matrix<float, 6, 6> R;// = MatrixXf::Identity(6, 6)*0.0001;
Matrix<float, 7 ,6> G;
Matrix<float, 3 ,1> Beta;

// CH5-gated, nominal 100 Hz RAM recorder: retain the newest 10 seconds.
// 1000 rows x 38 floats = 152,000 bytes of static RAM.
constexpr uint32_t DATANUM=38;
constexpr uint32_t LOG_ROWS=1000;
static_assert(LOG_ROWS*DATANUM*sizeof(float)==152000, "Unexpected log RAM size");
FlightLogRing<LOG_ROWS, DATANUM> flight_log;
Ch5CaptureGate log_capture_gate;
critical_section_t flight_log_guard;
volatile uint8_t Logflag=0;
volatile uint8_t Logoutputflag=0;
uint32_t log_output_position=0;
uint32_t log_output_count=0;
uint64_t log_output_first_sample=0;
bool log_output_started=false;

// Only the USB diagnostic snapshot is synchronized here. No USB printing
// or mutex acquisition is introduced in the 400 Hz rate controller.
// Core 1 publishes a coherent estimator snapshot under this short mutex;
// the USB thread on core 0 reads the copy, not the live Eigen state.
namespace {
mutex_t attitude_debug_mutex;
AttitudeDebugSnapshot published_attitude{};
uint32_t imu_debug_sample_count = 0;
uint32_t imu_debug_sample_time_us = 0;
}

//State Machine
volatile uint8_t LockMode=0;
namespace {
sbus::ReArmLatch rc_safety;
uint32_t last_seen_failsafe_event = 0;
uint8_t arm_gesture_counter = 0;
uint8_t arm_gesture_state = 0;
}
volatile bool rc_failsafe_latched = true;
float Disable_duty =0.10;
float Flight_duty  =0.18;//0.2/////////////////
uint8_t OverG_flag = 0;

//PID object and etc.
Filter acc_filter;
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;

void loop_400Hz(void);
void rate_control(void);
void sensor_read(void);
void angle_control(void);
void kalman_filter(void);
void logging(void);
void flight_log_freeze(void);
void flight_log_arm(void);
void motor_stop(void);
uint8_t lock_com(void);
bool arm_gesture_held(void);
void reset_arm_gesture(void);
uint8_t logdata_out_com(void);
void printPQR(void);

#define AVERAGE 2000
#define KALMANWAIT 6000

//Main loop
//This function is called from PWM Intrupt on 400Hz.
void loop_400Hz(void)
{
  static uint8_t led=1;
  S_time=time_us_32();

  // Check RC health every 2.5 ms, before any state/actuator decisions.
  // Communication recovery never returns straight into the armed state.
  const RadioStatus link = radio_update();
  const bool new_failsafe_event = link.failsafe_count != last_seen_failsafe_event;
  last_seen_failsafe_event = link.failsafe_count;
  const bool rc_allowed = rc_safety.update(
      link.healthy, new_failsafe_event, Chdata[2] <= CH3MIN + 80,
      !arm_gesture_held());
  rc_failsafe_latched = !rc_allowed;
  // Keep gyro-bias and Kalman startup independent of RC link availability.
  // A missing/unsafe SBUS link must prevent arming, not prevent calibration.
  // During Arm_flag=0/1 only STOP motor commands are emitted below.
  if (!rc_allowed && Arm_flag != 0 && Arm_flag != 1) {
    motor_stop(); // DShot 0 (STOP), not a physical motor power disconnect.
    if (Arm_flag == 2 || Arm_flag == 4) {
      flight_log_freeze();
      Arm_flag = 3;
      Logoutputflag = 0;
    }
    LockMode = 0;
    reset_arm_gesture();
    Pref = Qref = Rref = 0.0f;
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    phi_pid.reset();
    theta_pid.reset();
    psi_pid.reset();
    // After calibration the usual disarmed LED blink should remain
    // visible even with no SBUS signal. This does NOT release failsafe.
    if (Arm_flag == 3) {
      if (LedBlinkCounter < 10) {
        gpio_put(LED_PIN, 1);
        ++LedBlinkCounter;
      } else if (LedBlinkCounter < 100) {
        gpio_put(LED_PIN, 0);
        ++LedBlinkCounter;
      } else {
        LedBlinkCounter = 0;
      }
    } else {
      gpio_put(LED_PIN, 1);
    }
    return;
  }

  if (Arm_flag == 0 || Arm_flag == 1) {
    // LED off while the gyro bias and Kalman startup are still running.
    gpio_put(LED_PIN, 0);
  }

  if (Arm_flag==0)
  {
      // Continue DShot 0 (STOP) while the sensor calibration runs.
      // Keep the motor-control loop timing unchanged.
      motor_stop();
      Elevator_center = 0.0;
      Aileron_center = 0.0;
      Rudder_center = 0.0;
      Pbias = 0.0;
      Qbias = 0.0;
      Rbias = 0.0;
      Phi_bias = 0.0;
      Theta_bias = 0.0;
      Psi_bias = 0.0;
      return;
  }
  else if (Arm_flag==1)
  {
    motor_stop();
    //Gyro Bias Estimate
    if (BiasCounter < AVERAGE)
    {
      //Sensor Read
      sensor_read();
      Aileron_center  += Chdata[3];
      Elevator_center += Chdata[1];
      Rudder_center   += Chdata[0];
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      Mx_ave += Mx;
      My_ave += My;
      Mz_ave += Mz;
      BiasCounter++;
      return;
    }
    else if(BiasCounter<KALMANWAIT)
    {
      //Sensor Read
      sensor_read();
      if(BiasCounter == AVERAGE)
      {
        Elevator_center = Elevator_center/AVERAGE;
        Aileron_center  = Aileron_center/AVERAGE;
        Rudder_center   = Rudder_center/AVERAGE;
        Pbias = Pbias/AVERAGE;
        Qbias = Qbias/AVERAGE;
        Rbias = Rbias/AVERAGE;
        Mx_ave = Mx_ave/AVERAGE;
        My_ave = My_ave/AVERAGE;
        Mz_ave = Mz_ave/AVERAGE;

        Xe(4,0) = Pbias;
        Xe(5,0) = Qbias;
        Xe(6,0) = Rbias;
        Xp(4,0) = Pbias;
        Xp(5,0) = Qbias;
        Xp(6,0) = Rbias;
        MN = Mx_ave;
        ME = My_ave;
        MD = Mz_ave;
      }
      
      AngleControlCounter++;
      if(AngleControlCounter==4)
      {
        AngleControlCounter=0;
        sem_release(&sem);
      
      }
      Phi_bias   += Phi;
      Theta_bias += Theta;
      Psi_bias   += Psi;
      BiasCounter++;
      return;
    }
    else
    {
      Arm_flag = 3;
      Phi_bias   = Phi_bias/KALMANWAIT;
      Theta_bias = Theta_bias/KALMANWAIT;
      Psi_bias   = Psi_bias/KALMANWAIT;
      return;
    }
  }
  else if( Arm_flag==2)
  {
    if(LockMode==2)
    {
      if(lock_com()==1)
      {
        LockMode=3;//Disenable Flight
        flight_log_freeze();
        motor_stop();
        led=0;
        gpio_put(LED_PIN,led);
        return;
      }
      //Goto Flight
    }
    else if(LockMode==3)
    {
      if(lock_com()==0){
        LockMode=0;
        Arm_flag=3;
      }
      return;
    }
    //LED Blink
    gpio_put(LED_PIN, led);
    if(Logflag==1&&LedBlinkCounter<100){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
   
    //Rate Control (400Hz)
    rate_control();
   
    if(AngleControlCounter==4)
    {
      AngleControlCounter=0;
      //Angle Control (100Hz)
      sem_release(&sem);
    }
    AngleControlCounter++;
  }
  else if(Arm_flag==3)
  {
    motor_stop();
    OverG_flag = 0;
    if(LedBlinkCounter<10){
      gpio_put(LED_PIN, 1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      gpio_put(LED_PIN, 0);
      LedBlinkCounter++;
    }
    else LedBlinkCounter=0;
    
    //Get Stick Center 
    Aileron_center  = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center   = Chdata[0];
  
    if(LockMode==0)
    {
      if( lock_com()==1)
      {
        LockMode=1;
        return;
      }
      //Wait  output log
    }
    else if(LockMode==1)
    {
      if(lock_com()==0)
      {
        flight_log_arm(); // Require a CH5 OFF-then-ON transition after arming.
        LockMode=2;//Enable Flight
        Arm_flag=2;
      }
      return;
    }

    if(logdata_out_com()==1)
    {
      flight_log_freeze();
      log_output_started=false;
      Arm_flag=4;
      return;
    }
  }
  else if(Arm_flag==4)
  {
    motor_stop();
    Logoutputflag=1;
    //LED Blink
    gpio_put(LED_PIN, led);
    if(LedBlinkCounter<400){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
  }
  E_time=time_us_32();
  D_time=E_time-S_time;
}

void control_init(void)
{
  acc_filter.set_parameter(0.005, 0.0025);
  //Rate control
  // p_pid.set_parameter( 2.0, 0.145, 0.028, 0.015, 0.0025);//3.4
  // q_pid.set_parameter( 2.1, 0.125, 0.028, 0.015, 0.0025);//3.8
  // r_pid.set_parameter(12.0, 0.5, 0.008, 0.015, 0.0025);//9.4
  // //Angle control
  // phi_pid.set_parameter  ( 5.5, 9.5, 0.025, 0.018, 0.01);//6.0
  // theta_pid.set_parameter( 5.5, 9.5, 0.025, 0.018, 0.01);//6.0
  // psi_pid.set_parameter  ( 0.0, 10.0, 0.010, 0.03, 0.01);

  
  p_pid.set_parameter(1.05, 1000000.0, 0.0, 0.125, 0.0025); //ikaring(2.2, 5, 0.01) itocopter(2.5, 100, 0.009)
  q_pid.set_parameter(1.05, 1000000.0, 0.0, 0.125, 0.0025); //ikaring(1.5, 1, 0.01) itocopter(2.5, 100, 0.009)
  r_pid.set_parameter(0.21, 10000.0, 0.0, 0.125, 0.0025);  //ikaring(3.1, 1, 0.01) itocopter(3.5, 10, 0.009)
  // Angle control
  phi_pid.set_parameter(2.1, 1000.0, 0.013, 0.125, 0.01);   // 6.0  8.0,20,0.007      //1
  theta_pid.set_parameter(2.1, 1000.0, 0.013, 0.125, 0.01); // 6.0  8.0,20,0.007 // 1.6      
  psi_pid.set_parameter(0, 1000, 0, 0, 0.01);     //0 1000 0.01
  
  //Rate control
  //p_pid.set_parameter(3.3656, 0.1, 0.0112, 0.01, 0.0025);
  //q_pid.set_parameter(3.8042, 0.1, 0.0111, 0.01, 0.0025);
  //r_pid.set_parameter(9.4341, 0.11, 0.0056, 0.01, 0.0025);
  //Angle control
  //phi_pid.set_parameter  ( 9.0   , 0.07, 0.0352,  0.01, 0.01);
  //theta_pid.set_parameter( 8.5583, 0.1 , 0.0552,  0.01, 0.01);
  //psi_pid.set_parameter  ( 9.0256, 0.11, 0.0034,  0.01, 0.01);
}

bool arm_gesture_held(void)
{
  return Chdata[2] < CH3MIN+80
      && Chdata[0] > CH1MAX-80
      && Chdata[3] < CH4MIN+80
      && Chdata[1] > CH2MAX-80;
}

void reset_arm_gesture(void)
{
  arm_gesture_counter = 0;
  arm_gesture_state = 0;
}

uint8_t lock_com(void)
{
  if (arm_gesture_held())
  {
    if (arm_gesture_counter < 51) ++arm_gesture_counter;
    if (arm_gesture_counter > 50)
    {
      arm_gesture_counter = 50;
      arm_gesture_state = 1;
    }
  }
  else
  {
    reset_arm_gesture();
  }
  return arm_gesture_state;
}

uint8_t logdata_out_com(void)
{
  // Trigger once per continuous stick gesture, not repeatedly while held.
  static uint8_t hold_ticks=0;
  static bool ready_for_command=true;
  const bool held=Chdata[4]<(CH5MAX+CH5MIN)*0.5
      && Chdata[2]<CH3MIN+80
      && Chdata[0]<CH1MIN+80
      && Chdata[3]>CH4MAX-80
      && Chdata[1]>CH2MAX-80;
  if(!held)
  {
    hold_ticks=0;
    ready_for_command=true;
    return 0;
  }
  if(!ready_for_command) return 0;
  if(hold_ticks<51) ++hold_ticks;
  if(hold_ticks>=51)
  {
    hold_ticks=0;
    ready_for_command=false;
    return 1;
  }
  return 0;
}

// The IMU values are written on core 0 by the 400 Hz timer IRQ. Copy
// all nine values with that IRQ briefly disabled, matching the existing
// motor-control snapshot approach. Never take a blocking multicore lock
// from inside the rate-control interrupt.
ImuDebugSnapshot imu_debug_snapshot(void)
{
  ImuDebugSnapshot snapshot{};
  const uint32_t saved = save_and_disable_interrupts();
  snapshot.ax = Ax; snapshot.ay = Ay; snapshot.az = Az;
  snapshot.wp = Wp; snapshot.wq = Wq; snapshot.wr = Wr;
  snapshot.mx = Mx; snapshot.my = My; snapshot.mz = Mz;
  snapshot.sample_time_us = imu_debug_sample_time_us;
  snapshot.valid = imu_debug_sample_count != 0;
  restore_interrupts(saved);
  return snapshot;
}

AttitudeDebugSnapshot attitude_debug_snapshot(void)
{
  // Estimator runs on core 1; disabling core-0 IRQs does NOT freeze it.
  // Use a dedicated, short mutex only for publishing/copying the snapshot.
  mutex_enter_blocking(&attitude_debug_mutex);
  const AttitudeDebugSnapshot result = published_attitude;
  mutex_exit(&attitude_debug_mutex);
  return result;
}

ControlDebugSnapshot control_debug_snapshot(void)
{
  ControlDebugSnapshot snapshot{};
  // Snapshot the core-0 motor mixer variables without allowing the 400-Hz
  // PWM IRQ to update them halfway through this diagnostic read.
  const uint32_t saved = save_and_disable_interrupts();
  snapshot.t_ref = T_ref;
  snapshot.p_com = P_com;
  snapshot.q_com = Q_com;
  snapshot.r_com = R_com;
  snapshot.fr_duty = FR_duty;
  snapshot.fl_duty = FL_duty;
  snapshot.rr_duty = RR_duty;
  snapshot.rl_duty = RL_duty;
  snapshot.arm = Arm_flag;
  snapshot.lock_mode = LockMode;
  snapshot.logflag = Logflag;
  snapshot.over_g = OverG_flag;
  snapshot.failsafe_latched = rc_failsafe_latched;
  restore_interrupts(saved);
  return snapshot;
}

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref;
  float p_err, q_err, r_err;

  //Read Sensor Value
  sensor_read();

  //Get Bias
  //Pbias = Xe(4, 0);
  //Qbias = Xe(5, 0);
  //Rbias = Xe(6, 0);

  //Control angle velocity
  p_rate = Wp - Pbias;
  q_rate = Wq - Qbias;
  r_rate = Wr - Rbias;

  //Get reference
  p_ref = Pref;
  q_ref = Qref;
  r_ref = Rref;
  T_ref = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);

  //Error
  p_err = p_ref - p_rate;
  q_err = q_ref - q_rate;
  r_err = r_ref - r_rate;

  //PID
  P_com = p_pid.update(p_err);
  Q_com = q_pid.update(q_err);
  R_com = r_pid.update(r_err);

  // Motor duty scaling: the old direct (T_ref + corrections)*0.9
  // saturated BEFORE the motor-enable throttle threshold. Normalize the
  // usable CH3 range to [Disable_duty .. 0.95] and scale PID corrections by
  // BATTERY_VOLTAGE. PID gains, axis signs, yaw convention and motor STOP
  // threshold below are unchanged.
  const float minimum_duty = Disable_duty;
  const float maximum_duty = 0.95f;
  const motor_output::MotorDuties mixed = motor_output::mix(
      T_ref, P_com, Q_com, R_com,
      BATTERY_VOLTAGE, Disable_duty, minimum_duty, maximum_duty);
  FR_duty = mixed.fr;
  FL_duty = mixed.fl;
  RR_duty = mixed.rr;
  RL_duty = mixed.rl;

  if (FR_duty < minimum_duty) FR_duty = minimum_duty;
  if (FR_duty > maximum_duty) FR_duty = maximum_duty;

  if (FL_duty < minimum_duty) FL_duty = minimum_duty;
  if (FL_duty > maximum_duty) FL_duty = maximum_duty;

  if (RR_duty < minimum_duty) RR_duty = minimum_duty;
  if (RR_duty > maximum_duty) RR_duty = maximum_duty;

  if (RL_duty < minimum_duty) RL_duty = minimum_duty;
  if (RL_duty > maximum_duty) RL_duty = maximum_duty;

  //Duty set
  if(T_ref/BATTERY_VOLTAGE < Disable_duty)
  {
    motor_stop();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    Pref=0.0;
    Qref=0.0;
    Rref=0.0;
    Aileron_center  = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center   = Chdata[0];
    Phi_bias   = Phi;
    Theta_bias = Theta;
    Psi_bias   = Psi;
  }
  else
  {
    if (OverG_flag==0){
      set_duty_fr(FR_duty);
      set_duty_fl(FL_duty);
      set_duty_rr(RR_duty);
      set_duty_rl(RL_duty);
    }
    else motor_stop();
  }

  // Human-readable USB diagnostics are emitted from debug_usb.cpp only.
  // The CH5 flight recorder is independent of the USB status stream.
}

void angle_control(void)
{
  float phi_err,theta_err,psi_err;
  float q0,q1,q2,q3;
  float e23,e33,e13,e11,e12;
  while(1)
  {
    sem_acquire_blocking(&sem);
    sem_reset(&sem, 0);
    S_time2=time_us_32();
    kalman_filter();
    q0 = Xe(0,0);
    q1 = Xe(1,0);
    q2 = Xe(2,0);
    q3 = Xe(3,0);
    e11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    e12 = 2*(q1*q2 + q0*q3);
    e13 = 2*(q1*q3 - q0*q2);
    e23 = 2*(q2*q3 + q0*q1);
    e33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    Phi = atan2(e23, e33);
    Theta = atan2(-e13, sqrt(e23*e23+e33*e33));
    Psi = atan2(e12,e11);

    //Get angle ref 
    Phi_ref   = Phi_trim   + 0.3 *M_PI*(float)(Chdata[3] - (CH4MAX+CH4MIN)*0.5)*2/(CH4MAX-CH4MIN);
    Theta_ref = Theta_trim + 0.3 *M_PI*(float)(Chdata[1] - (CH2MAX+CH2MIN)*0.5)*2/(CH2MAX-CH2MIN);
    Psi_ref   = Psi_trim   + 0.8 *M_PI*(float)(Chdata[0] - (CH1MAX+CH1MIN)*0.5)*2/(CH1MAX-CH1MIN);

    //Error
    phi_err   = Phi_ref   - (Phi   - Phi_bias);
    theta_err = Theta_ref - (Theta - Theta_bias);
    psi_err   = Psi_ref   - (Psi   - Psi_bias);
    
    //PID Control
    if (T_ref/BATTERY_VOLTAGE < Flight_duty)
    {
      Pref=0.0;
      Qref=0.0;
      Rref=0.0;
      phi_pid.reset();
      theta_pid.reset();
      psi_pid.reset();
      Aileron_center  = Chdata[3];
      Elevator_center = Chdata[1];
      Rudder_center   = Chdata[0];
      /////////////////////////////////////
      Phi_bias   = Phi;
      Theta_bias = Theta;
      Psi_bias   = Psi;
      /////////////////////////////////////
    }
    else
    {
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
      Rref = Psi_ref;//psi_pid.update(psi_err);//Yawは角度制御しない
    }

    //Logging
    logging();

    // Publish a consistent debug-only copy after the 100 Hz estimator
    // update. USB formatting happens later on core 0's main thread.
    // This does not change the EKF, attitude controller or CH5 capture.
    AttitudeDebugSnapshot snapshot{};
    snapshot.roll = Phi;
    snapshot.pitch = Theta;
    snapshot.yaw = Psi;
    snapshot.roll_bias = Phi_bias;
    snapshot.pitch_bias = Theta_bias;
    snapshot.yaw_bias = Psi_bias;
    snapshot.quaternion[0] = Xe(0,0);
    snapshot.quaternion[1] = Xe(1,0);
    snapshot.quaternion[2] = Xe(2,0);
    snapshot.quaternion[3] = Xe(3,0);
    snapshot.gyro_bias[0] = Xe(4,0);
    snapshot.gyro_bias[1] = Xe(5,0);
    snapshot.gyro_bias[2] = Xe(6,0);
    snapshot.sample_time_us = S_time2;
    snapshot.valid = true;
    mutex_enter_blocking(&attitude_debug_mutex);
    published_attitude = snapshot;
    mutex_exit(&attitude_debug_mutex);

    E_time2=time_us_32();
    D_time2=E_time2-S_time2;

  }
}

void flight_log_init(void)
{
  critical_section_init(&flight_log_guard);
  critical_section_enter_blocking(&flight_log_guard);
  flight_log.reset();
  critical_section_exit(&flight_log_guard);
}

void flight_log_freeze(void)
{
  critical_section_enter_blocking(&flight_log_guard);
  log_capture_gate.stop();
  flight_log.freeze();
  critical_section_exit(&flight_log_guard);
  Logflag=0;
}

// Arming does not erase the most recent capture or begin recording.
// The first CH5 OFF->ON action during this arm starts a new log session.
void flight_log_arm(void)
{
  critical_section_enter_blocking(&flight_log_guard);
  log_capture_gate.on_arm();
  critical_section_exit(&flight_log_guard);
  Logflag=0;
}

void logging(void)
{
  // The attitude loop still runs at 100 Hz, but constructing/storing the
  // 38-field row happens ONLY while armed AND CH5 is in record position.
  if(Arm_flag!=2 || LockMode!=2) return;

  const bool ch5_on=Chdata[4]>(CH5MAX+CH5MIN)*0.5;
  bool capture=false;
  critical_section_enter_blocking(&flight_log_guard);
  if(Arm_flag==2 && LockMode==2)
  {
    const Ch5CaptureGate::Action action=log_capture_gate.update(ch5_on);
    if(action==Ch5CaptureGate::Action::Start)
    {
      // A new CH5 activation deliberately replaces the previous capture.
      flight_log.reset(); // Constant-time metadata reset: no large memset.
      capture=true;
    }
    else if(action==Ch5CaptureGate::Action::Capture)
    {
      capture=true;
    }
    else if(action==Ch5CaptureGate::Action::Stop)
    {
      // CH5 OFF preserves the newest 10 seconds for later USB export.
      flight_log.freeze();
    }
  }
  critical_section_exit(&flight_log_guard);
  if(!capture)
  {
    if(!ch5_on) Logflag=0;
    return;
  }

  float row[DATANUM];
  row[0] = Xe(0,0);                  //1
  row[1] = Xe(1,0);                  //2
  row[2] = Xe(2,0);                  //3
  row[3] = Xe(3,0);                  //4
  row[4] = Xe(4,0);                  //5
  row[5] = Xe(5,0);                  //6
  row[6] = Xe(6,0);                  //7
  row[7] = Wp;//-Pbias;              //8
  row[8] = Wq;//-Qbias;              //9
  row[9] = Wr;//-Rbias;              //10
  row[10] = Ax;                       //11
  row[11] = Ay;                       //12
  row[12] = Az;                       //13
  row[13] = Mx;                       //14
  row[14] = My;                       //15
  row[15] = Mz;                       //16
  row[16] = Pref;                     //17
  row[17] = Qref;                     //18
  row[18] = Rref;                     //19
  row[19] = Phi-Phi_bias;             //20
  row[20] = Theta-Theta_bias;         //21
  row[21] = Psi-Psi_bias;             //22
  row[22] = Phi_ref;                  //23
  row[23] = Theta_ref;                //24
  row[24] = Psi_ref;                  //25
  row[25] = P_com;                    //26
  row[26] = Q_com;                    //27
  row[27] = R_com;                    //28
  row[28] = p_pid.m_integral;//m_filter_output;    //29
  row[29] = q_pid.m_integral;//m_filter_output;    //30
  row[30] = r_pid.m_integral;//m_filter_output;    //31
  row[31] = phi_pid.m_integral;//m_filter_output;  //32
  row[32] = theta_pid.m_integral;//m_filter_output;//33
  row[33] = Pbias;                    //34
  row[34] = Qbias;                    //35
  row[35] = Rbias;                    //36
  row[36] = T_ref;                    //37
  row[37] = Acc_norm;                 //38

  // Publish a complete 38-field sample atomically with respect to the
  // core-0 disarm/freeze and USB export operations.
  critical_section_enter_blocking(&flight_log_guard);
  const bool saved=Arm_flag==2 && LockMode==2 && log_capture_gate.recording()
      && Chdata[4]>(CH5MAX+CH5MIN)*0.5 && flight_log.append(row);
  critical_section_exit(&flight_log_guard);
  if(saved) Logflag=1;
}

void log_output(void)
{
  if(!log_output_started)
  {
    critical_section_enter_blocking(&flight_log_guard);
    flight_log.freeze();
    log_output_count=flight_log.count();
    log_output_first_sample=flight_log.first_sample_number();
    critical_section_exit(&flight_log_guard);
    log_output_position=0;
    log_output_started=true;
    printPQR();
    printf("#Roll rate PID gain\n");
    p_pid.printGain();
    printf("#Pitch rate PID gain\n");
    q_pid.printGain();
    printf("#Yaw rate PID gain\n");
    r_pid.printGain();
    printf("#Roll angle PID gain\n");
    phi_pid.printGain();
    printf("#Pitch angle PID gain\n");
    theta_pid.printGain();
    printf("#CH5 flight log: 100 Hz nominal, 38 fields, %lu/%lu samples, oldest first\n",
           static_cast<unsigned long>(log_output_count),
           static_cast<unsigned long>(LOG_ROWS));
    printf("#time_s is nominal elapsed time since the latest CH5 record start\n");
  }
  if(log_output_position<log_output_count)
  {
    float row[DATANUM];
    critical_section_enter_blocking(&flight_log_guard);
    const bool ok=flight_log.read_oldest_first(log_output_position,row);
    critical_section_exit(&flight_log_guard);
    if(ok)
    {
      const double nominal_time=static_cast<double>(
          log_output_first_sample+log_output_position)*0.01;
      printf("%10.2f ",nominal_time);
      for(uint32_t i=0;i<DATANUM;++i) printf("%12.5f",row[i]);
      printf("\n");
      ++log_output_position;
      return;
    }
  }
  printf("#Flight log export complete: %lu samples\n",
         static_cast<unsigned long>(log_output_position));
  // Keep the frozen recording available until a new CH5 ON starts a capture.
  Arm_flag=3;
  Logoutputflag=0;
  LockMode=0;
  log_output_started=false;
}

void gyroCalibration(void)
{
  float wp,wq,wr;
  float sump,sumq,sumr;
  uint16_t N=400;
  for(uint16_t i=0;i<N;i++)
  {
    sensor_read();
    sump=sump+Wp;
    sumq=sumq+Wq;
    sumr=sumr+Wr;
  }
  Pbias=sump/N;
  Qbias=sumq/N;
  Rbias=sumr/N;
}

void sensor_read(void)
{
  float mx1, my1, mz1, mag_norm, acc_norm, rate_norm;

  imu_mag_data_read();
  Ax =-acceleration_mg[0]*GRAV*0.001;
  Ay =-acceleration_mg[1]*GRAV*0.001;
  Az = acceleration_mg[2]*GRAV*0.001;
  Wp = angular_rate_mdps[0]*M_PI*5.55555555e-6;//5.5.....e-6=1/180/1000
  Wq = angular_rate_mdps[1]*M_PI*5.55555555e-6;
  Wr =-angular_rate_mdps[2]*M_PI*5.55555555e-6;
  Mx0 =-magnetic_field_mgauss[0];
  My0 = magnetic_field_mgauss[1];
  Mz0 =-magnetic_field_mgauss[2];

  
  acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  if (acc_norm>250.0) OverG_flag = 1;
  Acc_norm = acc_filter.update(acc_norm);
  rate_norm = sqrt(Wp*Wp + Wq*Wq + Wr*Wr);
  if (rate_norm > 6.0) OverG_flag =1;

/*地磁気校正データ
回転行列
[[ 0.65330968  0.75327755 -0.07589064]
 [-0.75666134  0.65302622 -0.03194321]
 [ 0.02549647  0.07829232  0.99660436]]
中心座標
122.37559195017053 149.0184454603531 -138.99116060635413
W
-2.432054387460946
拡大係数
0.003077277151877191 0.0031893151610213463 0.0033832794976645804

//回転行列
const float rot[9]={0.65330968, 0.75327755, -0.07589064,
                   -0.75666134, 0.65302622, -0.03194321,
                    0.02549647, 0.07829232,  0.99660436};
//中心座標
const float center[3]={122.37559195017053, 149.0184454603531, -138.99116060635413};
//拡大係数
const float zoom[3]={0.003077277151877191, 0.0031893151610213463, 0.0033832794976645804};
*/
  //回転行列
  const float rot[9]={-0.78435472, -0.62015392, -0.01402787,
                       0.61753358, -0.78277935,  0.07686857,
                      -0.05865107,  0.05162955,  0.99694255};
  //中心座標
  const float center[3]={-109.32529343620176, 72.76584808916506, 759.2285249891385};
  //拡大係数
  const float zoom[3]={0.002034773458122364, 0.002173892202021849, 0.0021819494099235273};

//回転・平行移動・拡大
  mx1 = zoom[0]*( rot[0]*Mx0 +rot[1]*My0 +rot[2]*Mz0 -center[0]);
  my1 = zoom[1]*( rot[3]*Mx0 +rot[4]*My0 +rot[5]*Mz0 -center[1]);
  mz1 = zoom[2]*( rot[6]*Mx0 +rot[7]*My0 +rot[8]*Mz0 -center[2]);
//逆回転
  Mx = rot[0]*mx1 +rot[3]*my1 +rot[6]*mz1;
  My = rot[1]*mx1 +rot[4]*my1 +rot[7]*mz1;
  Mz = rot[2]*mx1 +rot[5]*my1 +rot[8]*mz1; 

  mag_norm=sqrt(Mx*Mx +My*My +Mz*Mz);
  Mx/=mag_norm;
  My/=mag_norm;
  Mz/=mag_norm;

  // Record the freshness of the most recent 400 Hz IMU update.
  // All source values above are already flight-control inputs; no extra
  // sensor transaction is performed for USB debugging.
  imu_debug_sample_time_us = S_time;
  ++imu_debug_sample_count;
}

void variable_init(void)
{
  mutex_init(&attitude_debug_mutex);
  published_attitude = {};
  imu_debug_sample_count = 0;
  imu_debug_sample_time_us = 0;
  //Variable Initalize
  Xe << 1.00, 0.0, 0.0, 0.0,0.0,0.0, 0.0;
  Xp =Xe;

  Q <<  6.0e-5, 0.0    , 0.0    ,  0.0    , 0.0    , 0.0   ,
        0.0   , 5.0e-5 , 0.0    ,  0.0    , 0.0    , 0.0   ,
        0.0   , 0.0    , 2.8e-5 ,  0.0    , 0.0    , 0.0   ,
        0.0   , 0.0    , 0.0    ,  5.0e-5 , 0.0    , 0.0   ,
        0.0   , 0.0    , 0.0    ,  0.0    , 5.0e-5 , 0.0   ,
        0.0   , 0.0    , 0.0    ,  0.0    , 0.0    , 5.0e-5;

  R <<  1.701e0, 0.0     , 0.0     , 0.0   , 0.0   , 0.0   ,
        0.0     , 2.799e0, 0.0     , 0.0   , 0.0   , 0.0   ,
        0.0     , 0.0     , 1.056e0, 0.0   , 0.0   , 0.0   ,
        0.0     , 0.0     , 0.0     , 2.3e-1, 0.0   , 0.0   ,
        0.0     , 0.0     , 0.0     , 0.0   , 1.4e-1, 0.0   ,
        0.0     , 0.0     , 0.0     , 0.0   , 0.0   , 0.49e-1;
          
  G <<   1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 
        -1.0, 1.0,-1.0, 0.0, 0.0, 0.0, 
        -1.0,-1.0, 1.0, 0.0, 0.0, 0.0, 
         1.0,-1.0,-1.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  
  G=G*0.01;

  Beta << 0.0, 0.0, 0.0;
  
  P <<  1e0,  0,   0,   0,   0,  0,   0,  
        0  ,1e0,   0,   0,   0,  0,   0,
        0  ,  0, 1e0,   0,   0,  0,   0,  
        0  ,  0,   0, 1e0,   0,  0,   0, 
        0  ,  0,   0, 0  , 1e0,  0,   0,  
        0  ,  0,   0, 0  ,   0,1e0,   0,  
        0  ,  0,   0, 0  ,   0,  0, 1e0;
}

void printPQR(void)
{
  volatile int m=0;
  volatile int n=0;
  //Print P
  printf("#P\n");
  for (m=0;m<7;m++)
  {
    printf("# ");
    for (n=0;n<7;n++)
    {
      printf("%12.4e ",P(m,n));
    }
    printf("\n");
  }
  //Print Q
  printf("#Q\n");
  for (m=0;m<6;m++)
  {
    printf("# ");
    for (n=0;n<6;n++)
    {
      printf("%12.4e ",Q(m,n));
    }
    printf("\n");
  }
  //Print R
  printf("#R\n");
  for (m=0;m<6;m++)
  {
    printf("# ");
    for (n=0;n<6;n++)
    {
      printf("%12.4e ",R(m,n));
    }
    printf("\n");
  }
}

void kalman_filter(void)
{
  //Kalman Filter
  float dt=0.01;
  Omega_m << Wp, Wq, Wr;
  Z << Ax, Ay, Az, Mx, My, Mz;
  ekf(Xe, Xp, P, Z, Omega_m, Q, R, G*dt, Beta, dt);
}


PID::PID()
{
  m_kp=1.0e-8;
  m_ti=1.0e8;
  m_td=0.0;
  m_integral=0.0;
  m_filter_time_constant=0.01;
  m_filter_output=0.0;
  m_err=0.0;
  m_h=0.01;
}

void PID::set_parameter(
    float kp, 
    float ti, 
    float td,
    float filter_time_constant, 
    float h)
{
  m_kp=kp;
  m_ti=ti;
  m_td=td;
  m_filter_time_constant=filter_time_constant;
  m_h=h;
}

void PID::reset(void)
{
  m_integral=0.0;
  m_filter_output=0.0;
  m_err=0.0;
  m_err2=0.0;
  m_err3=0.0;
}

void PID::i_reset(void)
{
  m_integral=0.0;
}
void PID::printGain(void)
{
  printf("#Kp:%8.4f Ti:%8.4f Td:%8.4f Filter T:%8.4f h:%8.4f\n",m_kp,m_ti,m_td,m_filter_time_constant,m_h);
}

float PID::filter(float x)
{
  m_filter_output = m_filter_output * m_filter_time_constant/(m_filter_time_constant + m_h) 
                  + x * m_h/(m_filter_time_constant + m_h);   
  return m_filter_output;
}

float PID::update(float err)
{
  float d;
  m_integral = m_integral + m_h * err;
  if(m_integral> 30000.0)m_integral = 30000.0;
  if(m_integral<-30000.0)m_integral =-30000.0;
  m_filter_output = filter((err-m_err3)/m_h);
  m_err3 = m_err2;
  m_err2 = m_err;
  m_err  = err;
  return m_kp*(err + m_integral/m_ti + m_td * m_filter_output); 
}

Filter::Filter()
{
  m_state = 0.0;
  m_T = 0.0025;
  m_h = 0.0025;
}

void Filter::reset(void)
{
  m_state = 0.0;
}

void Filter::set_parameter(float T, float h)
{
  m_T = T;
  m_h = h;
}

float Filter::update(float u)
{
  m_state = m_state * m_T /(m_T + m_h) + u * m_h/(m_T + m_h);
  m_out = m_state;
  return m_out;
}
