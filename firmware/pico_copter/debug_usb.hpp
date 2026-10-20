#ifndef PICO_COPTER_DEBUG_USB_HPP
#define PICO_COPTER_DEBUG_USB_HPP

// All ordinary USB CDC diagnostics are implemented in debug_usb.cpp.
// Called ONLY on core 0's main thread: never call from the 400 Hz IRQ or
// from angle_control() on core 1.
//
// Flight-log sample export is NOT debug output and remains in control.cpp,
// so its 100 Hz x 38 field data stream and timing are unchanged.
namespace debug_usb {

// Edit this ONE file to choose which human-readable USB categories
// appear. These switches never alter sensors, control, DShot, or CH5 logging.
// printf() is still done on core 0's non-real-time main thread only.
constexpr bool kEnablePeriodicDiagnostics = true;
constexpr bool kEnableRcStateMessages = true;
constexpr bool kEnableState = true;
constexpr bool kEnableSbus = true;
constexpr bool kEnableImu = true;        // Accelerometer and gyro
constexpr bool kEnableMag = true;        // Calibrated, normalized magnetometer
constexpr bool kEnableAttitude = true;   // Roll/pitch/yaw in degrees
constexpr bool kEnableEkf = true;        // Quaternion + EKF gyro bias
constexpr bool kEnablePidMixer = true;   // T/P/Q/R and FR/FL/RR/RL
constexpr bool kEnableDShot = true;      // eRPM, commands and RX errors
constexpr bool kEnableEscSerial = true;  // TX-pad ESC voltage/current/temp/mAh

void startup_countdown(int seconds_remaining);
void startup_complete();
void sensor_not_found();
void poll();

} // namespace debug_usb
#endif
