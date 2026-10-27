// comment
#include "pico_copter.hpp"
#include "debug_usb.hpp"
#include "esc_serial_uart.hpp"

//グローバル変数
volatile uint8_t Arm_flag=0;
semaphore_t sem;

int main(void)
{
  int start_wait=5;
  
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  
  //Initialize stdio for Pico
  stdio_init_all();
  
  //Initialize LSM9DS1
  imu_mag_init();
  
  //Initialize Radio
  radio_init();
  
  //Initialize Variavle
  variable_init();
  
  //Initilize Control
  control_init();
  
  //Initialize synchronized RAM flight recorder before enabling 400 Hz IRQ.
  flight_log_init();

  // Both Pico/RP2040 and Pico 2/RP2350: independent GP17 PIO UART.
  // Not used as flight-control feedback.
  esc_serial::init();

  //Initialize PWM
  //Start 400Hz Interval
  ESC_calib=0;
  pwm_init();

  while(start_wait)
  {
    start_wait--;
    debug_usb::startup_countdown(start_wait); 
    sleep_ms(1000);
  }
  debug_usb::startup_complete();

  //マルチコア関連の設定
  sem_init(&sem, 0, 1);
  multicore_launch_core1(angle_control);  

  Arm_flag=1;
  
  while(1)
  {
    // Ordinary USB output is owned by debug_usb.cpp, not the control loop.
    debug_usb::poll();
    tight_loop_contents();
    while (Logoutputflag==1){
      log_output();
    }
  }

  return 0;
}
