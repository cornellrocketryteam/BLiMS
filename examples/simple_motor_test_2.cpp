#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <cstdio>
#define PWM_PIN 28         // GPIO pin to output PWM signal
#define ODRIVE_STATE_PIN 0 // Choose a GPIO to send a state change signal
// want wrap to be as large as possible, increases the amount of steps so that we have as much control as possible
uint16_t wrap_cycle_count = 65535; // setting to max value for uint16
/*
setup_pwm_50hz configures the pwm signal
takes in gpio_pin
*/
void setup_pwm_50hz(uint gpio_pin)
{
  // Set GPIO as PWM
  gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
  // Set motor state gpio pin
  gpio_init(ODRIVE_STATE_PIN);
  gpio_set_dir(ODRIVE_STATE_PIN, GPIO_OUT);
  // Clock frequency of the Pico
  uint32_t clock = 125000000;
  // Desired PWM frequency
  uint32_t pwm_freq = 50;
  // Compute the divider (Avoids integer rounding issues)
  float divider = (float)clock / (pwm_freq * wrap_cycle_count);
  // Set PWM parameters
  pwm_set_clkdiv(slice_num, divider);
  pwm_set_wrap(slice_num, wrap_cycle_count);
  pwm_set_enabled(slice_num, true);
}
/*
set_motor_position turns the motor to a position between 0-1
The limits are set by our ODrive configuration -> https://gui.odriverobotics.com/configuration
0 = minimum position
0.5 = neutral
1 = maximum position
*/
void set_motor_position(uint gpio_pin, float position)
{
  // Position should be between 0-1
  // Should map between -26 to 26 turns (configured in web UI)
  uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
  // Map position to PWM duty cycle (typically 1ms to 2ms pulse width)
  uint16_t five_percent_duty_cycle = wrap_cycle_count * 0.05;
  // ranges between 5% and 10% duty cycle; 3276 ~= 5% duty, 6552 ~= 10% duty
  uint16_t duty = (uint16_t)(five_percent_duty_cycle + position * five_percent_duty_cycle);
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio_pin), duty);
}
int main()
{
  sleep_ms(5000);
  stdio_init_all();
  setup_pwm_50hz(PWM_PIN);
  while (1)
  {
    // set_motor_position(PWM_PIN, 0.0);
    // sleep_ms(3000);
    // send switch state command
    // Reset ODrive after an error
    gpio_put(ODRIVE_STATE_PIN, 0); // Pulse LOW to reset error state
    printf("pulse low\n");
    sleep_ms(500);
    gpio_put(ODRIVE_STATE_PIN, 1); // Enable again
    printf("enable\n");
    sleep_ms(5000);
    printf("turn 0.5\n");
    set_motor_position(PWM_PIN, 0.5);
    sleep_ms(5000);
    printf("turn 0.75\n");
    set_motor_position(PWM_PIN, 0.75);
    sleep_ms(5000);
    // set_motor_position(PWM_PIN, 1.0);
    // sleep_ms(5000);
  }
  return 0;
}