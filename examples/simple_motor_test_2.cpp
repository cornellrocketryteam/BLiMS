#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <cstdio>
#define PWM_PIN 28
#define ODRIVE_STATE_PIN 0
uint16_t wrap_cycle_count = 65535;

void setup_pwm_50hz(uint gpio_pin)
{
  gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
  gpio_init(ODRIVE_STATE_PIN);
  gpio_set_dir(ODRIVE_STATE_PIN, GPIO_OUT);
  uint32_t clock = 125000000;
  uint32_t pwm_freq = 50;
  float divider = (float)clock / (pwm_freq * wrap_cycle_count);
  pwm_set_clkdiv(slice_num, divider);
  pwm_set_wrap(slice_num, wrap_cycle_count);
  pwm_set_enabled(slice_num, true);
}

void set_motor_position(uint gpio_pin, float position)
{
  uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
  uint16_t five_percent_duty_cycle = wrap_cycle_count * 0.05;
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
    gpio_put(ODRIVE_STATE_PIN, 1);
    printf("enable\n");
    sleep_ms(500);
    gpio_put(ODRIVE_STATE_PIN, 0);
    printf("pulse low\n");
    sleep_ms(5000);
    printf("turn 0.5\n");
    set_motor_position(PWM_PIN, 0.5);
    sleep_ms(5000);
    printf("turn 0.75\n");
    set_motor_position(PWM_PIN, 0.75);
    sleep_ms(5000);
  }
  return 0;
}