/**
 * @file blims.cpp
 * @author gb486
 *
 * @brief BLIMS Functionality
 */

#include "blims.hpp"
#include "blims_state.hpp"
#include "blims_constants.hpp"
#include "hardware/pwm.h"
#include "hardware/timer.h"

BLIMS::begin(BLIMSMode mode)
{
  pwm_setup();
  flight::flight_mode = mode;
}

BLIMSDataOut BLIMS::execute(BLIMSDataIn data)
{
  if (flight::flight_mode == STANDBY)
  {
    print("Standby");
  }
  else if (flight::flight_mode == MVP_Flight)
  {
    print("MVP_Flight");
    add_alarm_in_ms(BLIMS_CONSTANTS_HPP::initial_hold_threshold, BLIMS::execute_MVP, NULL, true);
    // Note: could when we start the delay have any effect on when blims starts?
    // int64_t BLIMS::execute_MVP(alarm_id_t id, void *user_data);
  }
  else if (flight::flight_mode == MVP_Plus)
  {
    print("MVP_Plus");
  }
  else if (flight::flight_mode == LV)
  {
    print("LV");
  }

  return flight::data_out;
  // depending on what our mode is, execute different functions
}

int64_t BLIMS::execute_MVP(alarm_id_t id, void *user_data)
{

  // printf("in execute, action index = %d\n", state::blims::curr_action_index);
  MVP::curr_action_index++;

  if (MVP::curr_action_index >= 10)
  {
    MVP::curr_action_index = 0;
  }
  set_motor_position(action_arr[MVP::curr_action_index].position);
  // state::flight::events.emplace_back(Event::blims_threshold_reached); // we've completed a motor action in action_arr
  add_alarm_in_ms(action_arr[MVP::curr_action_index].duration, execute, NULL, false);
  return 0;
}

void BLIMS::set_motor_position(float position)
{
  // printf("setting motor position to %f\n", position);
  uint slice_num = pwm_gpio_to_slice_num(BLIMS_MOTOR);
  // Position should be between 0-1
  // Should map between -17 to 17 turns (configured in web UI)

  // Map position to PWM duty cycle (typically 1ms to 2ms pulse width)
  uint16_t five_percent_duty_cycle = wrap_cycle_count * 0.05;
  // ranges between 5% and 10% duty cycle; 3276 ~= 5% duty, 6552 ~= 10% duty
  uint16_t duty = (uint16_t)(five_percent_duty_cycle + position * five_percent_duty_cycle);
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(BLIMS_MOTOR), duty);

  // update state of motor (what is the position at the current time)
  // state::blims::motor_position = position;
}

void pwm_setup()
{
  // Set up the PWM configuration
  uint slice_num = pwm_gpio_to_slice_num(BLIMS_MOTOR);

  uint32_t clock = 125000000; // What our pico runs - set by the pico hardware itself

  uint32_t pwm_freq = 50; // desired pwm frequency Hz - what we want to achieve

  uint32_t divider_int = clock / pwm_freq / wrap_cycle_count; // div is important to get the frequency we want as we lose float info here

  uint32_t divider_frac = (clock % (pwm_freq * wrap_cycle_count)) * 16 / (pwm_freq * wrap_cycle_count); // gives us the fractional component of the divider
  // Clock divider: slows down pwm to get a certain amount of Hz
  // slice num - depends on what pin we're at, we set pin, pin has set slice num
  // integer divider - going to be 38
  pwm_set_clkdiv_int_frac(slice_num, divider_int, divider_frac);
  pwm_set_wrap(slice_num, wrap_cycle_count);
  pwm_set_enabled(slice_num, true);
}

// This was in FSW main -> flight_mode.cpp
//  state::blims::curr_action_duration = blims.action_arr[state::blims::curr_action_index].duration;

// Take a look at execute_MVP - how is everything supposed to be accessed? what about wrap_cyle_count

// how is the motor pin supposed to be set? see FSW pins.hpp/cpp

// why does action_arr need no reference to MVP?