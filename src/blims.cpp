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

void BLIMS::begin(BLIMSMode mode)
{
  printf("Begin BLiMS");
  pwm_setup();
  flight::flight_mode = mode;
}

BLIMSDataOut BLIMS::execute(BLIMSDataIn data_in)
{
  printf("Execute BLiMS");
  // update state vars with FSW data
  flight::latitude = data_in.latitude;
  flight::longitude = data_in.longitude;
  flight::speed = data_in.speed;
  flight::track_angle = data_in.track_angle;
  flight::heading = data_in.heading;

  if (flight::flight_mode == STANDBY)
  {
    printf("Standby Mode");
  }
  else if (flight::flight_mode == MVP_Flight)
  {
    printf("MVP_Flight Mode");
    if (!flight::blims_init)
    {
      flight::blims_init = true;
      add_alarm_in_ms(BLIMS_CONSTANTS_HPP::initial_hold_threshold, BLIMS::execute_MVP, NULL, true);
    }

    // Note: could when we start the delay have any effect on when blims starts?
    // int64_t BLIMS::execute_MVP(alarm_id_t id, void *user_data);

    // return data_out every cycle
  }
  else if (flight::flight_mode == MVP_Plus)
  {
    printf("MVP_Plus");
  }
  else if (flight::flight_mode == LV)
  {
    printf("LV");
  }

  return flight::data_out;
  // depending on what our mode is, execute different functions
}

int64_t BLIMS::execute_MVP(alarm_id_t id, void *user_data)
{
  printf("Execute MVP");

  // printf("in execute, action index = %d\n", state::blims::curr_action_index);
  MVP::curr_action_index++;

  if (MVP::curr_action_index >= 10)
  {
    MVP::curr_action_index = 0;
  }
  set_motor_position(MVP::action_arr[MVP::curr_action_index].position);
  // state::flight::events.emplace_back(Event::blims_threshold_reached); // we've completed a motor action in action_arr
  add_alarm_in_ms(MVP::action_arr[MVP::curr_action_index].duration, BLIMS::execute_MVP, NULL, false);

  // update statex
  return 0; // need this for add_alarm_in_ms

  //
}

void BLIMS::set_motor_position(float position)
{
  // printf("setting motor position to %f\n", position);
  uint slice_num = pwm_gpio_to_slice_num(blims_motor);
  // Position should be between 0-1
  // Should map between -17 to 17 turns (configured in web UI)

  // Map position to PWM duty cycle (typically 1ms to 2ms pulse width)
  uint16_t five_percent_duty_cycle = wrap_cycle_count * 0.05;
  // ranges between 5% and 10% duty cycle; 3276 ~= 5% duty, 6552 ~= 10% duty
  uint16_t duty = (uint16_t)(five_percent_duty_cycle + position * five_percent_duty_cycle);
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(blims_motor), duty);
  // update state of motor (what is the position at the current time)
  // state::blims::motor_position = position;

  flight::data_out.motor_position = position;
}

void BLIMS::pwm_setup()
{
  // Set up the PWM configuration
  uint slice_num = pwm_gpio_to_slice_num(blims_motor);

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