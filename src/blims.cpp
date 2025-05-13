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
#include <math.h>

static bool blims_start = false;
static bool pwm_start = false;

void BLIMS::begin(BLIMSMode mode, uint8_t pwm_pin, uint8_t enable_pin, int32_t target_lat, int32_t target_lon)
{

  blims::flight::flight_mode = mode;
  blims::flight::blims_pwm_pin = pwm_pin;
  blims::flight::blims_enable_pin = enable_pin;
  blims::LV::target_lat = target_lat;
  blims::LV::target_lon = target_lon;
  pwm_setup();
  // blims_start = false;
}
float compute_heading_error(float target, float current)
{
  float error = target - current;
  if (error > 180)
    error -= 360; // Wrap error to be within 180, -180. Want to always take the shortest turn to the target
  if (error < -180)
    error += 360;
  return error;
}
BLIMSDataOut BLIMS::execute(BLIMSDataIn *data_in)
{
  // update state vars with FSW data
  update_state_gps_vars(data_in);

  if (blims::flight::flight_mode == STANDBY)
  {
  }
  else if (blims::flight::flight_mode == MVP_Flight)
  {

    if (!blims::flight::blims_init)
    {
      blims::flight::blims_init = true;
      add_alarm_in_ms(initial_hold_threshold, BLIMS::execute_MVP, NULL, true);
    }
  }
  else if (blims::flight::flight_mode == LV)
  {
    // add blims state var --> 20 second wait is over
    // static funcition --> sets var to true, if false, don't do anything, if true, call execute_LV
    if (!blims::flight::blims_init)
    {
      add_alarm_in_ms(initial_hold_threshold, BLIMS::init_timer, NULL, true);
      blims::flight::blims_init = true;
    }
    if (blims_start == true)
    {
      gpio_put(blims::flight::blims_enable_pin, 1); // pull enable pin high to clear errors and put motor in the right state
      BLIMS::execute_LV();
      data_print_test();
    }
  }

  return blims::flight::data_out;
  // depending on what our mode is, execute different functions
}

int64_t BLIMS::init_timer(alarm_id_t id, void *user_data)
{
  blims_start = true;
  return 0;
}

int64_t BLIMS::pwm_setup_timer(alarm_id_t id, void *user_data)
{
  pwm_start = true;
  return 0;
}

void BLIMS::execute_LV()
{
  // TODO: how to set to true in FSW?
  if (blims::LV::gps_state)
  { // if gps status from FSW good then run

    if (blims::flight::fixType == 4 || blims::flight::fixType == 3 || blims::flight::fixType == 2)
    {                                                                  // only run the logic if we have satellite lock and are moving fast enough to have a clear direction. More relevant to car testing than actual flight but do make sure that the code doesn't break if the expected data isn't returned for a loop or two
      float dt_ms = (float)(blims::flight::currTime - blims::flight::prevTime); // calculate how long since last loop (delta time)
      blims::flight::prevTime = blims::flight::currTime;               // reset last time for the next loop
      if (dt_ms <= 0 || dt_ms > 1000) {
        dt_ms = 100;              // cap to prevent large jumps
      }
        
      float dt = dt_ms / 1000.0f; // convert to seconds

      BLIMS::calculate_bearing();
      float error = compute_heading_error(blims::LV::bearing, blims::flight::headMot);

      blims::LV::error_integral += error * dt; // integral = area under curve. This is the discritized version of that

      float limit = 0.5f / Ki; // because error_integral gets multiplied by Ki later, this calculation makes sure that the clamping limits on the I term are indeed 0.5
      if (blims::LV::error_integral > limit)
        blims::LV::error_integral = limit; // clamp term to prevent integral windup
      if (blims::LV::error_integral < -limit)
        blims::LV::error_integral = -limit;

      // calculate control terms
      blims::LV::pid_P = -1 * Kp * error;
      blims::LV::pid_I = -1 * Ki * blims::LV::error_integral;

      float correction = blims::LV::pid_P + blims::LV::pid_I;

      // control input is relative to 0.5 instead of 0, because neutral motor position is 0.5
      float position = 0.5f + correction;
      if (position < 0.0f)
        position = 0.0f;
      if (position > 1.0f)
        position = 1.0f;

      set_motor_position(position);
    }
    else
    {
      set_motor_position(0.5f); // set to neutral if no data
    }
  }
  // TODO
  // sleep_ms(100); // loop runs at 10Hz
}

int32_t BLIMS::calculate_timePassed()
{
  int32_t timePassed = (blims::flight::currTime - blims::flight::prevTime) / 1000;

  if (blims::flight::timePassed <= 0)
  {
    blims::flight::timePassed = 0.001;
  }
  return timePassed;
}

// int32_t BLIMS::calculate_pid_I()
// {
//   blims::LV::integralError += blims::LV::angError * blims::flight::timePassed;
//   if (blims::LV::integralError > integral_max)
//   {
//     blims::LV::integralError = integral_max;
//   }
//   else if (blims::LV::integralError < -integral_max)
//   {
//     blims::LV::integralError = -integral_max;
//   }
//   return Ki * blims::LV::integralError;
// }

void BLIMS::calculate_bearing()
{
  // convert all to radians
  blims::flight::gps_lat *= deg_to_rad;
  blims::flight::gps_lon *= deg_to_rad;
  blims::LV::target_lat *= deg_to_rad;
  blims::LV::target_lon *= deg_to_rad;

  float d_lon = blims::LV::target_lon - blims::flight::gps_lon;
  float d_lat = blims::LV::target_lat - blims::flight::gps_lat;
  float bearing = atan2(d_lat, d_lon) * rad_to_deg; // compute angle, then convert back to degrees

  if (bearing < 0)
  {
    bearing += 360.0f;
  } // atan2 is in range [-180,180). We want 0,360 for logic and plotting
  blims::LV::bearing = bearing;
}

// int32_t BLIMS::calculate_angError()
// {
//   int32_t angError = blims::flight::headMot - blims::LV::bearing;

//   // Normalize error to be within [-180, 180]. Want to take the shorter path
//   if (angError > 180)
//   {
//     angError -= 360;
//   }
//   else if (angError < -180)
//   {
//     angError += 360;
//   }
//   return angError;
// }

void BLIMS::set_motor_position(float position)
{
  uint slice_num = pwm_gpio_to_slice_num(blims::flight::blims_pwm_pin);
  // Position should be between 0-1
  // Should map between -17 to 17 turns (configured in web UI)

  // Map position to PWM duty cycle (typically 1ms to 2ms pulse width)
  uint16_t five_percent_duty_cycle = wrap_cycle_count * 0.05;
  // ranges between 5% and 10% duty cycle; 3276 ~= 5% duty, 6552 ~= 10% duty
  uint16_t duty = (uint16_t)(five_percent_duty_cycle + position * five_percent_duty_cycle);
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(blims::flight::blims_pwm_pin), duty);
  // update state of motor (what is the position at the current time)
  // state::blims::motor_position = position;
  blims::flight::data_out.motor_position = position;
  blims::flight::data_out.pid_I = blims::LV::pid_I;
  blims::flight::data_out.pid_P = blims::LV::pid_P;
}

void BLIMS::pwm_setup()
{
  gpio_set_function(blims::flight::blims_pwm_pin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(blims::flight::blims_pwm_pin);
  gpio_init(blims::flight::blims_enable_pin);
  gpio_set_dir(blims::flight::blims_enable_pin, GPIO_OUT);

  float divider = 125000000.0f / (50 * wrap_cycle_count);
  pwm_set_clkdiv(slice_num, divider);
  pwm_set_wrap(slice_num, wrap_cycle_count);
  pwm_set_enabled(slice_num, true);
}

void BLIMS::data_print_test()
{
#ifdef VERBOSE // set in FSW
  printf("blims_start: %d\n", blims_start);

  printf("\nLV Calculation Vars Print Statements\n");
  printf("bearing: %f\n", blims::LV::bearing);
  // printf("currTime: %d\n", blims::flight::currTime);
  // printf("timePassed: %d\n", blims::flight::timePassed);
  // printf("prevError: %d\n", blims::LV::prevError);
  // printf("prevTime: %d\n", blims::flight::prevTime);

  printf("\nController Print Statements\n");
  printf("pid_P: %f\n", blims::LV::pid_P);
  printf("pid_I: %f\n", blims::LV::pid_I);
#endif
}

void BLIMS::update_state_gps_vars(BLIMSDataIn *data_in)
{
  blims::flight::gps_lon = data_in->lon * 1e-7f;
  blims::flight::gps_lat = data_in->lat * 1e-7f;

  blims::flight::hAcc = data_in->hAcc;
  blims::flight::vAcc = data_in->vAcc;
  blims::flight::velN = data_in->velN;
  blims::flight::velE = data_in->velE;
  blims::flight::velD = data_in->velD;
  blims::flight::gSpeed = data_in->gSpeed;

  blims::flight::headMot = data_in->headMot;
  blims::flight::headMot = ((blims::flight::headMot * 1e-5f) - 90) * -1;
  if (blims::flight::headMot < 0)
  {
    blims::flight::headMot += 360.0f;
  }

  blims::flight::sAcc = data_in->sAcc;
  blims::flight::headAcc = data_in->headAcc;

  blims::flight::fixType = data_in->fixType;
  blims::LV::gps_state = data_in->gps_state;
}

int64_t BLIMS::execute_MVP(alarm_id_t id, void *user_data)
{
  gpio_put(blims::flight::blims_enable_pin, 1);
  blims::MVP::curr_action_index++;

  if (blims::MVP::curr_action_index >= 11)
  {
    blims::MVP::curr_action_index = 0;
  }
  set_motor_position(blims::MVP::action_arr[blims::MVP::curr_action_index].position);
  add_alarm_in_ms(blims::MVP::action_arr[blims::MVP::curr_action_index].duration, BLIMS::execute_MVP, NULL, false);

  // update statex
  return 0; // need this for add_alarm_in_ms

  //
}