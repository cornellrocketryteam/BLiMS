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

void BLIMS::begin(BLIMSMode mode, uint8_t pin, int32_t target_lat, int32_t target_lon)
{
  printf("Begin BLiMS");
  pwm_setup();
  blims::flight::flight_mode = mode;
  blims::flight::blims_motor_pin = pin;
  blims::LV::target_lat = target_lat;
  blims::LV::target_lon = target_lon;
  // blims_start = false;
}

BLIMSDataOut BLIMS::execute(BLIMSDataIn *data_in)
{
  printf("Execute BLiMS - Data In:\n");
  // update state vars with FSW data
  update_state_gps_vars(data_in);

  if (blims::flight::flight_mode == STANDBY)
  {
    printf("Standby Mode");
  }
  else if (blims::flight::flight_mode == MVP_Flight)
  {
    printf("MVP_Flight Mode");
    if (!blims::flight::blims_init)
    {
      blims::flight::blims_init = true;
      add_alarm_in_ms(initial_hold_threshold, BLIMS::execute_MVP, NULL, true);
    }
  }
  else if (blims::flight::flight_mode == LV)
  {
    printf("LV");
    // add blims state var --> 20 second wait is over
    // static funcition --> sets var to true, if false, don't do anything, if true, call execute_LV
    if (!blims::flight::blims_init)
    {
      add_alarm_in_ms(initial_hold_threshold, BLIMS::init_timer, NULL, true);
      blims::flight::blims_init = true;
    }
    if (blims_start == true)
    {
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

void BLIMS::execute_LV()
{
  printf("Execute LV\n");
  // Var Calculations
  blims::LV::LFP_lat = alpha * blims::flight::gps_lat + (1 - alpha) * blims::flight::gps_lat;
  blims::LV::LFP_lon = alpha * blims::flight::gps_lon + (1 - alpha) * blims::flight::gps_lon;
  blims::LV::deltaLat = blims::LV::target_lat - blims::LV::LFP_lat;
  blims::LV::deltaLon = blims::LV::target_lon - blims::LV::LFP_lon;
  blims::LV::bearing = BLIMS::calculate_bearing();
  blims::LV::angError = BLIMS::calculate_angError();
  blims::flight::currTime = to_ms_since_boot(get_absolute_time());
  printf("prevTime: %d\n", blims::flight::prevTime);
  printf("currTime: %d\n", blims::flight::currTime);
  blims::flight::timePassed = BLIMS::calculate_timePassed();

  // P term
  blims::LV::pid_P = Kp * blims::LV::angError;
  // I term
  blims::LV::pid_I = BLIMS::calculate_pid_I();
  // D term
  blims::LV::pid_D = Kd * ((blims::LV::angError - blims::LV::prevError) / blims::flight::timePassed);
  // Caluculate output
  blims::LV::controllerOutput = blims::LV::pid_P + blims::LV::pid_I + blims::LV::pid_D;
  BLIMS::set_motor_position(blims::LV::controllerOutput);

  blims::LV::prevError = blims::LV::angError;
  blims::flight::prevTime = blims::flight::currTime;
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

int32_t BLIMS::calculate_pid_I()
{
  blims::LV::integralError += blims::LV::angError * blims::flight::timePassed;
  if (blims::LV::integralError > integral_max)
  {
    blims::LV::integralError = integral_max;
  }
  else if (blims::LV::integralError < -integral_max)
  {
    blims::LV::integralError = -integral_max;
  }
  return Ki * blims::LV::integralError;
}

int32_t BLIMS::calculate_bearing()
{
  int32_t bearing = atan2(blims::LV::deltaLon, blims::LV::deltaLat) * 180 / M_PI;

  if (blims::LV::bearing < 0)
  {
    blims::LV::bearing += 360;
  }
  return bearing;
}

int32_t BLIMS::calculate_angError()
{
  ////TODO: MAKE SURE headMot IS WHAT YOU WANT
  int32_t angError = blims::flight::headMot - blims::LV::bearing;

  // Normalize error to be within [-180, 180]. Want to take the shorter path
  if (angError > 180)
  {
    angError -= 360;
  }
  else if (angError < -180)
  {
    angError += 360;
  }
  return angError;
}
void BLIMS::set_motor_position(float position)
{
  // printf("setting motor position to %f\n", position);
  uint slice_num = pwm_gpio_to_slice_num(blims::flight::blims_motor_pin);
  // Position should be between 0-1
  // Should map between -17 to 17 turns (configured in web UI)

  // Map position to PWM duty cycle (typically 1ms to 2ms pulse width)
  uint16_t five_percent_duty_cycle = wrap_cycle_count * 0.05;
  // ranges between 5% and 10% duty cycle; 3276 ~= 5% duty, 6552 ~= 10% duty
  uint16_t duty = (uint16_t)(five_percent_duty_cycle + position * five_percent_duty_cycle);
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(blims::flight::blims_motor_pin), duty);
  // update state of motor (what is the position at the current time)
  // state::blims::motor_position = position;

  blims::flight::data_out.motor_position = position;
}

void BLIMS::pwm_setup()
{
  // Set up the PWM configuration
  uint slice_num = pwm_gpio_to_slice_num(blims::flight::blims_motor_pin);

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

void BLIMS::data_print_test()
{
  printf("blims_start:%d\n", blims_start);

  printf("GPS Print Statements\n");
  printf("Latitude: %d\n", blims::flight::gps_lat);
  printf("Longitude: %d\n", blims::flight::gps_lon);
  printf("North Velocity: %d\n", blims::flight::velN);
  printf("Motor Position: %f\n", blims::flight::data_out.motor_position);

  printf("\nLV Calculation Vars Print Statements\n");
  printf("LFP_lat: %d\n", blims::LV::LFP_lat);
  printf("LFP_lon: %d\n", blims::LV::LFP_lon);
  printf("deltaLat: %d\n", blims::LV::deltaLat);
  printf("deltaLon: %d\n", blims::LV::deltaLon);
  printf("bearing: %d\n", blims::LV::bearing);
  printf("angError: %d\n", blims::LV::angError);
  printf("currTime: %d\n", blims::flight::currTime);
  printf("timePassed: %d\n", blims::flight::timePassed);
  printf("prevError: %d\n", blims::LV::prevError);
  printf("prevTime: %d\n", blims::flight::prevTime);

  printf("\nController Print Statements\n");
  printf("pid_P: %d\n", blims::LV::pid_P);
  printf("pid_I: %d\n", blims::LV::pid_I);
  printf("pid_D: %d\n", blims::LV::pid_D);
  printf("controllerOutput: %d\n", blims::LV::controllerOutput);
}

void BLIMS::update_state_gps_vars(BLIMSDataIn *data_in)
{
  blims::flight::gps_lon = data_in->lon;
  blims::flight::gps_lat = data_in->lat;
  blims::flight::hAcc = data_in->hAcc;
  blims::flight::vAcc = data_in->vAcc;
  blims::flight::velN = data_in->velN;
  blims::flight::velE = data_in->velE;
  blims::flight::velD = data_in->velD;
  blims::flight::gSpeed = data_in->gSpeed;
  blims::flight::headMot = data_in->headMot;
  blims::flight::sAcc = data_in->sAcc;
  blims::flight::headAcc = data_in->headAcc;
}

int64_t BLIMS::execute_MVP(alarm_id_t id, void *user_data)
{
  printf("Execute MVP\n");

  // printf("in execute, action index = %d\n", state::blims::curr_action_index);
  blims::MVP::curr_action_index++;

  if (blims::MVP::curr_action_index >= 11)
  {
    blims::MVP::curr_action_index = 0;
  }
  set_motor_position(blims::MVP::action_arr[blims::MVP::curr_action_index].position);
  // state::flight::events.emplace_back(Event::blims_threshold_reached); // we've completed a motor action in action_arr
  add_alarm_in_ms(blims::MVP::action_arr[blims::MVP::curr_action_index].duration, BLIMS::execute_MVP, NULL, false);

  // update statex
  return 0; // need this for add_alarm_in_ms

  //
}