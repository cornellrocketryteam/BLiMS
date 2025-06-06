/**
 * @file blims.hpp
 * @author gb486
 *
 * @brief BLiMS related definitions
 */
#ifndef BLIMS_HPP
#define BLIMS_HPP
#include "blims_constants.hpp"
#include "blims_state.hpp"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "cstdio"

class BLIMS
{
public:
  // associated with blims object because not static
  void begin(BLIMSMode mode, uint8_t pwm_pin, uint8_t enable_pin, float target_lat = 0, float target_lon = 0);
  BLIMSDataOut execute(BLIMSDataIn *data_in);

private:
  // configures the pwm signal
  void pwm_setup();
  // print test
  void data_print_test();
  // update state vars with FSW data
  void update_state_gps_vars(BLIMSDataIn *data_in);
  // sets position of motor on a 0-1 scale
  static void set_motor_position(float position);
  static int64_t execute_MVP(alarm_id_t id, void *user_data);
  void execute_LV();
  int32_t calculate_pid_I();
  void calculate_bearing();
  int32_t calculate_angError();
  int32_t calculate_timePassed();
  static int64_t init_timer(alarm_id_t id, void *user_data);

  static int64_t pwm_setup_timer(alarm_id_t id, void *user_data);
};

#endif