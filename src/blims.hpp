/**
 * @file blims.hpp
 * @author gb486
 *
 * @brief BLiMS related definitions
 * State Machine:
 *   Phase -1: HELD     - Invalid GPS/speed, motor at neutral
 *   Phase  0: TRACK    - PI control homing to target (>1000ft, outside set radius)
 *   Phase  1: DOWNWIND - Fly with wind (1000-600ft)
 *   Phase  2: BASE     - Perpendicular to wind (600-300ft)
 *   Phase  3: FINAL    - Into wind (300-100ft)
 *   Phase  4: NEUTRAL  - Hands off for landing (<100ft)
 *   Phase  5: LOITER   - Altitude bleed via alternating turns (>1000ft, inside set radius)
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
  void begin(BLIMSMode mode, uint8_t pwm_pin, uint8_t enable_pin);
  void set_target_lat(float lat);
  void set_target_lon(float lon);

  // NEW (L3-1): uplink wind direction "FROM" in degrees (0..360)
  void set_wind_from_deg(float wind_from_deg);

  BLIMSDataOut execute(BLIMSDataIn *data_in);

  static float wrap360(float deg);
  static float wrap180(float deg);

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