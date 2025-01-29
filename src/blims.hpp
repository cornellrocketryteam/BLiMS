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
  void begin(BLIMSMode mode);
  BLIMSDataOut execute(BLIMSDataIn data_in);

private:
  // configures the pwm signal
  void pwm_setup();
  // sets position of motor on a 0-1 scale
  static void set_motor_position(float position);
  static int64_t execute_MVP(alarm_id_t id, void *user_data);
};

#endif