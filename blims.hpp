/**
 * @file blims.hpp
 * @author gb486
 *
 * @brief BLiMS related definitions
 */
#ifndef BLIMS_HPP
#define BLIMS_HPP
#include "blims_constants.hpp"
#include "blims_pins.hpp"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

struct Action
{
  float position;
  uint32_t duration;
};

class BLIMS
{
public:
  void begin(BLIMSMode mode);
  BLIMSDataOut execute(BLIMSDataIn data);

private:
  // configures the pwm signal
  void pwm_setup();
  // sets position of motor on a 0-1 scale
  static void set_motor_position(float position);
  static int64_t execute_MVP(alarm_id_t id, void *user_data);
};

#endif