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
    // pwm_setup configures the pwm signal
  void pwm_setup();

  static int64_t execute_MVP(alarm_id_t id, void *user_data);

private:
  // sets position of motor on a 0-1 scale
  static void set_motor_position(float position);

  // want wrap to be as large as possible, increases the amount of steps so that we have as much control as possible
  uint16_t wrap_cycle_count = 65535;
};

#endif