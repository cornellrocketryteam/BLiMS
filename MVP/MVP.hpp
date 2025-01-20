/**
 * @file MVP.hpp
 * @author Gabriella Best, gb486
 *
 * @brief BLiMS MVP launch variables
 */

#ifndef MVP_HPP
#define MVP_HPP

#include <stdint.h>

struct Action
{
  float position;
  uint32_t duration;
};

namespace MVP
{
  namespace constants
  {
    extern uint32_t initial_hold_threshold;
    extern uint32_t turn_hold_threshold;
    extern uint32_t neutral_hold_threshold;
    extern float neutral_pos;
    // action_arr is effectively a cyle of instructions.
    // [_,0] index = % of motor's possible turning amount it should turn (this amount set in ODrive UI)
    // [_,1] index = length of pause after each turn
    extern static Action action_arr[10];
  }
  namespace state
  {
    float motor_position = 0;
    int32_t curr_action_duration = 0;
    int curr_action_index = 0;

  }
}

#endif