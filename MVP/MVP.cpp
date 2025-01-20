/**
 * @file MVP.cpp
 * @author Gabriella Best gb486
 *
 * @brief BLiMS MVP launch variables
 */

#include "MVP.hpp"
#include "../blims_constants.hpp"

namespace MVP
{
  namespace constants
  {
    constexpr uint32_t initial_hold_threshold = 10000;
    constexpr uint32_t turn_hold_threshold = 5000;
    constexpr uint32_t neutral_hold_threshold = 2000;
    float neutral_pos = 0.5;
    // action_arr is effectively a cyle of instructions.
    // [_,0] index = % of motor's possible turning amount it should turn (this amount set in ODrive UI)
    // [_,1] index = length of pause after each turn
    constexpr static Action action_arr[10] = {
        {0.6f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.3f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.8f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.1f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {1.0f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
    };
  }
  namespace state
  {
    float motor_position = 0;
    int32_t curr_action_duration = 0;
    int curr_action_index = 0;

  }
}