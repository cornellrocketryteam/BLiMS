/**
 * @file state.cpp
 * @author Gabriella Best gb486
 *
 * @brief BLiMS state variables
 */

#include "blims_state.hpp"
#include "blims_constants.hpp"

namespace flight
{
  bool blims_init = false;
  BLIMSMode flight_mode = STANDBY;
  float motor_position = 0;
  BLIMSDataOut data_out = {
      .motor_position = 0};
  float longitude = 0;
  float latitude = 0;
  float speed = 0;
  float track_angle = 0;
  float heading = 0;
}
namespace MVP
{
  int32_t curr_action_duration = 0;
  int curr_action_index = -1;

  // action_arr is effectively a cyle of instructions.
  // [_,[0,_]] index = % of motor's possible turning amount to turn (set in ODrive UI)
  // [_,[_,1]] index = length of pause after each turn
  Action action_arr[10] = {
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