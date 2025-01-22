/**
 * @file state.cpp
 * @author Gabriella Best gb486
 *
 * @brief BLiMS state variables
 */

#include "blims_state.hpp"
#include "blims_constants.hpp"
namespace general
{
  BLIMSMode flight_mode = STANDBY;

  BLIMSDataIn data_in = {
      longitude = -1,
      latitude = -1,
      speed = -1,       // speed over the ground in knots
      track_angle = -1, // degrees
      heading = -1};

}
namespace MVP
{
  float motor_position = 0;
  int32_t curr_action_duration = 0;
  int curr_action_index = 0;

  // action_arr is effectively a cyle of instructions.
  // [_,[0,_]] index = % of motor's possible turning amount to turn (set in ODrive UI)
  // [_,[_,1]] index = length of pause after each turn
  ActionArr action_arr[10] = {
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