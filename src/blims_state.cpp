/**
 * @file state.cpp
 * @author Gabriella Best gb486
 *
 * @brief BLiMS state variables
 */

#include "blims_state.hpp"
#include "blims_constants.hpp"
namespace blims
{
  namespace flight
  {
    bool blims_init = false;
    BLIMSMode flight_mode = STANDBY;
    float motor_position = 0;
    BLIMSDataOut data_out = {
        .motor_position = 0};
    int32_t lon = 0;
    int32_t lat = 0;
    uint32_t hAcc = 0;
    uint32_t vAcc = 0;
    int32_t velN = 0;
    int32_t velE = 0;
    int32_t velD = 0;
    int32_t gSpeed = 0;
    int32_t headMot = 0;
    uint32_t sAcc = 0;
    uint32_t headAcc = 0;
  }
  namespace MVP
  {
    int32_t curr_action_duration = 0;
    int curr_action_index = -1;

    // action_arr is effectively a cyle of instructions.
    // [_,[0,_]] index = % of motor's possible turning amount to turn (set in ODrive UI)
    // [_,[_,1]] index = length of pause after each turn
    // Action action_arr[10] = {
    //     {0.75f, turn_hold_threshold},
    //     {0.5f, neutral_hold_threshold},
    //     {0.25f, turn_hold_threshold},
    //     {0.5f, neutral_hold_threshold},
    //     {0.875f, turn_hold_threshold},
    //     {0.5f, neutral_hold_threshold},
    //     {0.25f, turn_hold_threshold},
    //     {0.5f, neutral_hold_threshold},
    //     {1.0f, turn_hold_threshold},
    //     {0.5f, 5000}};

    Action action_arr[11] = {
        {0.625f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.375f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.6875f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.375f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.75f, turn_hold_threshold},
        {0.917f, 5000},
        {0.5f, 5000}};
  }
}
