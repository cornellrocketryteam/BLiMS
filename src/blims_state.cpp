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
    uint8_t blims_pwm_pin = 0;
    uint8_t blims_enable_pin = 0;
    bool blims_init = false;
    BLIMSMode flight_mode = STANDBY;
    float motor_position = 0;
    BLIMSDataOut data_out = {
        .motor_position = 0};
    float gps_lon = 0;
    float gps_lat = 0;
    uint32_t hAcc = 0;
    uint32_t vAcc = 0;
    int32_t velN = 0;
    int32_t velE = 0;
    int32_t velD = 0;
    int32_t gSpeed = 0;
    int32_t headMot = 0;
    uint32_t sAcc = 0;
    uint32_t headAcc = 0;
    uint32_t currTime = 0;
    uint32_t prevTime = 0;
    uint32_t timePassed = 0;
    //////TODO////
    uint8_t fixType = 0;

  }
  namespace MVP
  {
    int32_t curr_action_duration = 0;
    int curr_action_index = -1;

    // action_arr is effectively a cyle of instructions.
    // [_,[0,_]] index = % of motor's possible turning amount to turn (set in ODrive UI)
    // [_,[_,1]] index = length of pause after each turn
    // see previous code pushed to git for other actino arrays
    Action action_arr[11] = {
        {0.75f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.25f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.875f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.25f, turn_hold_threshold},
        {0.5f, neutral_hold_threshold},
        {0.95f, turn_hold_threshold},
        {1.0f, 5000},
        {0.5f, neutral_hold_threshold}};

  }
  namespace LV
  {
    float target_lat = 0; // set in begin
    float target_lon = 0; // set in begin
    // int32_t LFP_lat = 0;    // GPS value
    // int32_t LFP_lon = 0;    // GPS value
    // int32_t deltaLat = 0;
    // int32_t deltaLon = 0;
    float bearing = 0;
    // int32_t magnitude = 0; // used for track with headMot
    float integralError = 0;
    // int32_t angError = 0;
    float prevError = 0;
    float pid_P = 0;
    float pid_I = 0;
    // int32_t pid_D = 0;
    // int32_t controllerOutput = 0;
    bool gps_state = false;
    float error_integral = 0;

  };
}
