#ifndef BLIMS_STATE_HPP
#define BLIMS_STATE_HPP
#include <cstdint>

enum BLIMSMode
{
  STANDBY,    //
  MVP_Flight, // Jan 2025 launch
  MVP_Plus,
  LV
};

struct BLIMSDataIn
{
  float longitude;
  float latitude;
  float speed;       // speed over the ground in knots
  float track_angle; // degrees
  float heading;     // degrees
};
struct BLIMSDataOut
{
  float motor_position;
};
struct Action
{
  float position;
  uint32_t duration;
};
namespace blims
{
  namespace flight
  {
    extern bool blims_init;
    extern BLIMSMode flight_mode;
    extern float motor_position;
    extern BLIMSDataOut data_out;
    extern float longitude;
    extern float latitude;
    extern float speed;       // speed over the ground in knots
    extern float track_angle; // degrees
    extern float heading;     // degrees
  }
  namespace MVP
  {
    extern int32_t curr_action_duration;
    extern int curr_action_index;
    extern Action action_arr[10];
  };
} // namespace name

#endif