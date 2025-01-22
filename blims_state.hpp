#ifndef BLIMS_STATE_HPP
#define BLIMS_STATE_HPP
#include <cstdint>

enum BLIMSMode
{
  STANDBY, //
  MVP,     // Jan 2025 launch
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
struct ActionArr
{
  float position;
  uint32_t duration;
};
namespace general
{
  BLIMSMode flight_mode;
  extern float motor_position;
}
namespace MVP
{
  extern int32_t curr_action_duration;
  extern int curr_action_index;
  extern ActionArr action_arr[10];
};

#endif