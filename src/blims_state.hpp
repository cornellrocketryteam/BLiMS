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
  int32_t lon;
  int32_t lat;
  uint32_t hAcc;
  uint32_t vAcc;
  int32_t velN;
  int32_t velE;
  int32_t velD;
  int32_t gSpeed;
  int32_t headMot;
  uint32_t sAcc;
  uint32_t headAcc;
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
  // TODO: break up into smaller sections
  namespace flight
  {
    extern float blims_motor_pin;
    extern bool blims_init;
    extern BLIMSMode flight_mode;
    extern float motor_position;
    extern BLIMSDataOut data_out;
    extern int32_t gps_lon;
    extern int32_t gps_lat;
    extern uint32_t hAcc;
    extern uint32_t vAcc;
    extern int32_t velN;
    extern int32_t velE;
    extern int32_t velD;
    extern int32_t gSpeed;
    extern int32_t headMot;
    extern uint32_t sAcc;
    extern uint32_t headAcc;
    extern uint32_t currTime;
    extern uint32_t prevTime;
    extern uint32_t timePassed;

  }
  namespace MVP
  {
    extern int32_t curr_action_duration;
    extern int curr_action_index;
    extern Action action_arr[11];
  }
  namespace LV
  {
    extern int32_t target_lat; // set in begin
    extern int32_t target_lon; // set in begin
    extern int32_t LFP_lat;    // GPS value
    extern int32_t LFP_lon;    // GPS value
    extern int32_t deltaLat;
    extern int32_t deltaLon;
    extern int32_t bearing;
    extern int32_t magnitude;     // used for track with headMot
    extern int32_t integralError; // could probably get rid of
    extern int32_t angError;
    extern int32_t prevError;
    extern int32_t pid_P;
    extern int32_t pid_I;
    extern int32_t pid_D;
    extern int32_t controllerOutput;

  };
} // namespace name

#endif