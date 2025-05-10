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
  uint8_t fixType;
  bool gps_state;
};
struct BLIMSDataOut
{
  float motor_position;
  float pid_P;
  float pid_I;
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
    extern uint8_t blims_pwm_pin;
    extern uint8_t blims_enable_pin;
    extern bool blims_init;
    extern BLIMSMode flight_mode;
    extern float motor_position;
    extern BLIMSDataOut data_out;
    extern float gps_lon;
    extern float gps_lat;
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
    extern uint32_t fixType;
  }
  namespace MVP
  {
    extern int32_t curr_action_duration;
    extern int curr_action_index;
    extern Action action_arr[11];
  }
  namespace LV
  {
    extern float target_lat; // set in begin
    extern float target_lon; // set in begin
    // extern int32_t LFP_lat;    // GPS value
    // extern int32_t LFP_lon;    // GPS value
    // extern int32_t deltaLat;
    // extern int32_t deltaLon;
    extern float bearing;
    // extern int32_t magnitude;     // used for track with headMot
    extern float integralError; // could probably get rid of
    // extern int32_t angError;
    extern float prevError;
    extern float pid_P;
    extern float pid_I;
    // extern int32_t pid_D;
    // extern int32_t controllerOutput;
    extern bool gps_state;
    extern float error_integral;

  };
} // namespace name

#endif