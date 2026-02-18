/**
 * @file blims_state.hpp
 * @brief BLiMS state definitions and data structures
 * 
 * This file defines the data structures used to communicate between
 * the flight software (FSW) and the BLiMS guidance controller.
 */

#ifndef BLIMS_STATE_HPP
#define BLIMS_STATE_HPP
#include <cstdint>

enum BLIMSMode
{
  STANDBY,    //system initialized but not active
  LV //Launch Vehicle - full L3 logic
};
/**
 * BLIMSDataIn: struct for data coming from FSW to BLiMS controller
 * passed to BLiMS::execute() every iteration (50 ms or 20 Hz)
 * All GPS fields come directly from the u-blox MAX-M10M UBX-NAV-PVT message.
 */
struct BLIMSDataIn
{
  //Position (from GPS)
  int32_t lon; ///< Longitude in degrees * 1e7
  int32_t lat; ///< Latitude in degrees * 1e7

  // Altitude (from BMP390 barometer, processed by FSW)
  float altitude_ft;  ///< Altitude AGL in feet

  //Accuracy estimates (from GPS)
  uint32_t hAcc; ///< Horizontal accuracy estimate in mm
  uint32_t vAcc; ///< Vertical accuracy estimate in mm

  //Velocity (from GPS)
  int32_t velN; ///< Velocity north in mm/s
  int32_t velE; ///< Velocity east in mm/s
  int32_t velD; ///<Down velocity in mm/positive = descending

  // Speed and heading (from GPS)
  int32_t gSpeed; ///< Ground speed in mm/s - **NOT BEING USED FIRST L3
  int32_t headMot; ///< Heading of motion in degrees * 1e5

  // Accuracy estimates (from GPS)
  uint32_t sAcc; ///< Speed accuracy estimate in mm/s
  uint32_t headAcc; ///< Heading accuracy estimate in degrees * 1e5

  //GPS Status
  uint8_t fixType; ///< GPS fix type: 0=none, 2=2D, 3=3D, 4=3D+DGPS
  bool gps_state; ///< GPS validity flag from FSW
};

struct BLIMSDataOut
{
  float motor_position; //neutral is 0.5, bound btwn 0.3 and 0.7, range is 0 to 1
  float pid_P;
  float pid_I;
  float bearing; //0 = North CW
  int8_t phase_id;  //Current flight phase (0-6): 0=HELD, 1=TRACK, 2=DOWNWIND, 3=BASE, 4=FINAL, 5=NEUTRAL, 6=LOITER
  int8_t loiter_step; ///< Current loiter sub-state (0-3), only valid in LOITER phase

};

namespace blims
{
  namespace flight
  {
    extern uint8_t blims_pwm_pin;
    extern uint8_t blims_enable_pin;
    extern bool blims_init;
    extern BLIMSMode flight_mode;
    extern float motor_position;
    extern BLIMSDataOut data_out;

    //Processed via GPS
    extern float gps_lon;
    extern float gps_lat;
    extern float altitude_ft;
    extern uint32_t hAcc;
    extern uint32_t vAcc;
    extern int32_t velN;
    extern int32_t velE;
    extern int32_t velD;
    extern int32_t gSpeed;
    extern int32_t headMot;
    extern uint32_t sAcc;
    extern uint32_t headAcc;
    extern uint8_t fixType;

    
    extern uint32_t currTime;
    extern uint32_t prevTime;
    extern uint32_t timePassed;//double check, but can use this for loiter time block? 
  }
  namespace LV
  {
    extern float target_lat; // set in begin
    extern float target_lon; // set in begin
    extern float bearing;
    extern float prevError;
    extern float pid_P;
    extern float pid_I;
    extern bool gps_state;
    extern float error_integral;
    // NEW (L3-1): wind direction "FROM" (deg 0-360), uploaded preflight
    extern float wind_from_deg;
  };
}

#endif