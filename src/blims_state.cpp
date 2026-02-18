/**
 * @file blims_state.cpp
 * @brief BLiMS state variable definitions
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
            .motor_position = 0,
            .pid_P = 0,
            .pid_I = 0,
            .bearing = 0,
            .phase_id = 0,
            .loiter_step = 0
        };
        
        float gps_lon = 0;
        float gps_lat = 0;
        float altitude_ft = 0;
        uint32_t hAcc = 0;
        uint32_t vAcc = 0;
        int32_t velN = 0;
        int32_t velE = 0;
        int32_t velD = 0;
        int32_t gSpeed = 0;
        int32_t headMot = 0;
        uint32_t sAcc = 0;
        uint32_t headAcc = 0;
        uint8_t fixType = 0;
        
        uint32_t currTime = 0;
        uint32_t prevTime = 0;
        uint32_t timePassed = 0;
    }

    namespace LV //stick with this
    {
        float target_lat = 0;
        float target_lon = 0;
        float wind_from_deg = 0;
        float bearing = 0;
        float prevError = 0;
        float pid_P = 0;
        float pid_I = 0;
        bool gps_state = false;
        float error_integral = 0;
    }
}