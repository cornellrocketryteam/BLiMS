/**
 * @file blims.hpp
 * @brief BLiMS class definition
 */

#ifndef BLIMS_HPP
#define BLIMS_HPP

#include "blims_constants.hpp"
#include "blims_state.hpp"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <cstdio>

class BLIMS
{
public:
    /**
     * @brief Initialize BLiMS system
     * @param mode Operating mode (STANDBY, LV)
     * @param pwm_pin GPIO pin for motor PWM
     * @param enable_pin GPIO pin for motor enable
     */
    void begin(BLIMSMode mode, uint8_t pwm_pin, uint8_t enable_pin);
    
    void set_target(float lat, float lon);
        
    /**
     * @brief Set wind direction (direction wind is coming FROM)
     * @param deg Wind direction in degrees [0, 360)
     */
    void set_wind_from_deg(float deg);

    /**
     * @brief Set wind profile from FSW (loaded via umbilical)
     * @param altitudes_m Array of altitudes in meters
     * @param directions_deg Array of wind directions (FROM) in degrees
     * @param size Number of layers
     */
    void set_wind_profile(const float* altitudes_m, const float* directions_deg, int size);
    
    /**
     * @brief Main execution function - call every control loop
     * @param data_in Pointer to input data from FSW
     * @return Output data for logging
     */
    BLIMSDataOut execute(BLIMSDataIn *data_in);

private:
    void pwm_setup();
    void data_print_test();
    void update_state_vars(BLIMSDataIn *data_in);
    
    static void set_motor_position(float position);
    
    // LV mode
    void execute_LV();
    void calculate_bearing();

    int32_t calculate_pid_I();
    int32_t calculate_angError();
    int32_t calculate_timePassed();
    static int64_t pwm_setup_timer(alarm_id_t id, void *user_data);
    
    // Loiter
    static int64_t loiter_alarm_callback(alarm_id_t id, void *user_data);
    void execute_loiter();
    void reset_loiter_state();
    
    // Init timer
    static int64_t init_timer(alarm_id_t id, void *user_data);


};

#endif // BLIMS_HPP