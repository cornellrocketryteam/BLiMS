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
#include <cstdint>
#include <cstdio>

// ============================================================================
// ENUMS (moved from blims.cpp so member functions can use them)
// ============================================================================

enum class Phase : int8_t {
    HELD     = 0,
    TRACK    = 1,
    DOWNWIND = 2,
    BASE     = 3,
    FINAL    = 4,
    NEUTRAL  = 5,
    LOITER   = 6
};

enum class LoiterStep : int8_t {
    TURN_RIGHT  = 0,
    PAUSE_RIGHT = 1,
    TURN_LEFT   = 2,
    PAUSE_LEFT  = 3
};

class BLIMS
{
public:
    // ---- Initialization ----
    void begin(BLIMSMode mode, uint8_t pwm_pin, uint8_t enable_pin);
    void set_target(float lat, float lon);
    void set_wind_from_deg(float deg);
    void set_wind_profile(const float* altitudes_m, const float* directions_deg, int size);

    // ---- Main loop ----
    BLIMSDataOut execute(BLIMSDataIn *data_in);

private:
    // ---- State (was file-scope static in blims.cpp) ----
    Phase last_phase = Phase::HELD;
    float error_integral = 0.0f;
    LoiterStep loiter_step = LoiterStep::TURN_RIGHT;
    alarm_id_t loiter_alarm_id = -1;
    volatile bool loiter_advance_pending = false;

    // ---- Utility ----
    static float wrap360(float angle);
    static float wrap180(float angle);
    static float compute_heading_error(float desired_heading, float actual_heading);

    // ---- Navigation ----
    float calculate_bearing_to_target();
    float calculate_distance_to_target();
    float get_wind_at_altitude(float altitude_m);
    Phase determine_phase(float altitude_ft, bool gps_valid);
    float get_desired_heading(Phase phase, float bearing_to_target, float altitude_ft);

    // ---- Motor ----
    void set_motor_position(float position);

    // ---- PI controller ----
    void execute_pi_control(float desired_heading, float current_heading, float dt);

    // ---- Loiter ----
    // Alarm callback MUST be static (C function pointer), uses user_data to get 'this'
    static int64_t loiter_alarm_callback(alarm_id_t id, void *user_data);
    static uint32_t get_loiter_step_duration(LoiterStep step);
    static LoiterStep get_next_loiter_step(LoiterStep current);
    void schedule_loiter_alarm(uint32_t duration_ms);
    void cancel_loiter_alarm();
    void apply_loiter_motor_position();
    void execute_loiter();
    void reset_loiter_state();
};

#endif // BLIMS_HPP