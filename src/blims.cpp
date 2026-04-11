/**
 * @file blims.cpp
 * @brief BLiMS (Brake Line Manipulation System) - Parafoil Guidance Controller
 * 
 * This file implements the guidance algorithm for steering a parafoil to a target
 * landing location. The system uses GPS track (direction of motion) as the control
 * variable and actuates a single motor that pulls brake lines to turn the parafoil.
 * 
 * CONTROL CONCEPT:
 * - Motor position 0.5 = neutral (no brake)
 * - Motor position < 0.5 = pull left brake (turn left)
 * - Motor position > 0.5 = pull right brake (turn right)
 * - We use GPS "track" (headMot) not compass heading, because track naturally
 *   accounts for wind drift and canopy oscillation
 * 
 * FLIGHT PHASES:
 * The controller transitions through phases based on altitude (AGL):
 * 
 *   Phase::HELD     (0) - GPS invalid or system not ready, motor at neutral
 *   Phase::TRACK    (1) - Above 1000ft AND outside 400ft of target, PI control toward target
 *   Phase::DOWNWIND (2) - 1000ft to 600ft, fly with the wind (downwind heading)
 *   Phase::BASE     (3) - 600ft to 300ft, fly perpendicular to wind (crosswind)
 *   Phase::FINAL    (4) - 300ft to 100ft, fly into the wind for slow ground speed landing
 *   Phase::NEUTRAL  (5) - Below 100ft, hands off for touchdown
 *   Phase::LOITER   (6) - Above 1000ft AND within 400ft of target, spiral to lose altitude
 * 
 * LANDING PATTERN:
 * The downwind-base-final pattern is a standard aviation approach that ensures
 * the parafoil arrives at the target heading into the wind, minimizing ground
 * speed at touchdown.
 * 
 *        Wind Direction
 *             ↓
 *     ┌───────────────┐
 *     │   DOWNWIND    │  (fly with wind, away from target)
 *     │   1000-600ft  │
 *     └───────┬───────┘
 *             │
 *     ┌───────▼───────┐
 *     │     BASE      │  (turn perpendicular to wind)
 *     │   600-300ft   │
 *     └───────┬───────┘
 *             │
 *     ┌───────▼───────┐
 *     │    FINAL      │  (turn into wind, approach target)
 *     │   300-100ft   │
 *     └───────┬───────┘
 *             │
 *           TARGET
 * 
 * @author Cornell Rocketry Team
 * @date 2025
 */

#include "blims.hpp"
#include "blims_constants.hpp"
#include "blims_state.hpp"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <cmath>

// Phase and LoiterStep enums are now defined in blims.hpp

// ============================================================================
// STATE VARIABLES
// ============================================================================
// These are now BLIMS class members (see blims.hpp):
//   last_phase, error_integral, loiter_step, loiter_alarm_id, loiter_advance_pending

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Wrap an angle to the range [0, 360)
 * @param angle Input angle in degrees
 * @return Angle wrapped to [0, 360)
 */
float BLIMS::wrap360(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief Wrap an angle to the range [-180, 180)
 * @param angle Input angle in degrees
 * @return Angle wrapped to [-180, 180)
 * 
 * This is useful for computing heading errors where we want the shortest
 * turn direction (positive = turn right, negative = turn left).
 */
float BLIMS::wrap180(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f) {
        angle -= 360.0f;
    } else if (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief Calculate bearing from current position to target
 * @return Bearing in degrees [0, 360), where 0 = North, 90 = East
 * 
 * Uses flat-earth approximation which is accurate to <0.1° for distances
 * under 2km at mid-latitudes. This is well within GPS accuracy limits.
 */
float BLIMS::calculate_bearing_to_target() {
    float d_lat = blims::LV::target_lat - blims::flight::gps_lat;
    float d_lon = blims::LV::target_lon - blims::flight::gps_lon;
    
    // Correct for longitude convergence at latitude
    float lat_rad = blims::flight::gps_lat * (M_PI / 180.0f);
    float d_lon_corrected = d_lon * cosf(lat_rad);
    
    // atan2(east, north) gives bearing from north, clockwise positive
    float bearing_rad = atan2f(d_lon_corrected, d_lat);
    float bearing_deg = bearing_rad * (180.0f / M_PI);
    
    return wrap360(bearing_deg);
}

/**
 * @brief Calculate distance from current position to target
 * @return Distance in meters
 * 
 * Uses flat-earth approximation with latitude correction.
 */
float BLIMS::calculate_distance_to_target() {
    float d_lat = blims::LV::target_lat - blims::flight::gps_lat;
    float d_lon = blims::LV::target_lon - blims::flight::gps_lon;
    
    // Convert to meters (111320 m per degree latitude at equator)
    float lat_rad = blims::flight::gps_lat * (M_PI / 180.0f);
    float d_north_m = d_lat * 111320.0f;
    float d_east_m = d_lon * 111320.0f * cosf(lat_rad);
    
    return sqrtf(d_north_m * d_north_m + d_east_m * d_east_m);
}

/**
 * @brief Compute heading error (desired - actual)
 * @param desired_heading Target heading in degrees
 * @param actual_heading Current heading in degrees
 * @return Error in degrees, wrapped to [-180, 180)
 *         Positive = need to turn right, Negative = need to turn left
 */
float BLIMS::compute_heading_error(float desired_heading, float actual_heading) {
    return wrap180(desired_heading - actual_heading);
}

/**
 * @brief Interpolate wind direction at given altitude
 */
float BLIMS::get_wind_at_altitude(float altitude_m) {
    if (blims::LV::wind_profile_size == 0) {
        return blims::LV::wind_from_deg;  // Fallback to single value
    }
    
    // Clamp to profile range
    if (altitude_m <= blims::LV::wind_altitudes_m[0]) {
        return blims::LV::wind_dirs_deg[0];
    }
    if (altitude_m >= blims::LV::wind_altitudes_m[blims::LV::wind_profile_size - 1]) {
        return blims::LV::wind_dirs_deg[blims::LV::wind_profile_size - 1];
    }
    
    // Linear interpolation
    for (int i = 0; i < blims::LV::wind_profile_size - 1; i++) {
        if (altitude_m >= blims::LV::wind_altitudes_m[i] && 
            altitude_m < blims::LV::wind_altitudes_m[i + 1]) {
            float t = (altitude_m - blims::LV::wind_altitudes_m[i]) / 
                      (blims::LV::wind_altitudes_m[i + 1] - blims::LV::wind_altitudes_m[i]);
            return blims::LV::wind_dirs_deg[i] + t * (blims::LV::wind_dirs_deg[i + 1] - blims::LV::wind_dirs_deg[i]);
        }
    }
    
    return blims::LV::wind_dirs_deg[0];
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

/**
 * @brief Set the motor position with clamping to safe limits
 * @param position Desired position [0.0, 1.0] where 0.5 is neutral
 * 
 * The position is clamped to [MOTOR_MIN, MOTOR_MAX] to prevent over-actuation
 * that could damage the brake lines or cause uncontrollable turns.
 */
void BLIMS::set_motor_position(float position) {
    // Clamp to safe operating range
    if (position < motor_min) {
        position = motor_min;
    } else if (position > motor_max) {
        position = motor_max;
    }
    
    blims::flight::motor_position = position;
    
    // Convert to PWM duty cycle and send to hardware
    uint16_t five_percent_duty = wrap_cycle_count * 0.05f;
    uint16_t duty = (uint16_t)(five_percent_duty + position * five_percent_duty);

    pwm_set_chan_level(
        pwm_gpio_to_slice_num(blims::flight::blims_pwm_pin),
        pwm_gpio_to_channel(blims::flight::blims_pwm_pin),
        duty
    );
}

// ============================================================================
// PHASE DETERMINATION
// ============================================================================

/**
 * @brief Determine the current flight phase based on altitude and position
 * @param altitude_ft Current altitude in feet AGL
 * @param gps_valid Whether GPS data is valid
 * @return The appropriate Phase for current conditions
 * 
 * Phase selection logic:
 * 1. If GPS invalid → HELD
 * 2. If below 100ft → NEUTRAL (hands off for landing)
 * 3. If above 1000ft:
 *    - If within 400ft of target → LOITER (bleed altitude)
 *    - Otherwise → TRACK (fly toward target)
 * 4. If 600-1000ft → DOWNWIND
 * 5. If 300-600ft → BASE
 * 6. If 100-300ft → FINAL
 */
Phase BLIMS::determine_phase(float altitude_ft, bool gps_valid) {
    // GPS must be valid for any active control
    if (!gps_valid) {
        return Phase::HELD;
    }
    
    // Below minimum altitude - hands off for touchdown
    if (altitude_ft < alt_neutral_ft) {
        return Phase::NEUTRAL;
    }
    
    // High altitude - either loiter or track toward target
    if (altitude_ft > alt_downwind_ft) {
        float distance_ft = calculate_distance_to_target() * 3.28084f;
        
        if (distance_ft < set_radius_ft) {
            return Phase::LOITER;  // Close to target, bleed altitude
        } else {
            return Phase::TRACK;   // Far from target, fly toward it
        }
    }
    
    // Landing pattern phases based on altitude
    if (altitude_ft > alt_base_ft) {
        return Phase::DOWNWIND;
    } else if (altitude_ft > alt_final_ft) {
        return Phase::BASE;
    } else {
        return Phase::FINAL;
    }
}

/**
 * @brief Get the desired heading for the current phase
 * @param phase Current flight phase
 * @param bearing_to_target Bearing from current position to target
 * @return Desired heading in degrees [0, 360)
 */
float BLIMS::get_desired_heading(Phase phase, float bearing_to_target, float altitude_ft) {
    float altitude_m = altitude_ft / 3.28084f;
    float wind_from = get_wind_at_altitude(altitude_m);
    float wind_to = wrap360(wind_from + 180.0f);  // Direction wind is blowing TO
    
    switch (phase) {
        case Phase::TRACK:
            // Fly directly toward target
            return bearing_to_target;
            
        case Phase::DOWNWIND:
            // Fly with the wind (same direction wind is blowing)
            return wind_to;
            
        case Phase::BASE: {
            // Fly perpendicular to wind
            // Choose the crosswind direction that's a shorter turn from current heading
            float crosswind_left = wrap360(wind_from - 90.0f);
            float crosswind_right = wrap360(wind_from + 90.0f);
            
            float current_heading = blims::flight::headMot * 1e-5f;
            float error_left = fabsf(wrap180(crosswind_left - current_heading));
            float error_right = fabsf(wrap180(crosswind_right - current_heading));
            
            return (error_left < error_right) ? crosswind_left : crosswind_right;
        }
            
        case Phase::FINAL:
            // Fly into the wind (opposite direction wind is blowing)
            return wind_from;
            
        default:
            // HELD, NEUTRAL, LOITER don't use heading control
            return 0.0f;
    }
}

void BLIMS::set_wind_profile(const float* altitudes_m, const float* directions_deg, int size) {
    if (size > blims::LV::MAX_WIND_LAYERS) {
        size = blims::LV::MAX_WIND_LAYERS;
    }
    blims::LV::wind_profile_size = size;
    for (int i = 0; i < size; i++) {
        blims::LV::wind_altitudes_m[i] = altitudes_m[i];
        blims::LV::wind_dirs_deg[i] = directions_deg[i];
    }
}

// ============================================================================
// LOITER CONTROL (using add_alarm_in_ms)
// ============================================================================

/**
 * @brief Get the duration for the current loiter step
 * @param step The loiter step
 * @return Duration in milliseconds
 */
uint32_t BLIMS::get_loiter_step_duration(LoiterStep step) {
    switch (step) {
        case LoiterStep::TURN_RIGHT:
        case LoiterStep::TURN_LEFT:
            return loiter_turn_duration_ms;
        case LoiterStep::PAUSE_RIGHT:
        case LoiterStep::PAUSE_LEFT:
            return loiter_pause_duration_ms;
        default:
            return loiter_turn_duration_ms;
    }
}

/**
 * @brief Get the next loiter step in the sequence
 * @param current Current loiter step
 * @return Next loiter step
 */
LoiterStep BLIMS::get_next_loiter_step(LoiterStep current) {
    switch (current) {
        case LoiterStep::TURN_RIGHT:  return LoiterStep::PAUSE_RIGHT;
        case LoiterStep::PAUSE_RIGHT: return LoiterStep::TURN_LEFT;
        case LoiterStep::TURN_LEFT:   return LoiterStep::PAUSE_LEFT;
        case LoiterStep::PAUSE_LEFT:  return LoiterStep::TURN_RIGHT;
        default:                      return LoiterStep::TURN_RIGHT;
    }
}

/**
 * @brief Alarm callback for loiter state transitions
 * @param id Alarm ID
 * @param user_data Pointer to BLIMS instance
 * @return 0 (do not reschedule - we'll schedule the next alarm manually)
 * 
 * This callback is triggered by add_alarm_in_ms when it's time to transition
 * to the next loiter step. It sets a flag that is processed in the main loop,
 * keeping the callback itself minimal and ISR-safe.
 * 
 * Must be static (C function pointer), so we recover 'this' from user_data.
 */
int64_t BLIMS::loiter_alarm_callback(alarm_id_t id, void *user_data) {
    (void)id;
    
    // Recover BLIMS instance from user_data (passed via schedule_loiter_alarm)
    BLIMS* self = static_cast<BLIMS*>(user_data);
    
    // Set flag for main loop to process
    // We don't do the state transition here to keep the ISR minimal
    self->loiter_advance_pending = true;
    
    return 0;  // Don't reschedule automatically
}

/**
 * @brief Schedule the next loiter alarm
 * @param duration_ms Time until next state transition
 * 
 * Schedules an alarm that will set loiter_advance_pending when it fires.
 * Passes 'this' as user_data so the static callback can find the instance.
 */
void BLIMS::schedule_loiter_alarm(uint32_t duration_ms) {
    loiter_alarm_id = add_alarm_in_ms(duration_ms, loiter_alarm_callback, this, false);
}

/**
 * @brief Cancel any pending loiter alarm
 * 
 * Called when exiting loiter phase to prevent stale callbacks.
 */
void BLIMS::cancel_loiter_alarm() {
    if (loiter_alarm_id >= 0) {
        cancel_alarm(loiter_alarm_id);
        loiter_alarm_id = -1;
    }
    loiter_advance_pending = false;
}

/**
 * @brief Apply motor position for current loiter step
 * 
 * Sets the motor position based on the current loiter sub-state.
 * Must be called every control loop iteration while in loiter.
 */
void BLIMS::apply_loiter_motor_position() {
    switch (loiter_step) {
        case LoiterStep::TURN_RIGHT:
            set_motor_position(loiter_right_pos);
            break;
        case LoiterStep::TURN_LEFT:
            set_motor_position(loiter_left_pos);
            break;
        case LoiterStep::PAUSE_RIGHT:
        case LoiterStep::PAUSE_LEFT:
            set_motor_position(neutral_pos);
            break;
    }
}

/**
 * @brief Execute loiter behavior - alternating turns to bleed altitude
 * 
 * Loiter uses alarm-based timing that alternates:
 *   TURN_RIGHT (6s) → PAUSE_RIGHT (2.5s) → TURN_LEFT (6s) → PAUSE_LEFT (2.5s) → repeat
 * 
 * This creates controlled spirals/figure-8s that lose altitude without
 * drifting far from the target. The pause periods let the parafoil
 * stabilize before reversing direction.
 * 
 * IMPORTANT: This function is non-blocking. The timing is handled by
 * add_alarm_in_ms callbacks, not by polling timestamps.
 */
void BLIMS::execute_loiter() {
    // Check if alarm fired and we need to advance to next step
    if (loiter_advance_pending) {
        loiter_advance_pending = false;
        
        // Advance to next step
        loiter_step = get_next_loiter_step(loiter_step);
        
        // Schedule alarm for next transition
        schedule_loiter_alarm(get_loiter_step_duration(loiter_step));
    }
    
    // Apply motor position for current step (called every iteration)
    apply_loiter_motor_position();
}

/**
 * @brief Reset loiter state machine and start first alarm
 * 
 * Called when entering LOITER phase to ensure consistent behavior.
 */
void BLIMS::reset_loiter_state() {
    // Cancel any existing alarm
    cancel_loiter_alarm();
    
    // Reset to initial state
    loiter_step = LoiterStep::TURN_RIGHT;
    loiter_advance_pending = false;
    
    // Schedule first alarm
    schedule_loiter_alarm(get_loiter_step_duration(loiter_step));
    
    // Apply motor position immediately
    apply_loiter_motor_position();
}

// ============================================================================
// PI CONTROLLER
// ============================================================================

/**
 * @brief Execute PI heading control
 * @param desired_heading Target heading in degrees
 * @param current_heading Actual heading in degrees  
 * @param dt Time step in seconds
 * 
 * Uses a PI (Proportional-Integral) controller to compute motor position:
 *   motor = neutral + Kp * error + Ki * integral(error)
 * 
 * The error is positive when we need to turn right, negative for left.
 * The output is clamped to [MOTOR_MIN, MOTOR_MAX].
 */
void BLIMS::execute_pi_control(float desired_heading, float current_heading, float dt) {
    float error = compute_heading_error(desired_heading, current_heading);
    
    // Update integral with anti-windup (clamp to prevent runaway)
    error_integral += error * dt;
    if (error_integral > integral_max) {
        error_integral = integral_max;
    } else if (error_integral < -integral_max) {
        error_integral = -integral_max;
    }
    
    // Compute PI output
    // Negative sign because positive error (need right turn) should increase motor position
    float p_term = Kp * error; //neg * pos. = neg term (turns)
    float i_term = Ki * error_integral;

    //its actually the case that left turn needs right turn - opposite way 
    
    float motor_position = neutral_pos + p_term + i_term;
    
    // Store for telemetry
    blims::LV::pid_P = p_term;
    blims::LV::pid_I = i_term;
    
    set_motor_position(motor_position);
}

// ============================================================================
// MAIN EXECUTION
// ============================================================================

/**
 * @brief Main BLiMS execution function - called every control loop iteration
 * @param data_in Pointer to GPS and sensor data from FSW
 * @return BLIMSDataOut struct with motor position and telemetry
 * 
 * This function is called at ~20Hz by the flight software. It:
 * 1. Processes GPS data
 * 2. Determines the current flight phase
 * 3. Computes and applies the appropriate motor command
 * 4. Returns telemetry data for logging
 * 
 * IMPORTANT: This function must be non-blocking. All timing is done via
 * timestamp comparisons, not delays.
 */
BLIMSDataOut BLIMS::execute(BLIMSDataIn* data_in) {
    // Update timing
    blims::flight::prevTime = blims::flight::currTime;
    blims::flight::currTime = to_ms_since_boot(get_absolute_time());
    float dt = (blims::flight::currTime - blims::flight::prevTime) / 1000.0f;
    if (dt > 0.2f || blims::flight::prevTime == 0) dt = 0.05f;
    
    // Process GPS data
    blims::flight::gps_lat = data_in->lat * 1e-7f;
    blims::flight::gps_lon = data_in->lon * 1e-7f;
    blims::flight::gSpeed = data_in->gSpeed;
    blims::flight::headMot = data_in->headMot;
    blims::flight::fixType = data_in->fixType;
    blims::LV::gps_state = data_in->gps_state;
    
    // Get altitude from barometer (passed in from FSW)
    float altitude_ft = data_in->altitude_ft;
    
    // Check GPS validity
    bool gps_valid = blims::LV::gps_state && (blims::flight::fixType >= 2);
    
    // Determine current phase
    Phase current_phase = determine_phase(altitude_ft, gps_valid);
    
    // Detect phase changes and reset state as needed
    if (current_phase != last_phase) {
        // Reset integral on any phase change to prevent windup carryover
        error_integral = 0.0f;
        
        // Cancel loiter alarm if we're leaving loiter phase
        if (last_phase == Phase::LOITER) {
            cancel_loiter_alarm();
        }
        
        // Reset loiter state when entering loiter
        if (current_phase == Phase::LOITER) {
            reset_loiter_state();
        }
        
        last_phase = current_phase;
    }
    
    // Calculate bearing for telemetry (and for TRACK phase control)
    float bearing_to_target = calculate_bearing_to_target();
    blims::LV::bearing = bearing_to_target;
    
    // Execute phase-specific control
    switch (current_phase) {
        case Phase::HELD:
        case Phase::NEUTRAL:
            // No active control - hold neutral
            set_motor_position(neutral_pos);
            blims::LV::pid_P = 0.0f;
            blims::LV::pid_I = 0.0f;
            break;
            
        case Phase::LOITER:
            // Timed alternating turns (uses add_alarm_in_ms internally)
            execute_loiter();
            break;
            
        case Phase::TRACK:
        case Phase::DOWNWIND:
        case Phase::BASE:
        case Phase::FINAL: {
            // PI heading control
            float desired_heading = get_desired_heading(current_phase, bearing_to_target, altitude_ft);
            float current_heading = blims::flight::headMot * 1e-5f;
            execute_pi_control(desired_heading, current_heading, dt);
            break;
        }
    }
    
    // Build and return output struct
    BLIMSDataOut data_out;
    data_out.motor_position = blims::flight::motor_position;
    data_out.pid_P = blims::LV::pid_P;
    data_out.pid_I = blims::LV::pid_I;
    data_out.bearing = blims::LV::bearing;
    data_out.phase_id = static_cast<int8_t>(current_phase);
    data_out.loiter_step = static_cast<int8_t>(loiter_step);
    
    return data_out;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * @brief Initialize the BLiMS system
 * @param mode Operating mode (STANDBY, LV)
 * @param pwm_pin GPIO pin for motor PWM signal
 * @param enable_pin GPIO pin for motor enable signal
 * 
 * Sets up PWM hardware and initializes state variables.
 * Target coordinates and wind direction should be set before flight.
 */
void BLIMS::begin(BLIMSMode mode, uint8_t pwm_pin, uint8_t enable_pin) {
    blims::flight::flight_mode = mode;
    blims::flight::blims_pwm_pin = pwm_pin;
    blims::flight::blims_enable_pin = enable_pin;
    
    // Configure PWM for 50Hz (standard servo frequency)
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pwm_pin);
    float divider = 125000000.0f / (50 * wrap_cycle_count);
    pwm_set_clkdiv(slice, divider);
    pwm_set_wrap(slice, wrap_cycle_count);
    pwm_set_enabled(slice, true);
    
    // Configure enable pin
    gpio_init(enable_pin);
    gpio_set_dir(enable_pin, GPIO_OUT);
    gpio_put(enable_pin, 1);  // Enable motor driver
    
    // Initialize motor to neutral
    set_motor_position(neutral_pos);
    
    // Initialize state
    error_integral = 0.0f;
    last_phase = Phase::HELD;
    loiter_step = LoiterStep::TURN_RIGHT;
    loiter_alarm_id = -1;
    loiter_advance_pending = false;
    
    blims::flight::blims_init = true;
}

/**
 * @brief Set the wind direction
 * @param wind_from_deg Direction wind is coming FROM in degrees [0, 360)
 *                      0 = from North, 90 = from East, etc.
 * 
 * This should be set before flight based on weather data.
 * The landing pattern headings are computed relative to this.
 */
void BLIMS::set_wind_from_deg(float wind_from_deg) {
    blims::LV::wind_from_deg = wrap360(wind_from_deg);
}

/**
 * @brief Set the target landing coordinates
 * @param lat Target latitude in degrees
 * @param lon Target longitude in degrees
 */
void BLIMS::set_target(float lat, float lon) {
    blims::LV::target_lat = lat;
    blims::LV::target_lon = lon;
}