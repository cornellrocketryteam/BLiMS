/**
 * @file sim_runner.cpp
 * @brief Simulation test runner for BLiMS landing logic
 * 
 * PURPOSE:
 * Run the landing logic tests without a physical car, using simulated altitude data.
 * This allows for rapid iteration and debugging of the landing state machine.
 * 
 * USAGE:
 * g++ -std=c++17 sim_runner.cpp sim_test.cpp -o sim_runner && ./sim_runner
 * 
 * Features:
 * - Altitude-based phase transitions (1000, 600, 300, 100 ft thresholds)
 * - PI controller simulation for motor control
 * - Loiter state machine (TURN_RIGHT, PAUSE_RIGHT, TURN_LEFT, PAUSE_LEFT)
 * - CSV output for plotting phase vs altitude
 */

#include "sim_test.hpp"
#include <cstdio>
#include <cmath>

// ============================================================================
// PHASE DEFINITIONS (from blims.cpp)
// ============================================================================
enum Phase {
    HELD = 0,      // Not started
    TRACK = 1,     // Heading to target
    DOWNWIND = 2,  // 1000 ft downwind leg
    BASE = 3,      // 600 ft base leg
    FINAL = 4,     // 300 ft final approach
    NEUTRAL = 5,   // 100 ft neutral wind hold
    LOITER = 6     // Land hold pattern
};

enum LoiterStep {
    TURN_RIGHT = 0,
    PAUSE_RIGHT = 1,
    TURN_LEFT = 2,
    PAUSE_LEFT = 3
};

// ============================================================================
// ALTITUDE THRESHOLDS (in feet AGL)
// ============================================================================
const float DOWNWIND_ALT = 1000.0f;
const float BASE_ALT = 600.0f;
const float FINAL_ALT = 300.0f;
const float NEUTRAL_ALT = 100.0f;

// ============================================================================
// PI CONTROLLER PARAMETERS (from blims_constants.hpp)
// ============================================================================
const float Kp = 0.009f;
const float Ki = 0.001f;
const float integral_max = 10.0f;
const float motor_min = 0.3f;
const float motor_max = 0.7f;

// ============================================================================
// LOITER TIMING (milliseconds)
// ============================================================================
const int TURN_TIME_MS = 6000;      // 6 second turn
const int PAUSE_TIME_MS = 2500;     // 2.5 second pause

// ============================================================================
// WIND PROFILE (matches blims.cpp structure)
// ============================================================================
const int MAX_WIND_LAYERS = 20;
int wind_profile_size = 11;  // Example profile

// Altitude in meters, wind direction in degrees (coming FROM)
float wind_altitudes_m[MAX_WIND_LAYERS] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
float wind_dirs_deg[MAX_WIND_LAYERS] = {45, 52, 58, 65, 70, 75, 80, 83, 86, 88, 90};

// Single value fallback
float wind_from_deg = 45.0f;

// ============================================================================
// HELPER FUNCTIONS - Wind
// ============================================================================

float wrap360(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

float wrap180(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    else if (angle < -180.0f) angle += 360.0f;
    return angle;
}

float get_wind_at_altitude(float altitude_ft) {
    float altitude_m = altitude_ft / 3.28084f;
    
    // If no profile, use fallback
    if (wind_profile_size == 0) {
        return wind_from_deg;
    }
    
    // Clamp to profile range
    if (altitude_m <= wind_altitudes_m[0]) {
        return wind_dirs_deg[0];
    }
    if (altitude_m >= wind_altitudes_m[wind_profile_size - 1]) {
        return wind_dirs_deg[wind_profile_size - 1];
    }
    
    // Linear interpolation
    for (int i = 0; i < wind_profile_size - 1; i++) {
        if (altitude_m >= wind_altitudes_m[i] && altitude_m < wind_altitudes_m[i + 1]) {
            float t = (altitude_m - wind_altitudes_m[i]) / 
                      (wind_altitudes_m[i + 1] - wind_altitudes_m[i]);
            return wind_dirs_deg[i] + t * (wind_dirs_deg[i + 1] - wind_dirs_deg[i]);
        }
    }
    
    return wind_dirs_deg[0];  // Fallback
}

float get_desired_heading(Phase phase, float altitude_ft, float bearing_to_target) {
    float wind_from = get_wind_at_altitude(altitude_ft);
    float wind_to = wrap360(wind_from + 180.0f);
    
    switch (phase) {
        case TRACK:
            return bearing_to_target;
            
        case DOWNWIND:
            return wind_to;  // Fly with wind
            
        case BASE: {
            // Pick shorter turn to crosswind
            float crosswind_right = wrap360(wind_from + 90.0f);
            return crosswind_right;
        }
            
        case FINAL:
            return wind_from;  // Fly into wind
            
        case NEUTRAL:
        case HELD:
        case LOITER:
        default:
            return 0.0f;  // Don't care
    }
}

// ============================================================================
// HELPER FUNCTIONS - Phase & Control
// ============================================================================

/**
 * Determine landing phase based on altitude, distance, and descent state
 */
Phase determine_phase(float altitude, float distance_ft, bool is_descending) {
    // Don't run landing phases during ascent
    if (!is_descending && altitude > 50.0f) {
        return HELD;
    }
    
    if (altitude >= DOWNWIND_ALT) {  // > 1000ft
        if (distance_ft < 400.0f) {
            return LOITER;
        } else {
            return TRACK;
        }
    } else if (altitude >= BASE_ALT) {  // 600-1000ft
        return DOWNWIND;
    } else if (altitude >= FINAL_ALT) {  // 300-600ft
        return BASE;
    } else if (altitude >= NEUTRAL_ALT) {  // 100-300ft
        return FINAL;
    } else {
        return NEUTRAL;  // < 100ft
    }
}

/**
 * Simulate heading error based on phase and wind
 */
float simulate_heading_error(Phase phase, float altitude_ft, float bearing_to_target, float& current_heading) {
    // Get target heading for this phase (uses wind at current altitude)
    float target_heading = get_desired_heading(phase, altitude_ft, bearing_to_target);
    
    // For NEUTRAL/HELD/LOITER, no heading control needed
    if (phase == NEUTRAL || phase == HELD || phase == LOITER) {
        return 0.0f;
    }
    
    // Calculate error (how far we are from target)
    float error = wrap180(target_heading - current_heading);
    
    // Simulate parafoil gradually turning toward target (~5 deg/sec with brake)
    // At 50ms per sample, that's ~0.25 deg per sample
    float turn_rate = 0.25f;
    if (error > 0) {
        current_heading += fminf(turn_rate, error);
    } else {
        current_heading -= fminf(turn_rate, -error);
    }
    current_heading = wrap360(current_heading);
    
    // Return current error for PI controller
    return error;
}

/**
 * PI controller for motor control with anti-windup
 */
float compute_motor_position(float heading_error, float& p_term, float& i_term) {
    p_term = Kp * heading_error;
    
    // Calculate what motor position WOULD be
    float tentative_i = i_term + Ki * heading_error;
    float tentative_motor = 0.5f + p_term + tentative_i;
    
    // Only accumulate integral if output not saturated (anti-windup)
    if (tentative_motor >= motor_min && tentative_motor <= motor_max) {
        i_term = tentative_i;
    }
    // Still clamp integral to absolute max
    if (i_term > integral_max) i_term = integral_max;
    if (i_term < -integral_max) i_term = -integral_max;
    
    float motor_pos = 0.5f + p_term + i_term;
    if (motor_pos > motor_max) motor_pos = motor_max;
    if (motor_pos < motor_min) motor_pos = motor_min;
    return motor_pos;
}

/**
 * Get motor position for loiter step
 */
float get_loiter_motor_position(int loiter_step) {
    switch (loiter_step) {
        case TURN_RIGHT:  return 0.65f;
        case TURN_LEFT:   return 0.35f;
        case PAUSE_RIGHT:
        case PAUSE_LEFT:
        default:          return 0.5f;
    }
}

/**
 * Update loiter step based on elapsed time since LOITER entry
 */
void update_loiter(int elapsed_ms, int& loiter_step) {
    int cycle_time = 2 * (TURN_TIME_MS + PAUSE_TIME_MS);  // 17000ms total
    int cycle_pos = elapsed_ms % cycle_time;
    
    if (cycle_pos < TURN_TIME_MS) {
        loiter_step = TURN_RIGHT;
    } else if (cycle_pos < TURN_TIME_MS + PAUSE_TIME_MS) {
        loiter_step = PAUSE_RIGHT;
    } else if (cycle_pos < 2 * TURN_TIME_MS + PAUSE_TIME_MS) {
        loiter_step = TURN_LEFT;
    } else {
        loiter_step = PAUSE_LEFT;
    }
}

// ============================================================================
// MAIN SIMULATION
// ============================================================================

int main() {
    printf("=== BLiMS Landing Logic Simulator ===\n");
    printf("Using real altitude data from flight log\n");
    printf("Simulating phase transitions and control logic\n\n");
    
    SimData sim;
    FILE* csv_file = fopen("sim_output.csv", "w");
    
    if (!csv_file) {
        printf("ERROR: Could not open sim_output.csv for writing\n");
        return 1;
    }
    
    // Write CSV header
    fprintf(csv_file, "sample,timestamp_ms,altitude_ft,phase,motor_pos,heading_error_deg,p_term,i_term,loiter_step,wind_dir,current_heading,target_heading,distance_ft,is_descending\n");

    int sample = 0;
    Phase current_phase = HELD;
    float p_term = 0.0f;
    float i_term = 0.0f;
    int loiter_step = TURN_RIGHT;
    int loiter_start_sample = 0;
    
    // Heading simulation state
    float current_heading = 0.0f;
    float bearing_to_target = 45.0f;
    
    // Descent detection state - once we pass apogee, stay in descent mode
    float max_altitude_seen = 0.0f;
    bool passed_apogee = false;
    bool is_descending = false;
    const float APOGEE_THRESHOLD = 50.0f;  // Must drop 50ft below max to confirm apogee
    
    printf("Running simulation...\n");
    
    while (sim.has_more_data()) {
        float altitude = sim.get_alt();
        
        // Clamp negative values to 0
        if (altitude < 0.0f) altitude = 0.0f;
        
        // Track max altitude and detect apogee
        if (altitude > max_altitude_seen) {
            max_altitude_seen = altitude;
        }
        
        // Once we've dropped APOGEE_THRESHOLD below max, we've passed apogee
        if (!passed_apogee && max_altitude_seen > 500.0f && 
            altitude < max_altitude_seen - APOGEE_THRESHOLD) {
            passed_apogee = true;
            printf("  ** APOGEE DETECTED at %.1f ft (max was %.1f ft) **\n", 
                   altitude, max_altitude_seen);
        }
        
        is_descending = passed_apogee;
        
        // Update simulated position (drift toward target)
        sim.update_position(0.05f);  // 50ms timestep
        float distance_ft = sim.get_distance_to_target_ft();
        
        // Determine phase based on altitude, distance, and descent
        Phase new_phase = determine_phase(altitude, distance_ft, is_descending);
        
        // Handle phase transitions
        if (new_phase != current_phase) {
            printf("PHASE CHANGE: %s -> %s at altitude %.1f ft (sample %d)\n",
                   SimData::phase_name(current_phase),
                   SimData::phase_name(new_phase),
                   altitude,
                   sample);
            
            // Reset integral on ANY phase change
            i_term = 0.0f;
            
            // Reset loiter timing on LOITER entry
            if (new_phase == LOITER) {
                loiter_start_sample = sample;
                loiter_step = TURN_RIGHT;
            }
            
            current_phase = new_phase;
        }
        
        // Compute heading error based on phase
        float heading_error = simulate_heading_error(current_phase, altitude, bearing_to_target, current_heading);
        
        // Phase-specific motor control
        float motor_pos;
        switch (current_phase) {
            case NEUTRAL:
            case HELD:
                motor_pos = 0.5f;  // Hands off
                p_term = 0.0f;
                i_term = 0.0f;
                break;
            case LOITER: {
                // Update loiter step based on time since LOITER entry
                int elapsed_ms = (sample - loiter_start_sample) * 50;  // 50ms per sample
                update_loiter(elapsed_ms, loiter_step);
                motor_pos = get_loiter_motor_position(loiter_step);
                p_term = 0.0f;
                i_term = 0.0f;
                break;
            }
            default:  // TRACK, DOWNWIND, BASE, FINAL
                motor_pos = compute_motor_position(heading_error, p_term, i_term);
                break;
        }
        
        // Log state
        sim.current_state = {
            altitude,
            (int)current_phase,
            motor_pos,
            heading_error,
            p_term,
            i_term,
            loiter_step,
            sample * 50
        };
        
        // Write to CSV
        float wind_at_alt = get_wind_at_altitude(altitude);
        float target_hdg = get_desired_heading(current_phase, altitude, bearing_to_target);
        int timestamp_ms = sample * 50;

        fprintf(csv_file, "%d,%d,%.2f,%d,%.4f,%.2f,%.6f,%.6f,%d,%.1f,%.1f,%.1f,%.1f,%d\n",
            sample,
            timestamp_ms,
            altitude,
            (int)current_phase,
            motor_pos,
            heading_error,
            p_term,
            i_term,
            loiter_step,
            wind_at_alt,
            current_heading,
            target_hdg,
            distance_ft,
            is_descending ? 1 : 0);
        
        // Print key transitions
        if (sample % 500 == 0) {
            printf("  Sample %5d: Alt=%.1f ft | Phase=%s | Motor=%.3f | Dist=%.1f ft | %s\n",
                   sample, altitude, SimData::phase_name(current_phase), motor_pos, 
                   distance_ft, is_descending ? "DESC" : "ASC");
        }
        
        sample++;
    }
    
    fclose(csv_file);
    printf("\nSimulation complete!\n");
    printf("Output written to: sim_output.csv\n");
    printf("Total samples processed: %d\n", sample);
    printf("Simulation duration: %.1f seconds\n\n", (sample * 50) / 1000.0f);
    
    // Print summary
    sim.print_summary();
    
    return 0;
}