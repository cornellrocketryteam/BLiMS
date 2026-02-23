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
// HELPER FUNCTIONS
// ============================================================================

/**
 * Determine landing phase based on altitude
 */
Phase determine_phase(float altitude) {
    if (altitude >= DOWNWIND_ALT) {
        return TRACK;
    } else if (altitude >= BASE_ALT) {
        return DOWNWIND;
    } else if (altitude >= FINAL_ALT) {
        return BASE;
    } else if (altitude >= NEUTRAL_ALT) {
        return FINAL;
    } else if (altitude > 0.0f) {
        return NEUTRAL;
    } else {
        return LOITER;
    }
}

/**
 * Simulate heading error (would normally come from GPS/compass)
 * For testing: oscillate between -30 and +30 degrees
 */
float simulate_heading_error(int timestamp_ms) {
    return 30.0f * sin(timestamp_ms / 1000.0f);
}

/**
 * PI controller for motor control
 * Maps heading error to motor position
 */
float compute_motor_position(float heading_error, float& p_term, float& i_term) {
    // Proportional term
    p_term = Kp * heading_error;
    
    // Integral term with clamping
    i_term += Ki * heading_error;
    if (i_term > integral_max) i_term = integral_max;
    if (i_term < -integral_max) i_term = -integral_max;
    
    // Total control signal
    float motor_pos = 0.5f + (p_term + i_term);  // 0.5 is center
    
    // Clamp to motor limits
    if (motor_pos > motor_max) motor_pos = motor_max;
    if (motor_pos < motor_min) motor_pos = motor_min;
    
    return motor_pos;
}

/**
 * Update loiter step based on elapsed time
 */
void update_loiter(int elapsed_ms, int& loiter_step) {
    int cycle_pos = elapsed_ms % (2 * (TURN_TIME_MS + PAUSE_TIME_MS));
    
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
    fprintf(csv_file, "sample,timestamp_ms,altitude_ft,phase,motor_pos,heading_error_deg,p_term,i_term,loiter_step\n");
    
    int sample = 0;
    Phase current_phase = HELD;
    Phase prev_phase = HELD;
    float p_term = 0.0f;
    float i_term = 0.0f;
    int loiter_step = TURN_RIGHT;
    int phase_entry_time = 0;
    
    printf("Running simulation...\n");
    
    while (sim.has_more_data()) {
        float altitude = sim.get_alt();
        
        // Convert to feet (data appears to be in feet already based on car test)
        // Clamp negative values to 0
        if (altitude < 0.0f) altitude = 0.0f;
        
        // Determine phase based on altitude
        Phase new_phase = determine_phase(altitude);
        if (new_phase != current_phase) {
            printf("PHASE CHANGE: %s -> %s at altitude %.1f ft (sample %d)\n",
                   SimData::phase_name(current_phase),
                   SimData::phase_name(new_phase),
                   altitude,
                   sample);
            current_phase = new_phase;
            phase_entry_time = sample;
            if (current_phase == LOITER) {
                i_term = 0.0f;  // Reset integral on landing
            }
        }
        
        // Simulate heading error and compute control
        float heading_error = simulate_heading_error(sample * 10);  // 10ms per sample
        float motor_pos = compute_motor_position(heading_error, p_term, i_term);
        
        // Update loiter step if in loiter
        if (current_phase == LOITER) {
            int loiter_elapsed = sample - phase_entry_time;
            update_loiter(loiter_elapsed * 10, loiter_step);
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
            sample * 10
        };
        
        // Write to CSV
        fprintf(csv_file, "%d,%d,%.2f,%d,%.4f,%.2f,%.6f,%.6f,%d\n",
                sample,
                sample * 10,
                altitude,
                (int)current_phase,
                motor_pos,
                heading_error,
                p_term,
                i_term,
                loiter_step);
        
        // Print key transitions
        if (sample % 500 == 0) {
            printf("  Sample %5d: Alt=%.1f ft | Phase=%s | Motor=%.3f\n",
                   sample, altitude, SimData::phase_name(current_phase), motor_pos);
        }
        
        sample++;
    }
    
    fclose(csv_file);
    printf("\nSimulation complete!\n");
    printf("Output written to: sim_output.csv\n");
    printf("Total samples processed: %d\n", sample);
    printf("Simulation duration: %.1f seconds\n\n", (sample * 10) / 1000.0f);
    
    // Print summary
    sim.print_summary();
    
    return 0;
}
