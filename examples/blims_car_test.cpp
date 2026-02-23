/**
 * @file l3_car_test.cpp
 * @brief Car test for BLiMS L3 landing pattern logic
 * 
 * PURPOSE:
 * Test the full L3 landing pattern logic (phase transitions, PI control, loiter)
 * while driving. Altitude is SIMULATED to trigger phase changes.
 * 
 * TEST MODES:
 * 1. MANUAL_ALTITUDE - Use buttons to manually set altitude (recommended)
 * 2. AUTO_DESCENT    - Altitude decreases automatically over time
 * 3. FIXED_PHASE     - Lock to a specific phase for isolated testing
 * 
 * HARDWARE:
 * - Raspberry Pi Pico
 * - SparkFun GPS (MAX-M10S) on I2C0
 * - ODrive motor via PWM
 * - Optional: 2 buttons for altitude control
 * 
 * WIRING:
 * - GPS SDA  -> GPIO 12
 * - GPS SCL  -> GPIO 13
 * - PWM      -> GPIO 27
 * - Enable   -> GPIO 28
 * - BTN_UP   -> GPIO 14 (optional, for manual altitude)
 * - BTN_DOWN -> GPIO 15 (optional, for manual altitude)
 * 
 * OUTPUT FORMAT (CSV for visualization):
 * lat,lon,target_lat,target_lon,heading,bearing,motor_pos,timestamp,P,I,phase,altitude,loiter_step
 */

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "ublox_mx.hpp"
#include "ublox_nav_pvt.hpp"

#include <cmath>
#include <cstdio>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define PWM_PIN 27
#define ODRIVE_STATE_PIN 28
#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13

// Optional buttons for manual altitude control - NOT USED for now***
#define BTN_UP 14      // Press to increase altitude - 
#define BTN_DOWN 15    // Press to decrease altitude
#define USE_BUTTONS 1  // Set to 0 if no buttons connected

// ============================================================================
// TEST CONFIGURATION
// ============================================================================

// Test modes
enum TestMode {
    MANUAL_ALTITUDE,  // Use buttons to change altitude
    AUTO_DESCENT,     // Altitude decreases over time
    FIXED_PHASE       // Lock to specific phase (set FIXED_PHASE_ID below)
};

// === CHANGE THESE FOR YOUR TEST ===
const TestMode TEST_MODE = AUTO_DESCENT;
const int FIXED_PHASE_ID = 1;  // Only used if TEST_MODE == FIXED_PHASE (1=TRACK)

// Target coordinates (SET THESE BEFORE TEST)
const double TARGET_LAT = 42.446610;  // Update for your test location
const double TARGET_LON = -76.461304;

// ============================================================================
// WIND PROFILE (copy values from blims_wind_profile.json)
// ============================================================================
// Altitude in meters, wind direction in degrees (coming FROM)
// Update these arrays with real data before flight

const int WIND_PROFILE_SIZE = 11;
const float WIND_ALTITUDES_M[] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
const float WIND_DIRS_DEG[] = {45, 52, 58, 65, 70, 75, 80, 83, 86, 88, 90};  // Example profile

// Interpolate wind direction at given altitude
float get_wind_at_altitude(float altitude_m) {
    // Clamp to profile range
    if (altitude_m <= WIND_ALTITUDES_M[0]) {
        return WIND_DIRS_DEG[0];
    }
    if (altitude_m >= WIND_ALTITUDES_M[WIND_PROFILE_SIZE - 1]) {
        return WIND_DIRS_DEG[WIND_PROFILE_SIZE - 1];
    }
    
    // Find bracketing indices
    for (int i = 0; i < WIND_PROFILE_SIZE - 1; i++) {
        if (altitude_m >= WIND_ALTITUDES_M[i] && altitude_m < WIND_ALTITUDES_M[i + 1]) {
            // Linear interpolation
            float t = (altitude_m - WIND_ALTITUDES_M[i]) / 
                      (WIND_ALTITUDES_M[i + 1] - WIND_ALTITUDES_M[i]);
            return WIND_DIRS_DEG[i] + t * (WIND_DIRS_DEG[i + 1] - WIND_DIRS_DEG[i]);
        }
    }
    
    return WIND_DIRS_DEG[0];  // Fallback
}

// Auto descent config
const float START_ALTITUDE_FT = 1500.0f;
const float DESCENT_RATE_FT_PER_SEC = 20.0f;  // Simulated descent rate
const float MIN_ALTITUDE_FT = 0.0f;

// Manual altitude config
const float ALTITUDE_STEP_FT = 100.0f;  // How much each button press changes altitude

// ============================================================================
// CONSTANTS (from blims_constants.hpp)
// ============================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

const float DEG_TO_RAD = M_PI / 180.0f;
const float RAD_TO_DEG = 180.0f / M_PI;

const uint16_t WRAP_CYCLE_COUNT = 65535;

const float NEUTRAL_POS = 0.5f;
const float MOTOR_MIN = 0.3f;
const float MOTOR_MAX = 0.7f;

const float Kp = 0.009f;
const float Ki = 0.001f;
const float INTEGRAL_MAX = 50.0f;

// Altitude thresholds (feet)
const float ALT_DOWNWIND_FT = 1000.0f;
const float ALT_BASE_FT = 600.0f;
const float ALT_FINAL_FT = 300.0f;
const float ALT_NEUTRAL_FT = 100.0f;

// Loiter config
const float SET_RADIUS_FT = 400.0f;
const uint32_t LOITER_TURN_DURATION_MS = 6000;
const uint32_t LOITER_PAUSE_DURATION_MS = 2500;
const float LOITER_RIGHT_POS = 0.65f;
const float LOITER_LEFT_POS = 0.35f;

// ============================================================================
// PHASE & LOITER ENUMS
// ============================================================================

enum Phase {
    PHASE_HELD     = 0,
    PHASE_TRACK    = 1,
    PHASE_DOWNWIND = 2,
    PHASE_BASE     = 3,
    PHASE_FINAL    = 4,
    PHASE_NEUTRAL  = 5,
    PHASE_LOITER   = 6
};

enum LoiterStep {
    LOITER_TURN_RIGHT  = 0,
    LOITER_PAUSE_RIGHT = 1,
    LOITER_TURN_LEFT   = 2,
    LOITER_PAUSE_LEFT  = 3
};

const char* phase_names[] = {
    "HELD", "TRACK", "DOWNWIND", "BASE", "FINAL", "NEUTRAL", "LOITER"
};

// ============================================================================
// GLOBAL STATE
// ============================================================================

GNSS gps(I2C_PORT);

float simulated_altitude_ft = START_ALTITUDE_FT;
float error_integral = 0.0f;
Phase current_phase = PHASE_HELD;
Phase last_phase = PHASE_HELD;

// Loiter state
LoiterStep loiter_step = LOITER_TURN_RIGHT;
alarm_id_t loiter_alarm_id = -1;
volatile bool loiter_advance_pending = false;

// Current GPS data
float current_lat = 0.0f;
float current_lon = 0.0f;
float current_heading = 0.0f;

// Telemetry
float pid_P = 0.0f;
float pid_I = 0.0f;
float bearing = 0.0f;

// ============================================================================
// UTILITY FUNCTIONS
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

float compute_heading_error(float target, float current) {
    return wrap180(target - current);
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setup_pwm_50hz(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    gpio_init(ODRIVE_STATE_PIN);
    gpio_set_dir(ODRIVE_STATE_PIN, GPIO_OUT);

    float divider = 125000000.0f / (50 * WRAP_CYCLE_COUNT);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, WRAP_CYCLE_COUNT);
    pwm_set_enabled(slice_num, true);
}

void set_motor_position(float position) {
    // Clamp
    if (position < MOTOR_MIN) position = MOTOR_MIN;
    if (position > MOTOR_MAX) position = MOTOR_MAX;
    
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    uint16_t five_percent_duty = WRAP_CYCLE_COUNT * 0.05f;
    uint16_t duty = (uint16_t)(five_percent_duty + position * five_percent_duty);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), duty);
}

// ============================================================================
// NAVIGATION FUNCTIONS
// ============================================================================

float compute_bearing_to_target() {
    float d_lat = TARGET_LAT - current_lat;
    float d_lon = TARGET_LON - current_lon;
    
    float lat_rad = current_lat * DEG_TO_RAD;
    float d_lon_corrected = d_lon * cosf(lat_rad);
    
    float bearing_rad = atan2f(d_lon_corrected, d_lat);
    float bearing_deg = bearing_rad * RAD_TO_DEG;
    
    return wrap360(bearing_deg);
}

float compute_distance_to_target_m() {
    float d_lat = TARGET_LAT - current_lat;
    float d_lon = TARGET_LON - current_lon;
    
    float lat_rad = current_lat * DEG_TO_RAD;
    float d_north_m = d_lat * 111320.0f;
    float d_east_m = d_lon * 111320.0f * cosf(lat_rad);
    
    return sqrtf(d_north_m * d_north_m + d_east_m * d_east_m);
}

// ============================================================================
// PHASE DETERMINATION
// ============================================================================

Phase determine_phase(float altitude_ft, bool gps_valid) {
    if (!gps_valid) {
        return PHASE_HELD;
    }
    
    if (altitude_ft < ALT_NEUTRAL_FT) {
        return PHASE_NEUTRAL;
    }
    
    if (altitude_ft > ALT_DOWNWIND_FT) {
        float distance_ft = compute_distance_to_target_m() * 3.28084f;
        if (distance_ft < SET_RADIUS_FT) {
            return PHASE_LOITER;
        } else {
            return PHASE_TRACK;
        }
    }
    
    if (altitude_ft > ALT_BASE_FT) {
        return PHASE_DOWNWIND;
    } else if (altitude_ft > ALT_FINAL_FT) {
        return PHASE_BASE;
    } else {
        return PHASE_FINAL;
    }
}

float get_desired_heading(Phase phase) {
    // Get wind direction at current simulated altitude
    float altitude_m = simulated_altitude_ft / 3.28084f;  // Convert ft to m
    float wind_from = get_wind_at_altitude(altitude_m);
    float wind_to = wrap360(wind_from + 180.0f);
    
    switch (phase) {
        case PHASE_TRACK:
            return bearing;
            
        case PHASE_DOWNWIND:
            return wind_to;
            
        case PHASE_BASE: {
            float crosswind_left = wrap360(wind_from - 90.0f);
            float crosswind_right = wrap360(wind_from + 90.0f);
            float err_left = fabsf(wrap180(crosswind_left - current_heading));
            float err_right = fabsf(wrap180(crosswind_right - current_heading));
            return (err_left < err_right) ? crosswind_left : crosswind_right;
        }
            
        case PHASE_FINAL:
            return wind_from;
            
        default:
            return 0.0f;
    }
}

// ============================================================================
// LOITER CONTROL
// ============================================================================

int64_t loiter_alarm_callback(alarm_id_t id, void *user_data) {
    (void)id;
    (void)user_data;
    loiter_advance_pending = true;
    return 0;
}

uint32_t get_loiter_step_duration(LoiterStep step) {
    if (step == LOITER_TURN_RIGHT || step == LOITER_TURN_LEFT) {
        return LOITER_TURN_DURATION_MS;
    }
    return LOITER_PAUSE_DURATION_MS;
}

LoiterStep get_next_loiter_step(LoiterStep current) {
    switch (current) {
        case LOITER_TURN_RIGHT:  return LOITER_PAUSE_RIGHT;
        case LOITER_PAUSE_RIGHT: return LOITER_TURN_LEFT;
        case LOITER_TURN_LEFT:   return LOITER_PAUSE_LEFT;
        case LOITER_PAUSE_LEFT:  return LOITER_TURN_RIGHT;
        default:                 return LOITER_TURN_RIGHT;
    }
}

float get_loiter_motor_position(LoiterStep step) {
    switch (step) {
        case LOITER_TURN_RIGHT: return LOITER_RIGHT_POS;
        case LOITER_TURN_LEFT:  return LOITER_LEFT_POS;
        default:                return NEUTRAL_POS;
    }
}

void reset_loiter_state() {
    if (loiter_alarm_id >= 0) {
        cancel_alarm(loiter_alarm_id);
    }
    loiter_step = LOITER_TURN_RIGHT;
    loiter_advance_pending = false;
    loiter_alarm_id = add_alarm_in_ms(get_loiter_step_duration(loiter_step),
                                       loiter_alarm_callback, NULL, false);
}

void execute_loiter() {
    if (loiter_advance_pending) {
        loiter_advance_pending = false;
        loiter_step = get_next_loiter_step(loiter_step);
        loiter_alarm_id = add_alarm_in_ms(get_loiter_step_duration(loiter_step),
                                           loiter_alarm_callback, NULL, false);
    }
    set_motor_position(get_loiter_motor_position(loiter_step));
}

void cancel_loiter_alarm() {
    if (loiter_alarm_id >= 0) {
        cancel_alarm(loiter_alarm_id);
        loiter_alarm_id = -1;
    }
    loiter_advance_pending = false;
}

// ============================================================================
// PI CONTROLLER
// ============================================================================

void execute_pi_control(float desired_heading, float dt) {
    float error = compute_heading_error(desired_heading, current_heading);
    
    error_integral += error * dt;
    if (error_integral > INTEGRAL_MAX) error_integral = INTEGRAL_MAX;
    if (error_integral < -INTEGRAL_MAX) error_integral = -INTEGRAL_MAX;
    
    pid_P = -Kp * error;
    pid_I = -Ki * error_integral;
    
    float position = NEUTRAL_POS + pid_P + pid_I;
    set_motor_position(position);
}

// ============================================================================
// ALTITUDE SIMULATION
// ============================================================================

void update_simulated_altitude(float dt) {
    if (TEST_MODE == AUTO_DESCENT) {
        simulated_altitude_ft -= DESCENT_RATE_FT_PER_SEC * dt;
        if (simulated_altitude_ft < MIN_ALTITUDE_FT) {
            simulated_altitude_ft = MIN_ALTITUDE_FT;
        }
    }
}

#if USE_BUTTONS
void setup_buttons() {
    gpio_init(BTN_UP);
    gpio_init(BTN_DOWN);
    gpio_set_dir(BTN_UP, GPIO_IN);
    gpio_set_dir(BTN_DOWN, GPIO_IN);
    gpio_pull_up(BTN_UP);
    gpio_pull_up(BTN_DOWN);
}

void check_buttons() {
    static bool btn_up_last = true;
    static bool btn_down_last = true;
    
    bool btn_up = gpio_get(BTN_UP);
    bool btn_down = gpio_get(BTN_DOWN);
    
    // Detect falling edge (button press, active low with pull-up)
    if (!btn_up && btn_up_last) {
        simulated_altitude_ft += ALTITUDE_STEP_FT;
        if (simulated_altitude_ft > 2000.0f) simulated_altitude_ft = 2000.0f;
        printf("# Altitude UP: %.0f ft\n", simulated_altitude_ft);
    }
    if (!btn_down && btn_down_last) {
        simulated_altitude_ft -= ALTITUDE_STEP_FT;
        if (simulated_altitude_ft < 0.0f) simulated_altitude_ft = 0.0f;
        printf("# Altitude DOWN: %.0f ft\n", simulated_altitude_ft);
    }
    
    btn_up_last = btn_up;
    btn_down_last = btn_down;
}
#endif

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================

void execute_control(float dt, bool gps_valid) {
    // Update bearing
    bearing = compute_bearing_to_target();
    
    // Determine phase (or use fixed phase for testing)
    if (TEST_MODE == FIXED_PHASE) {
        current_phase = (Phase)FIXED_PHASE_ID;
    } else {
        current_phase = determine_phase(simulated_altitude_ft, gps_valid);
    }
    
    // Handle phase transitions
    if (current_phase != last_phase) {
        printf("# Phase change: %s -> %s (alt=%.0f ft)\n", 
               phase_names[last_phase], phase_names[current_phase], simulated_altitude_ft);
        
        error_integral = 0.0f;
        
        if (last_phase == PHASE_LOITER) {
            cancel_loiter_alarm();
        }
        if (current_phase == PHASE_LOITER) {
            reset_loiter_state();
        }
        
        last_phase = current_phase;
    }
    
    // Execute phase-specific control
    switch (current_phase) {
        case PHASE_HELD:
        case PHASE_NEUTRAL:
            set_motor_position(NEUTRAL_POS);
            pid_P = 0.0f;
            pid_I = 0.0f;
            break;
            
        case PHASE_LOITER:
            execute_loiter();
            break;
            
        case PHASE_TRACK:
        case PHASE_DOWNWIND:
        case PHASE_BASE:
        case PHASE_FINAL: {
            float desired = get_desired_heading(current_phase);
            execute_pi_control(desired, dt);
            break;
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    setup_pwm_50hz(PWM_PIN);
    
#if USE_BUTTONS
    setup_buttons();
#endif
    
    // Setup I2C for GPS
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Wait for serial connection
    while (!tud_cdc_connected()) {
        sleep_ms(500);
    }
    
    printf("# ==========================================\n");
    printf("# BLiMS L3 Car Test\n");
    printf("# ==========================================\n");
    printf("# Test Mode: %s\n", 
           TEST_MODE == MANUAL_ALTITUDE ? "MANUAL_ALTITUDE" :
           TEST_MODE == AUTO_DESCENT ? "AUTO_DESCENT" : "FIXED_PHASE");
    printf("# Target: %.6f, %.6f\n", TARGET_LAT, TARGET_LON);
    printf("# Wind profile: %d layers loaded\n", WIND_PROFILE_SIZE);
    printf("# Start altitude: %.0f ft\n", START_ALTITUDE_FT);
    printf("# ==========================================\n");
    printf("# CSV: lat,lon,target_lat,target_lon,heading,bearing,motor,timestamp,P,I,phase,altitude,loiter_step\n");
    printf("# ==========================================\n");
    
    if (!gps.begin_PVT(20)) {
        printf("# ERROR: Failed to init GPS\n");
        return 1;
    }
    printf("# GPS initialized\n");
    
    gpio_put(ODRIVE_STATE_PIN, 1);  // Enable motor
    
    UbxNavPvt data = {0};
    absolute_time_t last_time = get_absolute_time();
    
    while (true) {
#if USE_BUTTONS
        check_buttons();
#endif
        
        if (gps.read_PVT_data(&data)) {
            current_lat = data.lat * 1e-7f;
            current_lon = data.lon * 1e-7f;
            
            // Convert heading: u-blox headMot to polar coords
            current_heading = ((data.headMot * 1e-5f) - 90.0f) * -1.0f;
            if (current_heading < 0) current_heading += 360.0f;
            current_heading = wrap360(current_heading);
            
            float ground_speed = data.gSpeed / 1000.0f;
            bool gps_valid = (data.fixType >= 2) && (ground_speed > 0.3f);
            
            // Calculate dt
            absolute_time_t now = get_absolute_time();
            float dt_ms = to_ms_since_boot(now) - to_ms_since_boot(last_time);
            last_time = now;
            if (dt_ms <= 0 || dt_ms > 1000) dt_ms = 100;
            float dt = dt_ms / 1000.0f;
            
            // Update simulated altitude
            update_simulated_altitude(dt);
            
            if (gps_valid) {
                // Run control
                execute_control(dt, true);
                
                // Get motor position for logging
                uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
                // (can't easily read back position, so track it separately if needed)
                float motor_pos = NEUTRAL_POS + pid_P + pid_I;  // Approximation
                if (current_phase == PHASE_LOITER) {
                    motor_pos = get_loiter_motor_position(loiter_step);
                }
                if (motor_pos < MOTOR_MIN) motor_pos = MOTOR_MIN;
                if (motor_pos > MOTOR_MAX) motor_pos = MOTOR_MAX;
                
                // Log CSV data
                printf("%.7f,%.7f,%.7f,%.7f,%.2f,%.2f,%.3f,%llu,%.4f,%.4f,%d,%.0f,%d\n",
                       current_lat, current_lon,
                       TARGET_LAT, TARGET_LON,
                       current_heading, bearing,
                       motor_pos,
                       to_ms_since_boot(now),
                       pid_P, pid_I,
                       (int)current_phase,
                       simulated_altitude_ft,
                       (int)loiter_step);
            } else {
                set_motor_position(NEUTRAL_POS);
                printf("# Waiting for valid fix (Fix: %d, Speed: %.2f m/s)\n", 
                       data.fixType, ground_speed);
            }
        }
        
        sleep_ms(50);  // 20 Hz loop
    }
    
    return 0;
}