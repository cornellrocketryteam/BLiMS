/**
 * @file blims_hil_test.cpp
 * @brief Hardware-in-Loop test for BLiMS
 * 
 * PURPOSE: Test the full BLiMS stack on Pico hardware with simulated GPS data.
 * Allows testing phase transitions, loiter logic, and motor response
 * without needing to actually fly or drive.
 * 
 * HOW IT WORKS:
 *   1. Python script on laptop sends fake GPS/altitude data via USB serial
 *   2. Pico runs full BLiMS controller code
 *   3. Motor responds as if in flight
 *   4. Pico sends telemetry back to laptop for logging/visualization
 * 
 * DATA FORMAT (laptop -> Pico):
 *   HIL,<lat>,<lon>,<alt_ft>,<heading>,<speed_mms>,<fix>\n
 *   Example: HIL,42.7080,-77.1710,800.0,90,5000,3\n
 * 
 * TELEMETRY FORMAT (Pico -> laptop):
 *   TEL,<time>,<phase>,<bearing>,<error>,<motor>,<loiter_step>\n
 * 
 * BUILD:
 *   Add to CMakeLists.txt, build as normal Pico project
 * 
 * USAGE WITH PYTHON:
 *   See blims_hil_driver.py for the companion laptop script
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define PWM_PIN        10
#define ENABLE_PIN     11

// PWM settings
constexpr uint16_t WRAP_CYCLE_COUNT = 65535;

// Motor positions
constexpr float NEUTRAL_POS = 0.5f;
constexpr float MOTOR_MIN = 0.3f;
constexpr float MOTOR_MAX = 0.7f;
constexpr float LOITER_RIGHT_POS = 0.65f;
constexpr float LOITER_LEFT_POS = 0.35f;

// Controller gains
constexpr float Kp = 0.009f;
constexpr float Ki = 0.001f;

// Altitude thresholds
constexpr float ALT_UTURN_START_FT = 1000.0f;
constexpr float ALT_BASE_START_FT = 600.0f;
constexpr float ALT_FINAL_START_FT = 300.0f;
constexpr float ALT_NEUTRAL_FT = 100.0f;
constexpr float SET_RADIUS_FT = 400.0f;
constexpr float FT_PER_M = 3.28084f;

// Loiter timing
constexpr uint32_t LOITER_TURN_MS = 6000;
constexpr uint32_t LOITER_NEUTRAL_MS = 2500;

// Speed threshold
constexpr int32_t GSPEED_MIN = 3000;

// Math
constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / 3.14159265f;

// ============================================================================
// STATE
// ============================================================================

// Target (set via serial or hardcoded)
float target_lat = 42.7080f;
float target_lon = -77.1710f;
float wind_from_deg = 270.0f;  // Wind from West

// Simulated GPS input (from HIL)
struct {
    float lat;
    float lon;
    float alt_ft;
    int32_t heading;
    int32_t speed_mms;
    uint8_t fix;
    bool valid;
} hil_input;

// Controller state
struct {
    float bearing;
    float heading_des;
    float error;
    float error_integral;
    float pid_P;
    float pid_I;
    float motor_position;
    int32_t phase;
    int32_t loiter_step;
    uint32_t loiter_step_start_ms;
    int32_t last_phase;
} ctrl;

static uint32_t current_time_ms = 0;
static uint32_t last_time_ms = 0;
static bool motor_enabled = false;

// Serial buffer
static char serial_buffer[128];
static int serial_index = 0;

// ============================================================================
// UTILITIES
// ============================================================================

float wrap360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

float wrap180(float deg) {
    deg = wrap360(deg);
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
}

float compute_heading_error(float target, float current) {
    float error = target - current;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    return error;
}

float distance_m(float lat1, float lon1, float lat2, float lon2) {
    float lr = lat1 * DEG_TO_RAD;
    float x = (lon2 - lon1) * DEG_TO_RAD * cosf(lr);
    float y = (lat2 - lat1) * DEG_TO_RAD;
    return 6371000.0f * sqrtf(x * x + y * y);
}

void calculate_bearing() {
    float d_lat = (target_lat - hil_input.lat) * DEG_TO_RAD;
    float d_lon = (target_lon - hil_input.lon) * DEG_TO_RAD;
    float lat_rad = hil_input.lat * DEG_TO_RAD;
    
    float x = d_lon * cosf(lat_rad);
    float y = d_lat;
    float bearing = atan2f(x, y) * RAD_TO_DEG;
    if (bearing < 0.0f) bearing += 360.0f;
    
    ctrl.bearing = bearing;
}

// ============================================================================
// PWM / MOTOR
// ============================================================================

static uint slice_num;

void pwm_init() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    
    float divider = 125000000.0f / (50.0f * WRAP_CYCLE_COUNT);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, WRAP_CYCLE_COUNT);
    pwm_set_enabled(slice_num, true);
}

void set_motor_position(float position) {
    if (position < MOTOR_MIN) position = MOTOR_MIN;
    if (position > MOTOR_MAX) position = MOTOR_MAX;
    
    uint16_t five_percent = (uint16_t)(WRAP_CYCLE_COUNT * 0.05f);
    uint16_t duty = five_percent + (uint16_t)(position * five_percent);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), duty);
    
    ctrl.motor_position = position;
}

void enable_motor(bool enable) {
    gpio_put(ENABLE_PIN, enable ? 1 : 0);
    motor_enabled = enable;
}

// ============================================================================
// MAIN CONTROLLER (mirrors blims.cpp execute_LV)
// ============================================================================

void execute_controller() {
    // Validity gates
    if (!hil_input.valid || hil_input.fix < 2 || hil_input.speed_mms < GSPEED_MIN) {
        ctrl.phase = -1;
        set_motor_position(NEUTRAL_POS);
        return;
    }
    
    // Time delta
    float dt = (current_time_ms - last_time_ms) / 1000.0f;
    last_time_ms = current_time_ms;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.05f;
    
    // Navigation
    calculate_bearing();
    float dist_ft = distance_m(hil_input.lat, hil_input.lon, target_lat, target_lon) * FT_PER_M;
    
    float W = wrap360(wind_from_deg);
    float DW = wrap360(W + 180.0f);
    float heading_des = ctrl.bearing;
    int32_t phase = 0;
    
    float alt_ft = hil_input.alt_ft;
    
    // Phase 4: NEUTRAL
    if (alt_ft <= ALT_NEUTRAL_FT) {
        phase = 4;
        if (ctrl.last_phase != phase) {
            ctrl.error_integral = 0;
            ctrl.last_phase = phase;
        }
        ctrl.phase = phase;
        ctrl.heading_des = heading_des;
        set_motor_position(NEUTRAL_POS);
        return;
    }
    
    // High altitude logic
    if (alt_ft > ALT_UTURN_START_FT) {
        if (dist_ft <= SET_RADIUS_FT) {
            // Phase 5: LOITER
            phase = 5;
            if (ctrl.last_phase != phase) {
                ctrl.error_integral = 0;
                ctrl.loiter_step = 0;
                ctrl.loiter_step_start_ms = current_time_ms;
                ctrl.last_phase = phase;
            }
            
            uint32_t elapsed = current_time_ms - ctrl.loiter_step_start_ms;
            
            switch (ctrl.loiter_step) {
                case 0:
                    if (elapsed >= LOITER_TURN_MS) {
                        ctrl.loiter_step = 1;
                        ctrl.loiter_step_start_ms = current_time_ms;
                    }
                    break;
                case 1:
                    if (elapsed >= LOITER_NEUTRAL_MS) {
                        ctrl.loiter_step = 2;
                        ctrl.loiter_step_start_ms = current_time_ms;
                    }
                    break;
                case 2:
                    if (elapsed >= LOITER_TURN_MS) {
                        ctrl.loiter_step = 3;
                        ctrl.loiter_step_start_ms = current_time_ms;
                    }
                    break;
                case 3:
                    if (elapsed >= LOITER_NEUTRAL_MS) {
                        ctrl.loiter_step = 0;
                        ctrl.loiter_step_start_ms = current_time_ms;
                    }
                    break;
            }
            
            float pos = NEUTRAL_POS;
            if (ctrl.loiter_step == 0) pos = LOITER_RIGHT_POS;
            if (ctrl.loiter_step == 2) pos = LOITER_LEFT_POS;
            
            ctrl.phase = phase;
            ctrl.heading_des = heading_des;
            set_motor_position(pos);
            return;
        } else {
            phase = 0;
            heading_des = ctrl.bearing;
            ctrl.loiter_step = 0;
        }
    }
    // Landing pattern
    else if (alt_ft <= ALT_FINAL_START_FT) {
        phase = 3;
        heading_des = W;
    }
    else if (alt_ft <= ALT_BASE_START_FT) {
        phase = 2;
        float b1 = wrap360(DW + 90.0f);
        float b2 = wrap360(DW - 90.0f);
        float e1 = fabsf(wrap180(b1 - hil_input.heading));
        float e2 = fabsf(wrap180(b2 - hil_input.heading));
        heading_des = (e1 <= e2) ? b1 : b2;
    }
    else {
        phase = 1;
        heading_des = DW;
    }
    
    ctrl.phase = phase;
    ctrl.heading_des = heading_des;
    
    if (phase != ctrl.last_phase) {
        ctrl.error_integral = 0;
        ctrl.last_phase = phase;
    }
    
    // PI Controller
    ctrl.error = compute_heading_error(heading_des, (float)hil_input.heading);
    ctrl.error_integral += ctrl.error * dt;
    
    float limit = 0.5f / Ki;
    if (ctrl.error_integral > limit) ctrl.error_integral = limit;
    if (ctrl.error_integral < -limit) ctrl.error_integral = -limit;
    
    ctrl.pid_P = -Kp * ctrl.error;
    ctrl.pid_I = -Ki * ctrl.error_integral;
    
    float position = NEUTRAL_POS + ctrl.pid_P + ctrl.pid_I;
    set_motor_position(position);
}

// ============================================================================
// SERIAL PROTOCOL
// ============================================================================

void parse_hil_command(char* cmd) {
    // HIL,<lat>,<lon>,<alt_ft>,<heading>,<speed_mms>,<fix>
    if (strncmp(cmd, "HIL,", 4) == 0) {
        char* token = strtok(cmd + 4, ",");
        int field = 0;
        
        while (token != NULL && field < 6) {
            switch (field) {
                case 0: hil_input.lat = atof(token); break;
                case 1: hil_input.lon = atof(token); break;
                case 2: hil_input.alt_ft = atof(token); break;
                case 3: hil_input.heading = atoi(token); break;
                case 4: hil_input.speed_mms = atoi(token); break;
                case 5: hil_input.fix = atoi(token); break;
            }
            field++;
            token = strtok(NULL, ",");
        }
        
        hil_input.valid = (field == 6);
    }
    // TARGET,<lat>,<lon>
    else if (strncmp(cmd, "TARGET,", 7) == 0) {
        sscanf(cmd + 7, "%f,%f", &target_lat, &target_lon);
        printf("ACK,TARGET,%.6f,%.6f\n", target_lat, target_lon);
    }
    // WIND,<deg>
    else if (strncmp(cmd, "WIND,", 5) == 0) {
        wind_from_deg = atof(cmd + 5);
        printf("ACK,WIND,%.1f\n", wind_from_deg);
    }
    // ENABLE
    else if (strcmp(cmd, "ENABLE") == 0) {
        enable_motor(true);
        printf("ACK,ENABLE\n");
    }
    // DISABLE
    else if (strcmp(cmd, "DISABLE") == 0) {
        enable_motor(false);
        printf("ACK,DISABLE\n");
    }
    // TIME,<ms>
    else if (strncmp(cmd, "TIME,", 5) == 0) {
        current_time_ms = atoi(cmd + 5);
    }
    // STATUS
    else if (strcmp(cmd, "STATUS") == 0) {
        printf("STATUS,target=%.6f:%.6f,wind=%.1f,motor=%s\n",
               target_lat, target_lon, wind_from_deg,
               motor_enabled ? "ON" : "OFF");
    }
}

void process_serial() {
    while (true) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) break;
        
        if (c == '\n' || c == '\r') {
            if (serial_index > 0) {
                serial_buffer[serial_index] = '\0';
                parse_hil_command(serial_buffer);
                serial_index = 0;
            }
        } else if (serial_index < sizeof(serial_buffer) - 1) {
            serial_buffer[serial_index++] = (char)c;
        }
    }
}

void send_telemetry() {
    printf("TEL,%lu,%d,%.1f,%.1f,%.1f,%.3f,%d\n",
           current_time_ms,
           ctrl.phase,
           ctrl.bearing,
           ctrl.heading_des,
           ctrl.error,
           ctrl.motor_position,
           ctrl.loiter_step);
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    stdio_init_all();
    sleep_ms(1000);
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║       BLiMS HIL Test - Hardware-in-Loop Simulation       ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Protocol:\n");
    printf("  Input:  HIL,<lat>,<lon>,<alt_ft>,<heading>,<speed_mms>,<fix>\n");
    printf("  Output: TEL,<time>,<phase>,<bearing>,<des>,<error>,<motor>,<loiter>\n");
    printf("\n");
    printf("Commands: TARGET,lat,lon | WIND,deg | ENABLE | DISABLE | STATUS\n");
    printf("\n");
    printf("READY\n");
    
    // Initialize hardware
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 0);
    
    pwm_init();
    set_motor_position(NEUTRAL_POS);
    
    // Initialize state
    memset(&hil_input, 0, sizeof(hil_input));
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.last_phase = -99;
    
    // Main loop
    uint32_t last_telemetry_ms = 0;
    
    while (true) {
        // Process incoming serial commands
        process_serial();
        
        // Run controller if we have valid HIL data
        if (hil_input.valid && motor_enabled) {
            execute_controller();
        }
        
        // Send telemetry at 20 Hz
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_telemetry_ms >= 50) {
            send_telemetry();
            last_telemetry_ms = now;
        }
        
        sleep_ms(1);
    }
    
    return 0;
}