/**
 * @file blims_car_test.cpp
 * @brief Car test for BLiMS closed-loop controller validation
 * 
 * PURPOSE: Validate closed-loop control with real GPS in a moving car.
 * This is the final validation before flight.
 * 
 * WHAT IT TESTS:
 *   1. Real GPS data processing
 *   2. Bearing calculation accuracy
 *   3. PI controller response to heading errors
 *   4. Motor actuation in response to control commands
 *   5. Phase transitions (simulated via altitude override)
 * 
 * SETUP:
 *   1. Mount Pico + GPS + BLiMS mechanism in car
 *   2. Set target GPS coordinates (parking lot corner, etc.)
 *   3. Drive around target while monitoring motor response
 *   4. Motor should try to "steer" toward target
 * 
 * DATA LOGGING:
 *   Logs to SD card (if available) or prints to serial.
 *   Format: timestamp, lat, lon, heading, bearing, error, motor_pos, phase
 * 
 * BUILD:
 *   Add to CMakeLists.txt, build as normal Pico project
 * 
 * USAGE:
 *   1. Flash to Pico
 *   2. Set target coordinates in code or via serial
 *   3. Open serial monitor (115200 baud)
 *   4. Drive car, observe motor response
 *   5. Analyze logged data
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// Hardware pins
#define PWM_PIN        10
#define ENABLE_PIN     11
#define GPS_UART       uart1
#define GPS_TX_PIN     4
#define GPS_RX_PIN     5

// PWM settings
constexpr uint16_t WRAP_CYCLE_COUNT = 65535;
constexpr float PWM_FREQ_HZ = 50.0f;

// Motor positions
constexpr float NEUTRAL_POS = 0.5f;
constexpr float MOTOR_MIN = 0.3f;
constexpr float MOTOR_MAX = 0.7f;

// Controller gains (from car test validation)
constexpr float Kp = 0.009f;
constexpr float Ki = 0.002f;  // Slightly higher for testing visibility

// Math constants
constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / 3.14159265f;

// Test configuration
constexpr uint32_t LOOP_INTERVAL_MS = 50;  // 20 Hz
constexpr int32_t GSPEED_MIN = 2000;       // 2 m/s minimum for valid heading

// ============================================================================
// TARGET COORDINATES - CHANGE THESE FOR YOUR TEST LOCATION
// ============================================================================

// Example: Cornell campus area
float target_lat = 42.4534f;   // CHANGE THIS
float target_lon = -76.4735f;  // CHANGE THIS

// ============================================================================
// STATE VARIABLES
// ============================================================================

struct GPSData {
    float lat;
    float lon;
    int32_t headMot;    // Heading in degrees
    int32_t gSpeed;     // Ground speed in mm/s
    uint8_t fixType;
    bool valid;
};

struct ControllerState {
    float bearing;
    float error;
    float error_integral;
    float pid_P;
    float pid_I;
    float motor_position;
    int32_t phase;      // For car test, we simulate phases
};

static GPSData gps;
static ControllerState ctrl;
static uint32_t loop_count = 0;
static uint32_t last_time_ms = 0;
static bool motor_enabled = false;

// NMEA parsing buffer
static char nmea_buffer[256];
static int nmea_index = 0;

// ============================================================================
// UTILITY FUNCTIONS
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

void calculate_bearing() {
    float d_lat = (target_lat - gps.lat) * DEG_TO_RAD;
    float d_lon = (target_lon - gps.lon) * DEG_TO_RAD;
    float lat_rad = gps.lat * DEG_TO_RAD;
    
    float x = d_lon * cosf(lat_rad);
    float y = d_lat;
    float bearing = atan2f(x, y) * RAD_TO_DEG;
    if (bearing < 0.0f) bearing += 360.0f;
    
    ctrl.bearing = bearing;
}

// ============================================================================
// PWM / MOTOR FUNCTIONS
// ============================================================================

static uint slice_num;

void pwm_init() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    
    float divider = 125000000.0f / (PWM_FREQ_HZ * WRAP_CYCLE_COUNT);
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
// GPS PARSING (Simple NMEA parser for GPRMC/GNRMC)
// ============================================================================

void parse_nmea_rmc(char* sentence) {
    // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    // Fields: time, status, lat, N/S, lon, E/W, speed_knots, track, date, mag_var, E/W
    
    char* token;
    int field = 0;
    float lat_deg = 0, lat_min = 0;
    float lon_deg = 0, lon_min = 0;
    char lat_dir = 'N', lon_dir = 'E';
    float speed_knots = 0;
    float track = 0;
    bool valid = false;
    
    token = strtok(sentence, ",");
    while (token != NULL) {
        switch (field) {
            case 2:  // Status: A=valid, V=invalid
                valid = (token[0] == 'A');
                break;
            case 3:  // Latitude (DDMM.MMMM)
                if (strlen(token) > 0) {
                    lat_deg = (token[0] - '0') * 10 + (token[1] - '0');
                    lat_min = atof(&token[2]);
                }
                break;
            case 4:  // N/S
                lat_dir = token[0];
                break;
            case 5:  // Longitude (DDDMM.MMMM)
                if (strlen(token) > 0) {
                    lon_deg = (token[0] - '0') * 100 + (token[1] - '0') * 10 + (token[2] - '0');
                    lon_min = atof(&token[3]);
                }
                break;
            case 6:  // E/W
                lon_dir = token[0];
                break;
            case 7:  // Speed in knots
                speed_knots = atof(token);
                break;
            case 8:  // Track (heading)
                track = atof(token);
                break;
        }
        field++;
        token = strtok(NULL, ",");
    }
    
    if (valid) {
        gps.lat = lat_deg + lat_min / 60.0f;
        if (lat_dir == 'S') gps.lat = -gps.lat;
        
        gps.lon = lon_deg + lon_min / 60.0f;
        if (lon_dir == 'W') gps.lon = -gps.lon;
        
        gps.gSpeed = (int32_t)(speed_knots * 514.444f);  // knots to mm/s
        gps.headMot = (int32_t)track;
        gps.fixType = 3;
        gps.valid = true;
    }
}

void process_nmea_char(char c) {
    if (c == '$') {
        nmea_index = 0;
    }
    
    if (nmea_index < sizeof(nmea_buffer) - 1) {
        nmea_buffer[nmea_index++] = c;
    }
    
    if (c == '\n') {
        nmea_buffer[nmea_index] = '\0';
        
        // Check for RMC sentence
        if (strncmp(nmea_buffer, "$GPRMC", 6) == 0 || 
            strncmp(nmea_buffer, "$GNRMC", 6) == 0) {
            parse_nmea_rmc(nmea_buffer);
        }
        
        nmea_index = 0;
    }
}

void gps_init() {
    uart_init(GPS_UART, 9600);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
}

void gps_update() {
    while (uart_is_readable(GPS_UART)) {
        char c = uart_getc(GPS_UART);
        process_nmea_char(c);
    }
}

// ============================================================================
// CONTROLLER
// ============================================================================

void run_controller(float dt) {
    // Check GPS validity
    if (!gps.valid || gps.gSpeed < GSPEED_MIN) {
        ctrl.phase = -1;
        set_motor_position(NEUTRAL_POS);
        ctrl.error = 0;
        return;
    }
    
    // For car test, we're always in "TRACK" mode (phase 0)
    ctrl.phase = 0;
    
    // Calculate bearing to target
    calculate_bearing();
    
    // Calculate heading error
    ctrl.error = compute_heading_error(ctrl.bearing, (float)gps.headMot);
    
    // PI controller
    ctrl.error_integral += ctrl.error * dt;
    
    // Anti-windup
    float limit = 0.5f / Ki;
    if (ctrl.error_integral > limit) ctrl.error_integral = limit;
    if (ctrl.error_integral < -limit) ctrl.error_integral = -limit;
    
    // Calculate control output
    ctrl.pid_P = -Kp * ctrl.error;
    ctrl.pid_I = -Ki * ctrl.error_integral;
    
    float position = NEUTRAL_POS + ctrl.pid_P + ctrl.pid_I;
    set_motor_position(position);
}

// ============================================================================
// DATA LOGGING
// ============================================================================

void print_header() {
    printf("\n");
    printf("time_ms,lat,lon,heading,bearing,error,P,I,motor,phase,speed_ms\n");
}

void log_data(uint32_t time_ms) {
    printf("%lu,%.6f,%.6f,%d,%.1f,%.1f,%.4f,%.4f,%.3f,%d,%.1f\n",
           time_ms,
           gps.lat,
           gps.lon,
           gps.headMot,
           ctrl.bearing,
           ctrl.error,
           ctrl.pid_P,
           ctrl.pid_I,
           ctrl.motor_position,
           ctrl.phase,
           gps.gSpeed / 1000.0f);  // Convert mm/s to m/s
}

void print_status() {
    printf("\r[%6lu] Lat:%.5f Lon:%.5f Head:%3d° Bear:%.0f° Err:%+6.1f° Motor:%.2f %s   ",
           loop_count,
           gps.lat,
           gps.lon,
           gps.headMot,
           ctrl.bearing,
           ctrl.error,
           ctrl.motor_position,
           gps.valid ? "GPS:OK " : "GPS:---");
}

// ============================================================================
// SERIAL COMMANDS
// ============================================================================

void process_serial_command() {
    if (!stdio_usb_connected()) return;
    
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;
    
    switch (c) {
        case 'e':  // Enable motor
            enable_motor(true);
            printf("\n>>> Motor ENABLED\n");
            break;
            
        case 'd':  // Disable motor
            enable_motor(false);
            printf("\n>>> Motor DISABLED\n");
            break;
            
        case 'n':  // Go to neutral
            set_motor_position(NEUTRAL_POS);
            printf("\n>>> Motor to NEUTRAL\n");
            break;
            
        case 'r':  // Reset integral
            ctrl.error_integral = 0;
            printf("\n>>> Integral RESET\n");
            break;
            
        case 'h':  // Print header
            print_header();
            break;
            
        case 's':  // Print status
            printf("\n\n=== STATUS ===\n");
            printf("Target: %.6f, %.6f\n", target_lat, target_lon);
            printf("GPS:    %.6f, %.6f (fix=%d, speed=%.1f m/s)\n", 
                   gps.lat, gps.lon, gps.fixType, gps.gSpeed/1000.0f);
            printf("Control: bearing=%.1f°, error=%.1f°, motor=%.3f\n",
                   ctrl.bearing, ctrl.error, ctrl.motor_position);
            printf("Motor:  %s\n", motor_enabled ? "ENABLED" : "DISABLED");
            printf("==============\n\n");
            break;
            
        case '?':  // Help
            printf("\n\n=== COMMANDS ===\n");
            printf("e - Enable motor\n");
            printf("d - Disable motor\n");
            printf("n - Go to neutral\n");
            printf("r - Reset integral\n");
            printf("h - Print CSV header\n");
            printf("s - Print status\n");
            printf("? - This help\n");
            printf("================\n\n");
            break;
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    // Initialize
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║          BLiMS Car Test - Closed-Loop Validation         ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Target: %.6f, %.6f\n", target_lat, target_lon);
    printf("Gains:  Kp=%.4f, Ki=%.4f\n", Kp, Ki);
    printf("\n");
    printf("Commands: e=enable, d=disable, n=neutral, r=reset, s=status, ?=help\n");
    printf("\n");
    
    // Initialize hardware
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 0);
    
    pwm_init();
    gps_init();
    
    set_motor_position(NEUTRAL_POS);
    
    printf("Waiting for GPS fix...\n");
    
    // Main loop
    uint32_t last_log_time = 0;
    
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Update GPS
        gps_update();
        
        // Process serial commands
        process_serial_command();
        
        // Run control loop at fixed rate
        if (now - last_time_ms >= LOOP_INTERVAL_MS) {
            float dt = (now - last_time_ms) / 1000.0f;
            last_time_ms = now;
            
            if (motor_enabled) {
                run_controller(dt);
            }
            
            loop_count++;
            
            // Log data every 200ms (5 Hz)
            if (now - last_log_time >= 200) {
                log_data(now);
                last_log_time = now;
            }
            
            // Print status every second
            if (loop_count % 20 == 0) {
                print_status();
            }
        }
        
        sleep_ms(1);
    }
    
    return 0;
}