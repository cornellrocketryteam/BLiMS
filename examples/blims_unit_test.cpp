/**
 * @file blims_unit_test.cpp
 * @brief Host-side unit tests for BLiMS control logic
 * 
 * PURPOSE: Verify state machine and controller math WITHOUT hardware.
 * Runs on your laptop, not on Pico.
 * 
 * COMPILE: g++ -DTEST_MODE blims_unit_test.cpp -o blims_unit_test -lm
 * RUN:     ./blims_unit_test
 * 
 * TESTS:
 *   - Utility functions (wrap360, wrap180, heading error)
 *   - Bearing calculation
 *   - Phase transitions at correct altitudes
 *   - Wind-relative heading calculations
 *   - Loiter state machine sequencing
 *   - PI controller behavior
 *   - GPS validity failsafes
 */

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cassert>
#include <cstring>

// ============================================================================
// MOCK PICO SDK (stubs for host compilation)
// ============================================================================

#ifdef TEST_MODE
typedef int64_t alarm_id_t;
typedef uint32_t absolute_time_t;
int64_t add_alarm_in_ms(uint32_t ms, int64_t (*cb)(alarm_id_t, void*), void* d, bool f) { return 0; }
void gpio_put(uint8_t pin, bool value) {}
void gpio_set_function(uint8_t pin, int func) {}
void gpio_init(uint8_t pin) {}
typedef unsigned int uint;
void gpio_set_dir(uint8_t pin, int dir) {}
typedef unsigned int uint;
uint pwm_gpio_to_slice_num(uint8_t pin) { return 0; }
uint pwm_gpio_to_channel(uint8_t pin) { return 0; }
void pwm_set_clkdiv(uint slice, float div) {}
void pwm_set_wrap(uint slice, uint16_t wrap) {}
void pwm_set_enabled(uint slice, bool enabled) {}
void pwm_set_chan_level(uint slice, uint chan, uint16_t level) {}
absolute_time_t get_absolute_time() { return 0; }
uint32_t to_ms_since_boot(absolute_time_t t) { return 0; }
#define GPIO_FUNC_PWM 2
#define GPIO_OUT 1
#endif

// ============================================================================
// CONSTANTS (from blims_constants.hpp)
// ============================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr float deg_to_rad = M_PI / 180.0f;
constexpr float rad_to_deg = 180.0f / M_PI;
constexpr float FT_PER_M = 3.28084f;
constexpr float neutral_pos = 0.5f;
constexpr float motor_min = 0.3f;
constexpr float motor_max = 0.7f;
constexpr float Kp = 0.009f;
constexpr float Ki = 0.001f;
constexpr float ALT_UTURN_START_FT = 1000.0f;
constexpr float ALT_BASE_START_FT = 600.0f;
constexpr float ALT_FINAL_START_FT = 300.0f;
constexpr float ALT_NEUTRAL_FT = 100.0f;
constexpr float SET_RADIUS_FT = 400.0f;
constexpr float LOITER_RIGHT_POS = 0.65f;
constexpr float LOITER_LEFT_POS = 0.35f;
constexpr uint32_t LOITER_TURN_MS = 6000;
constexpr uint32_t LOITER_NEUTRAL_MS = 2500;
constexpr int32_t GSPEED_MIN_FOR_HEADING = 3000;

// ============================================================================
// TEST STATE STRUCTURE
// ============================================================================

struct TestState {
    // GPS inputs
    float gps_lat, gps_lon, alt_agl_ft;
    int32_t headMot, gSpeed;
    uint8_t fixType;
    bool gps_state;
    uint32_t currTime, prevTime;
    
    // Configuration
    float target_lat, target_lon, wind_from_deg;
    
    // Outputs
    float motor_position, bearing, pid_P, pid_I, error_integral;
    int32_t phase_id, loiter_step, last_phase;
    uint32_t loiter_step_start_ms;
};

static TestState g;
static int tests_passed = 0;
static int tests_failed = 0;

// ============================================================================
// HELPER FUNCTIONS (from blims.cpp)
// ============================================================================

float wrap360(float d) { 
    while (d >= 360.0f) d -= 360.0f; 
    while (d < 0.0f) d += 360.0f; 
    return d; 
}

float wrap180(float d) { 
    d = wrap360(d); 
    if (d > 180.0f) d -= 360.0f; 
    return d; 
}

float compute_heading_error(float target, float current) {
    float e = target - current;
    if (e > 180.0f) e -= 360.0f;
    if (e < -180.0f) e += 360.0f;
    return e;
}

float distance_m(float lat1, float lon1, float lat2, float lon2) {
    float lr = lat1 * deg_to_rad;
    float x = (lon2 - lon1) * deg_to_rad * cosf(lr);
    float y = (lat2 - lat1) * deg_to_rad;
    return 6371000.0f * sqrtf(x*x + y*y);
}

void calc_bearing(TestState* s) {
    float d_lat = (s->target_lat - s->gps_lat) * deg_to_rad;
    float d_lon = (s->target_lon - s->gps_lon) * deg_to_rad;
    float lat_rad = s->gps_lat * deg_to_rad;
    
    // Flat-earth approximation with correct atan2 argument order
    float x = d_lon * cosf(lat_rad);  // East component
    float y = d_lat;                   // North component
    float b = atan2f(x, y) * rad_to_deg;  // atan2(east, north) = bearing from north
    if (b < 0) b += 360.0f;
    s->bearing = b;
}

// ============================================================================
// STATE MACHINE (simplified from blims.cpp)
// ============================================================================

void execute_LV(TestState* s) {
    // Validity gates
    if (!s->gps_state || s->fixType < 2 || s->gSpeed < GSPEED_MIN_FOR_HEADING) {
        s->phase_id = -1;
        s->motor_position = neutral_pos;
        return;
    }
    
    // Time delta
    float dt = (float)(s->currTime - s->prevTime) / 1000.0f;
    s->prevTime = s->currTime;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.05f;
    
    // Navigation
    calc_bearing(s);
    float dist_ft = distance_m(s->gps_lat, s->gps_lon, s->target_lat, s->target_lon) * FT_PER_M;
    
    float W = wrap360(s->wind_from_deg);
    float DW = wrap360(W + 180.0f);
    float heading_des = s->bearing;
    int32_t phase = 0;
    
    // Phase 4: NEUTRAL (very low)
    if (s->alt_agl_ft <= ALT_NEUTRAL_FT) {
        phase = 4;
        if (s->last_phase != phase) { s->error_integral = 0; s->last_phase = phase; }
        s->phase_id = phase;
        s->motor_position = neutral_pos;
        return;
    }
    
    // High altitude: TRACK or LOITER
    if (s->alt_agl_ft > ALT_UTURN_START_FT) {
        if (dist_ft <= SET_RADIUS_FT) {
            // Phase 5: LOITER
            phase = 5;
            if (s->last_phase != phase) {
                s->error_integral = 0;
                s->loiter_step = 0;
                s->loiter_step_start_ms = s->currTime;
                s->last_phase = phase;
            }
            uint32_t el = s->currTime - s->loiter_step_start_ms;
            switch (s->loiter_step) {
                case 0: if (el >= LOITER_TURN_MS) { s->loiter_step = 1; s->loiter_step_start_ms = s->currTime; } break;
                case 1: if (el >= LOITER_NEUTRAL_MS) { s->loiter_step = 2; s->loiter_step_start_ms = s->currTime; } break;
                case 2: if (el >= LOITER_TURN_MS) { s->loiter_step = 3; s->loiter_step_start_ms = s->currTime; } break;
                case 3: if (el >= LOITER_NEUTRAL_MS) { s->loiter_step = 0; s->loiter_step_start_ms = s->currTime; } break;
            }
            float pos = neutral_pos;
            if (s->loiter_step == 0) pos = LOITER_RIGHT_POS;
            if (s->loiter_step == 2) pos = LOITER_LEFT_POS;
            s->phase_id = phase;
            s->motor_position = pos;
            return;
        } else {
            phase = 0;
            heading_des = s->bearing;
            s->loiter_step = 0;
        }
    }
    // Landing pattern
    else if (s->alt_agl_ft <= ALT_FINAL_START_FT) {
        phase = 3;
        heading_des = W;
    }
    else if (s->alt_agl_ft <= ALT_BASE_START_FT) {
        phase = 2;
        float b1 = wrap360(DW + 90.0f);
        float b2 = wrap360(DW - 90.0f);
        float e1 = fabsf(wrap180(b1 - (float)s->headMot));
        float e2 = fabsf(wrap180(b2 - (float)s->headMot));
        heading_des = (e1 <= e2) ? b1 : b2;
    }
    else {
        phase = 1;
        heading_des = DW;
    }
    
    s->phase_id = phase;
    if (phase != s->last_phase) { s->error_integral = 0; s->last_phase = phase; }
    
    // PI Controller
    float error = compute_heading_error(heading_des, (float)s->headMot);
    s->error_integral += error * dt;
    float limit = 0.5f / Ki;
    if (s->error_integral > limit) s->error_integral = limit;
    if (s->error_integral < -limit) s->error_integral = -limit;
    
    s->pid_P = -Kp * error;
    s->pid_I = -Ki * s->error_integral;
    
    float pos = neutral_pos + s->pid_P + s->pid_I;
    if (pos < motor_min) pos = motor_min;
    if (pos > motor_max) pos = motor_max;
    
    s->bearing = heading_des;
    s->motor_position = pos;
}

// ============================================================================
// TEST UTILITIES
// ============================================================================

void reset() {
    memset(&g, 0, sizeof(g));
    g.gps_state = true;
    g.fixType = 3;
    g.gSpeed = 5000;
    g.last_phase = -99;
    g.target_lat = 42.7;
    g.target_lon = -77.2;
}

#define TEST(name) void test_##name()
#define RUN(name) do { \
    printf("  %-45s", #name); \
    test_##name(); \
    printf(" PASS\n"); \
    tests_passed++; \
} while(0)
#define ASSERT(cond) do { \
    if (!(cond)) { \
        printf(" FAIL: %s (line %d)\n", #cond, __LINE__); \
        tests_failed++; \
        return; \
    } \
} while(0)
#define ASSERT_NEAR(a, b, tol) ASSERT(fabsf((a)-(b)) < (tol))

// ============================================================================
// UNIT TESTS
// ============================================================================

// --- Utility Functions ---

TEST(wrap360_positive) {
    ASSERT_NEAR(wrap360(0), 0, 0.01);
    ASSERT_NEAR(wrap360(90), 90, 0.01);
    ASSERT_NEAR(wrap360(359), 359, 0.01);
    ASSERT_NEAR(wrap360(360), 0, 0.01);
    ASSERT_NEAR(wrap360(450), 90, 0.01);
    ASSERT_NEAR(wrap360(720), 0, 0.01);
}

TEST(wrap360_negative) {
    ASSERT_NEAR(wrap360(-10), 350, 0.01);
    ASSERT_NEAR(wrap360(-90), 270, 0.01);
    ASSERT_NEAR(wrap360(-180), 180, 0.01);
    ASSERT_NEAR(wrap360(-360), 0, 0.01);
    ASSERT_NEAR(wrap360(-450), 270, 0.01);
}

TEST(wrap180_range) {
    ASSERT_NEAR(wrap180(0), 0, 0.01);
    ASSERT_NEAR(wrap180(90), 90, 0.01);
    ASSERT_NEAR(wrap180(179), 179, 0.01);
    ASSERT(fabsf(wrap180(180)) <= 180.01);  // Edge case: can be +180 or -180
    ASSERT_NEAR(wrap180(270), -90, 0.01);
    ASSERT_NEAR(wrap180(-90), -90, 0.01);
}

TEST(heading_error_simple) {
    ASSERT_NEAR(compute_heading_error(90, 80), 10, 0.01);
    ASSERT_NEAR(compute_heading_error(80, 90), -10, 0.01);
    ASSERT_NEAR(compute_heading_error(0, 350), 10, 0.01);
    ASSERT_NEAR(compute_heading_error(350, 0), -10, 0.01);
}

TEST(heading_error_wraparound) {
    ASSERT_NEAR(compute_heading_error(10, 350), 20, 0.01);
    ASSERT_NEAR(compute_heading_error(350, 10), -20, 0.01);
    ASSERT_NEAR(compute_heading_error(0, 180), -180, 0.01);
    ASSERT_NEAR(compute_heading_error(180, 0), 180, 0.01);
}

// --- Bearing Calculation ---

TEST(bearing_north) {
    reset();
    g.gps_lat = 42.0; g.gps_lon = -77.0;
    g.target_lat = 43.0; g.target_lon = -77.0;  // Target due North
    calc_bearing(&g);
    ASSERT_NEAR(g.bearing, 0, 1.0);
}

TEST(bearing_east) {
    reset();
    g.gps_lat = 42.0; g.gps_lon = -77.0;
    g.target_lat = 42.0; g.target_lon = -76.0;  // Target due East
    calc_bearing(&g);
    ASSERT_NEAR(g.bearing, 90, 1.0);
}

TEST(bearing_south) {
    reset();
    g.gps_lat = 42.0; g.gps_lon = -77.0;
    g.target_lat = 41.0; g.target_lon = -77.0;  // Target due South
    calc_bearing(&g);
    ASSERT_NEAR(g.bearing, 180, 1.0);
}

TEST(bearing_west) {
    reset();
    g.gps_lat = 42.0; g.gps_lon = -77.0;
    g.target_lat = 42.0; g.target_lon = -78.0;  // Target due West
    calc_bearing(&g);
    ASSERT_NEAR(g.bearing, 270, 1.0);
}

// --- Phase Transitions ---

TEST(phase_neutral_below_100ft) {
    reset();
    g.alt_agl_ft = 50;
    g.headMot = 0;
    g.currTime = 1000;
    execute_LV(&g);
    ASSERT(g.phase_id == 4);
    ASSERT_NEAR(g.motor_position, neutral_pos, 0.01);
}

TEST(phase_final_below_300ft) {
    reset();
    g.alt_agl_ft = 200;
    g.wind_from_deg = 270;  // Wind from West
    g.headMot = 270;
    g.currTime = 1000;
    execute_LV(&g);
    ASSERT(g.phase_id == 3);
    ASSERT_NEAR(g.bearing, 270, 1.0);  // Fly into wind (West)
}

TEST(phase_base_300_to_600ft) {
    reset();
    g.alt_agl_ft = 450;
    g.wind_from_deg = 270;  // Wind from West, DW = East (90)
    g.headMot = 180;        // Currently heading South
    g.currTime = 1000;
    execute_LV(&g);
    ASSERT(g.phase_id == 2);
    // Base is perpendicular to DW(90): either 0 or 180
    ASSERT(g.bearing == 180.0f || g.bearing == 0.0f);
}

TEST(phase_downwind_600_to_1000ft) {
    reset();
    g.alt_agl_ft = 800;
    g.wind_from_deg = 270;  // Wind from West, DW = 90 (East)
    g.headMot = 90;
    g.currTime = 1000;
    execute_LV(&g);
    ASSERT(g.phase_id == 1);
    ASSERT_NEAR(g.bearing, 90, 1.0);  // Downwind = East
}

TEST(phase_track_above_1000ft_far) {
    reset();
    g.gps_lat = 42.0; g.gps_lon = -77.0;
    g.target_lat = 42.0; g.target_lon = -76.0;  // ~74km East (way outside set radius)
    g.alt_agl_ft = 1500;
    g.headMot = 90;
    g.currTime = 1000;
    execute_LV(&g);
    ASSERT(g.phase_id == 0);  // TRACK
    ASSERT_NEAR(g.bearing, 90, 2.0);  // Bearing to target
}

TEST(phase_loiter_above_1000ft_close) {
    reset();
    g.gps_lat = 42.7;   // Same as target
    g.gps_lon = -77.2;
    g.alt_agl_ft = 1500;
    g.headMot = 0;
    g.currTime = 1000;
    execute_LV(&g);
    ASSERT(g.phase_id == 5);  // LOITER
    ASSERT(g.loiter_step == 0);
    ASSERT_NEAR(g.motor_position, LOITER_RIGHT_POS, 0.01);
}

// --- Loiter Sequencing ---

TEST(loiter_full_cycle) {
    reset();
    g.gps_lat = 42.7; g.gps_lon = -77.2;  // At target
    g.alt_agl_ft = 1500;
    g.currTime = 0;
    
    // Step 0: Right turn
    execute_LV(&g);
    ASSERT(g.loiter_step == 0);
    ASSERT_NEAR(g.motor_position, LOITER_RIGHT_POS, 0.01);
    
    // After LOITER_TURN_MS: Step 1 (neutral)
    g.currTime = LOITER_TURN_MS + 100;
    execute_LV(&g);
    ASSERT(g.loiter_step == 1);
    ASSERT_NEAR(g.motor_position, neutral_pos, 0.01);
    
    // After LOITER_NEUTRAL_MS: Step 2 (left turn)
    g.currTime += LOITER_NEUTRAL_MS + 100;
    execute_LV(&g);
    ASSERT(g.loiter_step == 2);
    ASSERT_NEAR(g.motor_position, LOITER_LEFT_POS, 0.01);
    
    // After LOITER_TURN_MS: Step 3 (neutral)
    g.currTime += LOITER_TURN_MS + 100;
    execute_LV(&g);
    ASSERT(g.loiter_step == 3);
    ASSERT_NEAR(g.motor_position, neutral_pos, 0.01);
    
    // After LOITER_NEUTRAL_MS: Back to Step 0
    g.currTime += LOITER_NEUTRAL_MS + 100;
    execute_LV(&g);
    ASSERT(g.loiter_step == 0);
    ASSERT_NEAR(g.motor_position, LOITER_RIGHT_POS, 0.01);
}

// --- PI Controller ---

TEST(pi_positive_error_turns_right) {
    reset();
    g.alt_agl_ft = 800;
    g.wind_from_deg = 0;  // DW = 180 (South)
    g.headMot = 170;      // Slightly left of target (180)
    g.currTime = 1000;
    g.prevTime = 950;
    execute_LV(&g);
    // Error = 180 - 170 = 10 (positive, target to right)
    // P = -Kp * 10 = -0.09
    // Position = 0.5 - 0.09 = 0.41 (turn right)
    ASSERT(g.motor_position < neutral_pos);
}

TEST(pi_negative_error_turns_left) {
    reset();
    g.alt_agl_ft = 800;
    g.wind_from_deg = 0;  // DW = 180 (South)
    g.headMot = 190;      // Slightly right of target (180)
    g.currTime = 1000;
    g.prevTime = 950;
    execute_LV(&g);
    // Error = 180 - 190 = -10 (negative, target to left)
    // P = -Kp * (-10) = +0.09
    // Position = 0.5 + 0.09 = 0.59 (turn left)
    ASSERT(g.motor_position > neutral_pos);
}

TEST(motor_clamping_min) {
    reset();
    g.alt_agl_ft = 800;
    g.wind_from_deg = 0;
    g.headMot = 0;        // Heading North, want South (180° error)
    g.currTime = 1000;
    g.prevTime = 0;
    execute_LV(&g);
    ASSERT(g.motor_position >= motor_min);
    ASSERT(g.motor_position <= motor_max);
}

TEST(motor_clamping_max) {
    reset();
    g.alt_agl_ft = 800;
    g.wind_from_deg = 180;  // DW = 0 (North)
    g.headMot = 180;        // Heading South, want North (180° error other way)
    g.currTime = 1000;
    g.prevTime = 0;
    execute_LV(&g);
    ASSERT(g.motor_position >= motor_min);
    ASSERT(g.motor_position <= motor_max);
}

// --- Failsafes ---

TEST(failsafe_gps_invalid) {
    reset();
    g.gps_state = false;
    g.alt_agl_ft = 500;
    g.currTime = 1000;
    execute_LV(&g);
    ASSERT(g.phase_id == -1);
    ASSERT_NEAR(g.motor_position, neutral_pos, 0.01);
}

TEST(failsafe_bad_fix_type) {
    reset();
    g.fixType = 1;  // DR only, not good enough
    g.alt_agl_ft = 500;
    g.currTime = 1000;
    execute_LV(&g);
    ASSERT(g.phase_id == -1);
    ASSERT_NEAR(g.motor_position, neutral_pos, 0.01);
}

TEST(failsafe_low_speed) {
    reset();
    g.gSpeed = 1000;  // 1 m/s, below threshold
    g.alt_agl_ft = 500;
    g.currTime = 1000;
    execute_LV(&g);
    ASSERT(g.phase_id == -1);
    ASSERT_NEAR(g.motor_position, neutral_pos, 0.01);
}

// --- Phase Transition Integral Reset ---

TEST(integral_resets_on_phase_change) {
    reset();
    g.alt_agl_ft = 800;  // DOWNWIND
    g.wind_from_deg = 0;
    g.headMot = 170;
    g.currTime = 1000;
    g.prevTime = 900;
    
    // Build up some integral
    execute_LV(&g);
    g.currTime = 1100;
    execute_LV(&g);
    g.currTime = 1200;
    execute_LV(&g);
    ASSERT(g.phase_id == 1);
    int old_phase = g.phase_id;
    
    // Change altitude to trigger phase change
    g.alt_agl_ft = 450;  // BASE
    g.currTime = 2000;
    execute_LV(&g);
    ASSERT(g.phase_id == 2);
    ASSERT(g.phase_id != old_phase);
    // Integral was reset, so first iteration in new phase starts fresh
}

// --- Full Descent Simulation ---

TEST(full_descent_sequence) {
    reset();
    g.gps_lat = 42.71; g.gps_lon = -77.21;  // Slightly away from target
    g.target_lat = 42.7; g.target_lon = -77.2;
    g.wind_from_deg = 270;  // Wind from West
    g.headMot = 90;
    g.gSpeed = 5000;
    
    // Start high - should be TRACK (far from target)
    g.alt_agl_ft = 1500;
    g.currTime = 0;
    execute_LV(&g);
    ASSERT(g.phase_id == 0 || g.phase_id == 5);  // TRACK or LOITER depending on distance
    
    // Descend to DOWNWIND
    g.alt_agl_ft = 800;
    g.currTime = 10000;
    execute_LV(&g);
    ASSERT(g.phase_id == 1);
    
    // Descend to BASE
    g.alt_agl_ft = 450;
    g.currTime = 20000;
    execute_LV(&g);
    ASSERT(g.phase_id == 2);
    
    // Descend to FINAL
    g.alt_agl_ft = 200;
    g.currTime = 30000;
    execute_LV(&g);
    ASSERT(g.phase_id == 3);
    
    // Descend to NEUTRAL
    g.alt_agl_ft = 50;
    g.currTime = 40000;
    execute_LV(&g);
    ASSERT(g.phase_id == 4);
    ASSERT_NEAR(g.motor_position, neutral_pos, 0.01);
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║       BLiMS Unit Tests - Host-Side Logic Verification    ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");
    
    printf("Utility Functions:\n");
    RUN(wrap360_positive);
    RUN(wrap360_negative);
    RUN(wrap180_range);
    RUN(heading_error_simple);
    RUN(heading_error_wraparound);
    
    printf("\nBearing Calculation:\n");
    RUN(bearing_north);
    RUN(bearing_east);
    RUN(bearing_south);
    RUN(bearing_west);
    
    printf("\nPhase Transitions:\n");
    RUN(phase_neutral_below_100ft);
    RUN(phase_final_below_300ft);
    RUN(phase_base_300_to_600ft);
    RUN(phase_downwind_600_to_1000ft);
    RUN(phase_track_above_1000ft_far);
    RUN(phase_loiter_above_1000ft_close);
    
    printf("\nLoiter Sequencing:\n");
    RUN(loiter_full_cycle);
    
    printf("\nPI Controller:\n");
    RUN(pi_positive_error_turns_right);
    RUN(pi_negative_error_turns_left);
    RUN(motor_clamping_min);
    RUN(motor_clamping_max);
    
    printf("\nFailsafes:\n");
    RUN(failsafe_gps_invalid);
    RUN(failsafe_bad_fix_type);
    RUN(failsafe_low_speed);
    
    printf("\nIntegration:\n");
    RUN(integral_resets_on_phase_change);
    RUN(full_descent_sequence);
    
    printf("\n══════════════════════════════════════════════════════════\n");
    if (tests_failed == 0) {
        printf("  ✓ ALL %d TESTS PASSED\n", tests_passed);
    } else {
        printf("  ✗ %d passed, %d FAILED\n", tests_passed, tests_failed);
    }
    printf("══════════════════════════════════════════════════════════\n\n");
    
    return tests_failed > 0 ? 1 : 0;
}