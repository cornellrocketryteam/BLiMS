/**
 * @file blims_car_test.cpp
 * @brief Car test using BLIMS class begin/execute — matches FSW 24-25 pattern
 *
 * PURPOSE:
 * Test the full BLiMS LV flight logic by calling begin() once then execute()
 * at 20 Hz (50ms cycle), exactly as FSW does in MainDeployedMode.
 * Real GPS provides lat/lon/heading, simulated altitude from L3 Launch 4
 * descent data drives phase transitions through the landing pattern.
 *
 * FSW 24-25 PATTERN (what this replicates):
 *   StartupMode::execute()       -> blims_obj.begin(mode, PWM, EN)
 *   MainDeployedMode::execute()  -> pack BLIMSDataIn, call blims_obj.execute()
 *   Flight::execute()            -> sleep remaining cycle time (50ms target)
 *
 * NOTE - FSW BUG:
 * FSW MainDeployedMode does NOT populate altitude_ft in BLIMSDataIn.
 * It defaults to 0, meaning BLIMS always sees PHASE_NEUTRAL in flight.
 * Before flight, FSW needs:
 *     data_in.altitude_ft = state::alt::altitude * 3.28084f;
 *
 * WIRING (breadboard):
 *   PWM    -> GPIO 28    Enable -> GPIO 0
 *   SDA    -> GPIO 12    SCL    -> GPIO 13    (I2C0)
 *
 * COMPILE: Link with blims.cpp, blims_state.cpp
 *
 * OUTPUT CSV (13 fields, for car_test_visualizer.py):
 *   lat,lon,target_lat,target_lon,heading,bearing,motor_pos,
 *   timestamp_ms,P,I,phase,altitude,loiter_step
 */

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "ublox_mx.hpp"
#include "ublox_nav_pvt.hpp"

#include "blims.hpp"
#include "blims_state.hpp"
#include "blims_constants.hpp"
#include "descent_alt_data.hpp"

#include <cstdio>

// ============================================================================
// PIN DEFINITIONS (breadboard)
// ============================================================================

#define PWM_PIN     28
#define ENABLE_PIN  0
#define I2C_PORT    i2c0
#define I2C_SDA     12
#define I2C_SCL     13

// ============================================================================
// TEST CONFIGURATION - UPDATE BEFORE EACH TEST
// ============================================================================

static const double TARGET_LAT = 42.446610;
static const double TARGET_LON = -76.461304;

// Minimum ground speed (m/s) for GPS heading to be trustworthy
static const float MIN_GROUND_SPEED_MPS = 0.3f;

// Cycle time - matches FSW constants::cycle_time (50ms = 20Hz)
static const uint32_t CYCLE_TIME_MS = 50;

// ============================================================================
// WIND PROFILE
// Altitude in meters AGL, wind direction in degrees (coming FROM)
// Update with real sounding data or forecast before test/flight
// ============================================================================

static const int WIND_PROFILE_SIZE = 11;
static const float WIND_ALTITUDES_M[] = {
    0, 50, 100, 150, 200, 250, 300, 400, 500, 550, 610
};
static const float WIND_DIRS_DEG[] = {
    45, 48, 52, 56, 60, 64, 68, 75, 80, 85, 90
};

// Fallback single value (used if wind_profile_size == 0)
static const float WIND_FROM_DEG = 45.0f;

// ============================================================================
// GLOBALS
// ============================================================================

GNSS gps(I2C_PORT);
BLIMS blims_obj;

static int  alt_index       = 0;
static bool descent_started = false;

static const char* PHASE_NAMES[] = {
    "HELD", "TRACK", "DOWNWIND", "BASE", "FINAL", "NEUTRAL", "LOITER"
};

// ============================================================================
// MAIN
// ============================================================================

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    // ---- I2C for GPS ----
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // ---- BLIMS begin (mirrors FSW StartupMode::execute) ----
    // New API: begin() enables motor immediately, no 10s delay
    blims_obj.begin(LV, PWM_PIN, ENABLE_PIN);
    sleep_ms(5000);
    blims_obj.set_target((float)TARGET_LAT, (float)TARGET_LON);

    // Load wind profile (new API)
    // Falls back to single value if set_wind_profile is called with size 0
    blims_obj.set_wind_from_deg(WIND_FROM_DEG);
    blims_obj.set_wind_profile(WIND_ALTITUDES_M, WIND_DIRS_DEG, WIND_PROFILE_SIZE);

    // ---- Wait for serial ----
    while (!tud_cdc_connected())
    {
        sleep_ms(500);
    }

    // ---- Banner ----
    printf("# ================================================\n");
    printf("# BLiMS Car Test (FSW begin/execute pattern)\n");
    printf("# ================================================\n");
    printf("# Target:     %.6f, %.6f\n", TARGET_LAT, TARGET_LON);
    printf("# Wind:       profile with %d layers, surface %.0f deg\n",
           WIND_PROFILE_SIZE, WIND_DIRS_DEG[0]);
    printf("# Descent:    %d samples (%.1f sec), %.0f -> %.0f ft\n",
           DESCENT_DATA_SIZE, DESCENT_DATA_SIZE * 0.05f,
           descent_alt_ft[0], descent_alt_ft[DESCENT_DATA_SIZE - 1]);
    printf("# Cycle:      %d ms (20 Hz)\n", CYCLE_TIME_MS);
    printf("# Pins:       PWM=%d EN=%d SDA=%d SCL=%d\n",
           PWM_PIN, ENABLE_PIN, I2C_SDA, I2C_SCL);
    printf("# ================================================\n");
    printf("# CSV: lat,lon,target_lat,target_lon,heading,bearing,"
           "motor_pos,timestamp_ms,P,I,phase,altitude,loiter_step\n");
    printf("# ================================================\n");

    // ---- Init GPS at 20 Hz ----
    if (!gps.begin_PVT(20))
    {
        printf("# ERROR: GPS init failed\n");
        return 1;
    }
    printf("# GPS initialized at 20 Hz\n");
    // ---- I2C bus scan (debug) ----
    printf("# I2C scan on i2c0:\n");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int ret = i2c_read_blocking(I2C_PORT, addr, &dummy, 1, false);
        if (ret >= 0) {
            printf("#   Found device at 0x%02X\n", addr);
        }
    }
    printf("# I2C scan complete\n");
    printf("# Waiting for GPS fix to start descent...\n");

    // ---- Loop state ----
    UbxNavPvt pvt = {0};
    int8_t last_phase_id = -1;

    // ================================================================
    // MAIN LOOP - FSW-style cycle timing
    //
    // FSW flight_loop.cpp:
    //   cycle_start = now
    //   mode->execute()       <-- sensor reads + blims.execute()
    //   mode->transition()
    //   sleep(cycle_time - elapsed)
    // ================================================================
    gpio_put(ENABLE_PIN, 1);
    printf("enable\n");
    sleep_ms(500);
    gpio_put(ENABLE_PIN, 0);
    printf("pulse low\n");
    while (true)
    {
        uint32_t cycle_start = to_ms_since_boot(get_absolute_time());

        // ==========================================================
        // 1. SENSOR READ
        // ==========================================================
        gps.read_PVT_data(&pvt);

        float ground_speed_mps = pvt.gSpeed / 1000.0f;
        bool gps_valid = (pvt.fixType >= 2) &&
                         (ground_speed_mps > MIN_GROUND_SPEED_MPS);

        // ==========================================================
        // 2. ALTITUDE SIMULATION
        //    Descent starts on first valid GPS fix
        // ==========================================================
        if (!descent_started && gps_valid)
        {
            descent_started = true;
            printf("# DESCENT STARTED - alt %.0f ft (0/%d)\n",
                   descent_alt_ft[0], DESCENT_DATA_SIZE);
        }

        float current_alt_ft = descent_started
            ? descent_alt_ft[alt_index]
            : descent_alt_ft[0];

        // ==========================================================
        // 3. PACK BLIMSDataIn (mirrors FSW MainDeployedMode)
        // ==========================================================
        BLIMSDataIn data_in;
        data_in.lon         = pvt.lon;
        data_in.lat         = pvt.lat;
        data_in.altitude_ft = current_alt_ft;
        data_in.hAcc        = pvt.hAcc;
        data_in.vAcc        = pvt.vAcc;
        data_in.velN        = pvt.velN;
        data_in.velE        = pvt.velE;
        data_in.velD        = pvt.velD;
        data_in.gSpeed      = pvt.gSpeed;
        data_in.headMot     = pvt.headMot;
        data_in.sAcc        = pvt.sAcc;
        data_in.headAcc     = pvt.headAcc;
        data_in.fixType     = pvt.fixType;
        data_in.gps_state   = gps_valid;

        // ==========================================================
        // 4. EXECUTE BLIMS
        // ==========================================================
        BLIMSDataOut data_out = blims_obj.execute(&data_in);

        // ==========================================================
        // 5. ADVANCE ALTITUDE (one step per cycle)
        //    Runs through all data to ground, then stays at last value
        // ==========================================================
        if (descent_started && alt_index < DESCENT_DATA_SIZE - 1)
        {
            alt_index++;
        }

        // ==========================================================
        // 6. LOG
        // ==========================================================
        if (data_out.phase_id != last_phase_id)
        {
            int p = data_out.phase_id;
            const char* name = (p >= 0 && p <= 6) ? PHASE_NAMES[p] : "???";
            printf("# PHASE: %s (alt=%.0f ft, sample %d/%d)\n",
                   name, current_alt_ft, alt_index, DESCENT_DATA_SIZE);
            last_phase_id = data_out.phase_id;
        }

        if (pvt.fixType >= 2)
        {
            float heading_deg = blims::flight::headMot * 1e-5f;
            float lat_f       = pvt.lat * 1e-7f;
            float lon_f       = pvt.lon * 1e-7f;
            uint64_t now_ms   = to_ms_since_boot(get_absolute_time());

            printf("%.7f,%.7f,%.7f,%.7f,%.2f,%.2f,%.3f,%llu,%.4f,%.4f,%d,%.1f,%d\n",
                   lat_f, lon_f,
                   (float)TARGET_LAT, (float)TARGET_LON,
                   heading_deg,
                   data_out.bearing,
                   data_out.motor_position,
                   now_ms,
                   data_out.pid_P,
                   data_out.pid_I,
                   (int)data_out.phase_id,
                   current_alt_ft,
                   (int)data_out.loiter_step);
        }
        else
        {
            printf("# No fix (type=%d speed=%.2f m/s)\n",
                   pvt.fixType, ground_speed_mps);
        }

        // ==========================================================
        // 7. CYCLE TIMING (mirrors FSW flight_loop.cpp)
        // ==========================================================
        uint32_t cycle_duration =
            to_ms_since_boot(get_absolute_time()) - cycle_start;

        if (cycle_duration < CYCLE_TIME_MS)
        {
            sleep_ms(CYCLE_TIME_MS - cycle_duration);
        }
        else
        {
            printf("# WARN: cycle overrun %d ms\n", cycle_duration);
        }
    }

    return 0;
}