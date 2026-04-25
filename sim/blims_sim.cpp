/**
 * @file blims_sim.cpp
 * @brief Desktop simulation harness for BLIMS — no Pico hardware needed
 *
 * Calls blims_obj.execute() in a loop with synthetic BLIMSDataIn, writes
 * the same 13-field CSV that blims_car_test.cpp produces, so you can pipe
 * it directly into car_test_visualizer_qt.py.
 *
 * ── Project layout assumption ──────────────────────────────────────────────
 *   BLiMS/
 *   ├── src/
 *   │   ├── blims.cpp / blims.hpp
 *   │   ├── blims_state.cpp / blims_state.hpp
 *   │   └── blims_constants.hpp
 *   └── test/
 *       └── blims_sim/          <── put this file here
 *           ├── blims_sim.cpp
 *           └── sim_stubs.hpp
 *
 * ── Compile ────────────────────────────────────────────────────────────────
 *   From BLiMS/test/blims_sim/ :
 *
 *   g++ -std=c++17 \
 *       -include sim_stubs.hpp \
 *       -I../../src \
 *       -o blims_sim \
 *       blims_sim.cpp ../../src/blims.cpp ../../src/blims_state.cpp \
 *       -lm
 *
 * ── Run ────────────────────────────────────────────────────────────────────
 *   ./blims_sim sweep   > sweep.csv    # heading sweep — sign-flip detector
 *   ./blims_sim phase   > phase.csv    # altitude descent — phase transitions
 *   ./blims_sim drive   > drive.csv    # closed-loop drive toward target
 *
 * ── Visualize ──────────────────────────────────────────────────────────────
 *   python car_test_visualizer_qt.py --file drive.csv
 *   (or add --file support to the visualizer, or just inspect the CSV)
 *
 * ── CSV format (matches blims_car_test.cpp exactly) ───────────────────────
 *   lat, lon, target_lat, target_lon, heading, bearing,
 *   motor_pos, timestamp_ms, P, I, phase, altitude, loiter_step
 */

// sim_stubs.hpp MUST be included before any blims headers (via -include flag,
// but also listed here for editors / IDEs)
#include "sim_stubs.hpp"

#include "blims.hpp"
#include "blims_state.hpp"
#include "blims_constants.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// SCENARIO CONFIGURATION — update to match your field / test conditions
// ─────────────────────────────────────────────────────────────────────────────

static const float  TARGET_LAT          = 42.446610f;
static const float  TARGET_LON          = -76.461304f;
static const float  WIND_FROM_DEG       = 45.0f;    // degrees, wind coming FROM
static const float  VEHICLE_SPEED_MPS   = 4.0f;     // simulated forward speed

// Heading turn-rate model:
//   deflection = motor_position - 0.5   (range -0.5 to +0.5)
//   turn_rate  = deflection * TURN_GAIN  (deg/sec)
// Set to match your glider's real turn rate at full deflection.
static const float  TURN_GAIN_DEG_S     = 40.0f;

static const int    CYCLE_MS            = 50;       // 20 Hz — matches FSW
static const float  DT                  = CYCLE_MS / 1000.0f;

static const float  METERS_PER_DEG_LAT = 111320.0f;

// Wind profile (matches blims_car_test.cpp defaults)
static const int    WIND_PROFILE_SIZE   = 11;
static const float  WIND_ALTITUDES_M[] = { 0,  50, 100, 150, 200, 250, 300, 400, 500, 550, 610 };
static const float  WIND_DIRS_DEG[]    = { 45, 48,  52,  56,  60,  64,  68,  75,  80,  85,  90 };

// ─────────────────────────────────────────────────────────────────────────────
// COORDINATE HELPERS
// ─────────────────────────────────────────────────────────────────────────────

// Bearing from (lat1,lon1) → (lat2,lon2), degrees geographic [0,360)
static float bearing_to(float lat1, float lon1, float lat2, float lon2) {
    float dlat = (lat2 - lat1) * (M_PI / 180.0f);
    float dlon = (lon2 - lon1) * (M_PI / 180.0f);
    float y = sinf(dlon) * cosf(lat2 * (M_PI / 180.0f));
    float x = cosf(lat1 * (M_PI / 180.0f)) * sinf(lat2 * (M_PI / 180.0f))
            - sinf(lat1 * (M_PI / 180.0f)) * cosf(lat2 * (M_PI / 180.0f)) * cosf(dlon);
    return fmodf(atan2f(y, x) * (180.0f / M_PI) + 360.0f, 360.0f);
}

// Distance in metres (flat-earth, good enough within a few km)
static float distance_m(float lat1, float lon1, float lat2, float lon2) {
    float dlat  = (lat2 - lat1) * METERS_PER_DEG_LAT;
    float dlon  = (lon2 - lon1) * METERS_PER_DEG_LAT * cosf(lat1 * (M_PI / 180.0f));
    return sqrtf(dlat * dlat + dlon * dlon);
}

// Propagate lat/lon by (vN m/s, vE m/s) over DT seconds
static void step_position(float& lat, float& lon, float vN, float vE) {
    lat += (vN * DT) / METERS_PER_DEG_LAT;
    lon += (vE * DT) / (METERS_PER_DEG_LAT * cosf(lat * (M_PI / 180.0f)));
}

// ─────────────────────────────────────────────────────────────────────────────
// BLIMSDataIn FACTORY
// Fills every field in the same units blims.cpp expects (UBX native units).
// ─────────────────────────────────────────────────────────────────────────────

static BLIMSDataIn make_data_in(float lat, float lon, float alt_ft,
                                 float heading_deg, float speed_mps,
                                 bool gps_valid = true)
{
    BLIMSDataIn d = {};

    // Position: degrees * 1e7  (int32)
    d.lat = (int32_t)(lat * 1e7f);
    d.lon = (int32_t)(lon * 1e7f);

    // Altitude: already in feet, passed straight through
    d.altitude_ft = alt_ft;

    // Fix / validity
    d.fixType   = gps_valid ? 3 : 0;
    d.gps_state = gps_valid;

    // Velocity: mm/s
    float speed_mms = speed_mps * 1000.0f;
    float rad = heading_deg * (M_PI / 180.0f);
    d.velN   = (int32_t)( cosf(rad) * speed_mms);
    d.velE   = (int32_t)( sinf(rad) * speed_mms);
    d.velD   = 0;
    d.gSpeed = (int32_t)speed_mms;

    // Heading of motion: degrees * 1e5  (int32)
    d.headMot = (int32_t)(heading_deg * 1e5f);

    // Accuracy estimates — realistic values so blims doesn't reject them
    d.hAcc    = 500;      // 0.5 m
    d.vAcc    = 1000;     // 1.0 m
    d.sAcc    = 200;      // 0.2 m/s
    d.headAcc = 2000000;  // 20 deg (in units of 1e-5 deg)

    return d;
}

// ─────────────────────────────────────────────────────────────────────────────
// CSV OUTPUT
// ─────────────────────────────────────────────────────────────────────────────

static void print_header() {
    printf("# lat,lon,target_lat,target_lon,heading,bearing,"
           "motor_pos,timestamp_ms,P,I,phase,altitude,loiter_step\n");
}

static void print_row(float lat, float lon, float heading_deg,
                      const BLIMSDataOut& out, float alt_ft, uint32_t ts_ms)
{
    printf("%.7f,%.7f,%.7f,%.7f,%.2f,%.2f,%.3f,%u,%.4f,%.4f,%d,%.1f,%d\n",
           lat, lon,
           TARGET_LAT, TARGET_LON,
           heading_deg,
           out.bearing,
           out.motor_position,
           ts_ms,
           out.pid_P,
           out.pid_I,
           (int)out.phase_id,
           alt_ft,
           (int)out.loiter_step);
}

// ─────────────────────────────────────────────────────────────────────────────
// BLIMS INIT HELPER
// ─────────────────────────────────────────────────────────────────────────────

static void init_blims(BLIMS& blims) {
    blims.begin(LV, 0, 0);   // pin numbers are no-ops in sim
    blims.set_target(TARGET_LAT, TARGET_LON);
    blims.set_wind_from_deg(WIND_FROM_DEG);
    blims.set_wind_profile(WIND_ALTITUDES_M, WIND_DIRS_DEG, WIND_PROFILE_SIZE);
}

// ─────────────────────────────────────────────────────────────────────────────
// SCENARIO 1: HEADING SWEEP
//
// What it tests: Bug 1 (wrong turn direction / sign flip)
//
// Fixed position 200 m NW of target, fixed altitude in TRACK phase.
// Sweeps heading 0→360° in 1° steps, one call to execute() per step.
//
// What to look for in the output:
//   - When heading ≈ bearing (pointing at target), error ≈ 0 → motor ≈ 0.5
//   - When target is 90° to the RIGHT of heading, motor should be > 0.5
//   - When target is 90° to the LEFT  of heading, motor should be < 0.5
//   - If it's backwards, the sign fix in execute_pi_control() is still wrong
// ─────────────────────────────────────────────────────────────────────────────

static void scenario_sweep() {
    fprintf(stderr, "# SCENARIO: heading sweep\n");
    fprintf(stderr, "# Fixed position 200m NW of target, alt=1500ft (TRACK phase)\n");
    fprintf(stderr, "# Check: motor > 0.5 when target is to your RIGHT\n");
    print_header();

    BLIMS blims;
    init_blims(blims);

    // 200 m north + 200 m west of target
    float lat = TARGET_LAT + (200.0f / METERS_PER_DEG_LAT);
    float lon = TARGET_LON - (200.0f / (METERS_PER_DEG_LAT * cosf(TARGET_LAT * (M_PI/180.0f))));
    float alt_ft  = 1500.0f;  // well above alt_downwind_ft (1000 ft) → TRACK

    uint32_t ts = 0;
    for (float heading = 0.0f; heading < 360.0f; heading += 1.0f) {
        BLIMSDataIn  din  = make_data_in(lat, lon, alt_ft, heading, VEHICLE_SPEED_MPS);
        BLIMSDataOut dout = blims.execute(&din);
        print_row(lat, lon, heading, dout, alt_ft, ts);
        ts += CYCLE_MS;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SCENARIO 2: ALTITUDE / PHASE SWEEP
//
// What it tests: phase transitions, I-term accumulation per phase
//
// Fixed position on the downwind leg, descends from 2000 ft → 0 ft.
// One execute() call per 5 ft of descent.
//
// What to look for:
//   - Phase column (col 11) changes at exactly 1000 / 600 / 300 / 100 ft
//   - I term (col 10) accumulates while in TRACK, DOWNWIND, BASE, FINAL
//   - I term resets to 0 on every phase transition (by design)
//   - I term stays 0 the whole time → still gated by phase or dt bug
// ─────────────────────────────────────────────────────────────────────────────

static void scenario_phase() {
    fprintf(stderr, "# SCENARIO: phase / altitude sweep\n");
    fprintf(stderr, "# Descending 2000→0 ft, fixed position, heading South\n");
    fprintf(stderr, "# Check: phase transitions at 1000/600/300/100 ft, I term grows\n");
    print_header();

    BLIMS blims;
    init_blims(blims);

    // 500 m north of target — sitting on the downwind leg
    float lat     = TARGET_LAT + (500.0f / METERS_PER_DEG_LAT);
    float lon     = TARGET_LON;
    float heading = 180.0f;   // facing South (toward target)

    uint32_t ts = 0;
    int steps   = 400;        // 2000 ft / 5 ft per step
    for (int i = 0; i <= steps; i++) {
        float alt_ft = 2000.0f - (2000.0f * i / (float)steps);
        BLIMSDataIn  din  = make_data_in(lat, lon, alt_ft, heading, VEHICLE_SPEED_MPS);
        BLIMSDataOut dout = blims.execute(&din);
        print_row(lat, lon, heading, dout, alt_ft, ts);
        ts += CYCLE_MS;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SCENARIO 3: CLOSED-LOOP DRIVE
//
// What it tests: end-to-end convergence — sign, I term, phase logic together
//
// Vehicle starts offset from target. Motor output steers heading each cycle.
// Simple kinematic model propagates position.
//
// What to look for in the visualizer:
//   - Trail spirals INTO the target (not away from it)
//   - I term grows over ~5–10 seconds and helps correct steady-state error
//   - Reaches target (prints "# REACHED TARGET") within the 60 s run
//   - If trail spirals outward → sign is still wrong
//   - If it reaches target but oscillates → Kp / Ki need tuning
// ─────────────────────────────────────────────────────────────────────────────

static void scenario_drive() {
    fprintf(stderr, "# SCENARIO: closed-loop drive\n");
    fprintf(stderr, "# Start 300m north of target, heading East, alt=1500ft\n");
    fprintf(stderr, "# Check: trail converges to target, I term grows\n");
    print_header();

    BLIMS blims;
    init_blims(blims);

    float lat     = TARGET_LAT + (300.0f / METERS_PER_DEG_LAT);
    float lon     = TARGET_LON;
    float heading = 90.0f;    // facing East — must correct ~180° to face target
    float alt_ft  = 1500.0f;  // TRACK phase throughout

    uint32_t ts   = 0;
    int max_cycles = 1200;    // 60 seconds at 20 Hz

    for (int cycle = 0; cycle < max_cycles; cycle++) {
        BLIMSDataIn  din  = make_data_in(lat, lon, alt_ft, heading, VEHICLE_SPEED_MPS);
        BLIMSDataOut dout = blims.execute(&din);

        print_row(lat, lon, heading, dout, alt_ft, ts);
        ts += CYCLE_MS;

        // ── Kinematic model ──────────────────────────────────────────
        // Motor 0.5 = neutral. Deflection [-0.5, +0.5] drives turn rate.
        float deflection = dout.motor_position - 0.5f;
        float turn_rate  = deflection * TURN_GAIN_DEG_S;   // deg/sec
        heading = fmodf(heading + turn_rate * DT + 360.0f, 360.0f);

        float rad = heading * (M_PI / 180.0f);
        float vN  = VEHICLE_SPEED_MPS * cosf(rad);
        float vE  = VEHICLE_SPEED_MPS * sinf(rad);
        step_position(lat, lon, vN, vE);

        // ── Stop when close enough ────────────────────────────────────
        float dist = distance_m(lat, lon, TARGET_LAT, TARGET_LON);
        if (dist < 5.0f) {
            fprintf(stderr, "# REACHED TARGET at cycle %d  (t=%.1fs, dist=%.1fm)\n",
                    cycle, ts / 1000.0f, dist);
            break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const char* scenario = (argc > 1) ? argv[1] : "drive";

    if      (strcmp(scenario, "sweep") == 0) scenario_sweep();
    else if (strcmp(scenario, "phase") == 0) scenario_phase();
    else if (strcmp(scenario, "drive") == 0) scenario_drive();
    else {
        fprintf(stderr,
            "Usage: blims_sim [sweep|phase|drive]\n"
            "\n"
            "  sweep  — heading 0→360, fixed pos/alt. Checks sign of motor output.\n"
            "  phase  — altitude 2000→0 ft. Checks phase transitions + I term.\n"
            "  drive  — closed-loop sim. Checks full convergence to target.\n"
        );
        return 1;
    }

    return 0;
}