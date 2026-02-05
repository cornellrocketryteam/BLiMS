// - Added altitude thresholds for DOWNWIND/BASE/FINAL phases
// - Added loiter parameters for energy management
// - Added motor limits and speed gates
// - Defined FT_PER_M conversion factor
#ifndef BLIMS_CONSTANTS_HPP
#define BLIMS_CONSTANTS_HPP
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

//unit conversions
constexpr float deg_to_rad = M_PI / 180.0f;
constexpr float rad_to_deg = 180.0f / M_PI;
constexpr float FT_PER_M = 3.28084f; // feet per meter conversion

//replaced by ALT_NEUTRAL_FT - no longer our brake altitude
constexpr float brake_alt = 10; // To be updated for when we want BLiMS to brake

constexpr uint32_t initial_hold_threshold = 10000; //parafoil stabilization
// want wrap to be as large as possible, increases the amount of steps so that we have as much control as possible
constexpr uint16_t wrap_cycle_count = 65535;

//////////MVP Specific Constants//////////
constexpr uint32_t turn_hold_threshold = 10000; //10 sec per turn
constexpr uint32_t neutral_hold_threshold = 7500; //7.5 sec neutral
//0.0 to 1.0 maps to ODrive configuration of -17 to 17 turns
constexpr float neutral_pos = 0.5f; //straight flight
constexpr float motor_min = 0.3f; //max left
constexpr float motor_max = 0.7f; //max right

//////////LV Specific Constants//////////
constexpr float alpha = 0.1f;       // low pass filter value. Higher values increase resistance to noise but slow down the responsiveness of the data to fast changing values - ununsed in current implementation
constexpr float integral_max = 10.0f; // clamp value for integral term to prevent too much integral windup
//revalidate via car testing**
constexpr float Kp = 0.009f;       // for controller
constexpr float Ki = 0.001f;       // for controller - revalidate 


//////////L3-1 Landing Pattern: altitude-band U-turn, WIND-FROM. feet AGL//////
constexpr float ALT_UTURN_START_FT = 1000.0f; //begin landing pattern
constexpr float ALT_BASE_START_FT  = 600.0f; //turn perpendiular to downwind
constexpr float ALT_FINAL_START_FT = 300.0f; //turn into wind
constexpr float ALT_NEUTRAL_FT     = 100.0f; //hands off for landing flare

// Set radius ("vicinity") + Loiter / altitude bleed
// Units: feet (horizontal distance) and milliseconds
constexpr float SET_RADIUS_FT = 400.0f;      // TODO: pick based on field + expected drift

// Loiter pattern: alternate turn right/neutral/left/neutral...
constexpr float LOITER_RIGHT_POS = 0.65f;    // right turn: **TODO: tune (more aggressive if desired)
constexpr float LOITER_LEFT_POS  = 0.35f; // left turn
constexpr uint32_t LOITER_TURN_MS    = 6000; // turn duration - 6 sec.
constexpr uint32_t LOITER_NEUTRAL_MS = 2500; // neutral pause duration btwn turns - 2.5 sec.

// Minimum groundspeed for reliable heading (mm/s)
//GPS heading is unreliable at low speeds due to position noise
// 3000 mm/s = 3 m/s.
constexpr int32_t GSPEED_MIN_FOR_HEADING = 3000; // 3m/s

#endif // BLIMS_CONSTANTS_HPP