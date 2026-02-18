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
constexpr float ft_per_m = 3.28084f; // feet per meter conversion

//replaced by ALT_NEUTRAL_FT - no longer our brake altitude
constexpr float brake_alt = 10; // To be updated for when we want BLiMS to brake

constexpr uint32_t initial_hold_threshold = 10000; //parafoil stabilization
// want wrap to be as large as possible, increases the amount of steps so that we have as much control as possible
constexpr uint16_t wrap_cycle_count = 65535;

//0.0 to 1.0 maps to ODrive configuration of -17 to 17 turns
constexpr float neutral_pos = 0.5f; //straight flight
constexpr float motor_min = 0.3f; //max left
constexpr float motor_max = 0.7f; //max right

//////////LV Specific Constants//////////
constexpr float alpha = 0.1f;       // low pass filter value. Higher values increase resistance to noise but slow down the responsiveness of the data to fast changing values - unused in current implementation
constexpr float integral_max = 10.0f; // clamp value for integral term to prevent too much integral windup
//revalidate via car testing**
constexpr float Kp = 0.009f;       // for controller
constexpr float Ki = 0.001f;       // for controller - revalidate 


//////////L3-1 Landing Pattern: altitude-band U-turn, WIND-FROM. feet AGL//////
constexpr float alt_downwind_ft = 1000.0f;   // Start landing pattern
constexpr float alt_base_ft = 600.0f;        // Transition to base leg
constexpr float alt_final_ft = 300.0f;       // Transition to final approach
constexpr float alt_neutral_ft = 100.0f;     // Hands off for landing

// Set radius ("vicinity") + Loiter / altitude bleed
// Units: feet (horizontal distance) and milliseconds
constexpr float set_radius_ft = 400.0f;      // TODO: pick based on field + expected drift

// Loiter pattern: alternate turn right/neutral/left/neutral...
constexpr uint32_t loiter_turn_duration_ms = 6000;   // 6s per turn
constexpr uint32_t loiter_pause_duration_ms = 2500;  // 2.5s pause between turns
constexpr float loiter_right_pos = 0.65f;            // Right turn position
constexpr float loiter_left_pos = 0.35f;             // Left turn position

#endif // BLIMS_CONSTANTS_HPP