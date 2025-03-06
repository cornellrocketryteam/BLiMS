#ifndef BLIMS_CONSTANTS_HPP
#define BLIMS_CONSTANTS_HPP
#include <cstdint>

constexpr float brake_alt = 10; // To be updated for when we want BLiMS to brake
constexpr uint32_t initial_hold_threshold = 20000;
// want wrap to be as large as possible, increases the amount of steps so that we have as much control as possible
constexpr uint16_t wrap_cycle_count = 65535;

//////////MVP Specific Constants//////////
constexpr uint32_t turn_hold_threshold = 10000;
constexpr uint32_t neutral_hold_threshold = 7500;
constexpr float neutral_pos = 0.5;

//////////LV Specific Constants//////////
constexpr float alpha = 0.1;       // low pass filter value. Higher values increase resistance to noise but slow down the responsiveness of the data to fast changing values
constexpr float integral_max = 10; // clamp value for integral term to prevent too much integral windup
constexpr float Kp = 20;           // for controller
constexpr float Ki = 0;            // for controller
constexpr float Kd = 0;            // for controller

#endif // BLIMS_CONSTANTS_HPP