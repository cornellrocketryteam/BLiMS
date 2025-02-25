#ifndef BLIMS_CONSTANTS_HPP
#define BLIMS_CONSTANTS_HPP
#include <cstdint>

constexpr float brake_alt = 10; // To be updated for when we want BLiMS to brake
/////NEED TO FIX -  GET FROM BLIMS MODULE
constexpr float blims_motor = 10; // pin for motor
constexpr uint32_t initial_hold_threshold = 20000;
// want wrap to be as large as possible, increases the amount of steps so that we have as much control as possible
constexpr uint16_t wrap_cycle_count = 65535;

//////////MVP Specific Constants//////////
constexpr uint32_t turn_hold_threshold = 10000;
constexpr uint32_t neutral_hold_threshold = 7500;
constexpr float neutral_pos = 0.5;

#endif // BLIMS_CONSTANTS_HPP