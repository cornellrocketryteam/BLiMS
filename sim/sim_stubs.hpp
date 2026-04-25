/**
 * @file sim_stubs.hpp
 * @brief Pico SDK stubs for desktop (non-Pico) simulation builds
 *
 * Include this before any blims headers when compiling on desktop.
 * Every Pico SDK call used by blims.cpp / blims_state.cpp is stubbed
 * out as a no-op so the logic compiles and runs without hardware.
 *
 * Usage:
 *   g++ -std=c++17 -include sim_stubs.hpp -I../../src -o blims_sim \
 *       blims_sim.cpp ../../src/blims.cpp ../../src/blims_state.cpp
 */

#pragma once
#include <cstdint>
#include <cstdlib>
#include <ctime>

// ── Basic integer types the SDK re-exports ────────────────────────────────────
typedef unsigned int uint;

// ── Alarm / timer ─────────────────────────────────────────────────────────────
typedef int alarm_id_t;

struct _absolute_time { uint64_t _private_us_since_boot; };
typedef struct _absolute_time absolute_time_t;

inline absolute_time_t get_absolute_time() {
    absolute_time_t t;
    t._private_us_since_boot = (uint64_t)(clock()) * 1000000ULL / CLOCKS_PER_SEC;
    return t;
}

inline uint32_t to_ms_since_boot(absolute_time_t t) {
    return (uint32_t)(t._private_us_since_boot / 1000ULL);
}

// Alarm callbacks are no-ops in simulation — loiter timing isn't tested here
inline alarm_id_t add_alarm_in_ms(uint32_t, int64_t(*)(alarm_id_t, void*), void*, bool) {
    return 1;
}
inline bool cancel_alarm(alarm_id_t) { return true; }

// ── GPIO ──────────────────────────────────────────────────────────────────────
#define GPIO_FUNC_PWM   4
#define GPIO_FUNC_I2C   3
#define GPIO_OUT        1
#define GPIO_IN         0

inline void gpio_set_function(uint, uint) {}
inline void gpio_init(uint) {}
inline void gpio_set_dir(uint, uint) {}
inline void gpio_put(uint, int) {}
inline void gpio_pull_up(uint) {}

// ── PWM ───────────────────────────────────────────────────────────────────────
inline uint pwm_gpio_to_slice_num(uint) { return 0; }
inline uint pwm_gpio_to_channel(uint)   { return 0; }
inline void pwm_set_clkdiv(uint, float) {}
inline void pwm_set_wrap(uint, uint16_t) {}
inline void pwm_set_enabled(uint, bool) {}
inline void pwm_set_chan_level(uint, uint, uint16_t) {}

// ── pico/stdlib.h minimal surface ─────────────────────────────────────────────
inline void sleep_ms(uint32_t) {}