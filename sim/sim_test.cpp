#include "sim_test.hpp"

SimData::SimData() {
    reset();
}

float SimData::get_alt() {
    if (alt_i == 5537) {
        return -2;
    }
    return alt_sim[alt_i++];
}

bool SimData::has_more_data() const {
    return alt_i < 5537;
}

void SimData::reset() {
    alt_i = 0;
    current_state = {0, 0, 0.5f, 0.0f, 0.0f, 0.0f, 0, 0};
    start_time_ms = 0;
}

void SimData::log_state(const char* label) {
    printf("[%s] Alt=%.1f ft | Phase=%s | Motor=%.3f | Error=%.1f° | P=%.4f | I=%.4f | Loiter=%d | t=%dms\n",
           label,
           current_state.altitude,
           phase_name(current_state.phase),
           current_state.motor_position,
           current_state.heading_error,
           current_state.p_term,
           current_state.i_term,
           current_state.loiter_step,
           current_state.timestamp_ms);
}

void SimData::print_summary() {
    printf("\n=== SIMULATION SUMMARY ===\n");
    printf("Total altitude samples: %d / 5538\n", alt_i);
    printf("Data usage: %.1f%%\n", (alt_i / 5538.0f) * 100.0f);
    printf("Final phase: %s\n", phase_name(current_state.phase));
    printf("Final altitude: %.1f ft\n", current_state.altitude);
    printf("Final motor position: %.3f\n", current_state.motor_position);
    printf("========================\n\n");
}

const char* SimData::phase_name(int phase) {
    switch (phase) {
        case 0: return "HELD";
        case 1: return "TRACK";
        case 2: return "DOWNWIND";
        case 3: return "BASE";
        case 4: return "FINAL";
        case 5: return "NEUTRAL";
        case 6: return "LOITER";
        default: return "UNKNOWN";
    }
}
