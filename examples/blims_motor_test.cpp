/**
 * @file blims_motor_test.cpp
 * @brief Hardware motor test for BLiMS on Raspberry Pi Pico
 * 
 * PURPOSE: Verify motor/PWM hardware works correctly.
 * Tests all motor positions including new phase-specific positions.
 * Runs on Pico, requires ODrive + motor connected.
 * 
 * WHAT IT TESTS:
 *   1. PWM initialization at 50 Hz
 *   2. Motor neutral position (0.5)
 *   3. Motor min/max limits (0.3, 0.7)
 *   4. Loiter positions (0.35, 0.65)
 *   5. Full range sweep (0.0 to 1.0)
 *   6. Position hold stability
 * 
 * WIRING:
 *   - PWM_PIN (GP10) -> ODrive PWM input
 *   - ENABLE_PIN (GP11) -> ODrive enable
 *   - GND -> ODrive GND
 * 
 * BUILD:
 *   Add to CMakeLists.txt in examples folder, then:
 *   mkdir build && cd build && cmake .. && make blims_motor_test
 * 
 * USAGE:
 *   1. Flash to Pico
 *   2. Open serial monitor (115200 baud)
 *   3. Watch motor move through positions
 *   4. Verify visually that motor responds correctly
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define PWM_PIN        10    // GP10 - PWM to ODrive
#define ENABLE_PIN     11    // GP11 - Enable to ODrive

// PWM settings (must match blims_constants.hpp)
constexpr uint16_t WRAP_CYCLE_COUNT = 65535;
constexpr float PWM_FREQ_HZ = 50.0f;

// Motor positions (must match blims_constants.hpp)
constexpr float NEUTRAL_POS = 0.5f;
constexpr float MOTOR_MIN = 0.3f;
constexpr float MOTOR_MAX = 0.7f;
constexpr float LOITER_LEFT_POS = 0.35f;
constexpr float LOITER_RIGHT_POS = 0.65f;

// Test timing
constexpr uint32_t POSITION_HOLD_MS = 2000;   // Hold each position for 2 sec
constexpr uint32_t SWEEP_STEP_MS = 100;       // Delay between sweep steps

// ============================================================================
// PWM FUNCTIONS
// ============================================================================

static uint slice_num;

void pwm_init_50hz() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    
    // 125 MHz / (50 Hz * 65535) ≈ 38.15 divider
    float divider = 125000000.0f / (PWM_FREQ_HZ * WRAP_CYCLE_COUNT);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, WRAP_CYCLE_COUNT);
    pwm_set_enabled(slice_num, true);
    
    printf("PWM initialized: Pin=%d, Slice=%d, Freq=%.1f Hz\n", 
           PWM_PIN, slice_num, PWM_FREQ_HZ);
}

void set_motor_position(float position) {
    // Clamp to valid range
    if (position < 0.0f) position = 0.0f;
    if (position > 1.0f) position = 1.0f;
    
    // Map 0-1 to 5%-10% duty cycle
    uint16_t five_percent = (uint16_t)(WRAP_CYCLE_COUNT * 0.05f);
    uint16_t duty = five_percent + (uint16_t)(position * five_percent);
    
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), duty);
}

void enable_motor(bool enable) {
    gpio_put(ENABLE_PIN, enable ? 1 : 0);
    printf("Motor %s\n", enable ? "ENABLED" : "DISABLED");
}

// ============================================================================
// TEST FUNCTIONS
// ============================================================================

void test_position(const char* name, float position, uint32_t hold_ms) {
    printf("\n>> Testing: %s (position=%.3f)\n", name, position);
    set_motor_position(position);
    sleep_ms(hold_ms);
    printf("   Hold complete\n");
}

void test_neutral() {
    printf("\n═══════════════════════════════════════\n");
    printf("TEST 1: Neutral Position\n");
    printf("═══════════════════════════════════════\n");
    printf("Motor should be centered (no brake line pull)\n");
    test_position("NEUTRAL", NEUTRAL_POS, POSITION_HOLD_MS);
}

void test_limits() {
    printf("\n═══════════════════════════════════════\n");
    printf("TEST 2: Motor Limits (Safe Range)\n");
    printf("═══════════════════════════════════════\n");
    printf("Testing min/max positions used during flight\n");
    
    test_position("MIN (max left turn)", MOTOR_MIN, POSITION_HOLD_MS);
    test_position("NEUTRAL", NEUTRAL_POS, POSITION_HOLD_MS);
    test_position("MAX (max right turn)", MOTOR_MAX, POSITION_HOLD_MS);
    test_position("NEUTRAL", NEUTRAL_POS, POSITION_HOLD_MS);
}

void test_loiter_positions() {
    printf("\n═══════════════════════════════════════\n");
    printf("TEST 3: Loiter Positions\n");
    printf("═══════════════════════════════════════\n");
    printf("Testing positions used during altitude bleed loiter\n");
    
    test_position("LOITER_RIGHT", LOITER_RIGHT_POS, POSITION_HOLD_MS);
    test_position("NEUTRAL", NEUTRAL_POS, POSITION_HOLD_MS);
    test_position("LOITER_LEFT", LOITER_LEFT_POS, POSITION_HOLD_MS);
    test_position("NEUTRAL", NEUTRAL_POS, POSITION_HOLD_MS);
}

void test_full_range_sweep() {
    printf("\n═══════════════════════════════════════\n");
    printf("TEST 4: Full Range Sweep\n");
    printf("═══════════════════════════════════════\n");
    printf("Sweeping from 0.0 to 1.0 in 0.1 increments\n");
    printf("WARNING: This exceeds safe flight limits!\n");
    
    // Sweep up
    printf("\nSweeping UP (0.0 -> 1.0):\n");
    for (float pos = 0.0f; pos <= 1.0f; pos += 0.1f) {
        printf("  Position: %.1f\n", pos);
        set_motor_position(pos);
        sleep_ms(SWEEP_STEP_MS * 5);
    }
    
    // Hold at max
    sleep_ms(POSITION_HOLD_MS);
    
    // Sweep down
    printf("\nSweeping DOWN (1.0 -> 0.0):\n");
    for (float pos = 1.0f; pos >= 0.0f; pos -= 0.1f) {
        printf("  Position: %.1f\n", pos);
        set_motor_position(pos);
        sleep_ms(SWEEP_STEP_MS * 5);
    }
    
    // Return to neutral
    printf("\nReturning to neutral\n");
    set_motor_position(NEUTRAL_POS);
    sleep_ms(POSITION_HOLD_MS);
}

void test_rapid_transitions() {
    printf("\n═══════════════════════════════════════\n");
    printf("TEST 5: Rapid Transitions\n");
    printf("═══════════════════════════════════════\n");
    printf("Testing quick position changes (simulates active control)\n");
    
    for (int i = 0; i < 5; i++) {
        printf("  Cycle %d/5\n", i + 1);
        set_motor_position(MOTOR_MIN);
        sleep_ms(500);
        set_motor_position(MOTOR_MAX);
        sleep_ms(500);
    }
    
    set_motor_position(NEUTRAL_POS);
    sleep_ms(POSITION_HOLD_MS);
}

void test_hold_stability() {
    printf("\n═══════════════════════════════════════\n");
    printf("TEST 6: Position Hold Stability\n");
    printf("═══════════════════════════════════════\n");
    printf("Holding neutral for 10 seconds - watch for drift\n");
    
    set_motor_position(NEUTRAL_POS);
    for (int i = 10; i > 0; i--) {
        printf("  %d seconds remaining...\n", i);
        sleep_ms(1000);
    }
    printf("  Hold complete - motor should not have drifted\n");
}

void simulate_landing_pattern() {
    printf("\n═══════════════════════════════════════\n");
    printf("TEST 7: Simulated Landing Pattern\n");
    printf("═══════════════════════════════════════\n");
    printf("Simulating phase transitions during descent\n");
    
    printf("\n[Phase 0: TRACK] - Homing to target\n");
    set_motor_position(0.45f);  // Slight correction
    sleep_ms(3000);
    
    printf("\n[Phase 1: DOWNWIND] - Flying with wind\n");
    set_motor_position(0.52f);  // Small trim
    sleep_ms(3000);
    
    printf("\n[Phase 2: BASE] - Perpendicular turn\n");
    set_motor_position(MOTOR_MIN);  // Hard turn
    sleep_ms(2000);
    set_motor_position(NEUTRAL_POS);
    sleep_ms(1000);
    
    printf("\n[Phase 3: FINAL] - Into wind\n");
    set_motor_position(0.48f);  // Hold heading
    sleep_ms(3000);
    
    printf("\n[Phase 4: NEUTRAL] - Hands off for landing\n");
    set_motor_position(NEUTRAL_POS);
    sleep_ms(3000);
    
    printf("\nLanding pattern simulation complete!\n");
}

void simulate_loiter_cycle() {
    printf("\n═══════════════════════════════════════\n");
    printf("TEST 8: Simulated Loiter Cycle\n");
    printf("═══════════════════════════════════════\n");
    printf("Simulating one full loiter cycle (17 seconds)\n");
    
    printf("\n[Step 0] Right turn (6 sec)\n");
    set_motor_position(LOITER_RIGHT_POS);
    sleep_ms(6000);
    
    printf("\n[Step 1] Neutral (2.5 sec)\n");
    set_motor_position(NEUTRAL_POS);
    sleep_ms(2500);
    
    printf("\n[Step 2] Left turn (6 sec)\n");
    set_motor_position(LOITER_LEFT_POS);
    sleep_ms(6000);
    
    printf("\n[Step 3] Neutral (2.5 sec)\n");
    set_motor_position(NEUTRAL_POS);
    sleep_ms(2500);
    
    printf("\nLoiter cycle complete!\n");
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    // Initialize stdio for serial output
    stdio_init_all();
    sleep_ms(2000);  // Wait for serial connection
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║        BLiMS Motor Test - Hardware Verification          ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Configuration:\n");
    printf("  PWM Pin:    GP%d\n", PWM_PIN);
    printf("  Enable Pin: GP%d\n", ENABLE_PIN);
    printf("  PWM Freq:   %.1f Hz\n", PWM_FREQ_HZ);
    printf("  Neutral:    %.2f\n", NEUTRAL_POS);
    printf("  Min/Max:    %.2f / %.2f\n", MOTOR_MIN, MOTOR_MAX);
    printf("  Loiter L/R: %.2f / %.2f\n", LOITER_LEFT_POS, LOITER_RIGHT_POS);
    printf("\n");
    
    // Initialize hardware
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 0);  // Start disabled
    
    pwm_init_50hz();
    set_motor_position(NEUTRAL_POS);
    
    printf("Press any key to start tests (or wait 5 seconds)...\n");
    sleep_ms(5000);
    
    // Enable motor
    enable_motor(true);
    sleep_ms(500);
    
    // Run tests
    test_neutral();
    test_limits();
    test_loiter_positions();
    test_rapid_transitions();
    test_hold_stability();
    simulate_landing_pattern();
    simulate_loiter_cycle();
    
    // Full range sweep (optional - can damage if limits wrong)
    printf("\n");
    printf("Full range sweep test is OPTIONAL and exceeds safe limits.\n");
    printf("Skipping for safety. Uncomment in code if needed.\n");
    // test_full_range_sweep();
    
    // Cleanup
    printf("\n═══════════════════════════════════════\n");
    printf("ALL TESTS COMPLETE\n");
    printf("═══════════════════════════════════════\n");
    
    set_motor_position(NEUTRAL_POS);
    sleep_ms(1000);
    enable_motor(false);
    
    printf("\nMotor disabled. Test complete!\n");
    printf("Review serial output to verify all positions were correct.\n");
    
    // Idle loop
    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}