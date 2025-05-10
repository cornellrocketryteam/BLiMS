#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "ublox_mx.hpp"
#include "ublox_nav_pvt.hpp"

#include <cmath>
#include <iostream>
#include <iomanip>

#define PWM_PIN 28
#define ODRIVE_STATE_PIN 0 // enable pin. Check all these pins with the wiring diagram as this test script might not match what the LV is wired for
#define I2C_PORT i2c0
#define I2C_SDA 12 // gps pins
#define I2C_SCL 13

// ==========================
// Motor control setup
// ==========================
uint16_t wrap_cycle_count = 65535;

void setup_pwm_50hz(uint gpio_pin)
{
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    gpio_init(ODRIVE_STATE_PIN);
    gpio_set_dir(ODRIVE_STATE_PIN, GPIO_OUT);

    float divider = 125000000.0f / (50 * wrap_cycle_count);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, wrap_cycle_count);
    pwm_set_enabled(slice_num, true);
}

void set_motor_position(uint gpio_pin, float position)
{
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    uint16_t five_percent_duty_cycle = wrap_cycle_count * 0.05;
    uint16_t duty = five_percent_duty_cycle + position * five_percent_duty_cycle;
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio_pin), duty);
}

// ==========================
// GPS + Control
// ==========================
GNSS gps(I2C_PORT);
const float Kp = 0.009f; // gains for controller
const float Ki = 0.002f;

const double target_lat = 42.446610; // target is hardcoded in here
const double target_lon = -76.461304;

float compute_heading_error(float target, float current)
{
    float error = target - current;
    if (error > 180)
        error -= 360; // Wrap error to be within 180, -180. Want to always take the shortest turn to the target
    if (error < -180)
        error += 360;
    return error;
}

float compute_bearing(float lat1, float lon1, float lat2, float lon2)
{
    const float deg_to_rad = M_PI / 180.0f; // constants used for rad/deg conversion
    const float rad_to_deg = 180.0f / M_PI;

    // convert all to radians
    lat1 *= deg_to_rad; // gps lat
    lon1 *= deg_to_rad; // gps lon
    lat2 *= deg_to_rad; // target lat
    lon2 *= deg_to_rad; // target lon

    float d_lon = lon2 - lon1;
    float d_lat = lat2 - lat1;
    float bearing = atan2(d_lat, d_lon) * rad_to_deg; // compute angle, then convert back to degrees

    if (bearing < 0)
        bearing += 360.0f; // atan2 is in range [-180,180). We want 0,360 for logic and plotting
    return bearing;
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);
    setup_pwm_50hz(PWM_PIN);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    while (!tud_cdc_connected())
    { // this test script only starts pulling gps values and sending motor commands after the serial port has been opened
        sleep_ms(500);
    }
    printf("Connected\n");

    if (!gps.begin_PVT(20))
    {
        printf("Failed to init GPS\n");
        return 1;
    }

    UbxNavPvt data = {0};

    gpio_put(ODRIVE_STATE_PIN, 1); // pull enable pin high to clear errors and put motor in the right state

    float error_integral = 0.0f;                     // initialize accumulated error as zero
    absolute_time_t last_time = get_absolute_time(); // initialize variable that keeps track of timing for integral calculation

    while (true)
    {
        if (gps.read_PVT_data(&data))
        { // if gps status from FSW good then run
            float current_lat = data.lat * 1e-7f;
            float current_lon = data.lon * 1e-7f;
            float heading = ((data.headMot * 1e-5f) - 90) * -1; // IMPORTANT: Transform to polar coords by shifting 90 degrees and flipping
            if (heading < 0)
                heading += 360.0f; // then wrap to 0,360. This variable is technically heading of motion, couldve been more precise with my naming

            float ground_speed = data.gSpeed / 1000.0f; // ground speed isn't actually used for anything rn

            if (data.fixType >= 3 && ground_speed > 0.3)
            { // only run the logic if we have satellite lock and are moving fast enough to have a clear direction. More relevant to car testing than actual flight but do make sure that the code doesn't break if the expected data isn't returned for a loop or two
                absolute_time_t now = get_absolute_time();
                float dt_ms = to_ms_since_boot(now) - to_ms_since_boot(last_time); // calculate how long since last loop (delta time)
                last_time = now;                                                   // reset last time for the next loop
                if (dt_ms <= 0 || dt_ms > 1000)
                    dt_ms = 100;            // cap to prevent large jumps
                float dt = dt_ms / 1000.0f; // convert to seconds

                float target_heading = compute_bearing(current_lat, current_lon, target_lat, target_lon);
                float error = compute_heading_error(target_heading, heading);

                error_integral += error * dt; // integral = area under curve. This is the discritized version of that
                float limit = 0.5f / Ki;      // because error_integral gets multiplied by Ki later, this calculation makes sure that the clamping limits on the I term are indeed 0.5
                if (error_integral > limit)
                    error_integral = limit; // clamp term to prevent integral windup
                if (error_integral < -limit)
                    error_integral = -limit;

                // calculate control terms
                float P_term = -1 * Kp * error;
                float I_term = -1 * Ki * error_integral;

                float correction = P_term + I_term;

                // control input is relative to 0.5 instead of 0, because neutral motor position is 0.5
                float position = 0.5f + correction;
                if (position < 0.0f)
                    position = 0.0f;
                if (position > 1.0f)
                    position = 1.0f;

                set_motor_position(PWM_PIN, position);

                // Log everything. Print statements for the visualizer, but in FSW should be logged to SD card if possible
                printf("%.7f,%.7f,%.7f,%.7f,%.2f,%.2f,%.3f,%llu,%.3f,%.3f\n",
                       current_lat, current_lon,
                       target_lat, target_lon,
                       heading, target_heading,
                       position, // motor position
                       to_ms_since_boot(now),
                       P_term, I_term);
            }
            else
            {
                printf("Waiting for valid fix or movement (Fix: %d, Speed: %.2f)\n", data.fixType, ground_speed);
                set_motor_position(PWM_PIN, 0.5f); // set to neutral if no data
            }
        }
        sleep_ms(100); // loop runs at 10Hz
    }

    return 0;
}
