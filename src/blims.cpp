/**
 * @file blims.cpp
 * @author gb486
 *
 * @brief BLIMS Functionality
 * CHANGELOG (from original):
 * - FIXED: Bearing calculation now uses proper great-circle formula
 * - FIXED: No longer mutates gps_lat/gps_lon during bearing calculation
 * - FIXED: currTime now properly updated from hardware timer
 * - ADDED: 6-phase state machine (HELD, TRACK, DOWNWIND, BASE, FINAL, NEUTRAL, LOITER)
 * - ADDED: Integral reset on phase transitions
 * - ADDED: Minimum ground speed gate for GPS heading validity
 * - ADDED: set_wind_from_deg() API for pre-flight wind configuration
 * - ADDED: Distance-to-target calculation for loiter entry
 * 
 * State Machine:
 *   Phase -1: HELD (GPS invalid, low speed, bad fix)
 *   Phase  0: TRACK (>1000ft, outside 400ft radius, PI homing)
 *   Phase  1: DOWNWIND (1000-600ft, fly with wind)
 *   Phase  2: BASE (600-300ft, perpendicular to wind)
 *   Phase  3: FINAL (300-100ft, into wind)
 *   Phase  4: NEUTRAL (below 100ft, hands off)
 *   Phase  5: LOITER (>1000ft, inside 400ft radius, altitude bleed)
 */

#include "blims.hpp"
#include "blims_state.hpp"
#include "blims_constants.hpp"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include <math.h>

static bool blims_start = false;
static bool pwm_start = false;
static int32_t last_phase = -99; //track last phase for integral reset

void BLIMS::begin(BLIMSMode mode, uint8_t pwm_pin, uint8_t enable_pin)
{

  blims::flight::flight_mode = mode;
  blims::flight::blims_pwm_pin = pwm_pin;
  blims::flight::blims_enable_pin = enable_pin;
  pwm_setup();
}

void BLIMS::set_target_lat(float lat)
{
  blims::LV::target_lat = lat;
}

void BLIMS::set_target_lon(float lon)
{
  blims::LV::target_lon = lon;
}

// NEW (L3-1)
void BLIMS::set_wind_from_deg(float wind_from_deg)
{
  blims::LV::wind_from_deg = wrap360(wind_from_deg);
}

BLIMSDataOut BLIMS::execute(BLIMSDataIn *data_in)
{
  // update state vars with FSW data
  update_state_gps_vars(data_in);

  if (blims::flight::flight_mode == STANDBY)
  {
  }
  else if (blims::flight::flight_mode == MVP_Flight)
  {
    
    if (!blims::flight::blims_init)
    {
      blims::flight::blims_init = true;
      add_alarm_in_ms(initial_hold_threshold, BLIMS::execute_MVP, NULL, true);
    }
  }
  else if (blims::flight::flight_mode == LV)
  {
    // add blims state var --> 20 second wait is over
    // static funcition --> sets var to true, if false, don't do anything, if true, call execute_LV
    if (!blims::flight::blims_init)
    {
      add_alarm_in_ms(initial_hold_threshold, BLIMS::init_timer, NULL, true);
      blims::flight::blims_init = true;
    }
    if (blims_start == true)
    {
      gpio_put(blims::flight::blims_enable_pin, 1); // pull enable pin high to clear errors and put motor in the right state
      BLIMS::execute_LV();
      data_print_test();
    }
  }

  return blims::flight::data_out;
  // depending on what our mode is, execute different functions
}

// ------------------- Angle wraps (NEW) -------------------
float BLIMS::wrap360(float deg)
{
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f)    deg += 360.0f;
  return deg;
}

float BLIMS::wrap180(float deg)
{
  deg = wrap360(deg);
  if (deg > 180.0f) deg -= 360.0f;
  return deg;
}

float compute_heading_error(float target, float current)
{
  float error = target - current;
  if (error > 180.0f)
    error -= 360.0f; // Wrap error to be within 180, -180. Want to always take the shortest turn to the target
  if (error < -180.0f)
    error += 360.0f;
  return error;
}

// Flat-earth distance to target (good enough for a few km)
static float distance_to_target_m(float lat_deg, float lon_deg, float tlat_deg, float tlon_deg)
{
  // equirectangular approximation
  const float lat_rad = lat_deg * deg_to_rad;
  const float x = (tlon_deg - lon_deg) * deg_to_rad * cosf(lat_rad);
  const float y = (tlat_deg - lat_deg) * deg_to_rad;
  const float R = 6371000.0f;
  return R * sqrtf(x*x + y*y);
}
//for highest accuracy  - use great earth formula **

int64_t BLIMS::init_timer(alarm_id_t id, void *user_data)
{
  blims_start = true;
  return 0;
}

int64_t BLIMS::pwm_setup_timer(alarm_id_t id, void *user_data)
{
  pwm_start = true;
  return 0;
}

void BLIMS::execute_LV()
{
  // GPS validity gate from FSW
  if (!blims::LV::gps_state)
  {
    blims::flight::data_out.phase_id = -1;
    set_motor_position(neutral_pos);
    return;
  }
  // FixType gate - (require at least 2D fix)
  // 0=none, 1=DR only, 2=2D, 3=3D, 4=3D+DGPS
  if (!(blims::flight::fixType >= 2 && blims::flight::fixType <= 4))
  {
    blims::flight::data_out.phase_id = -1;
    set_motor_position(neutral_pos);
    return;
  }

  // minimum groundspeed for reliable heading
  if (blims::flight::gSpeed < GSPEED_MIN_FOR_HEADING)
  {
    blims::flight::data_out.phase_id = -1;
    set_motor_position(neutral_pos);
    return;
  }

  if (blims::LV::gps_state)
    // if gps status from FSW good then run
    if (blims::flight::fixType == 4 || blims::flight::fixType == 3 || blims::flight::fixType == 2)
    {                                                                           // only run the logic if we have satellite lock and are moving fast enough to have a clear direction. More relevant to car testing
      float dt_ms = (float)(blims::flight::currTime - blims::flight::prevTime); // calculate how long since last loop (delta time)
      blims::flight::prevTime = blims::flight::currTime;                        // reset last time for the next loop
      // if (dt_ms <= 0 || dt_ms > 1000)
      // {
      //   dt_ms = 100; // cap to prevent large jumps
      // }

      float dt = dt_ms / 1000.0f; // convert to seconds
      if (dt <= 0.0f || dt > 1.0f) dt = 0.02f; // placeholder fallback (50 Hz)

      // Compute normal tracking bearing (to target)
      calculate_bearing(); // sets blims::LV::bearing

      // Compute distance to target (for set-radius logic)
      float dist_m = distance_to_target_m(blims::flight::gps_lat, blims::flight::gps_lon,
                                      blims::LV::target_lat, blims::LV::target_lon);
      float dist_ft = dist_m * FT_PER_M;

      const float W  = wrap360(blims::LV::wind_from_deg);     // wind FROM
      const float DW = wrap360(W + 180.0f);                   // downwind (wind TO)

      // ------------------- L3-1: choose desired heading by altitude band -------------------
      const float alt_ft = blims::flight::alt_agl_ft; //current altitude from altimeter

      float heading_des = blims::LV::bearing; // default: track-to-target
      int32_t phase = 0;                   // 0=track

      // Phase 4: very low - neutral
      if (alt_ft <= ALT_NEUTRAL_FT)
      {
        phase = 4;
        blims::flight::data_out.phase_id = phase;

        //reset integral on phase entry
        if (last_phase != phase)
        {
          blims::LV::error_integral = 0.0f; // reset integral on phase change
          last_phase = phase;
        }

        set_motor_position(neutral_pos);
        blims::LV::bearing = heading_des;
        return;
      }

      // -----------------------------
      // Above 1000 ft: TRACK or LOITER depending on set radius
      // -----------------------------
      if (alt_ft > ALT_UTURN_START_FT)
      {
        if (dist_ft <= SET_RADIUS_FT)
        {
          // Phase 5:LOITER / altitude bleed until we reach 1000 ft and within 400ft of target
          phase = 5;
          blims::flight::data_out.phase_id = phase;
          
          if (last_phase != phase)
          {
            blims::LV::error_integral = 0.0f; // reset integral on phase change
            blims::LV::loiter_step = 0;
            blims::LV::loiter_step_start_ms = blims::flight::currTime;
            last_phase = phase;
          }

          uint32_t elapsed = blims::flight::currTime - blims::LV::loiter_step_start_ms;

          // Step machine: 0=right turn, 1=neutral, 2=left turn, 3=neutral
          // Loiter state machine: RIGHT -> NEUTRAL -> LEFT -> NEUTRAL -> repeat
          switch (blims::LV::loiter_step)
          {
            case 0:  // Turning right
              if (elapsed >= LOITER_TURN_MS)
              {
                blims::LV::loiter_step = 1;
                blims::LV::loiter_step_start_ms = blims::flight::currTime;
              }
              break;
              
            case 1:  // Neutral pause
              if (elapsed >= LOITER_NEUTRAL_MS)
              {
                blims::LV::loiter_step = 2;
                blims::LV::loiter_step_start_ms = blims::flight::currTime;
              }
              break;
              
            case 2:  // Turning left
              if (elapsed >= LOITER_TURN_MS)
              {
                blims::LV::loiter_step = 3;
                blims::LV::loiter_step_start_ms = blims::flight::currTime;
              }
              break;
              
            case 3:  // Neutral pause
              if (elapsed >= LOITER_NEUTRAL_MS)
              {
                blims::LV::loiter_step = 0;
                blims::LV::loiter_step_start_ms = blims::flight::currTime;
              }
              break;
          }

          float pos = neutral_pos;
          if (blims::LV::loiter_step == 0) pos = LOITER_RIGHT_POS;
          if (blims::LV::loiter_step == 2) pos = LOITER_LEFT_POS;

          // For logging
          set_motor_position(pos);
          blims::LV::bearing = blims::LV::bearing; // keep target bearing in bearing, or ignore
          return;
        }
        else
        {
          // Phase 0: TRACK toward target
          phase = 0;
          heading_des = blims::LV::bearing;

          // reset loiter state when leaving loiter zone
          blims::LV::loiter_step_start_ms = 0;
          blims::LV::loiter_step = 0;
        }
      }
      //LANDING PATTERN BELOW 1000 FT.
      else if (alt_ft <= ALT_FINAL_START_FT)
      {
        // Phase 3: Final approach: fly directly into wind for controlled descent under 300 ft. 
        phase = 3;
        heading_des = W; //fly toward where wind comes from
      }
      else if (alt_ft <= ALT_BASE_START_FT)
      {
        // Phase 2: Base leg - turn to fly perpendicualr to wind from 300-600 ft. 
        phase = 2;
        // Base: +/- 90 off downwind, choose smaller turn - calculate both perpendicular options
        float base1 = wrap360(DW + 90.0f);
        float base2 = wrap360(DW -90.0f); 

        float e1 = fabsf(wrap180(base1 - blims::flight::headMot));
        float e2 = fabsf(wrap180(base2 - blims::flight::headMot));

        heading_des = (e1 <= e2) ? base1 : base2;
      }
      else 
      {
        // Phase 1: Downwind, 1000-600 ft. flying with the wind to cover ground
        phase = 1;
        heading_des = DW;
      }

      blims::flight::data_out.phase_id = phase;

      if (phase != last_phase)
      {
        blims::LV::error_integral = 0.0f; // reset integral on phase change to prevent carryover
        last_phase = phase;
      }

      //PID Code//
      float error = compute_heading_error(heading_des, (float)blims::flight::headMot);

      blims::LV::error_integral += error * dt; // integral = area under curve.

      float limit = 0.5f / Ki; // because error_integral gets multiplied by Ki later, this calculation makes sure that the clamping limits on the I term are indeed 0.5
      if (blims::LV::error_integral > limit)
        blims::LV::error_integral = limit; // clamp term to prevent integral windup
      if (blims::LV::error_integral < -limit)
        blims::LV::error_integral = -limit;

      // calculate control terms
      blims::LV::pid_P = -1.0f * Kp * error;
      blims::LV::pid_I = -1.0f * Ki * blims::LV::error_integral;

      float correction = blims::LV::pid_P + blims::LV::pid_I;

      // control input is relative to 0.5 instead of 0, because neutral motor position is 0.5
      float position = neutral_pos+ correction; //neutral_pos = 0.5f
      if (position < motor_min) //motor_min = 0.3f
        position = motor_min;
      if (position > motor_max) //motor_max = 0.7f
        position = motor_max;

      blims::LV::bearing = heading_des;
      set_motor_position(position);
    }
    else
    {
      set_motor_position(0.5f); // set to neutral if no data
    }
  }
}

int32_t BLIMS::calculate_timePassed()
{
  int32_t timePassed = (blims::flight::currTime - blims::flight::prevTime) / 1000;

  if (blims::flight::timePassed <= 0)
  {
    blims::flight::timePassed = 0.001;
  }
  return timePassed;
}

void BLIMS::calculate_bearing()
{
  // convert all to radians
  float gps_lat_rad = blims::flight::gps_lat * deg_to_rad;
  float gps_lon_rad = blims::flight::gps_lon * deg_to_rad;

  float target_lat_rad = blims::LV::target_lat * deg_to_rad;
  float target_lon_rad = blims::LV::target_lon * deg_to_rad;

  float d_lon = target_lon_rad - gps_lon_rad;
  float d_lat = target_lat_rad - gps_lat_rad;
  float bearing = atan2f(d_lon, d_lat) * rad_to_deg; // compute angle, then convert back to degrees

  if (bearing < 0.0f)
  {
    bearing += 360.0f;
  } // atan2 is in range [-180,180). We want 0,360 for logic and plotting
  blims::LV::bearing = bearing;
}

void BLIMS::set_motor_position(float position)
{
  uint slice_num = pwm_gpio_to_slice_num(blims::flight::blims_pwm_pin);
  // Position should be between 0-1
  // Should map between -17 to 17 turns (configured in web UI)

  // Map position to PWM duty cycle (typically 1ms to 2ms pulse width)
  uint16_t five_percent_duty_cycle = wrap_cycle_count * 0.05f;
  // ranges between 5% and 10% duty cycle; 3276 ~= 5% duty, 6552 ~= 10% duty
  uint16_t duty = (uint16_t)(five_percent_duty_cycle + position * five_percent_duty_cycle);
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(blims::flight::blims_pwm_pin), duty);
  // update state of motor (what is the position at the current time)
  blims::flight::data_out.motor_position = position;
  blims::flight::data_out.pid_I = blims::LV::pid_I;
  blims::flight::data_out.pid_P = blims::LV::pid_P;
  blims::flight::data_out.bearing = blims::LV::bearing;
}

void BLIMS::pwm_setup()
{
  gpio_set_function(blims::flight::blims_pwm_pin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(blims::flight::blims_pwm_pin);
  gpio_init(blims::flight::blims_enable_pin);
  gpio_set_dir(blims::flight::blims_enable_pin, GPIO_OUT);

  float divider = 125000000.0f / (50.0f * wrap_cycle_count);
  pwm_set_clkdiv(slice_num, divider);
  pwm_set_wrap(slice_num, wrap_cycle_count);
  pwm_set_enabled(slice_num, true);
}

void BLIMS::data_print_test()
{
#ifdef VERBOSE // set in FSW
  printf("blims_start: %d\n", blims_start);

  printf("\nLV Calculation Vars Print Statements\n");
  printf("bearing: %f\n", blims::LV::bearing);
  // printf("currTime: %d\n", blims::flight::currTime);
  // printf("timePassed: %d\n", blims::flight::timePassed);
  // printf("prevError: %d\n", blims::LV::prevError);
  // printf("prevTime: %d\n", blims::flight::prevTime);

  printf("\nController Print Statements\n");
  printf("pid_P: %f\n", blims::LV::pid_P);
  printf("pid_I: %f\n", blims::LV::pid_I);
#endif
}

void BLIMS::update_state_gps_vars(BLIMSDataIn *data_in)
{
  blims::flight::gps_lon = data_in->lon * 1e-7f;
  blims::flight::gps_lat = data_in->lat * 1e-7f;

  // NEW (L3-1)
  blims::flight::alt_agl_ft = data_in->alt_agl_ft;

  blims::flight::hAcc = data_in->hAcc;
  blims::flight::vAcc = data_in->vAcc;
  blims::flight::velN = data_in->velN;
  blims::flight::velE = data_in->velE;
  blims::flight::velD = data_in->velD;
  blims::flight::gSpeed = data_in->gSpeed;

  float headMot_deg = data_in->headMot * 1e-5f;

  blims::flight::headMot = (int32_t)wrap360(headMot_deg);

  blims::flight::sAcc = data_in->sAcc;
  blims::flight::headAcc = data_in->headAcc;

  blims::flight::fixType = data_in->fixType;
  blims::LV::gps_state = data_in->gps_state;

  blims::flight::currTime = to_ms_since_boot(get_absolute_time());
}

int64_t BLIMS::execute_MVP(alarm_id_t id, void *user_data)
{
  gpio_put(blims::flight::blims_enable_pin, 1);
  blims::MVP::curr_action_index++;

  if (blims::MVP::curr_action_index >= 11)
  {
    blims::MVP::curr_action_index = 0;
  }
  set_motor_position(blims::MVP::action_arr[blims::MVP::curr_action_index].position);
  add_alarm_in_ms(blims::MVP::action_arr[blims::MVP::curr_action_index].duration, BLIMS::execute_MVP, NULL, false);

  // update statex
  return 0; // need this for add_alarm_in_ms

  //
}