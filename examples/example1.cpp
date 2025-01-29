/**
 * @file example1.cpp
 * @author Gabriella Best, gb486
 *
 * @brief example of turning the motor
 */

#include "pico/stdlib.h"
#include "../src/blims.hpp"
#include "../src/blims_constants.hpp"
#include "tusb.h"

int main()
{
  stdio_init_all();
  while (!tud_cdc_connected())
  { // while not connected to serial monitor
    sleep_ms(500);
  }
  BLIMS blims;
  blims.begin(STANDBY);
  printf("Success");
  return 0;
}
