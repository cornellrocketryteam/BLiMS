/**
 * @file example1.cpp
 * @author Gabriella Best, gb486
 *
 * @brief example of turning the motor
 */

#include "pico/stdlib.h"
#include "../src/blims.hpp"
#include "../src/blims_constants.hpp"

int main()
{
  stdio_init_all();
  BLIMS blims;
  blims.begin(STANDBY);
  printf("Success");
  return 0;
}
