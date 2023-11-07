#include <Arduino.h>


////////Which libraries do we need to include?
// #include <cstdio>

// #include "pico/stdlib.h"
// #include "hardware/i2c.h"

// #include "fsw.hpp"

// #define I2C_PORT i2c0
// #define I2C_SDA 4
// #define I2C_SCL 5

void setup(){
    Serial.begin(9600);
}
void loop() {
    Serial.println("hello"); //will execute forever
}