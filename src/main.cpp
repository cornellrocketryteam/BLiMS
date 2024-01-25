#include <Arduino.h>
#include <string.h>//need the .h for arduino
#include "core/state.hpp"
#include "core/fsw.hpp"
#include <SPI.h>
#include <SD.h>

Flight flight;

void setup(){
    //don't need serial lines for actual rocket, just for debugging in console
    Serial.begin(9600);
    while(!Serial){
        ; // wait for serial to connect
    }



//trying to see if connected
//if it doesn't get the response, will fail
//all sensors need this begin
    if (!state::altimeter::bmp.begin_I2C()) {   
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");//error message
    while (1);
  }

}
void loop() {
    flight.execute();
}