#include "flight_mode.hpp"
#include "state.hpp"
#include <SPI.h>
#include <SD.h>

File sdLog;


void FlightMode::execute() {
    // TODO: Poll sensors
    //takes current pressure and compares with ground pressure
    float altitude = bmp.readAltitude(constants::altimeter::ref_pressure); 
    
    //https://docs.arduino.cc/learn/programming/sd-guide

}

void beforeMainDeployed::transition() {
    // Try to init all sensors
    //add a bunch of if statements

}

void afterMainDeployed::execute() {
    // Try to init all sensors
}

void afterMainDeployed::transition() {
    // Try to init all sensors
}

void groundState::transition() {
    // Try to init all sensors
}