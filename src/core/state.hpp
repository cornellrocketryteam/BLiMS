//declares variables
#include <Adafruit_Sensor.h> //common code used for adafruit
#include "Adafruit_BMP3XX.h" //library for altimeter

namespace state {
    namespace altimeter{
        extern Adafruit_BMP3XX bmp; // creates instance of altimeter
    }
}
    