//container for all data to be accessed
//declares variables

//neeed to check what vars each component actually provides

#include "state.hpp"

#include <stdint.h>
 



class FlightMode;

namespace state {
    namespace flight {
        //add modes
        
    }

    namespace altimeter{
        Adafruit_BMP3XX bmp; // creates instance of altimeter
    }

    namespace stepper_1 {
        extern bool init;
        extern float position; //what are the actual vars?
    }

    namespace stepper_2 {
        extern bool init;
        extern float position;
    }

    namespace compass {
        extern bool init;
        //what variables
    }

    namespace altimeter {
        extern bool init;
        extern float altitude;
    }

    namespace gps {
        extern bool init;
        extern float latitude;
        extern float longitude;
        extern float altitude;
        extern uint8_t siv;
    }

    namespace sd {
        extern bool init;
    }




}