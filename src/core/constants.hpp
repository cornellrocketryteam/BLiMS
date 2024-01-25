#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_



namespace constants {
    //constexpr = won't ever change value
    //rocket starts going down
    namespace altimeter{
        constexpr float ref_pressure = 1013.25; //ground pressure in pascals from library
    }
}


#endif // CONSTANTS_HPP_