
//////Which libraries do we need?
//Are these headers referenced later?


//says what flight execute does

#include "fsw.hpp"


void Flight::execute() {
    // sleep_ms(5000);
    state::flight::mode->execute();
    state::flight::mode->transition();
}

//gets state from state.hpp