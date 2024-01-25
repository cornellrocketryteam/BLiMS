#ifndef FLIGHT_MODE_
#define FLIGHT_MODE_
//this files cares about altimeter, other files only care about data from altimeter

#include <string.h> //allows me to make std string

#include "constants.hpp"





class FlightMode {
public:
    void execute();
    virtual void transition() = 0;
    virtual int id() = 0;
    virtual std::string name() = 0;//does it need string in here?
};

class beforeMainDeployed : public FlightMode {//what exactly is this line doing
    public:
    //void execute(); i don't need this right? because nothing is being executed at this time
    void transition(); 

    int id() { return 0; }
    std::string name() { return "BeforeMainDeployed"; };
};

class afterMainDeployed : public FlightMode {
    public:
    void execute();
    void transition();

    int id() { return 1; }
    std::string name() { return "AfterMainDeployed"; };
};


class groundState : public FlightMode {//what exactly is this line doing
    public:
    void transition(); //do we want a transition or execute here if this is the final stage?

    int id() { return 2; }
    std::string name() { return "GroundState"; };
};


#endif // FLIGHT_MODE_