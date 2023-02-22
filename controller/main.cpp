#include <cnoid/SimpleController>
#include <cnoid/Body>

#include "myrobot.h"

using namespace cnoid;
using namespace cnoid::hvac2022;

class HVAC_2022_Controller : public SimpleController{
public:
    MyRobot* robot;

public:
    virtual bool configure(SimpleControllerConfig* config){
        return true;
    }

    virtual bool initialize(SimpleControllerIO* io){
        robot = new MyRobot;
        robot->Init(io);
        return true;
    }

    virtual bool control(){
        robot->Control();
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HVAC_2022_Controller)