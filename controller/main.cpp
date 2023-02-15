#include <cnoid/SimpleController>
#include <cnoid/Body>

#include "myrobot.h"

using namespace cnoid;
//using namespace cnoid::hvac2022;

class HVAC_2022_Controller : public SimpleController{
    public:
        //MyRobot* robot;

    public:
        virtual bool configure(SimpleControllerConfig* config){
            return true;
        }

        virtual bool initialize(SimpleControllerIO* io){
            // âΩÇÁÇ©ÇÃèâä˙âªÇçsÇ§
            return true;
        }

        virtual bool control(){
            //robot->Control();Å@Ç±ÇÒÇ»ä¥Ç∂Ç≈âΩÇÁÇ©ÇÃêßå‰ÇçsÇ§
            return true;
        }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HVAC_2022_Controller)