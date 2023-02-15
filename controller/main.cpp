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
            // 何らかの初期化を行う
            return true;
        }

        virtual bool control(){
            //robot->Control();　こんな感じで何らかの制御を行う
            return true;
        }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HVAC_2022_Controller)