#pragma once 

#include "centroid.h"
#include "footstep.h"
#include "param.h"
#include "robot.h"

namespace cnoid {
namespace hvac2022 {

class MyRobot : public Robot {
public:
	double standby_period;
	double standby_com_height;

	int plan_cycle;
	
	Param param;
	Centroid centroid;
	Footstep footstep;

public:
	virtual void Init(SimpleControllerIO* io);
	virtual void Control();

	MyRobot();
};
}
}