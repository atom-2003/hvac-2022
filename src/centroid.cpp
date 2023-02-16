#include "footstep.h"
#include "centroid.h"
#include "robot.h"

#include <cmath>

using namespace cnoid;
using namespace cnoid::hvac2022;

Centroid::Centroid() {
	com_pos = Vector3(0.0, 0.0, 0.0);
	com_vel = Vector3(0.0, 0.0, 0.0);
	dcm_pos = Vector3(0.0, 0.0, 0.0);
	dcm_vel = Vector3(0.0, 0.0, 0.0);
	zmp_pos = Vector3(0.0, 0.0, 0.0);
	zmp_vel = Vector3(0.0, 0.0, 0.0);
	vrp_pos = Vector3(0.0, 0.0, 0.0);
}

void Centroid::CalcVrpRef(Footstep &footstep, Param &param, double dt) {
	double T = std::sqrt(param.com_height / param.g.x());
	
	Footstep::step s = footstep.steps[0];
	zmp_pos = footstep.steps[0].foot_pos[s.side];
	vrp_pos = zmp_pos + Vector3(0.0, 0.0, param.com_height);
}

void Centroid::CalcDcmRef(double dt) {

}