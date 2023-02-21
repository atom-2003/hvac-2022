#include "footstep.h"
#include "centroid.h"
#include "robot.h"
#include "param.h"

#include <cmath>

using namespace cnoid;
using namespace cnoid::hvac2022;

Centroid::Centroid() {
	com_pos_ref = Vector3(0.0, 0.0, 0.0);
	com_vel_ref = Vector3(0.0, 0.0, 0.0);
	dcm_pos_ref = Vector3(0.0, 0.0, 0.0);
	dcm_vel_ref = Vector3(0.0, 0.0, 0.0);
	dcm_pos     = Vector3(0.0, 0.0, 0.0);

	zmp_pos = Vector3(0.0, 0.0, 0.0);
	zmp_vel = Vector3(0.0, 0.0, 0.0);
	vrp_pos = Vector3(0.0, 0.0, 0.0);

	k_dcm = 10.0;
}

// current reference vrp
//void Centroid::CalcVrpRef(Footstep &footstep, Param &param, double dt) {
//	double T = std::sqrt(param.com_height / param.g.z());
//	
//	step s  = footstep.steps[0];
//	zmp_pos_ref = footstep.steps[0].foot_pos[s.side];
//	vrp_pos_ref = zmp_pos_ref + Vector3(0.0, 0.0, param.com_height);
//}

// current reference dcm
void Centroid::CalcDcmRef(Footstep& footstep, Param &param, double dt) {
	double T = std::sqrt(param.com_height / param.g.z());
	Vector3 dcm = footstep.steps[0].dcm_pos;
	Vector3 vrp = footstep.steps[0].vrp_pos;

	dcm_pos_ref = vrp_pos_ref + exp(dt / T) * (dcm_pos_ref - vrp);
}

// update com command
void Centroid::Update(Footstep& footstep, Param &param, double dt) {
	CalcDcmRef(footstep, param, dt);

	double T = std::sqrt(param.com_height / param.g.z());
	
	// dcm tracking control
	Vector3 dcm = footstep.steps[0].dcm_pos;
	Vector3 vrp = footstep.steps[0].vrp_pos;
	vrp_pos_ref = vrp + (1 + k_dcm * T) * (dcm_pos - dcm_pos_ref);
	zmp_pos_ref = vrp_pos_ref - Vector3(0.0, 0.0, param.com_height);

	// update state variables of centroid
	Vector3 dcm_d = (1 / T) * (dcm_pos_ref - vrp_pos_ref);
	dcm_pos     += dcm_d * dt;
	com_vel_ref += (1 / T) * (dcm_pos_ref - com_pos_ref);
	com_pos_ref += com_vel_ref * dt;

	// additional information
	com_acc_ref = (1 / T) * (dcm_d - com_vel_ref);
}