#include "myrobot.h"
using namespace std;

namespace cnoid {
namespace hvac2022 {

MyRobot::MyRobot() {
	base_actuation = false;
}

void MyRobot::Init(SimpleControllerIO* io) {
	Robot::Init(io);

	centroid.com_pos_ref = Vector3(0.0, 0.0, param.com_height);
	centroid.dcm_pos_ref = Vector3(0.0, 0.0, param.com_height);
	step s0, s1;
	s0.stride  = 0.0; s1.stride  = 0.0;
	s0.spacing = 0.2; s1.spacing = 0.0;
	s0.side    = 0  ; s1.side    = 1;

	footstep.steps.push_back(s0);
	footstep.steps.push_back(s1);
	footstep.GeneratePos(param);

}

void MyRobot::Control() {
	Robot::Sense();
	
	while (footstep.steps.size() > 2)
		footstep.steps.pop_back();

	step step;
	step.stride = 0.1;
	step.spacing = 0.2;
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
	step.stride = 0.0;
	footstep.steps.push_back(step);
	footstep.GeneratePos(param);

	// TODO: ‚©‚È‚è‰ö‚µ‚¢
	centroid.Update(footstep, param, dt);

	//foot[0].pos_ref = Vector3(0.0, 0.0, 0.0);
	//foot[0].ori_ref = Vector3(0.0, 0.0, 0.0);
	//foot[1].pos_ref = Vector3(0.0, 0.0, 0.0);
	//foot[1].ori_ref = Vector3(0.0, 0.0, 0.0);
	//hand[0].pos_ref = Vector3(0.0, 0.0, 0.0);
	//hand[0].ori_ref = Vector3(0.0, 0.0, 0.0);
	//hand[1].pos_ref = Vector3(0.0, 0.0, 0.0);
	//hand[1].ori_ref = Vector3(0.0, 0.0, 0.0);

	// ‚±‚±‚Å‹t‰^“®ŠwŒvŽZ‚·‚é
	Robot::Actuate();
	time += dt;
}
}
}