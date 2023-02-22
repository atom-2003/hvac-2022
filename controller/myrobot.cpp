#include "myrobot.h"
#include <cnoid/EigenUtil>

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
	VectorXd foot_r(6);
	VectorXd foot_l(6);
	VectorXd hand_r(6);
	VectorXd hand_l(6);
	Isometry3 T;
	T.linear() = rotFromRpy(Vector3(foot_r.tail<3>()));
	T.translation() = foot_r.head<3>();
	if (baseToFoot_R->calcInverseKinematics(T))
	{
		for (int i = 0; i < baseToFoot_R->numJoints(); i++)
		{
			Link* jnt = baseToFoot_R->joint(i);
			joint[jnt->jointId()].qref = jnt->q();
		}
	}
	baseToFoot_R->calcInverseKinematics();

	T.linear() = rotFromRpy(Vector3(foot_l.tail<3>()));
	T.translation() = foot_l.head<3>();
	if (baseToFoot_L->calcInverseKinematics(T))
	{
		for (int i = 0; i < baseToFoot_L->numJoints(); i++)
		{
			Link* jnt = baseToFoot_L->joint(i);
			joint[jnt->jointId()].qref = jnt->q();
		}
	}
	baseToFoot_L->calcInverseKinematics(T);

	// ƒgƒ‹ƒNŽw—ß’l
	Robot::Actuate();
	time += dt;
}
}
}