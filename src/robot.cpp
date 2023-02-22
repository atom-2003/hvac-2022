#include "robot.h"
#include "centroid.h"
#include "footstep.h"
#include "param.h"
#include <cnoid/Body>

namespace cnoid {
namespace hvac2022{

Robot::Robot() {

	base_acc_sensor_name    = "gsensor";
	base_gyro_sensor_name   = "gyrosensor";
	right_force_sensor_name = "rfsensor";
	left_force_sensor_name  = "rfsensor";

	gyro_axis_x = Vector3(1.0, 0.0, 0.0);
	gyro_axis_y = Vector3(0.0, 1.0, 0.0);
	gyro_axis_z = Vector3(0.0, 0.0, 1.0);

	base_actuation = false;
}

// configure and initialize handle of robot body and sensors
void Robot::Init(SimpleControllerIO* io) {
	time = 0.0;
	dt   = 0.001;

	joint[0] .pgain = 1000.0; joint[0] .dgain = 200.0;
	joint[1] .pgain = 1000.0; joint[1] .dgain = 200.0;
	joint[2] .pgain = 1000.0; joint[2] .dgain = 200.0;
	joint[3] .pgain = 1000.0; joint[3] .dgain = 200.0;
	joint[4] .pgain = 1000.0; joint[4] .dgain = 200.0;
	joint[5] .pgain = 1000.0; joint[5] .dgain = 200.0;
	joint[6] .pgain = 1000.0; joint[6] .dgain = 200.0;
	joint[7] .pgain = 1000.0; joint[7] .dgain = 200.0;
	joint[8] .pgain = 1000.0; joint[8] .dgain = 200.0;
	joint[9] .pgain = 1000.0; joint[9] .dgain = 200.0;
	joint[10].pgain = 1000.0; joint[10].dgain = 200.0;
	joint[11].pgain = 1000.0; joint[11].dgain = 200.0;
	joint[12].pgain = 1000.0; joint[12].dgain = 200.0;
	joint[13].pgain = 1000.0; joint[13].dgain = 200.0;
	joint[14].pgain = 1000.0; joint[14].dgain = 200.0;
	joint[15].pgain = 1000.0; joint[15].dgain = 200.0;
	joint[16].pgain = 1000.0; joint[16].dgain = 200.0;
	joint[17].pgain = 1000.0; joint[17].dgain = 200.0;
	joint[18].pgain = 1000.0; joint[18].dgain = 200.0;
	joint[19].pgain = 1000.0; joint[19].dgain = 200.0;
	joint[20].pgain = 1000.0; joint[20].dgain = 200.0;
	joint[21].pgain = 1000.0; joint[21].dgain = 200.0;
	joint[22].pgain = 1000.0; joint[22].dgain = 200.0;
	joint[23].pgain = 1000.0; joint[23].dgain = 200.0;
	joint[24].pgain = 1000.0; joint[24].dgain = 200.0;
	joint[25].pgain = 1000.0; joint[25].dgain = 200.0;
	joint[26].pgain = 1000.0; joint[26].dgain = 200.0;
	joint[27].pgain = 1000.0; joint[27].dgain = 200.0;
	joint[28].pgain = 1000.0; joint[28].dgain = 200.0;
	joint[29].pgain = 1000.0; joint[29].dgain = 200.0;

	io_body = io->body();

	ik_body = io_body->clone();
	ik_foot[0] = ik_body->link("R_FOOT_R");
	ik_foot[1] = ik_body->link("L_FOOT_R");
	ik_hand[0] = ik_body->link("R_HAND_R");
	ik_hand[1] = ik_body->link("L_HAND_R");

	baseToFoot_R = JointPath::getCustomPath(ik_body, ik_body->rootLink(), ik_foot[0]);
	baseToFoot_L = JointPath::getCustomPath(ik_body, ik_body->rootLink(), ik_foot[1]);
	baseToHand_R = JointPath::getCustomPath(ik_body, ik_body->rootLink(), ik_hand[0]);
	baseToHand_L = JointPath::getCustomPath(ik_body, ik_body->rootLink(), ik_hand[1]);

	if (base_actuation)
	{
		io_body->link(0)->setActuationMode(cnoid::Link::LinkPosition);
		io->enableIO(io_body->link(0));
	}
	io->enableInput(io_body->link(0), cnoid::Link::LinkPosition);

	for (int i = 0; i < io_body->numJoints(); i++)
	{
		cnoid::Link* jnt = io_body->joint(i);

		jnt->setActuationMode(cnoid::Link::JointTorque);
		io->enableIO(jnt);
		io->enableInput(jnt, cnoid::Link::JointVelocity);

		joint[i].qref  = joint[i].q = jnt->q();
		joint[i].dqref = joint[i].dq = jnt->dq();
		joint[i].u     = 0.0;
	}

	{
		accel_sensor = io_body->findDevice<AccelerationSensor>(base_acc_sensor_name );
		gyro_sensor  = io_body->findDevice<RateGyroSensor    >(base_gyro_sensor_name);

		if (accel_sensor) io->enableInput(accel_sensor);
		if (gyro_sensor)  io->enableInput(gyro_sensor );
	}

	for (int i = 0; i < 2; i++)
	{
		foot_force_sensor[i] = io_body->findDevice<ForceSensor>(i == 0 ? right_force_sensor_name : right_force_sensor_name);
		if (foot_force_sensor[i])
			io->enableInput(foot_force_sensor[i]);
	}

	dt = io->timeStep();
}

void Robot::SolveFK(){

}
void Robot::SolveIK(){}


void Robot::Sense(){
	Link* lnk = io_body->link(0);
	base.pos = lnk->p();

	if (accel_sensor)
	{
		Vector3 a = accel_sensor->dv();
		base.acc[0] = gyro_axis_x.dot(a);
		base.acc[1] = gyro_axis_y.dot(a);
		base.acc[2] = gyro_axis_z.dot(a);
	}
	if (gyro_sensor)
	{
		Vector3 w = gyro_sensor->w();
		base.angvel[0] = gyro_axis_x.dot(w);
		base.angvel[1] = gyro_axis_y.dot(w);
		base.angvel[2] = gyro_axis_z.dot(w);
	}

	base.angle[0] += (base.angvel[0]) * dt;
	base.angle[1] += (base.angvel[1]) * dt;
	base.angle[2] += (base.angvel[2]) * dt;

	for (int i = 0; i < 2; i++)
	{
		if (foot_force_sensor[i])
		{
			Vector3 f = foot_force_sensor[i]->F().segment<3>(0);
			Vector3 m = foot_force_sensor[i]->F().segment<3>(3);
			for (int j = 0; j < 3; j++)
			{
				//foot[i].force[j]  = f[j];
				//foot[i].moment[j] = m[j];
			}
		}
	}
}
void Robot::Actuate(){
	if (base_actuation)
	{
		Link* lnk = io_body->link(0);
		lnk->p() = base.pos_ref;
		//lnk->R() = base.ori_ref.matrix(); //TODO: ori_ref‚ÍQuartanionŒ^‚É‚·‚é
	}
	for (int i = 0; i < io_body->numJoints(); i++)
	{
		Link* jnt = io_body->joint(i);
		joint[i].u = joint[i].pgain * (joint[i].qref - jnt->q()) + joint[i].dgain * (joint[i].dqref - jnt->dq());
		jnt->u() = joint[i].u;
	}
}
}
}