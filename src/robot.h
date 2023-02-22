#pragma once

#include <cnoid/Body>
#include <cnoid/SimpleController>
#include <cnoid/BasicSensors>
#include <cnoid/JointPath>

#include <string>
//using namespace std;

namespace cnoid {
namespace hvac2022 {

struct Hand {
	// hand position and orientation 
};

// foot position and orientation
struct Foot {
	Vector3 pos_ref;
	Vector3 vel_ref;
	Vector3 acc_ref;
	Vector3 ori_ref;

	Vector3 force_ref;
	Vector3 moment_ref;
};
// base link position and orientation
struct Base {
	Vector3 pos;
	Vector3 pos_ref;
	Vector3 vel;
	Vector3 acc;
	Vector3 angle;
	Vector3 angvel;
	Vector3 ori;
	Vector3 ori_ref;
};

struct Joint {
	double pgain;
	double dgain;

	double q;
	double qref;
	double dq;
	double dqref;

	double u;
	double uref;
};

class Robot {
public:
	bool base_actuation;
	std::string base_acc_sensor_name;
	std::string base_gyro_sensor_name;
	std::string right_force_sensor_name;
	std::string left_force_sensor_name;

	Body* io_body;
	Body* ik_body;    // body handle for IK, cloned from io_body
	Link* ik_foot[2]; // foot link for IK. 0: right 1: left
	Link* ik_hand[2]; // hand link for IK

	std::shared_ptr<JointPath> baseToFoot_R;
	std::shared_ptr<JointPath> baseToFoot_L;
	std::shared_ptr<JointPath> baseToHand_R;
	std::shared_ptr<JointPath> baseToHand_L;

	AccelerationSensor* accel_sensor;          ///< handle to acceleration sensor of Choreonoid
	RateGyroSensor*     gyro_sensor;           ///< handle to rate gyro sensor of Choreonoid
	ForceSensor*        foot_force_sensor[2];  ///< handle to force sensro of Choreonoid attached to each foot

	Vector3 gyro_axis_x;
	Vector3 gyro_axis_y;
	Vector3 gyro_axis_z;

	double dt;   // time step length
	double time; // plannning timer

	// 0: right, 1: left
	Hand hand[2];
	Foot foot[2];
	Joint joint[30];

	Base base;

	Robot();
	virtual void Init(SimpleControllerIO* io);
	void SolveIK(); // Inverse Kinematics
	void SolveFK(); // Forward Kinematics
	void Sense();
	void Actuate();
};



}
}