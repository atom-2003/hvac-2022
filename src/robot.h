#pragma once

#include <cnoid/Body>
#include <cnoid/SimpleController>
#include <cnoid/BasicSensors>

namespace cnoid {
namespace hvac2022 {

class Robot {
public:
	bool base_actuation;
	Body* io_body;
	AccelerationSensor* accel_sensor;          ///< handle to acceleration sensor of Choreonoid
	RateGyroSensor*     gyro_sensor;           ///< handle to rate gyro sensor of Choreonoid
	ForceSensor*        foot_force_sensor[2];  ///< handle to force sensro of Choreonoid attached to each foot

	double dt;   // time step length
	double time; // simulation timer

	// 0: right, 1: left
	Hand hand[2];
	Foot foot[2];

	void Init(SimpleControllerIO* io);
	void SolveIK(); // Inverse Kinematics
	void SolveFK(); // Forward Kinematics
	Robot();
};

struct Hand {
	// hand position and orientation 
};
struct Foot {
	// foot position and orientation
};
struct Base {
	// base link position and orientation
};

}
}