#pragma once

#include <cnoid/Body>
#include <cnoid/SimpleController>

namespace cnoid {
namespace hvac2022 {

class Robot {
public:
	bool base_actuation;
	Body* io_body;
	double dt;   // time step length
	double time; // simulation timer

	struct Hand {
		// hand position and arm joint state 
	};
	struct Foot {
		// foot position and leg joint state
	};

	void Init(SimpleControllerIO* io);
	void SolveIK(); // Inverse Kinematics
	void SolveFK(); // Forward Kinematics
	Robot();
};
}
}