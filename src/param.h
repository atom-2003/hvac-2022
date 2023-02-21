#include <cnoid/EigenTypes>

namespace cnoid
{
struct Param
{
	Vector3 g;
	double  mass;
	double  Td; // double support duration
	double  Ts; // single support duration
	double  com_height;
	double  swing_height;

	Param();
};

Param::Param() {
	g    = Vector3(0.0, 0.0, 9.8);
	mass = 50;
	Td   = 0.0;
	Ts   = 0.5;

	com_height   = 0.75;
	swing_height = 0.05;
}
}