#include <cnoid/EigenTypes>

namespace cnoid
{
struct Param
{
	Vector3 g;
	double  mass;
	double  com_height;
	double  Td; // double support duration
	double  Ts; // single support duration

	Param();
};

Param::Param() {
	g = Vector3(0.0, 0.0, 9.8);
	mass = 50;
	com_height = 0.75;
	Td   = 0.0;
	Ts   = 0.5;
}
}