#include <cnoid/EigenTypes>

namespace cnoid
{
struct Param
{
	Vector3 g;
	double  mass;
	double  com_height;
	double  Td;
	double  Ts;

	Param();
};

Param::Param() {
	g = Vector3(0.0, 0.0, 9.8);
	mass = 50;
	Td   = 0.0;
	Ts   = 0.5;
}
}