#include "footstep.h"

namespace cnoid{
namespace hvac2022{

Footstep::Footstep(/* args */)
{
    step s;
    s.stride = 0.0;
    s.turn   = 0.0;
    s.foot_pos[0] = Vector3(0.0, 0.0, 0.0);
    s.foot_pos[1] = Vector3(0.0, 0.0, 0.0);

    steps.push_back(s);
}


// generate foot position
void Footstep::GeneratePos()
{
    int nstep = steps.size();
    for (int i = 0; i < nstep-1; i++)
    {
        step& st0 = steps[i];
        step& st1 = steps[i + 1];

        int sup = st0.side;
        int swg = !st0.side;

        st1.side = !st0.side;
        st1.foot_pos[sup] = st0.foot_pos[swg];
        st1.foot_pos[swg] = st1.foot_pos[sup] + Vector3(st1.stride, (swg - sup)*st1.spacing, 0.0);
    }
}

}
}