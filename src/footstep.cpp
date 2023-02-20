#include "footstep.h"
#include "robot.h"

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


// generate foot position, ZMP position and DCM position
void Footstep::GeneratePos(Param &param)
{
    // footprint pos
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

    // zmp and dcm pos by recursion
    double T     = sqrt(param.com_height / param.g.z());
    double Tstep = param.Ts + param.Td;
    for (int i = nstep; i > 0; i--)
    {
        step& st = steps[i];
        st.zmp_pos = st.foot_pos[st.side];
        st.vrp_pos = st.zmp_pos + Vector3(0.0, 0.0, param.com_height);
        if (i == nstep) st.dcm_pos = st.vrp_pos;
        else
        {
            step& st_next = steps[i + 1];
            st.dcm_pos = st.vrp_pos + exp(-(param.Ts + param.Td)/T)*(st_next.dcm_pos - st.vrp_pos);
        }
    }
}

} // namespace hvac2022
} // namespace cnoid