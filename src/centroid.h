#pragma once

#include <cnoid/EigenTypes>
#include "footstep.h"
#include "centroid.h"

namespace cnoid{
namespace hvac2022{


class Centroid
{
    public:
        Vector3 com_pos;
        Vector3 com_vel;
        Vector3 zmp_pos;
        Vector3 zmp_vel;

        Vector3 dcm_pos;
        Vector3 dcm_vel;
        Vector3 vrp_pos;
        Vector3 vrp_vel;

        void CalcComState(Vector3 &pos, Vector3 &vel); // Englsberger とかの方法で重心軌道計算する
        void CalcDcmState(Vector3 &pos, Vector3 &vel);
        void CalcVrpState(Vector3 &pos, Vector3 &vel);
    	Centroid();
    private:

};
}
}