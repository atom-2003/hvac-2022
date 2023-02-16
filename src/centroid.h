#pragma once

#include <cnoid/EigenTypes>
#include "footstep.h"
#include "robot.h"

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
        //Vector3 vrp_vel;

        double  k_dcm;

        void CalcComRef(Vector3 &pos, Vector3 &vel); // Englsberger とかの方法で重心軌道計算する
        void CalcDcmRef(double dt);
        void CalcVrpRef(Footstep &footstep, Param &param, double dt);

        void Update(Vector3& pos, Vector3& vel, double dt);
    	Centroid();

};
}
}