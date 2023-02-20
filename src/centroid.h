#pragma once

#include <cnoid/EigenTypes>
#include "footstep.h"
#include "robot.h"

namespace cnoid{
namespace hvac2022{


class Centroid
{
    public:
        Vector3 com_pos_ref;
        Vector3 com_vel_ref;
        Vector3 com_acc_ref;

        Vector3 zmp_pos;
        Vector3 zmp_pos_ref;
        Vector3 zmp_vel;

        Vector3 dcm_pos_ref;
        Vector3 dcm_vel_ref;
        Vector3 dcm_pos;
        Vector3 vrp_pos;
        Vector3 vrp_pos_ref;
        //Vector3 vrp_vel;

        double  k_dcm;

        void CalcDcmRef(Footstep &footstep, Param &param, double dt);
        //void CalcVrpRef(Footstep &footstep, Param &param, double dt);

        void Update(Footstep& footstep, Param& param, double dt); // ëΩï™Ç±Ç±Ç≈DCMÇêßå‰Ç∑ÇÈÇ©Ç»

    	Centroid();

};
}
}