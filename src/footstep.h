#pragma once

#include <cnoid/EigenTypes>
#include <vector>
#include "param.h"
using namespace std;

namespace cnoid {
namespace hvac2022 {

struct step
{
    double stride;
    double spacing;
    double turn;
    int    side;

    Vector3 foot_pos[2];
    Vector3 zmp_pos;
    Vector3 vrp_pos;
    Vector3 dcm_pos;

    step();
};

class Footstep{
    private:
        /* data */
    public:
        vector<step> steps;
        void GeneratePos(Param &param);

        Footstep(/* args */);
};

}
}