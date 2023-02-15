#pragma once

#include <cnoid/EigenTypes>
#include <vector>
using namespace std;

namespace cnoid {
namespace hvac2022 {

class Footstep{
    private:
        /* data */
    public:
        struct step
        {
            double stride;
            double spacing;
            double turn;
            int    side;

            Vector3 foot_pos[2];
        };

    public:
        vector<step> steps;
        void GeneratePos();

        Footstep(/* args */);
    };
    }


}
}