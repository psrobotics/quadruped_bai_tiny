#ifndef _GAIT_CONTROLLER__
#define _GAIT_CONTROLLER__

#include "kinematics.h"

class gait_controller
{
    private:
    int gait_state;

    public:
    body *target_body;

    int link_body(body &_target_body);

};

#endif
