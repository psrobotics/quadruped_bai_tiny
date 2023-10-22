#include "gait_controller.h"

//link the controlled body
int gait_controller::link_body(body &_target_body)
{
    target_body = &_target_body;
    return 0;
}
