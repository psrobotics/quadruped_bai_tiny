#include "leg_bridge.h"

leg_bridge::leg_bridge(int _id)
{
    leg_id = _id;
    //init all data
    for(int s=0;s<3;s++)
    {
        rad_now[s].f = 0;
        rad_tar[s].f = 0;
        torque_now[s].f = 0;
        torque_tar[s].f = 0;
    }
}

int leg_bridge::change_rad(float *_rad)
{
    for(int s=0;s<3;s++)
        rad_tar[s].f = _rad[s];

    return 0;
}

int leg_bridge::change_tor(float *_tor)
{
    for(int s=0;s<3;s++)
        torque_tar[s].f = _tor[s];

    return 0;
}

void leg_bridge::pack_to_float(float in_1, float in_2, floatint &result)
{
    unsigned int up = (in_1+32.5)*1000;
    unsigned int down = (in_2+32.5)*1000;
    result.i = (((unsigned long)up << 16) | down);
}

void leg_bridge::unpack_from_float(float &out_1, float &out_2, floatint result)
{
    unsigned int up_d = (unsigned int)(result.i>>16);
    unsigned int down_d = (unsigned int)(result.i & 0xffff);
    out_1 = up_d/1000.0f-32.5;
    out_2 = down_d/1000.0f-32.5;    
}

//write to sub driver, currently built for 8dof robot
int leg_bridge::write_driver(SoftwareSerial **_s_arr)
{
    //8dof config comes with 2 drivers
    for(int s=0;s<2;s++)
    {
        //combine, torque comes with a 2000 offset
        //to dis from rad data
        int rad_norm = (rad_tar[s].f+16)*1000.0;
        int tor_norm = (torque_tar[s].f-16)*1000.0;
        _s_arr[s]->println(rad_norm);
        _s_arr[s]->println(tor_norm);
    }

    return 0;
}

int leg_bridge::read_master()
{

    return 0;
}
