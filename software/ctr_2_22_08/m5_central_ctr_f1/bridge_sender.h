#ifndef _BRIDGE_SENDER__
#define _BRIDGE_SENDER__

#include "Wire.h"
#include "Arduino.h"
#include "hardware_para.h"

union float_byte {
  float f;
  char c[sizeof(f)];
};


//hardware iic link to leg bridge
class bridge_sender
{
    public:
    bridge_sender();
    //send caled rad and ff torque to specified bridge
    int iic_send(int leg_type, float *rad_data, float *tor_data);

    private:
    //4* leg bridge address, 0->fl, 1->fr, 2->bl, 3->br
    int bridge_address[4];

};

#endif