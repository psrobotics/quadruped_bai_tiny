#include "bridge_sender.h"

bridge_sender::bridge_sender()
{
    bridge_address[0]=10; //fl
    bridge_address[1]=11; //fr
    bridge_address[2]=12; //bl
    bridge_address[3]=13; //br
}

int bridge_sender::iic_send(int leg_type, float *rad_data, float *tor_data)
{
    //for leg_type 0->fl, 1->fr, 2->bl, 3->br
    int address_tmp = bridge_address[leg_type];
    float_byte rad_data_send_tmp[2];
    float_byte tor_data_send_tmp[2];

    for(int i=0;i<2;i++)
    {
        rad_data_send_tmp[i].f = rad_data[i];
        tor_data_send_tmp[i].f = tor_data[i];
    }

    Wire.beginTransmission(address_tmp);
    //rad session 
    Wire.write('r');
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(rad_data_send_tmp[s].c[i]);
    //torque session
    Wire.write('t');
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(tor_data_send_tmp[s].c[i]);
    Wire.endTransmission();  

    return 0;
}