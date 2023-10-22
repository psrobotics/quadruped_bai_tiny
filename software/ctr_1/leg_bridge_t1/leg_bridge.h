#include <SoftwareSerial.h>
#include <Wire.h>
#include "Arduino.h"

#ifndef _LEG_BRIDGE_H__
#define _LEG_BRIDGE_H__

//leg type define
#define FL 0
#define FR 1
#define BL 2
#define BR 3

union float2byte {
  float f; byte b[sizeof(float)];
};

union floatint {
  uint32_t i; float f; char b[4];
};

class leg_bridge 
{
    public:

    leg_bridge(int _id);

    int write_driver(SoftwareSerial **_s_arr); //write pos, torque to write_driver
    int read_driver(SoftwareSerial **_s_arr); //read rad from read_driver
    int read_master(); //read target rad, torque from read_master
    int write_master(); //write rad to read_master

    int change_rad(float *_rad);
    int change_tor(float *_tor);

    private:
    int leg_id;

    float2byte rad_tar[3];
    float2byte torque_tar[3];

    float2byte rad_now[3];
    float2byte torque_now[3];

    void pack_to_float(float in_1, float in_2, floatint &result);
    void unpack_from_float(float &out_1, float &out_2, floatint result);

};

#endif