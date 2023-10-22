#ifndef _KINEMATICS__
#define _KINEMATICS__

#include "matrix.h"
#include "bridge_sender.h"
#include "hardware_para.h"
#include "Arduino.h"

class leg
{
    public:
    //for 2dof type
    int leg_type;

    //joint rad
    float joint_rad_arr[2];
    //motor rad, considering reducer and mounting direction
    //1->long link, 0->short link
    float motor_rad_arr[2];

    //calcuated motor feedforward torque
    float torque_feedforward[2];
    //scaled motor current sent to driver
    float torque_feedforward_to_motor[2];

    //since we're using 8dof design, using only xy cod to declear foot position
    float local_foot_cod_tar[2];
    //desired force in local cod
    float local_foot_force_tar[2];

    leg(int _leg_type);

    int leg_ik();
    int leg_fk();
    int leg_force_to_tor();

    int get_local_foot_cod_tar(vec3d &foot_root, vec3d &foot_end);

    private:
    //hardware link length 
    float l1,l2,l3,l4,l5,l6;
    //joint offset when homing
    float rad_offset[2];
    //reducer reduction ratio
    float reducer_ratio;

    float get_tri_rad(float ll1, float ll2, float ll3);


};

class body
{
    public:
    vec3d leg_root_fl;
    vec3d leg_root_fr;
    vec3d leg_root_bl;
    vec3d leg_root_br;

    vec3d leg_end_fl;
    vec3d leg_end_fr;
    vec3d leg_end_bl;
    vec3d leg_end_br;

    leg *leg_fl;
    leg *leg_fr;
    leg *leg_bl;
    leg *leg_br;

    //body pose state - target
    matrix3d body_rot_mat;
    vec3d body_rot_angle_tar;
    vec3d body_pos_vec;

    //body rot angle measure by onboard imu
    vec3d body_rot_angle_now;

    bridge_sender *sender;

    body();

    int init_body_mat();
    int refresh_body_mat();
    int body_ik();

    //send rad/torque to leg
    int send_to_leg();

    int set_body_pos_tar(float _x, float _y, float _z);
    int set_body_rot_tar(float _pitch, float _roll, float _yaw);
    int get_body_rot_now(float _pitch_n, float _roll_n, float _yaw_n);

    private:
    //hardware para
    float body_width;
    float body_length;


};

#endif
