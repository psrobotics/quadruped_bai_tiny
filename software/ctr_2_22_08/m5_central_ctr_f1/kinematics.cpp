#include "kinematics.h"

leg::leg(int _leg_type)
{
    //hardware para setup
    l1 = LEG_LINK_LEN_L1;
    l2 = LEG_LINK_LEN_L2;
    l3 = LEG_LINK_LEN_L3;
    l6 = LEG_LINK_LEN_L6;
    l5 = LEG_LINK_LEN_L5;
    l4 = LEG_LINK_LEN_L4;
    leg_type = _leg_type;

    rad_offset[1] = J_OFFSET_1;
    rad_offset[0] = J_OFFSET_0;

    reducer_ratio = REDUCER_RATIO;
}

int leg::leg_ik()
{
    float y = local_foot_cod_tar[1] * -1;
    float x = local_foot_cod_tar[0];
    if (leg_type == FL || leg_type == BL)
        x *= -1;

    float b1_p = pow(x, 2) + pow(y, 2);
    float b1 = sqrt(b1_p);
    float a1 = atan2(y, x);
    float a0 = pi - a1;
    float b2_p = pow((l1 / 2), 2) + b1_p - l1 * b1 * cos(a0);
    float b2 = sqrt(b2_p);
    float a2 = get_tri_rad(b1, b2, l1 / 2);
    float a3 = get_tri_rad(b2, l3 + l6, l2);
    float a4 = get_tri_rad(l2, l3 + l6, b2);
    float a5 = 2 * pi - a0 - a2 - a3 - a4;
    //rad at longer link
    joint_rad_arr[0] = 1.5 * pi - a5;

    float a7 = a4 + a5 - pi;
    float p1x = x - l6 * cos(a7);
    float p1y = y - l6 * sin(a7);
    float e1_p = pow(p1x, 2) + pow(p1y, 2);
    float e1 = sqrt(e1_p);
    float c1 = atan2(p1y, p1x);
    float c2 = get_tri_rad(l5, e1, l4);
    //rad at short link
    joint_rad_arr[1] = 0.5 * pi + c1 - c2;

    if (leg_type == FL || leg_type == BR)
    {
        motor_rad_arr[1] = (joint_rad_arr[0] - rad_offset[0]) * -1 * reducer_ratio;
        motor_rad_arr[0] = (joint_rad_arr[1] - rad_offset[1]) * -1 * reducer_ratio;
    }
    else if (leg_type == FR || leg_type == BL)
    {
        motor_rad_arr[1] = (joint_rad_arr[0] - rad_offset[0]) * reducer_ratio;
        motor_rad_arr[0] = (joint_rad_arr[1] - rad_offset[1]) * reducer_ratio;
    }

    return 0;
}

float leg::get_tri_rad(float ll1, float ll2, float ll3)
{
    return acos((pow(ll1, 2) + pow(ll2, 2) - pow(ll3, 2)) / (2 * ll1 * ll2));
}

int leg::get_local_foot_cod_tar(vec3d &foot_root, vec3d &foot_end)
{
    //considering this is a model with 2dof leg
    //dont consider 3d cod x
    vec3d tmp = foot_end - foot_root;
    local_foot_cod_tar[0] = tmp.vec[1]; //local x<->3d y
    local_foot_cod_tar[1] = tmp.vec[2]; //local y<->3d z

    return 0;
}

body::body()
{
    leg_fl = new leg(FL);
    leg_fr = new leg(FR);
    leg_bl = new leg(BL);
    leg_br = new leg(BR);

    sender = new bridge_sender;

    body_width = BODY_WIDTH;
    body_length = BODY_LENGTH;
}

int body::init_body_mat()
{
    leg_root_fl.vec[0] = -1 * body_width / 2; //width-x
    leg_root_fl.vec[1] = body_length / 2;

    leg_root_fr.vec[0] = body_width / 2;
    leg_root_fr.vec[1] = body_length / 2;

    leg_root_bl.vec[0] = -1 * body_width / 2;
    leg_root_bl.vec[1] = -1 * body_length / 2;

    leg_root_br.vec[0] = body_width / 2;
    leg_root_br.vec[0] = -1 * body_length / 2;

    leg_root_fl = leg_root_fl + body_pos_vec;
    leg_root_fr = leg_root_fr + body_pos_vec;
    leg_root_bl = leg_root_bl + body_pos_vec;
    leg_root_br = leg_root_br + body_pos_vec;

    return 0;
}

int body::refresh_body_mat()
{
    //cal rot point
    leg_root_fl = body_rot_mat * leg_root_fl;
    leg_root_fr = body_rot_mat * leg_root_fr;
    leg_root_bl = body_rot_mat * leg_root_bl;
    leg_root_br = body_rot_mat * leg_root_br;

    //translate
    leg_root_fl = leg_root_fl + body_pos_vec;
    leg_root_fr = leg_root_fr + body_pos_vec;
    leg_root_bl = leg_root_bl + body_pos_vec;
    leg_root_br = leg_root_br + body_pos_vec;

    return 0;
}

int body::body_ik()
{
    refresh_body_mat();
    //trans 3d foot cod to local foot cod
    leg_fl->get_local_foot_cod_tar(leg_root_fl, leg_end_fl);
    leg_fr->get_local_foot_cod_tar(leg_root_fr, leg_end_fr);
    leg_bl->get_local_foot_cod_tar(leg_root_bl, leg_end_bl);
    leg_br->get_local_foot_cod_tar(leg_root_br, leg_end_br);

    leg_fl->leg_ik();
    leg_fr->leg_ik();
    leg_bl->leg_ik();
    leg_br->leg_ik();

    return 0;
}

int body::send_to_leg()
{
    sender->iic_send(leg_fl->leg_type, leg_fl->motor_rad_arr, leg_fl->torque_feedforward_to_motor);
    sender->iic_send(leg_fr->leg_type, leg_fr->motor_rad_arr, leg_fr->torque_feedforward_to_motor);
    sender->iic_send(leg_bl->leg_type, leg_bl->motor_rad_arr, leg_bl->torque_feedforward_to_motor);
    sender->iic_send(leg_br->leg_type, leg_br->motor_rad_arr, leg_br->torque_feedforward_to_motor);

    return 0;
}

int body::set_body_pos_tar(float _x, float _y, float _z)
{
    body_pos_vec.vec[0]=_x;
    body_pos_vec.vec[1]=_y;
    body_pos_vec.vec[2]=_z;
    return 0;
}

int body::set_body_rot_tar(float _pitch, float _roll, float _yaw)
{
    body_rot_angle_tar.vec[0]=_pitch;
    body_rot_angle_tar.vec[1]=_roll;
    body_rot_angle_tar.vec[2]=_yaw;

    body_rot_mat.gen_rotation(body_rot_angle_tar.vec);

    return 0;
}

int body::get_body_rot_now(float _pitch_n, float _roll_n, float _yaw_n)
{
    body_rot_angle_now.vec[0]=_pitch_n;
    body_rot_angle_now.vec[1]=_roll_n;
    body_rot_angle_now.vec[2]=_yaw_n;
    return 0;
}
