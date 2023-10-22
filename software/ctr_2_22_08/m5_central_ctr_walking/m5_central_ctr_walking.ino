#include <M5Atom.h>   // http://librarymanager/All#M5Atom  https://github.com/m5stack/M5Atom
#include <FastLED.h>  // http://librarymanager/All#FastLED https://github.com/FastLED/FastLED
#include <Wire.h>
#include <arduino-timer.h>
#include "quick_math.h"

//auto timer_iic = timer_create_default();  // create a timer with default settings
//auto timer = timer_create_default();      // create a timer with default settings

auto main_timer = timer_create_default();

unsigned long global_time;

#define FL 10
#define FR 11
#define BL 12
#define BR 13

union float_byte {
  float f;
  char c[sizeof(f)];
};

float step_timer = 0;
int step_state = 0;

float y = 0;

float test_y_coner_1;
float test_y_coner_2;
float test_x_coner_1;
float test_x_coner_2;

float step_len = 8;

float_byte rad_tmp_fl[2];
float_byte tor_tmp_fl[2];

float_byte rad_tmp_fr[2];
float_byte tor_tmp_fr[2];

float_byte rad_tmp_bl[2];
float_byte tor_tmp_bl[2];

float_byte rad_tmp_br[2];
float_byte tor_tmp_br[2];

float rad_ori_fl[2];  //[0] - shorter link [1] - longer link
float rad_ori_fr[2];
float rad_ori_bl[2];
float rad_ori_br[2];

//ori shortlink/longlink 2.039242698, 1.458746189

float offset_arr[2] = { 1.3347, 2.2138 };

float fforward_torque = 0;
float roll_e = 0;
float roll_e_old = 0;
float roll_e_sum = 0;
float roll_k = 0.14 / 1.8;  //0.14
float roll_d = 0.18 / 1.1;  //0.18
float roll_i = 0;

//0.074 0.37
float fforward_torque_pitch = 0;
float pitch_e = 0;
float pitch_e_old = 0;
float pitch_e_sum = 0;
float pitch_k = 0.06;  //0.06
float pitch_d = 0.2;   //0.2
int state = 0;

float test_f = 0;

//foot traj paraments
float x_s = 8;
float y_s = 100;
float h_s = 18;  //10
float hl_s = 2;
float l_s = 6;  //12
float p_x_ref[10];
float p_y_ref[10];
float item_ref[10] = { 1, 9, 36, 84, 126, 126, 84, 36, 9, 1 };

float t_period = 1.0;

float global_clock_t1 = 0*t_period;
float global_clock_t2 = 0.5*t_period;
float global_clock_t3 = 0.5*t_period;
float global_clock_t4 = 0*t_period;

//jumping parameters
float j_start_y = 45;
float j_end_y = 110;
float j_touchdown_y = 80;

float j_start_x = 0;
float j_end_x = 0;
float j_touchdown_x = 0;

//jumping time parameters in ms
int j_t_stance = 2000;
int j_t_jump = 400;
int j_t_ground = 500;
int j_t_trans = 800;
int j_cyc_time = j_t_stance+j_t_jump+j_t_ground+j_t_trans;

int jt_1 = j_t_stance;
int jt_2 = j_t_stance + j_t_jump;
int jt_3 = j_t_stance + j_t_jump + j_t_ground;
int jt_4 = j_t_stance + j_t_jump + j_t_ground + j_t_trans;

float l_current_x;
float l_current_y;

void setup() {
  M5.begin(true, true, true);
  M5.IMU.Init();  //init imu

  Wire.begin(26, 32, 400000);  //external iic

  // LED(0-24)
  for (int i = 0; i < 25; i++) {
    M5.dis.drawpix(i, CRGB::Blue);
  }

  main_timer.every(5,main_loop_handle);
}

bool main_loop_handle(void *)
{
  if (global_clock_t1 > t_period) global_clock_t1 = 0;
  if (global_clock_t2 > t_period) global_clock_t2 = 0;
  if (global_clock_t3 > t_period) global_clock_t3 = 0;
  if (global_clock_t4 > t_period) global_clock_t4 = 0;

  float clock_as = 0.017;//0.017;  // 0.045;//0.017;//0.002;

  global_clock_t1 += clock_as;
  global_clock_t2 += clock_as;
  global_clock_t3 += clock_as;
  global_clock_t4 += clock_as;

  float tar_x_fl = 0;
  float tar_y_fl = 0;

  float tar_x_fr = 0;
  float tar_y_fr = 0;

  float tar_x_bl = 0;
  float tar_y_bl = 0;

  float tar_x_br = 0;
  float tar_y_br = 0;

  float pitch = 0, roll = 0, yaw = 0;
  //getAhrsData pitch roll yaw using another filter
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  if (roll < 0)
    roll = 360 + roll;

  roll_e = roll - 190;  //187
  pitch_e = pitch - 0;

  float roll_adjust = -0.07 * roll_e - 0 * roll_e_sum;
  float pitch_adjust = 0.08 * pitch_e;

  if (roll_e > 60 || roll_e < -60)
    roll_adjust = 0;
  else
    roll_e_sum += roll_e;

  if (pitch_e > 60 || pitch_e < -60)
    pitch_adjust = 0;
  else
    pitch_e_sum += pitch_e;

  Serial.println(roll_e);

  
  p_x_ref[0] = x_s;
  p_x_ref[1] = x_s - 0.8 * l_s;
  p_x_ref[2] = x_s - 1.8 * l_s;
  p_x_ref[3] = x_s + 0.5 * l_s;
  p_x_ref[4] = x_s + 0.5 * l_s;
  p_x_ref[5] = x_s + 2.8 * l_s;
  p_x_ref[6] = x_s + 1.3 * l_s;
  p_x_ref[7] = x_s + l_s;
  p_x_ref[8] = x_s + 0.5 * l_s;
  p_x_ref[9] = x_s;

  p_y_ref[0] = y_s;
  p_y_ref[2] = y_s;
  p_y_ref[1] = y_s - 1.4 * h_s;
  p_y_ref[3] = y_s - 0.9 * h_s;
  p_y_ref[5] = y_s - 1.3 * h_s;
  p_y_ref[4] = y_s - 1.6 * h_s;
  p_y_ref[6] = y_s;
  p_y_ref[7] = y_s;
  p_y_ref[8] = y_s + hl_s;
  p_y_ref[9] = y_s;
  

/*
  p_x_ref[0] = x_s;
  p_x_ref[1] = x_s - 4 * l_s;
  p_x_ref[2] = x_s - 1.8 * l_s;
  p_x_ref[3] = x_s + 0.5 * l_s;
  p_x_ref[4] = x_s + 0.5 * l_s;
  p_x_ref[5] = x_s + 2.2 * l_s;
  p_x_ref[6] = x_s + 1.3 * l_s;
  p_x_ref[7] = x_s + 4 * l_s;
  p_x_ref[8] = x_s + 0.5 * l_s;
  p_x_ref[9] = x_s;

  p_y_ref[0] = y_s;
  p_y_ref[2] = y_s;
  p_y_ref[1] = y_s - 1.4 * h_s;
  p_y_ref[3] = y_s - 0.9 * h_s;
  p_y_ref[5] = y_s - 1.3 * h_s;
  p_y_ref[4] = y_s - 1.6 * h_s;
  p_y_ref[6] = y_s;
  p_y_ref[7] = y_s;
  p_y_ref[8] = y_s + hl_s;
  p_y_ref[9] = y_s;*/

  foot_planner(tar_x_fl, tar_y_fl, p_x_ref, p_y_ref, item_ref, global_clock_t1>1?1:global_clock_t1);
  foot_planner(tar_x_fr, tar_y_fr, p_x_ref, p_y_ref, item_ref, global_clock_t2>1?1:global_clock_t2);

  foot_planner(tar_x_bl, tar_y_bl, p_x_ref, p_y_ref, item_ref, global_clock_t3>1?1:global_clock_t3);
  foot_planner(tar_x_br, tar_y_br, p_x_ref, p_y_ref, item_ref, global_clock_t4>1?1:global_clock_t4);

  leg_ik_t1(tar_x_fl, tar_y_fl - roll_adjust - pitch_adjust, FL);
  leg_ik_t1(tar_x_fr, tar_y_fr + roll_adjust - pitch_adjust+2, FR);
  leg_ik_t1(tar_x_bl * -1 + 16, tar_y_bl - roll_adjust + pitch_adjust+2, BL);
  leg_ik_t1(tar_x_br * -1 + 16, tar_y_br + roll_adjust + pitch_adjust+2, BR);


  send_to_bridge(FL);
  send_to_bridge(FR);
  send_to_bridge(BL);
  send_to_bridge(BR);

  return 1;
}

void loop() {
  main_timer.tick();
}

void leg_ik_t1(float x, float y, int leg_type) {
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
  float j1 = 1.5 * pi - a5;  //longer link

  float a7 = a4 + a5 - pi;
  float p1x = x - l6 * cos(a7);
  float p1y = y - l6 * sin(a7);
  float e1_p = pow(p1x, 2) + pow(p1y, 2);
  float e1 = sqrt(e1_p);
  float c1 = atan2(p1y, p1x);
  float c2 = get_tri_rad(l5, e1, l4);
  float j2 = 0.5 * pi + c1 - c2;  //shorter link

  if (leg_type == FL) {
    rad_ori_fl[0] = j2;  //shorter link - //offset
    rad_ori_fl[1] = j1;  //longer link

    rad_tmp_fl[1].f = (j1 - offset_arr[1]) * 5.367;  //5.167 reducer ra
    rad_tmp_fl[0].f = (j2 - offset_arr[0]) * 5.367;
  } else if (leg_type == BR) {
    rad_ori_br[0] = j2;  //shorter link
    rad_ori_br[1] = j1;  //longer link

    rad_tmp_br[1].f = (j1 - offset_arr[1]) * 5.367;
    rad_tmp_br[0].f = (j2 - offset_arr[0]) * 5.367;
  } else if (leg_type == FR) {
    rad_ori_fr[0] = j2;  //shorter link
    rad_ori_fr[1] = j1;  //longer link

    rad_tmp_fr[1].f = (j1 - offset_arr[1]) * -5.367;
    rad_tmp_fr[0].f = (j2 - offset_arr[0]) * -5.367;
  } else if (leg_type == BL) {
    rad_ori_bl[0] = j2;  //shorter link
    rad_ori_bl[1] = j1;  //longer link

    rad_tmp_bl[1].f = (j1 - offset_arr[1]) * -5.367;
    rad_tmp_bl[0].f = (j2 - offset_arr[0]) * -5.367;
  }

  //Serial.print (j1);
  //Serial.print (" ");
  //Serial.println(j2);
}

float get_tri_rad(float ll1, float ll2, float ll3) {
  return acos((pow(ll1, 2) + pow(ll2, 2) - pow(ll3, 2)) / (2 * ll1 * ll2));
}

int get_imu(float &pitch, float &roll, float &yaw) {
  //float pitch = 0, roll = 0, yaw = 0;
  //getAhrsData pitch roll yaw using another filter
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  if (roll < 0)
    roll = 360 + roll;

  roll_e = roll - 180 -11;  //187
  pitch_e = pitch - 0;

  return true;
}


void send_tor_to_bridge(int leg_type) {
  Wire.beginTransmission(leg_type);  // transmit to device FL
  Wire.write('t');
  if (leg_type == FL)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(tor_tmp_fl[s].c[i]);
  else if (leg_type == FR)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(tor_tmp_fr[s].c[i]);
  else if (leg_type == BL)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(tor_tmp_bl[s].c[i]);
  else if (leg_type == BR)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(tor_tmp_br[s].c[i]);
  Wire.endTransmission();  // stop transmitting
}

void send_to_bridge(int leg_type) {
  Wire.beginTransmission(leg_type);  // transmit to device FL
  Wire.write('r');                   // sends start byte
  if (leg_type == FL)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(rad_tmp_fl[s].c[i]);
  else if (leg_type == FR)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(rad_tmp_fr[s].c[i]);
  else if (leg_type == BL)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(rad_tmp_bl[s].c[i]);
  else if (leg_type == BR)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < 4; i++)
        Wire.write(rad_tmp_br[s].c[i]);
  Wire.endTransmission();  // stop transmitting
}

float bezier_item_cal(float _t, int num, int nn) {
  float item;
  item = pow(_t, num) * pow(1 - _t, nn - num);
  return item;
}

void foot_planner(float &tar_x, float &tar_y, float *p_x_ref, float *p_y_ref, float *item_ref, float _t) {
  //9th, 10 points
  tar_x = 0;
  tar_y = 0;
  for (int s = 0; s < 10; s++) {
    float item_sr = bezier_item_cal(_t, s, 9);
    tar_x = tar_x + item_ref[s] * item_sr * p_x_ref[s];
    tar_y = tar_y + item_ref[s] * item_sr * p_y_ref[s];
  }
}