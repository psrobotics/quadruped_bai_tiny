//walk test2 for ee599, apr 22 2023 shuang

#include <M5Atom.h>   // http://librarymanager/All#M5Atom  https://github.com/m5stack/M5Atom
#include <FastLED.h>  // http://librarymanager/All#FastLED https://github.com/FastLED/FastLED
#include <Wire.h>
#include <arduino-timer.h>
#include "quick_math.h"

auto timer_iic = timer_create_default(); // create a timer with default settings
auto timer = timer_create_default(); // create a timer with default setting

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

// gait params
float step_len = 7;
float step_fre = 1.95; //1.6 1.95

int moving_command = 0; 
// 0-forward, 1 left, 2 right

float_byte rad_tmp_fl[2];
float_byte tor_tmp_fl[2];

float_byte rad_tmp_fr[2];
float_byte tor_tmp_fr[2];

float_byte rad_tmp_bl[2];
float_byte tor_tmp_bl[2];

float_byte rad_tmp_br[2];
float_byte tor_tmp_br[2];

float rad_ori_fl[2]; //[0] - shorter link [1] - longer link
float rad_ori_fr[2];
float rad_ori_bl[2];
float rad_ori_br[2];

//ori shortlink/longlink 2.039242698, 1.458746189

float rad_offset_fl[2] = {1.3347, 2.2138};
float rad_offset_fr[2] = {1.3347, 2.0138};
float rad_offset_bl[2] = {1.3347, 2.2138};
float rad_offset_br[2] = {1.3347, 2.2138};

float test_x = 0;
float test_y = 80;

float s_s_p = 0;
float s_timer_p = 0;
float jump_timer = 0;

float fforward_torque = 0;
float roll_e = 0;
float roll_e_old = 0;
float roll_sum = 0;
float roll_k = -0.14 / 1.8; //0.14
float roll_d = -0.18 / 1.1; //0.18
float roll_i = 0;

//0.074 0.37
float fforward_torque_pitch = 0;
float pitch_e = 0;
float pitch_e_old = 0;
float pitch_k = -0.06; //0.06
float pitch_d = -0.2; //0.2
int state = 0;


//TwoWire bridge_iic(1);

void setup() {
  M5.begin(true, true, true);
  M5.IMU.Init();//init imu
  // 初期化
  //M5.begin(true, false, true);  // (Serial, I2C, NeoPixel)
  //Wire.begin(25, 21, 10000);    // 内蔵I2C 0x68(MPU6886)
  //low speed mode 10000, standard speed mode 100000, fast 400000
  Wire.begin(26, 32, 400000); //external iic

  // GPIO初期化
  //pinMode(22, INPUT); // PIN  (INPUT, OUTPUT,       )
  //pinMode(19, INPUT); // PIN  (INPUT, OUTPUT,       )
  //pinMode(23, INPUT); // PIN  (INPUT, OUTPUT,       )
  //pinMode(33, INPUT); // PIN  (INPUT, OUTPUT, ANALOG)
  //pinMode(26, INPUT); // GROVE(INPUT, OUTPUT, ANALOG)無線利用時にはANALOG利用不可, DAC出力可
  //pinMode(32, INPUT); // GROVE(INPUT, OUTPUT, ANALOG)

  // 内部ピン初期化
  //pinMode(12, OUTPUT_OPEN_DRAIN); // IR赤外線(LOWで出力)
  //digitalWrite(12, HIGH);

  // LED(0-24)
  for (int i = 0; i < 25; i++) {
    M5.dis.drawpix(i, CRGB::Blue);
  }

  timer_iic.every(5, iic_sent);
  timer.every(4, change_motion);
  //evey 10 milis, run the cycle
}

void send_to_bridge(int leg_type)
{
  Wire.beginTransmission(leg_type); // transmit to device FL
  Wire.write('r');        // sends start byte
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

  Wire.endTransmission();    // stop transmitting
}

void loop()
{
  timer.tick();
  timer_iic.tick();

  if(Serial.available())
  {
    char cc = Serial.read();
    if(cc == 'a')
      moving_command = 0;
    else if(cc == 'b')
      moving_command = 1;
    else if(cc == 'c')
      moving_command = 2;
  }
}


void leg_ik_t1(float x, float y, int leg_type)
{
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
  float j1 = 1.5 * pi - a5; //longer link

  float a7 = a4 + a5 - pi;
  float p1x = x - l6 * cos(a7);
  float p1y = y - l6 * sin(a7);
  float e1_p = pow(p1x, 2) + pow(p1y, 2);
  float e1 = sqrt(e1_p);
  float c1 = atan2(p1y, p1x);
  float c2 = get_tri_rad(l5, e1, l4);
  float j2 = 0.5 * pi + c1 - c2; //shorter link

  if (leg_type == FL)
  {
    rad_ori_fl[0] = j2; //shorter link
    rad_ori_fl[1] = j1; //longer link

    rad_tmp_fl[1].f = (j1 - rad_offset_fl[1]) * -5.367; //5.167 reducer ra
    rad_tmp_fl[0].f = (j2 - rad_offset_fl[0]) * -5.367;
  }
  else if (leg_type == BR)
  {
    rad_ori_br[0] = j2; //shorter link
    rad_ori_br[1] = j1; //longer link

    rad_tmp_br[1].f = (j1 - rad_offset_br[1]) * -5.367;
    rad_tmp_br[0].f = (j2 - rad_offset_br[0]) * -5.367;
  }
  else if (leg_type == FR)
  {
    rad_ori_fr[0] = j2; //shorter link
    rad_ori_fr[1] = j1; //longer link

    rad_tmp_fr[1].f = (j1 - rad_offset_fr[1]) * 5.367;
    rad_tmp_fr[0].f = (j2 - rad_offset_fr[0]) * 5.367;
  }
  else if (leg_type == BL)
  {
    rad_ori_bl[0] = j2; //shorter link
    rad_ori_bl[1] = j1; //longer link

    rad_tmp_bl[1].f = (j1 - rad_offset_bl[1]) * 5.367;
    rad_tmp_bl[0].f = (j2 - rad_offset_bl[0]) * 5.367;
  }

  //Serial.print (j1);
  //Serial.print (" ");
  //Serial.println(j2);
}

float get_tri_rad(float ll1, float ll2, float ll3)
{
  return acos((pow(ll1, 2) + pow(ll2, 2) - pow(ll3, 2)) / (2 * ll1 * ll2));
}

bool change_motion(void *)
{
  s_s_p += 0.018 * abs(sin(s_timer_p)); //0.03 //0.018
  s_timer_p += 0.001;

  if (step_state == 0 || step_state == 2)
  {
    step_timer += 0.018 * 4 * step_fre;
  }
  else
  {
    step_timer += 0.018 * 11 * step_fre;
  }

  if (s_s_p > 2 * 3.1415926)
    s_s_p = 0;

  if (s_timer_p > 2 * 3.1415926)
    s_timer_p = 0;

  if (step_timer > 3.1415926) //half cycle
  {
    step_timer = 0;

    //change sub state
    if (step_state == 0)
      step_state = 1;
    else if (step_state == 1)
      step_state = 2;
    else if (step_state == 2)
      step_state = 3;
    else if (step_state == 3)
      step_state = 0;
  }

  float walk_height = 90;
  float step_height_gain = 2; //2
  float ground_height_gain = 0.95; //0.9
  
  if (step_state == 0)
  {
    test_y_coner_1 = walk_height - step_len * step_height_gain * sin(step_timer);
    test_x_coner_1 = 0 + step_len * cos(step_timer); //+

    test_y_coner_2 = walk_height + ground_height_gain * sin(step_timer);
    test_x_coner_2 = 0 - step_len * cos(step_timer); //-
  }
  else if (step_state == 1)
  {
    //test_y_coner_1 = 90;
    //test_x_coner_1 = 0 - step_len * cos(step_timer);
  }

  else if (step_state == 2)
  {
    test_y_coner_1 = walk_height + ground_height_gain * sin(step_timer);
    test_x_coner_1 = 0 - step_len * cos(step_timer);

    test_y_coner_2 = walk_height - step_len * step_height_gain * sin(step_timer);
    test_x_coner_2 = 0 + step_len * cos(step_timer);
  }
  else if (step_state == 3)
  {
    //test_y_coner_2 = 90;
    //test_x_coner_2 = 0 - step_len * cos(step_timer);
  }

  //ff torque

  float ground_ff = 0.15; //4.5

  if (state == 2)
  {

    for (int i = 0; i < 25; i++)
    {
      M5.dis.drawpix(i, CRGB::Red);
    }
    tor_tmp_fl[0].f = fforward_torque - 1 * fforward_torque_pitch;
    tor_tmp_fl[1].f = fforward_torque - 1 * fforward_torque_pitch;

    tor_tmp_br[0].f = -1 * fforward_torque + fforward_torque_pitch;
    tor_tmp_br[1].f = -1 * fforward_torque + fforward_torque_pitch;

    tor_tmp_bl[0].f = -1 * fforward_torque - 1 * fforward_torque_pitch;
    tor_tmp_bl[1].f = -1 * fforward_torque - 1 * fforward_torque_pitch;

    tor_tmp_fr[0].f = fforward_torque + fforward_torque_pitch;
    tor_tmp_fr[1].f = fforward_torque + fforward_torque_pitch;

    if (step_state == 0) //fr bl on ground
    {
      //kill sky foot
      tor_tmp_fl[0].f = 0;
      tor_tmp_fl[1].f = 0;

      tor_tmp_br[0].f = 0;
      tor_tmp_br[1].f = 0;

      //enhance ground foot
      tor_tmp_fr[0].f += ground_ff;
      tor_tmp_fr[1].f += ground_ff;

      tor_tmp_bl[0].f += ground_ff;
      tor_tmp_bl[1].f += ground_ff;
    }
    else if (step_state == 2) //fl br on ground
    {
      //kill sky foot
      tor_tmp_fr[0].f = 0;
      tor_tmp_fr[1].f = 0;

      tor_tmp_bl[0].f = 0;
      tor_tmp_bl[1].f = 0;

      //enhance ground foot
      tor_tmp_fl[0].f -= ground_ff;
      tor_tmp_fl[1].f -= ground_ff;

      tor_tmp_br[0].f -= ground_ff;
      tor_tmp_br[1].f -= ground_ff;
    }
  }

  else if (state == 1)
  {
    //balanced stand
    for (int i = 0; i < 25; i++) {
      M5.dis.drawpix(i, CRGB::White);
    }

    test_x_coner_1 = 0;
    test_x_coner_2 = 0;
    test_y_coner_1 = walk_height;
    test_y_coner_2 = walk_height;


    tor_tmp_fl[0].f = fforward_torque - 1 * fforward_torque_pitch;
    tor_tmp_fl[1].f = fforward_torque - 1 * fforward_torque_pitch;

    tor_tmp_br[0].f = -1 * fforward_torque + fforward_torque_pitch;
    tor_tmp_br[1].f = -1 * fforward_torque + fforward_torque_pitch;

    tor_tmp_bl[0].f = -1 * fforward_torque - 1 * fforward_torque_pitch;
    tor_tmp_bl[1].f = -1 * fforward_torque - 1 * fforward_torque_pitch;

    tor_tmp_fr[0].f = fforward_torque + fforward_torque_pitch;
    tor_tmp_fr[1].f = fforward_torque + fforward_torque_pitch;
  }
  else if (state == 0 || state == 3)
  {
    //static stand
    for (int i = 0; i < 25; i++) {
      M5.dis.drawpix(i, CRGB::Blue);
    }
    tor_tmp_fl[0].f = 0;
    tor_tmp_fl[1].f = 0;

    tor_tmp_br[0].f = 0;
    tor_tmp_br[1].f = 0;

    tor_tmp_bl[0].f = 0;
    tor_tmp_bl[1].f = 0;

    tor_tmp_fr[0].f = 0;
    tor_tmp_fr[1].f = 0;

    test_x_coner_1 = 0;
    test_x_coner_2 = 0;
    test_y_coner_1 = walk_height;
    test_y_coner_2 = walk_height;
  }

  float pitch = 0, roll = 0, yaw = 0;
  //getAhrsData pitch roll yaw using another filter
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  if (roll < 0)
    roll = 360 + roll;

  roll_e = roll - 200; //187 178
  pitch_e = pitch - 0;

  roll_sum += roll_e;

  fforward_torque = roll_e * roll_k + roll_d * (roll_e - roll_e_old) + roll_i * roll_sum;
  fforward_torque_pitch = pitch_e * pitch_k + pitch_d * (pitch_e - pitch_e_old);

  roll_e_old = roll_e;
  pitch_e_old = pitch_e;

  M5.update();
  //Serial.printf("%.2f,%.2f\n", pitch, roll);

  if (M5.Btn.wasPressed())
  {
    if (state == 0)
      state = 1;
    else if (state == 1)
      state = 2;
    else if (state == 2)
      state = 3;
    else if (state == 3)
      state = 0;

  }
  return true;
}

bool iic_sent(void *)
{

if(moving_command == 0) // forward
{
  leg_ik_t1(-1 * test_x_coner_1 + 10, test_y_coner_1 - 4, FL);
  leg_ik_t1(-1 * test_x_coner_2 + 10, test_y_coner_2 - 4, FR);
  leg_ik_t1(1 * test_x_coner_2 + 10, test_y_coner_2 - 10, BL);
  leg_ik_t1(1 * test_x_coner_1 + 10, test_y_coner_1 - 10, BR);
}
else if(moving_command == 1) // right
{
  leg_ik_t1(1 * test_x_coner_1 + 10, test_y_coner_1 - 4, FL);
  leg_ik_t1(-1 * test_x_coner_2 + 10, test_y_coner_2 - 4, FR);
  leg_ik_t1(-1 * test_x_coner_2 + 10, test_y_coner_2 - 10, BL);
  leg_ik_t1(1 * test_x_coner_1 + 10, test_y_coner_1 - 10, BR);
}
else if(moving_command == 2) // left
{
  leg_ik_t1(-1 * test_x_coner_1 + 10, test_y_coner_1 - 4, FL);
  leg_ik_t1(1 * test_x_coner_2 + 10, test_y_coner_2 - 4, FR);
  leg_ik_t1(1 * test_x_coner_2 + 10, test_y_coner_2 - 10, BL);
  leg_ik_t1(-1 * test_x_coner_1 + 10, test_y_coner_1 - 10, BR);
}

  //leg_inv_dny(rad_ori_fl[0], rad_ori_fl[1], 0, -0.02, FL);

  send_to_bridge(FL);
  send_to_bridge(FR);
  send_to_bridge(BL);
  send_to_bridge(BR);

  return true;

}
