#include <M5Atom.h>  // http://librarymanager/All#M5Atom  https://github.com/m5stack/M5Atom
#include <FastLED.h> // http://librarymanager/All#FastLED https://github.com/FastLED/FastLED
#include <Wire.h>
#include <arduino-timer.h>

#include "matrix.h"
#include "kinematics.h"
#include "bridge_sender.h"
#include "hardware_para.h"
#include "gait_controller.h"

auto timer_iic = timer_create_default(); // create a timer with default settings
auto timer = timer_create_default();     // create a timer with default settings

body body_test;

void setup()
{

  M5.begin(true, true, true);
  M5.IMU.Init(); //init imu
  // 初期化
  //M5.begin(true, false, true);  // (Serial, I2C, NeoPixel)
  //Wire.begin(25, 21, 10000);    // 内蔵I2C 0x68(MPU6886)
  //low speed mode 10000, standard speed mode 100000, fast 400000
  Wire.begin(26, 32, 400000); //external iic for leg bridge

  // LED(0-24)
  for (int i = 0; i < 25; i++)
  {
    M5.dis.drawpix(i, CRGB::Blue);
  }

  //init timers
  timer_iic.every(10, iic_sent);
  timer.every(4, ik_cal);

  body_test.init_body_mat();
  //x,y,z
  body_test.set_body_pos_tar(0, 0, 90);
  //rot along x,y,z axis, aka pitch,roll,yaw
  //considering the 2dof leg structure, yaw always set to 0
  body_test.set_body_rot_tar(0, 0, 0);
}

bool iic_sent(void *)
{
  body_test.send_to_leg();
  //if return false, the kernal would be killed
  return true;
}

bool ik_cal(void *)
{
  //get imu reading, maybe intergate in later version
  float pitch = 0, roll = 0, yaw = 0;
  //getAhrsData pitch roll yaw using another filter
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  //offset clear roll_true-=187
  if (roll < 0)
    roll = 360 + roll;
  //M5 button update
  M5.update();

  //imu serial test 
  Serial.print(pitch); Serial.print(" ");
  Serial.print(roll); Serial.print(" ");
  Serial.print(yaw); Serial.println(" ");


  body_test.body_ik();

  return true;
}

void loop()
{
  
  timer_iic.tick();
  timer.tick();
}