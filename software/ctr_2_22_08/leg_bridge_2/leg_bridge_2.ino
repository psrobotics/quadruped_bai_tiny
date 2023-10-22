#include <SoftwareSerial.h>
#include <Wire.h>
#include "Arduino.h"

#define FL 10
#define FR 11
#define BL 12
#define BR 13

#define POSITION_CTR 0
#define TORQUE_CTR 1

union float_byte {
  float f;
  char c[sizeof(f)];
};

float_byte rad_rev[2];
float_byte tor_rev[2];

float_byte sent_tmp[2];

float rad_tmp[3];
float tor_tmp[3];

int led = 9;
SoftwareSerial *s_arr[3];

int control_type = POSITION_CTR;

void setup()
{

  Serial.begin(115200);
  pinMode(led, OUTPUT);

  Wire.begin(BR);
  Wire.onReceive(receiveEvent); // register event

  s_arr[0] = new SoftwareSerial(3, 2);
  s_arr[1] = new SoftwareSerial(5, 4);

  s_arr[0]->begin(115200);
  s_arr[1]->begin(115200);

  digitalWrite(led, HIGH);
  sent_tmp[0].f = 0;
  sent_tmp[1].f = 0;

  //wait untill all joints are working
  delay(2000);
}

void loop()
{

}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  while (1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    if (c == 'r')//rad
    {
      for (int s = 0; s < 2; s++)
        for (int i = 0; i < 4; i++)
          rad_rev[s].c[i] = Wire.read();

      rad_tmp[0] = rad_rev[0].f;
      rad_tmp[1] = rad_rev[1].f;

      //write rad to joints
      write_rad(rad_tmp[0], rad_tmp[1]);
    }
    else if (c == 't')//torque
    {
      for (int s = 0; s < 2; s++)
        for (int i = 0; i < 4; i++)
          tor_rev[s].c[i] = Wire.read();

      tor_tmp[0] = tor_rev[0].f;
      tor_tmp[1] = tor_rev[1].f;

      //write torque to joints
      write_tor(tor_tmp[0], tor_tmp[1]);
    }
    else if (c == 'c')
    {
      //change to position control mode
      for (int s = 0; s < 2; s++)
      {
        s_arr[0]->println("MC2");
        s_arr[1]->println("MC2");
      }
    }
    else if (c == 'v')
    {
      //change to torque control mode
      for (int s = 0; s < 2; s++)
      {
        s_arr[0]->println("MC0");
        s_arr[1]->println("MC0");
      }
    }
    else if (c == 'e')
    {
      //enable joints
      mot_enable();
    }
    else if (c == 's')
    {
      //disable joints
      mot_disable();
    }
  }
  //send to serial only after the iic callback,
  //so the programmer can be controlled with external source
}

int change_to_pos_ctr()
{
  if (control_type == POSITION_CTR)
    return;
  else if (control_type == TORQUE_CTR)
  {
    for (int s = 0; s < 2; s++)
    {
      s_arr[0]->println("MC2");
      s_arr[1]->println("MC2");
    }
    control_type = POSITION_CTR;
  }
  return 0;
}

int change_to_tor_ctr()
{
  if (control_type == TORQUE_CTR)
    return;
  else if (control_type == POSITION_CTR)
  {
    for (int s = 0; s < 2; s++)
    {
      s_arr[0]->println("MC0");
      s_arr[1]->println("MC0");
    }
    control_type = TORQUE_CTR;
  }
  return 0;
}

int write_rad(float rad_1, float rad_2)
{
  s_arr[0]->print('M');
  s_arr[0]->println(double(rad_1), 3);

  s_arr[1]->print('M');
  s_arr[1]->println(double(rad_2), 3);

  return 0;
}

int write_tor(float tor_1, float tor_2)
{
  s_arr[0]->print('M');
  s_arr[0]->println(double(tor_1), 3);

  s_arr[1]->print('M');
  s_arr[1]->println(double(tor_2), 3);

  return 0;
}

int mot_enable()
{
  for (int s = 0; s < 2; s++)
  {
    s_arr[0]->println("ME0");
    s_arr[1]->println("ME0");
  }
  return 0;
}

int mot_disable()
{
  for (int s = 0; s < 2; s++)
  {
    s_arr[0]->println("ME1");
    s_arr[1]->println("ME1");
  }
  return 0;
}
