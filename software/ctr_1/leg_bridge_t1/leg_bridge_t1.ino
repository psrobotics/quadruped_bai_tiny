#include "Arduino.h"
#include "leg_bridge.h"

#define FL 10
#define FR 11
#define BL 12
#define BR 13

union float_byte {
  float f;
  char c[sizeof(f)];
};

float_byte rad_rev[2];
float_byte tor_rev[2];

float_byte sent_tmp[2];

leg_bridge *bridge;

float rad_tmp[3];
float tor_tmp[3];

int led = 9;
SoftwareSerial *s_arr[3];

void setup()
{

  Serial.begin(115200);
  pinMode(led, OUTPUT);

  bridge = new leg_bridge(FL);

  Wire.begin(BR);
  Wire.onReceive(receiveEvent); // register event

  s_arr[0] = new SoftwareSerial(3, 2);
  s_arr[1] = new SoftwareSerial(5, 4);

  s_arr[0]->begin(115200);
  s_arr[1]->begin(115200);

  //digitalWrite(led, HIGH);
  sent_tmp[0].f = 0;
  sent_tmp[1].f = 0;
}

void loop()
{

  //bridge->change_rad(rad_tmp);
  //bridge->change_tor(tor_tmp);

  //Serial.println(rad_tmp[0]);
  //Serial.println(rad_tmp[1]);

  //bridge->write_driver(s_arr);
  //delay(2);
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
          rad_rev[s].c[i]=Wire.read();

      rad_tmp[0]=rad_rev[0].f;
      rad_tmp[1]=rad_rev[1].f;
    }
    else if (c == 't')//torque
    {
      for (int s = 0; s < 2; s++)
        for (int i = 0; i < 4; i++)
          tor_rev[s].c[i]=Wire.read();

       tor_tmp[0]=tor_rev[0].f;
       tor_tmp[1]=tor_rev[1].f;
    }
  }

  //send to serial after the callback,
  //so the programmer can be controlled with external source
  bridge->change_rad(rad_tmp);
  bridge->change_tor(tor_tmp);

  bridge->write_driver(s_arr);
}
