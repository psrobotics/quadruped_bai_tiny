#include <SimpleFOC.h>
int LED = 10;

union floatint {
  uint32_t i; float f; char b[4];
};

// magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 9, 7);

// target voltage to be set to the motor
float target_voltage = 0;
float init_angle = 0;
float torque_feedforward = 0;
//target joint position
float target_angle = 0;

//pd para
float err_now = 0;
float err_sum = 0;
float err_old = 0;
float pos_k = 4.8; //5.25
float pos_i = 0.019;
float pos_d = 12; //17.59

void setup() {

  //display led
  pinMode(LED, OUTPUT);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // power supply voltage
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 4;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  //Serial.println("Motor ready.");
  //Serial.println("Set the target voltage using serial terminal:");
  _delay(500);
  
  digitalWrite(LED, HIGH);
  //wait user to calibrate legs into init pos
  _delay(10000);
  digitalWrite(LED, LOW);
  _delay(500);
  
  init_angle = motor.shaftAngle();

  digitalWrite(LED, HIGH);
  _delay(500);
  //digitalWrite(LED, LOW);
}

void loop() 
{
  motor.loopFOC();

  //add the feedforward torque
  motor.move(target_voltage + torque_feedforward);

  serialReceiveUserCommand();

  //offset now
  err_now = target_angle - motor.shaft_angle + init_angle;

  //i item
  if(abs(err_now)<0.5)
    err_sum += err_now;

  err_sum = constrain(err_sum, -100.0, 100.0);
  target_voltage = pos_k * err_now + pos_i * err_sum + pos_d*(err_now-err_old);
  err_old = err_now;

  //analogWrite(LED, abs((target_voltage + torque_feedforward) / driver.voltage_power_supply) * 255);
}


// utility function enabling serial communication with the user to set the target values
// this function can be implemented in serialEvent function as well
void serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  //floatint data_read;
  //init pack value
  //data_read.i = (((unsigned long)32500 << 16) | 32500);
  
  while (Serial.available()) {
    // get the new byte:
    char inChar = Serial.read(); //init char insert
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') 
    {  
      // change the motor target
      //received_chars.toCharArray(data_read.b,4);
      int receive_tmp = received_chars.toInt();
      // reset the command buffer 
      received_chars = "";
      //unpack_from_float (target_angle, torque_feedforward, data_read);
      if(receive_tmp < 0) //torque offset
        torque_feedforward = receive_tmp/1000.0 + 16;
      else //target angle
        target_angle = receive_tmp/1000.0 - 16;
    }
  }
}
