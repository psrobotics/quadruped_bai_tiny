/**

   Position/angle motion control example
   Steps:
   1) Configure the motor and magnetic sensor
   2) Run the code
   3) Set the target angle (in radians) from serial terminal

*/
#include <SimpleFOC.h>

int led = 10;

// magnetic sensor instance - SPI
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// magnetic sensor instance - MagneticSensorI2C
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 7);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void onMotion(char* cmd) {
  command.motion(&motor, cmd);
}

void setup() {
  pinMode(led, OUTPUT);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  // after mod, the velocity pid is position-torque pid
  
  motor.PID_velocity.P = 8.1;//0.05;//0.085;//0.12 //7.1
  motor.PID_velocity.I = 0.1;//0.8;//15
  motor.PID_velocity.D = 0.02;//0.015;//0;//0.001 //0.029

  //disabele ramp
  motor.PID_velocity.output_ramp = -1;
  
  // maximal voltage to be set to the motor
  motor.voltage_limit = 12;

  // velocity low pass filtering time constant
  // the lower the less filtered
  //motor.LPF_velocity.Tf = 0.005;

  // angle P controller 
  //motor.P_angle.P = 0.05;//80;
  //motor.P_angle.I = 0;
  //motor.P_angle.D = 0;//0.008;
  // maximal velocity of the position control
  motor.velocity_limit = 50;

  //motor.motion_downsample = 2;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);


  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('M', onMotion, "motion control");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));

  //offset, measured for each joint, change with different joints
  motor.sensor_offset = -5;

  ////////////////////////////////////////////////////////////////
  digitalWrite(led, HIGH);
  _delay(1000);

  //motor.disable();

  //motor.disable();
  //motor.target = motor.shaftAngle();
  //calibration process, reset shaft angle

  /*
  float shaft_ang_tmp = 0;
  for(int s=0;s<10;s++)
  {
    shaft_ang_tmp += motor.shaftAngle();
    Serial.println(motor.shaftAngle());
    _delay(20);
  }
  shaft_ang_tmp /= 10.0;
  motor.sensor_offset = shaft_ang_tmp;
  //motor.enable();
  digitalWrite(led, HIGH);
  _delay(500);
*/

}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  //shaft_ang_tmp - offset, target - ang setpoint, shift_angle - current router angle
  
  motor.move();


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();

  // user communication
  command.run();

  //Serial.println(motor.shaft_angle);
}
