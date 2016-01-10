#include <PS3BT.h>
#include <Servo.h>
#include <usbhub.h>
#include <Wire.h>
#include "MPUSensor.h"
#include "PID_v1.h"


// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif


#define EDVCOPTER_DEBUG  // use this, if you want to connect quadcopter to my logging software

#define ESC_A 5 //b
#define ESC_D 6 //w
#define ESC_B 3 //blk
#define ESC_C 9 //y



#define ESC_MIN 10
#define ESC_MAX 180
#define PITCH_P 0.0f
#define PITCH_I 0.0f
#define PITCH_D 0.0f
#define PITCH_MAX_MOTOR_BALANCE_SPEED 30              // max amount of thrust that will be applied to balance this axis
#define PITCH_PID_OUTPUT 20
#define PITCH_ERROR_CORRECTION -0.010f



float ROLL_P = 0.100f;
float ROLL_I = 0.000f;
float ROLL_D =  0.000f;
int ROLL_MAX_MOTOR_BALANCE_SPEED = 30;                  // max amount of thrust that will be applied to balance this axis
#define ROLL_PID_OUTPUT 20
#define ROLL_ERROR_CORRECTION 0.05f

#define YAW_P 0.3f
#define YAW_I 0.04f
#define YAW_D 0.308f
#define YAW_PID_OUTPUT 20
#define YAW_MAX_MOTOR_BALANCE_SPEED 30                   // max amount of thrust that will be applied to balance this axis
#define YAW_ERROR_CORRECTION 0.0948f

#define HOVER_MOTOR_SPEED 10

#define ESC_ARM_TIME 2000                                // in milliseconds, this requires so much time because I also need to wait for sensor values to normalize.
#define SENSOR_NORMALIZE_TIME 1000*20


double pitchSp, rollSp, yawSp = 0;                       // setpoints
bool yawSpSet = false;
double P, I, D;                                          // PID values
float velocity = 20.0;                                          // global velocity
double bal_ac = 0, bal_bd, bal_axes = 0;                 // motor balances can vary between -100 & 100, motor balance between axes -100:ac , +100:bd
float deltaTime = 0;

double va, vb, vc, vd, v_ac, v_bd = 0;                   // velocities
double pitch, roll, yaw  = 0.0;                          // angles in degrees


USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57);

Servo a, b, c, d;                                        // motors

PID pitchReg(&pitch, &bal_bd, &pitchSp, PITCH_P, PITCH_I, PITCH_D, DIRECT);
PID rollReg(&roll, &bal_ac, &rollSp, ROLL_P, ROLL_I, ROLL_D, DIRECT);
PID yawReg(&yaw, &bal_axes, &yawSp, YAW_P, YAW_I, YAW_D, DIRECT);

float sentTime = 0;

MPUSensor sensor;




void setup() {
  Serial.begin(115200);
  
  
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));

  sensor.init();
  initPIDs();
  initESCs();
  armESCs();

}

void loop() {
  
  Usb.Task();

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {


    if (PS3.getButtonClick(CROSS)) {
      ROLL_P = ROLL_P - 0.005;
    }

    if (PS3.getButtonClick(TRIANGLE)) {
      ROLL_P = ROLL_P + 0.005;
    }

    if (PS3.getButtonClick(SQUARE)) {
      ROLL_I = ROLL_I - 0.005;
    }

    if (PS3.getButtonClick(CIRCLE)) {
      ROLL_I = ROLL_I + 0.005;
    }

    
    if (PS3.getAnalogButton(R2)) {
      velocity += 0.005;
    }

    
    if (PS3.getAnalogButton(L2)) {
     velocity -= 0.005;
    }
    
    // Left yaw
    if (PS3.getAnalogButton(L1)) {
      Serial.print(F("\r\nLeftYaw"));

    }
    // Right yaw
    // Don't allow both L1 and R1 at the same time
    else if (PS3.getAnalogButton(R1)) {
      Serial.print(F("\r\nRightYaw"));


    }

    // Stop the motors
    if (PS3.getButtonClick(START)) {

    }

    // Disconnect the controller
    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS3.disconnect();
    }

    int pitchRollAmount = 5;

    // Pitch forward
    if (PS3.getButtonClick(UP)) {
       ROLL_MAX_MOTOR_BALANCE_SPEED = ROLL_MAX_MOTOR_BALANCE_SPEED + 5;
        Serial.print("Balance Speed Increase ");
        Serial.print(ROLL_MAX_MOTOR_BALANCE_SPEED);
        Serial.print("\n");

    }

    // Pitch backwards
    if (PS3.getButtonClick(DOWN)) {
      ROLL_MAX_MOTOR_BALANCE_SPEED = ROLL_MAX_MOTOR_BALANCE_SPEED - 5;
        Serial.print("Balance Speed Decrease ");
        Serial.print(ROLL_MAX_MOTOR_BALANCE_SPEED);
        Serial.print("\n");
      // TODO: Add min max checks

    }

    // Roll left
    if (PS3.getButtonClick(LEFT)) {
       ROLL_D = ROLL_D - 0.005;
    }

    // Roll right
    if (PS3.getButtonClick(RIGHT)) {
       ROLL_D = ROLL_D + 0.005;
    }
     Serial.print("#");
  }

   Serial.print(("Roll P: "));
   Serial.print(ROLL_P);
   Serial.print((" I: "));
   Serial.print(ROLL_I);
   Serial.print((" D: "));
   Serial.print(ROLL_D);
   Serial.print(("; "));

   Serial.print(("Velocity is "));
   Serial.print(velocity);
   Serial.println((" "));

  setSetPoint();
  computeRotation();
  computeVelocities();
  updateMotors();
  sensor.calculate(); 
}

void setSetPoint() {
  rollSp = 0;
  pitchSp = 0;
  yawSp = 0;
}

void computeRotation()
{
  pitch = ((sensor.getPitch() + PITCH_ERROR_CORRECTION) * (180 / M_PI)); // Get value from sensor, correct it, and convert from radians to degrees
  roll = ((sensor.getRoll() + ROLL_ERROR_CORRECTION) * (180 / M_PI)); // Same thing here
  yaw = ((sensor.getYaw() + YAW_ERROR_CORRECTION) * (180 / M_PI));

//     Serial.print("rotation: ");
//    Serial.print(sensor.getPitch()+ PITCH_ERROR_CORRECTION);Serial.print(" ");
//    Serial.print(sensor.getRoll()+ ROLL_ERROR_CORRECTION);Serial.print(" ");
//    Serial.print(sensor.getYaw()+ YAW_ERROR_CORRECTION);Serial.print(" ");
//    Serial.println(" ");


  //if(abs(pitch) <= 1.5f) pitch = 0;
  //if(abs(roll) <= 1.5f) roll = 0;
}

void computeVelocities()
{

  //velocity = 20.0;


  if (pitchReg.Compute()) {

    bal_bd /= PITCH_PID_OUTPUT;

    vd = velocity + (bal_bd * PITCH_MAX_MOTOR_BALANCE_SPEED);
    vb = velocity - (bal_bd * PITCH_MAX_MOTOR_BALANCE_SPEED);
    //   Serial.print(" Pitch: ");
    //    Serial.print(coef(vd));Serial.print(" "); Serial.print(coef(vb));Serial.print(" ");
    //
  }

  if (rollReg.Compute()) {

    bal_ac /= ROLL_PID_OUTPUT;


    va = velocity + (bal_ac * ROLL_MAX_MOTOR_BALANCE_SPEED);
    vc = velocity - (bal_ac * ROLL_MAX_MOTOR_BALANCE_SPEED);

    //   Serial.print(" Roll: ");
    //     Serial.print(coef(va));Serial.print(" "); Serial.print(coef(vc));Serial.print(" ");
    //    Serial.println(" ");
  }

  if (yawReg.Compute()) {

    bal_axes /= YAW_PID_OUTPUT;
    va -= bal_axes * YAW_MAX_MOTOR_BALANCE_SPEED;
    vc -= bal_axes * YAW_MAX_MOTOR_BALANCE_SPEED;

    vb += bal_axes * YAW_MAX_MOTOR_BALANCE_SPEED;
    vd += bal_axes * YAW_MAX_MOTOR_BALANCE_SPEED;
  }
}

void updateMotors() {

  a.writeMicroseconds(coef(va));
  c.writeMicroseconds(coef(vc));
  b.writeMicroseconds(coef(vb));
  d.writeMicroseconds(coef(vd));
}

int coef(int val) {
  int coef = 1000 / 180;
  int v = (val * coef) + 1000;

  return v;
}

void initESCs() {
  // attach ESCs to servos
  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
}

void armESCs() {
  va = vb = vc = vd = ESC_MIN;
  updateMotors();
  delay(ESC_ARM_TIME);
}

void initPIDs() {
  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PITCH_PID_OUTPUT, PITCH_PID_OUTPUT);
  pitchReg.SetSampleTime(14);

  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-ROLL_PID_OUTPUT, ROLL_PID_OUTPUT);
  rollReg.SetSampleTime(14);

  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-YAW_PID_OUTPUT, YAW_PID_OUTPUT);
  yawReg.SetSampleTime(14);
}

