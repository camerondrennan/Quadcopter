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

#define EDVCOPTER_DEBUG 
#define ESC_MIN 10;
#define ESC_A 5 //b
#define ESC_D 6 //w
#define ESC_B 3 //blk
#define ESC_C 4 //y

float PITCH_P = 0.5f;
float PITCH_I = 0.06f;
float PITCH_D = 0.06f;
int PITCH_MAX_MOTOR_BALANCE_SPEED = 40;              // max amount of thrust that will be applied to balance this axis
float PITCH_PID_OUTPUT = 40;
float PITCH_ERROR_CORRECTION = -0.02f;

float ROLL_P = 0.5f;
float ROLL_I = 0.06f;
float ROLL_D = 0.06f;
int ROLL_MAX_MOTOR_BALANCE_SPEED = 40;                  // max amount of thrust that will be applied to balance this axis
float ROLL_PID_OUTPUT = 40;
float ROLL_ERROR_CORRECTION = -0.07f;

float YAW_P = 0.5f;
float YAW_I = 0.04f;
float YAW_D = 0.16f;
float YAW_PID_OUTPUT = 20;
int YAW_MAX_MOTOR_BALANCE_SPEED =  40;                 // max amount of thrust that will be applied to balance this axis
float YAW_ERROR_CORRECTION = -0.15f;

#define HOVER_MOTOR_SPEED 10

#define ESC_ARM_TIME 2000                                // in milliseconds, this requires so much time because I also need to wait for sensor values to normalize.
#define SENSOR_NORMALIZE_TIME 1000*20

int tiltRoll, tiltPitch = 0;
boolean start = false;
double pitchSp, rollSp, yawSp = 0;                       // setpoints
bool yawSpSet = false;
double P, I, D;                                          // PID values
float velocity = 0.0;                                          // global velocity
double bal_ac, bal_bd, bal_axes = 0;                 // motor balances can vary between -100 & 100, motor balance between axes -100:ac , +100:bd
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

    if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) {

      tiltPitch = (PS3.getAnalogHat(LeftHatX) - 122) / 10;

      tiltRoll = (PS3.getAnalogHat(LeftHatY) - 122) / 10;



    } else {
      tiltPitch = tiltRoll = 0;


    }

    if (PS3.getButtonClick(CROSS)) {
      ROLL_P = ROLL_P - 0.01f;
      PITCH_P = PITCH_P - 0.01f;
//        YAW_P = YAW_P - 0.01f;

      rollReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      pitchReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      yawReg.SetTunings(YAW_P, YAW_I, YAW_D);

    }

    if (PS3.getButtonClick(TRIANGLE)) {
      ROLL_P = ROLL_P + 0.01f;
      PITCH_P = PITCH_P + 0.01f;
//       YAW_P = YAW_P + 0.01f;

      rollReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      pitchReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      yawReg.SetTunings(YAW_P, YAW_I, YAW_D);
    }

    if (PS3.getButtonClick(SQUARE)) {
      ROLL_I = ROLL_I - 0.01f;
      PITCH_I = PITCH_I - 0.01f;
//        YAW_I = YAW_I - 0.01f;

      rollReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      pitchReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      yawReg.SetTunings(YAW_P, YAW_I, YAW_D);
    }

    if (PS3.getButtonClick(CIRCLE)) {
      ROLL_I = ROLL_I + 0.01f;
      PITCH_I = PITCH_I + 0.01f;
//      YAW_I = YAW_I + 0.01f;

      rollReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      pitchReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      yawReg.SetTunings(YAW_P, YAW_I, YAW_D);
    }

    if (PS3.getAnalogButton(R2)) {
      velocity += 1;
    }

    if (PS3.getAnalogButton(L2)) {
      velocity -= 1;
    }

    // Left yaw
    if (PS3.getAnalogButton(L1)) {

    }
    // Right yaw

    else if (PS3.getAnalogButton(R1)) {

    }

    // Stop the motors
    if (PS3.getButtonClick(START)) {
      if (start == false) {
        start = true;
      } else start = false;
    }

    // Disconnect the controller
    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS3.disconnect();
    }

    if (PS3.getButtonClick(UP)) {

      //       PITCH_MAX_MOTOR_BALANCE_SPEED += 10;
      //       ROLL_MAX_MOTOR_BALANCE_SPEED += 10;
      ROLL_PID_OUTPUT += 1;
      PITCH_PID_OUTPUT += 1;
//      YAW_PID_OUTPUT +=1;

    }

    if (PS3.getButtonClick(DOWN)) {
      //       PITCH_MAX_MOTOR_BALANCE_SPEED -= 10;
      //       ROLL_MAX_MOTOR_BALANCE_SPEED -= 10;

      ROLL_PID_OUTPUT -= 1;
      PITCH_PID_OUTPUT -= 1;
//        YAW_PID_OUTPUT -=1;

    }

    if (PS3.getButtonClick(LEFT)) {
      ROLL_D = ROLL_D - 0.01;
      PITCH_D = PITCH_D - 0.01;
//      YAW_D = YAW_D - 0.01;
//
      rollReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      pitchReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      yawReg.SetTunings(YAW_P, YAW_I, YAW_D);

    }

    // Roll right
    if (PS3.getButtonClick(RIGHT)) {
      ROLL_D = ROLL_D + 0.01;
      PITCH_D = PITCH_D + 0.01;
      
      rollReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
      pitchReg.SetTunings(ROLL_P, ROLL_I, ROLL_D);
    }
    Serial.print("# ");
    if (start) Serial.print("ON ");
    else Serial.print("OFF ");
  } else {
    start = false;
  }

  if (start == true) {
    Serial.print(("Roll P: "));
    Serial.print(ROLL_P);
    Serial.print((" I: "));
    Serial.print(ROLL_I);
    Serial.print((" D: "));
    Serial.print(ROLL_D);
    Serial.print(("; "));

    Serial.print(("Velocity is "));
    Serial.print(velocity);
    Serial.print((" "));

    Serial.print(("Out is "));
    Serial.print(ROLL_PID_OUTPUT);
    Serial.print((" "));

    //    Serial.print((" Pitch is "));
    //    Serial.print(tiltPitch);
    //    Serial.print((" Roll is "));
    //    Serial.print(tiltRoll);
    //    Serial.print((" "));

    setSetPoint();
    computeRotation();
    computeVelocities();
    updateMotors();

    if (sensor.isReady()) {
      sensor.calculate();
    }
  } else {

    va = vb = vc = vd = 0;
    updateMotors();
  }

  if (PITCH_P < 0) PITCH_P = 0;
  if (PITCH_I < 0) PITCH_I = 0;
  if (PITCH_D < 0) PITCH_D = 0;

  if (ROLL_P < 0) ROLL_P = 0;
  if (ROLL_I < 0) ROLL_I = 0;
  if (ROLL_D < 0) ROLL_D = 0;

  if (YAW_P < 0) YAW_P = 0;
  if (YAW_I < 0) YAW_I = 0;
  if (YAW_D < 0) YAW_D = 0;
}

void setSetPoint() {
  rollSp = 0 + tiltRoll;
  pitchSp = 0 + tiltPitch;
  yawSp = 0;
}

void computeRotation()
{
  pitch = ((sensor.getPitch() + PITCH_ERROR_CORRECTION) * (180 / M_PI)); // Get value from sensor, correct it, and convert from radians to degrees
  roll = ((sensor.getRoll() + ROLL_ERROR_CORRECTION) * (180 / M_PI)); // Same thing here
  yaw = ((sensor.getYaw() + YAW_ERROR_CORRECTION) * (180 / M_PI));

      Serial.print("Rotation: ");
      Serial.print(sensor.getPitch()+ PITCH_ERROR_CORRECTION);Serial.print(" ");
      Serial.print(sensor.getRoll()+ ROLL_ERROR_CORRECTION);Serial.print(" ");
      Serial.print(sensor.getYaw()+ YAW_ERROR_CORRECTION);Serial.print(" ");
      Serial.print(" ");
//  Serial.print("Rotation: ");
//  Serial.print(pitch);
//  Serial.print(" ");
//  Serial.print(roll);
//  Serial.print(" ");
//  Serial.print(yaw);
//  Serial.print(" ");

  //if(abs(pitch) <= 1.5f) pitch = 0;
  //if(abs(roll) <= 1.5f) roll = 0;
}

void computeVelocities()
{

  if (pitchReg.Compute()) {

    bal_bd /= PITCH_PID_OUTPUT;

    vd = velocity + (bal_bd * PITCH_MAX_MOTOR_BALANCE_SPEED);
    vb = velocity - (bal_bd * PITCH_MAX_MOTOR_BALANCE_SPEED);

    //    Serial.print(" bal_bd ");
    //    Serial.print(bal_bd);

  }

  if (rollReg.Compute()) {

    bal_ac /= ROLL_PID_OUTPUT;

    va = velocity + (bal_ac * ROLL_MAX_MOTOR_BALANCE_SPEED);
    vc = velocity - (bal_ac * ROLL_MAX_MOTOR_BALANCE_SPEED);

    Serial.print(" bal_ac ");
    Serial.print(bal_ac);

  }

  if (yawReg.Compute()) {

    bal_axes /= YAW_PID_OUTPUT;

        va += bal_axes * YAW_MAX_MOTOR_BALANCE_SPEED;
        vc += bal_axes * YAW_MAX_MOTOR_BALANCE_SPEED;
    
        vb -= bal_axes * YAW_MAX_MOTOR_BALANCE_SPEED;
        vd -= bal_axes * YAW_MAX_MOTOR_BALANCE_SPEED;

    //    Serial.print(" bal_axes ");
    //    Serial.print(bal_axes);
  }

  vb += tiltPitch;
  vc += tiltPitch;
  va -= tiltPitch;
  vd -= tiltPitch;

  va += tiltRoll;
  vb += tiltRoll;
  vd -= tiltRoll;
  vc -= tiltRoll;

  if (va < 20) va = 20;
  if (vb < 20) vb = 20;
  if (vc < 20) vc = 20;
  if (vd < 20) vd = 20;

}

void updateMotors() {
  Serial.print(" uSeconds: ");
  Serial.print(coef(va));
  Serial.print(" ");
  Serial.print(coef(vb));
  Serial.print(" ");
  Serial.print(coef(vc));
  Serial.print(" ");
  Serial.print(coef(vd));
  Serial.println(" ");

  a.writeMicroseconds(coef(va));
  c.writeMicroseconds(coef(vc));
  b.writeMicroseconds(coef(vb));
  d.writeMicroseconds(coef(vd));
}

int coef(int val) {
  int coef = 800 / 180;
  int v = (val * coef) + 1060;
  return v;
}

void initESCs() {
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

