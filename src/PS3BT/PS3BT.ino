#include <PS3BT.h>
#include <Servo.h>
#include <usbhub.h>


// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
BTD Btd(&Usb); 
PS3BT PS3(&Btd); 


Servo motor;
Servo motor2;
Servo motor3;
Servo motor4;

int val;



void setup() {
  Serial.begin(9600);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));


  motor.attach(5);
  motor2.attach(3);
  motor3.attach(7);
  motor4.attach(9);

  

  motor.writeMicroseconds(1060);
  motor2.writeMicroseconds(1060);
  motor3.writeMicroseconds(1060);
  motor4.writeMicroseconds(1060);

  //delay(100);

//  //myMotor.writeMicroseconds(1440);
//  myMotor2.writeMicroseconds(1860);
//  //myMotor3.writeMicroseconds(1440);
//  //myMotor4.writeMicroseconds(1440);

  
}


void loop() {
  Usb.Task();



  int currentSpeed = 1400;

  
  //motor.writeMicroseconds(currentSpeed);
  //motor2.writeMicroseconds(currentSpeed);
  //motor3.writeMicroseconds(currentSpeed);
  //motor4.writeMicroseconds(currentSpeed);
  
    
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (PS3.getAnalogButton(R2)) {  
      Serial.print(F("\r\nR2: "));
      Serial.print(PS3.getAnalogButton(R2));

      //increaseMotorSpeed(currentSpeed);
       motor.writeMicroseconds(1440);
  motor2.writeMicroseconds(1440);
  motor3.writeMicroseconds(1440);
  motor4.writeMicroseconds(1440);
    }

    if (PS3.getAnalogButton(L2)) {
      Serial.print(F("\r\nL2: "));
      Serial.print(PS3.getAnalogButton(L2));

      //decreaseMotorSpeed(currentSpeed);
       motor.writeMicroseconds(1060);
       motor2.writeMicroseconds(1060);
        motor3.writeMicroseconds(1060);
    motor4.writeMicroseconds(1060);
    }
    
    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS3.disconnect();
    }
  }
}
