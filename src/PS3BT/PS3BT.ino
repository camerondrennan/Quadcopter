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
int currentSpeed;

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
  motor2.attach(7);
  motor3.attach(6);
  motor4.attach(3);

  currentSpeed = 1100;

  

  motor.writeMicroseconds(1060);
  motor2.writeMicroseconds(1060);
  motor3.writeMicroseconds(1060);
  motor4.writeMicroseconds(1060);



  
}
// 1% = 1127, 100% = 1860
void writeMotorSpeed(int newSpeed) {
  currentSpeed = newSpeed;

  motor.writeMicroseconds(currentSpeed);
  motor2.writeMicroseconds(currentSpeed);
  motor3.writeMicroseconds(currentSpeed);
  motor4.writeMicroseconds(currentSpeed);
}


void loop() {
  Usb.Task();

    
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (PS3.getAnalogButton(R2)) {  
      Serial.print(F("\r\nR2: "));
      Serial.print(PS3.getAnalogButton(R2));
      Serial.print(F("\r\nSpeed: "));
      Serial.print(currentSpeed);

      writeMotorSpeed(currentSpeed + 5);
    }

    if (PS3.getAnalogButton(L2)) {
      Serial.print(F("\r\nL2: "));
      Serial.print(PS3.getAnalogButton(L2));
      Serial.print(F("\r\nSpeed: "));
      Serial.print(currentSpeed);


        writeMotorSpeed(currentSpeed - 5);
    }
    

    if (PS3.getAnalogButton(L1)) {

        writeMotorSpeed(1060);
    }
    
    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS3.disconnect();
    }
  }
}
