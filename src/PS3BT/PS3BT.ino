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

/*Servo myMotor;
Servo myMotor2;
Servo myMotor3;
Servo myMotor4;*/

int val;

/*void setMotor(int val){
  myMotor.writeMicroseconds(val);
  myMotor2.writeMicroseconds(val);
  myMotor3.writeMicroseconds(val);
  myMotor4.writeMicroseconds(val);
}*/

void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void loop() {
  Usb.Task();
    
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (PS3.getAnalogButton(R2)) {  
      Serial.print(F("\r\nR2: "));
      Serial.print(PS3.getAnalogButton(R2));
    }

    if (PS3.getAnalogButton(L2)) {
      Serial.print(F("\r\nL2: "));
      Serial.print(PS3.getAnalogButton(L2));
    }
    
    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS3.disconnect();
    }
  }
}
