#include <PS3BT.h>
#include <usbhub.h>
#include <Servo.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
USBHub Hub1(&Usb); // Some dongles have a hub inside

Servo myMotor;
Servo myMotor2;
Servo myMotor3;
Servo myMotor4;

BTD Btd(&Usb); 
PS3BT PS3(&Btd); 

bool printTemperature;
bool printAngle;

int val;

void setup() {
  myMotor.attach(3);
  myMotor2.attach(5);
  myMotor3.attach(6);
  myMotor4.attach(10);

  setMotor(1000);

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
    Serial.print("we are here");
    
    // Analog button values can be read from almost all buttons
    if (PS3.getAnalogButton(R2)) {  
      //Serial.print(F("\r\nR2: "));
      setMotor(1200+PS3.getAnalogButton(R2));
    }
    
    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS3.disconnect();
    }
  }
}

void setMotor(int val){
  myMotor.writeMicroseconds(val);
  myMotor2.writeMicroseconds(val);
  myMotor3.writeMicroseconds(val);
  myMotor4.writeMicroseconds(val);
}
