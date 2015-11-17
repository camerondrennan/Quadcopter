


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


Servo myMotor, myMotor3;
Servo myMotor2, myMotor4;

BTD Btd(&Usb); 
PS3BT PS3(&Btd); 

bool printTemperature;
bool printAngle;

void setup() {

  myMotor.attach(3);
  myMotor2.attach(4);
  myMotor3.attach(5);
  myMotor4.attach(6);

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
  int val;
  
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) {
      Serial.print(F("\r\nLeftHatX: "));
      Serial.print(PS3.getAnalogHat(LeftHatX));
      Serial.print(F("\tLeftHatY: "));
      Serial.print(PS3.getAnalogHat(LeftHatY));
      Serial.print(F("\tRightHatX: "));
      Serial.print(PS3.getAnalogHat(RightHatX));
      Serial.print(F("\tRightHatY: "));
      Serial.print(PS3.getAnalogHat(RightHatY));
    
    }

    // Analog button values can be read from almost all buttons
    if (PS3.getAnalogButton(L2)) {
     // Serial.print(F("\r\nL2: "));

  
      
       if(PS3.getAnalogButton(L2) > 10 && PS3.getAnalogButton(L2) <65) { 
       myMotor.writeMicroseconds(val);
       myMotor2.writeMicroseconds(val);
       myMotor3.writeMicroseconds(val);
       myMotor4.writeMicroseconds(val);
       Serial.print("L2 Minimum\n");

       
       }
       else if (PS3.getAnalogButton(L2) >= 65 && PS3.getAnalogButton(L2) < 130) {
       myMotor.writeMicroseconds(val);
       myMotor2.writeMicroseconds(val);
       myMotor3.writeMicroseconds(val);
       myMotor4.writeMicroseconds(val);
         Serial.print("L2 Lower Middle\n");
      
    }
      else if (PS3.getAnalogButton(L2) >= 130 && PS3.getAnalogButton(L2) < 195) {
       myMotor.writeMicroseconds(val);
       myMotor2.writeMicroseconds(val);
       myMotor3.writeMicroseconds(val);
       myMotor4.writeMicroseconds(val);
       Serial.print("L2 Upper Middle\n");
        
      }
      else if (PS3.getAnalogButton(L2) >= 195 && PS3.getAnalogButton(L2) < 255) {
       myMotor.writeMicroseconds(val);
       myMotor2.writeMicroseconds(val);
       myMotor3.writeMicroseconds(val);
       myMotor4.writeMicroseconds(val);
       Serial.print("L2 Maximum\n");
      }
      
    }

   if (PS3.getAnalogButton(R2)) {
    
    if (PS3.getAnalogButton(R2) < 10) {
    //  Serial.print(F("\tR2: "));
       myMotor.writeMicroseconds(val);
       myMotor2.writeMicroseconds(val);
       myMotor3.writeMicroseconds(val);
       myMotor4.writeMicroseconds(val);
       Serial.print("R2 Zero\n");
    }
    else  if (PS3.getAnalogButton(R2) > 10 && PS3.getAnalogButton(R2) <65) {
        
       myMotor.writeMicroseconds(val);
       myMotor2.writeMicroseconds(val);
       myMotor3.writeMicroseconds(val);
       myMotor4.writeMicroseconds(val);
       Serial.print("R2 Minimum\n");
}
else if (PS3.getAnalogButton(R2) >= 65 && PS3.getAnalogButton(R2) <130) {
       myMotor.writeMicroseconds(val);
       myMotor2.writeMicroseconds(val);
       myMotor3.writeMicroseconds(val);
       myMotor4.writeMicroseconds(val);
       Serial.print("R2 Lower Middle\n");
  
}
  else if (PS3.getAnalogButton(R2) >= 130 && PS3.getAnalogButton(R2) <195) {
       myMotor.writeMicroseconds(val);
       myMotor2.writeMicroseconds(val);
       myMotor3.writeMicroseconds(val);
       myMotor4.writeMicroseconds(val);
       Serial.print("R2 Upper Middle\n");
  }
  else if (PS3.getAnalogButton(R2) >= 195 && PS3.getAnalogButton(R2) <255) {
       myMotor.writeMicroseconds(val);
       myMotor2.writeMicroseconds(val);
       myMotor3.writeMicroseconds(val);
       myMotor4.writeMicroseconds(val);
       Serial.print("R2 Maximum Power.\n");
  }

    }

    
    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS3.disconnect();
    }
    else {
      if (PS3.getButtonClick(TRIANGLE))
        Serial.print(F("\r\nTraingle"));
      if (PS3.getButtonClick(CIRCLE))
        Serial.print(F("\r\nCircle"));
      if (PS3.getButtonClick(CROSS))
        Serial.print(F("\r\nCross"));
      if (PS3.getButtonClick(SQUARE))
        Serial.print(F("\r\nSquare"));

      if (PS3.getButtonClick(UP)) {
        Serial.print(F("\r\nUp"));
        if (PS3.PS3Connected) {
          PS3.setLedOff();
          PS3.setLedOn(LED4);
        }
      }
      if (PS3.getButtonClick(RIGHT)) {
        Serial.print(F("\r\nRight"));
        if (PS3.PS3Connected) {
          PS3.setLedOff();
          PS3.setLedOn(LED1);
        }
      }
      if (PS3.getButtonClick(DOWN)) {
        Serial.print(F("\r\nDown"));
        if (PS3.PS3Connected) {
          PS3.setLedOff();
          PS3.setLedOn(LED2);
        }
      }
      if (PS3.getButtonClick(LEFT)) {
        Serial.print(F("\r\nLeft"));
        if (PS3.PS3Connected) {
          PS3.setLedOff();
          PS3.setLedOn(LED3);
        }
      }

      if (PS3.getButtonClick(L1))
        Serial.print(F("\r\nL1"));
      if (PS3.getButtonClick(L3))
        Serial.print(F("\r\nL3"));
      if (PS3.getButtonClick(R1))
        Serial.print(F("\r\nR1"));
      if (PS3.getButtonClick(R3))
        Serial.print(F("\r\nR3"));

      if (PS3.getButtonClick(SELECT)) {
        Serial.print(F("\r\nSelect - "));
        PS3.printStatusString();
      }
      if (PS3.getButtonClick(START)) {
        Serial.print(F("\r\nStart"));
        printAngle = !printAngle;
      }
    }

  }

}
