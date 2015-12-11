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

// 1 + 2 = CCW
// 3 + 4 = CW
Servo motor;
Servo motor2;
Servo motor3;
Servo motor4;


int motor1Speed;
int motor2Speed;
int motor3Speed;
int motor4Speed;


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


  // Specify which pins the motors are attached to
  motor.attach(5);
  motor2.attach(9);
  motor3.attach(6);
  motor4.attach(3);

  

  // Give the motor speed variables a baseline value
  motor1Speed = 1100;
  motor2Speed = 1100;
  motor3Speed = 1100;
  motor4Speed = 1100;

  
  // Initialise the motors
  motor.writeMicroseconds(1060);
  motor2.writeMicroseconds(1060);
  motor3.writeMicroseconds(1060);
  motor4.writeMicroseconds(1060);

  
}

// Writes the motor speed to the motors
// 1% = 1127, 100% = 1860
void writeMotorSpeed() {
  
  motor.writeMicroseconds(motor1Speed);
  motor2.writeMicroseconds(motor2Speed);
  motor3.writeMicroseconds(motor3Speed);
  motor4.writeMicroseconds(motor4Speed);
}


void loop() {
  Usb.Task();

  // Update the motor speed to the last value set
  writeMotorSpeed();

    
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    
    // Increase throttle
    if (PS3.getAnalogButton(R2)) {  
      Serial.print(F("\r\nR2: "));
      Serial.print(PS3.getAnalogButton(R2));
      Serial.print(F("\r\nSpeed: "));
      Serial.print(motor1Speed);

        motor1Speed += 5;
        motor2Speed += 5;
        motor3Speed += 5;
        motor4Speed += 5;
        //writeMotorSpeed();
    }

    // Decrease throttle
    if (PS3.getAnalogButton(L2)) {
      Serial.print(F("\r\nL2: "));
      Serial.print(PS3.getAnalogButton(L2));
      Serial.print(F("\r\nSpeed: "));
      Serial.print(motor1Speed);


        motor1Speed -= 5;
        motor2Speed -= 5;
        motor3Speed -= 5;
        motor4Speed -= 5;
        
    }
    

    
    int yawAmount = 2;
    // Left yaw
    if (PS3.getAnalogButton(L1)) {
      Serial.print(F("\r\nLeftYaw"));
      
      // Increase the counter clockwise rotors by yawAmount
      // and decrease the clockwise rotors by yawAmount
      motor1Speed += yawAmount;
      motor2Speed += yawAmount;
      
      motor3Speed -= yawAmount;
      motor4Speed -= yawAmount;      
    }
    // Right yaw
    // Don't allow both L1 and R1 at the same time
    else if (PS3.getAnalogButton(R1)) {
      Serial.print(F("\r\nRightYaw"));
      
      // Increase the clockwise rotors by yawAmount
      // and decrease the ccw rotors by yawAmount
      
      motor3Speed += yawAmount;
      motor4Speed += yawAmount;

      motor1Speed -= yawAmount;
      motor2Speed -= yawAmount;
    }

    // Stop the motors
    if (PS3.getButtonClick(START)) {
        // Should kill the motors
        motor1Speed = 1120;
        motor2Speed = 1120;
        motor3Speed = 1120;
        motor4Speed = 1120;
        
    }

    // Disconnect the controller
    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS3.disconnect();
    }

    int pitchRollAmount = 5;

    // Pitch forward
    if (PS3.getButtonClick(UP)) {
      // TODO: Add min max checks
      motor1Speed -= pitchRollAmount;
      motor2Speed += pitchRollAmount;
    }

    // Pitch backwards
    if (PS3.getButtonClick(DOWN)) {
      // TODO: Add min max checks
      motor1Speed += pitchRollAmount;
      motor2Speed -= pitchRollAmount;
    }

    // Roll left
    if (PS3.getButtonClick(LEFT)) {
      motor3Speed -= pitchRollAmount;
      motor4Speed += pitchRollAmount;
    }

    // Roll right
    if (PS3.getButtonClick(RIGHT)) {
      motor3Speed += pitchRollAmount;
      motor4Speed -= pitchRollAmount;
    }
    
  }
  else {
    // Fail safe if the controller disconnects?
    motor1Speed = 1120;
    motor2Speed = 1120;
    motor3Speed = 1120;
    motor4Speed = 1120;
  }
}
