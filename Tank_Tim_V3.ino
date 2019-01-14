/*************************************************************
  
 ______     __  __     __     __         _____        ______        ______   ______     __   __     __  __    .......(\_/) 
/\  == \   /\ \/\ \   /\ \   /\ \       /\  __-.     /\  __ \      /\__  _\ /\  __ \   /\ "-.\ \   /\ \/ /     ......( '_') 
\ \  __<   \ \ \_\ \  \ \ \  \ \ \____  \ \ \/\ \    \ \  __ \     \/_/\ \/ \ \  __ \  \ \ \-.  \  \ \  _"-.    ..../""""""""""""\======░ ▒▓▓█D    
 \ \_____\  \ \_____\  \ \_\  \ \_____\  \ \____-     \ \_\ \_\       \ \_\  \ \_\ \_\  \ \_\\"\_\  \ \_\ \_\     /"""""""""""""""""""""""\   
  \/_____/   \/_____/   \/_/   \/_____/   \/____/      \/_/\/_/        \/_/   \/_/\/_/   \/_/ \/_/   \/_/\/_/     \_@_@_@_@_@_@_@_/
                                                                                                              

  This sketch was based on existing sketches from the following:

    DFRobot:  https://www.dfrobot.com/blog-494.html
    jlmyra:   https://github.com/jlmyra/Arduino-Blynk-Joystick-4-Motor-Robot-Rover

 
 *************************************************************
  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Blynk community:            http://community.blynk.cc
    Social networks:            http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.
  
 *************************************************************

  This example shows how to use Arduino with HC-08
  Bluetooth BLE module to connect your project to Blynk.

  Feel free to apply it to any other example. It's simple!

  NOTE: Bluetooth support is in beta!

  You can receive x and y coords for joystick movement within App.

  App project setup:
    Two Axis Joystick on V2 in MERGE output mode.
    MERGE mode means device will receive both x and y within 1 message
 *************************************************************/

/* Comment this out to disable prints and save space */

/* Tim Ohling 
  Version 3 - includes "arcade" drive steering based on WPILib DifferentialDrive setArcadeDrive
  as of 2018 WPILib used for FRC
  Also included is an ultrasonic sensor assumed to be front-mounted for collision avoidance.
  maybe.
*/

#define BLYNK_USE_DIRECT_CONNECT
#define BLYNK_PRINT Serial
#include <BlynkSimpleSerialBLE.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <math.h>
#define trigPin 18
#define echoPin 19

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "d3ed6502233241faad6a0f1ffdd2523c"; // <<< INSERT YOUR AUTH CODE HERE

SoftwareSerial SerialBLE(10, 11); // RX, TX

//*************************************************************

AF_DCMotor motorDriveRight(3, MOTOR3_A); // create motor #2, 64KHz pwm
AF_DCMotor motorDriveLeft(4, MOTOR4_A);
int state = 0;
double dThrottleMult = 0.5;

double dTurnRate;
double dSpeed;

//######### SETUP ######################################
void setup()
{
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  delay(1000);

  SerialBLE.begin(9600);
  delay(5000);
  SerialBLE.print("AT+NAMEOhling");
  Blynk.begin(SerialBLE, auth);
  Serial.println("Waiting for connections...");
  Serial.println("Adafruit Motorshield v1 - DC Motor");
}
 

//**********VOID LOOP**********
void loop()
{
  Blynk.run(); // To Run Blynk
}
//**********END VOID LOOP**********
// get Joystick Scale 
// where dAxisValue is a double for a desigated joystick axis position 
// The result will be 0 if position is inside the Deadband (near centered)
// Otherwise it is a 0-100% scale
// positive for reverse and negative for forward
// 6/10/2018 tro
//
double getJoystickScale(double dAxisValue) {

  double dReturnVal = 0;
  int iJoystickCenter = 128;
  int iDeadbandRange = 30;
  int DEADBAND_LOW = iJoystickCenter - (iDeadbandRange / 2);
  int DEADBAND_HIGH= iJoystickCenter + (iDeadbandRange / 2);   

  if (dAxisValue >= DEADBAND_HIGH) {
    dReturnVal = (iJoystickCenter - 1 - dAxisValue) / (iJoystickCenter - 1);
  } else if (dAxisValue <= DEADBAND_LOW) {
    dReturnVal = (iJoystickCenter - dAxisValue) / iJoystickCenter;
  }
  return dReturnVal;
}

double getObstacleFactor(double dRSpeed, double dLSpeed) {
  double dReturnVal = 1.0;
  long duration, distance;
  if (dRSpeed < 0 && dLSpeed < 0) {
    digitalWrite(trigPin, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPin, HIGH);
  //  delayMicroseconds(1000); - Removed this line
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2) / 29.1;

    if (distance >= 200 || distance <= 0) {
      Serial.println("Out of range");
    }
    else {
      Serial.print(distance);
      Serial.println(" cm");
      if (distance < 20) {
        dReturnVal = distance / 20.0;
        if (dReturnVal < 0.2) {
          dReturnVal = 0.0;
        }
      }
    }
  }
  Serial.print("Obstacle Factor");
  Serial.println(dReturnVal);
  return dReturnVal;
}

//*******************************************************
// common joystick logging to serial (use serial monitor on PC)
// tro 6/7/2018
//
void joystick_log(double dTurnRate, double dSpeed) {
  Serial.print("JOYSTICK: ");
  Serial.print(" Turn Rate: ");
  Serial.print(dTurnRate);
  Serial.print(" Speed: ");
  Serial.println(dSpeed);
}

//*******************************************************
// common arcade drive
// y axis as forward/BACKWARD
// x axis as left/right 
// combination controls wheel speed based on axis positions
// speed and turn rate are doubles ranging from -1 to 1, with zero being stop
//
// tro 6/7/2018
//
void setArcadeDrive(double dSpeed, double dTurnRate) {
  
  double leftMotorOutput;
  double rightMotorOutput;
  double maxInput = copysign(max(fabs(dSpeed), fabs(dTurnRate)), dSpeed);

  if (dSpeed >= 0.0) {
    // First quadrant, else second quadrant
    if (dTurnRate >= 0.0) {
      leftMotorOutput = maxInput;
      rightMotorOutput = dSpeed - dTurnRate;
    } else {
      leftMotorOutput = dSpeed + dTurnRate;
      rightMotorOutput = maxInput;
    }
  } else {
    // Third quadrant, else fourth quadrant
    if (dTurnRate >= 0.0) {
      leftMotorOutput = dSpeed + dTurnRate;
      rightMotorOutput = maxInput;
    } else {
      leftMotorOutput = maxInput;
      rightMotorOutput = dSpeed - dTurnRate;
    }
  }

//  Serial.print("Motor Speed Left: ");
//  Serial.print(leftMotorOutput);
//  Serial.print(" Right: ");
//  Serial.print(rightMotorOutput); 
//  Serial.print(" maxInput");
//  Serial.println(maxInput); 

  double dObstacleFactor = getObstacleFactor(rightMotorOutput, leftMotorOutput);
  motorDriveRight.setSpeed(scaleSpeed(rightMotorOutput * dObstacleFactor));
  motorDriveLeft.setSpeed(scaleSpeed(leftMotorOutput * dObstacleFactor)); 
  motorDriveRight.run(getDirection(rightMotorOutput * dObstacleFactor));
  motorDriveLeft.run(getDirection(leftMotorOutput * dObstacleFactor)); 

}

// scale speed from double to an integer range valid for this motor controller
// absolute value is used as direction is a separate control
// tro 6/10/2018 
//
int scaleSpeed(double dSpeed) {
  double dMaxSpeed = 255.0;
  int iSpeed = (int) fabs(limit(dSpeed) * dMaxSpeed) + 0.5;
  return iSpeed;
}

// get motor direction based on provided speed
// tro 6/10/2018 
//
int getDirection(double dSpeed) {
  int iDirection = RELEASE;
  if (dSpeed < 0) {
    iDirection = FORWARD;
  } else if (dSpeed > 0) {
    iDirection = BACKWARD;
  }
  return iDirection;
}

double limit(double dAxisVal) {
  if (dAxisVal > 1.0) {
    dAxisVal = 1.0;
  } else if (dAxisVal < -1.0) {
    dAxisVal = -1.0;
  }
  return dAxisVal;
}

//**********Blynk Subroutines**********

//**********Save Throttle Setting**********
// This function saves the throttle from the Blynk slider
// the variable ranges from 0-255
// On the phone app set the Slider Ouput to Virtual V2
// 

BLYNK_WRITE(V2) {
  double dThrottleVal = param.asDouble(); // assigning incoming value from pin V1 to a variable
  dThrottleMult = (256 + dThrottleVal) / 511;
  Serial.print("THROTTLE: ");
  Serial.print(dThrottleVal);
  Serial.print(" Throttle Percent: ");
  Serial.println(dThrottleMult);
  
}
//**********END Set Throttle**********


//**********Translate the Joystick Position to motor speeds**********
//
//This function translates the joystick movement to a Rover direction.
//Blynk Joysick is centered at y=128, x=128 with a range of 0-255. 
// Deadband defines the Joystick "centered" limits - these may vary by device 
//
//  Joystick Movement along x, y Axis
// (Inside the * is the Threshold Area)
//            y=255--(y_position=255, x_position=128; y_direction=+1, x_direction=0)
//           * | *
//           * | *
//           * | *
//   ********* | *********
//x=0---------------------x=255--(y_position=128, x_position=255; y_direction=0, x_direction=0)
//   ********* | *********
//           * | *
//           * | * (Inside the * is the Threshold Area)
//           * | *
//            y=0--(y_position=0, x_position=128; y_direction=-1, x_direction=0)
BLYNK_WRITE(V0) {

  double dTurnRate = getJoystickScale(param[0].asDouble());  //Read the Blynk Joystick x Position 0-255
  double dSpeed = getJoystickScale(param[1].asDouble());  //Read the Blynk Joystick y Position 0-255

//Determine the direction of the Joystick Movement

//  joystick_log(dTurnRate, dSpeed);
  setArcadeDrive(dSpeed * dThrottleMult, dTurnRate * dThrottleMult);

}
