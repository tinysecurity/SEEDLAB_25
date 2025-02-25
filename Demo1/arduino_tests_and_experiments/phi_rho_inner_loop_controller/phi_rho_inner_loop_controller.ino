#ifndef MACRO_DEFAULTS
#define MACRO_DEFAULTS
#define LEFT 0
#define RIGHT 1
#define WHEEL_RADIUS 0.0762 // NOTE: we need the wheel radius to calculate linear vel
#define WIDTH_OF_WHEELBASE 0.3683 // NOTE: we need wheelbase width to calculate angle
#endif


#include "encoder.hpp"
#include "position.hpp"
#include "pid_movement.hpp"

const uint8_t i2cAddress = 8;
volatile uint8_t offset;
volatile uint8_t msgLength = 0;
volatile uint8_t instruction[32] = {0};

// Initialize desired position and angle
float desiredDistance = 0;
float desiredAngleDegrees = 0;
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 50;

// Initialize control values
float vBar = 0;
float vDelta = 0;
float rhoDot = 0;
float phiDot = 0;
float rhoDotDesired = 0;
float phiDotDesired = 0;
float kRhoDot = 0;
float kPhiDot = 0;

// Step input parameters
float pathTime = 10;
float pathRadius = 1;

// Time tracking variables
long int timeOffSet = 0;
long int time = 0;

void setup() {
  Serial.begin(115200);

  // ---------- ENCODER ----------
  encoderSetup();
  
  // ---------- MOTOR ----------
  pinMode(motorEnable, OUTPUT);
  pinMode(motorDirection[LEFT], OUTPUT);
  pinMode(motorDirection[RIGHT], OUTPUT);
  pinMode(motor[LEFT], OUTPUT);
  pinMode(motor[RIGHT], OUTPUT);

  digitalWrite(motorEnable, HIGH);
  digitalWrite(motorDirection[LEFT], HIGH);
  digitalWrite(motorDirection[RIGHT], HIGH);

  // ---------- CSV ----------
  Serial.println("TIME,vBar,vDelta,rhoDot,phiDot");
  
}

void loop() {
  currTime = (float)(millis()-timeOffSet)/1000;
  time = millis();
  // Step loop
  if((currTime <= 2 || currTime >= 6) && currTime < 7){
    // Step loop down time
    rhoDotDesired = (PI * pathRadius) / pathTime;
    phiDotDesired = PI / pathTime;
  }else if (currTime > 2 && currTime < 12){
    // Step loop up time
    rhoDotDesired = (PI * pathRadius) / pathTime;
    phiDotDesired = PI / pathTime; 
  }else if (currTime >= 7){
    //Go back to staart of step loop
    currTime = 0;
    timeOffSet = millis();
    Serial.println("TIME,vBar,vDelta,rhoDot,phiDot");
  }else{
    // how did you get here
    currTime = 0;
    timeOffSet = millis();
  }
  vBar = kRhoDot * rhoDotDesired;
  vDelta = kPhiDot * phiDotDesired;
  voltage[RIGHT] = (vBar + vDelta) / 2;
  voltage[LEFT] = -(vBar - vDelta) / 2;
  
  for(int motorChoice = 0; motorChoice < 2; motorChoice++){

    // Correct voltage sign
    if(voltage[motorChoice] >= 0){
      digitalWrite(motorDirection[motorChoice], HIGH);
    }
    else {
      digitalWrite(motorDirection[motorChoice], LOW);
    }

    // Write voltage to motor
    PWM = 255*abs(voltage[motorChoice])/batteryVoltage;
    analogWrite(motor[motorChoice], min(PWM,255));

    // Read position and angular velocity
    actualPos[motorChoice] = countsToRadians(readEncoder(motorChoice));
    actualSpeed[motorChoice] = rad2AngVel(prevPos[motorChoice], prevTime, actualPos[motorChoice], currTime);

    // update postition
    prevPos[motorChoice] = actualPos[motorChoice];
  }

  // Calculate rhoDot and phiDot
  rhoDot = WHEEL_RADIUS * ((actualSpeed[RIGHT] + (-actualSpeed[LEFT])) / 2);
  phiDot = WHEEL_RADIUS * ((actualSpeed[RIGHT] - (-actualSpeed[LEFT])) / WIDTH_OF_WHEELBASE);

  
  // Print relevant info
  if((time - lastUpdateTime) >= updateInterval){
    Serial.print(currTime);
    Serial.print(",");
    Serial.print(vBar);
    Serial.print(",");
    Serial.print(vDelta);
    Serial.print(",");
    Serial.print(rhoDot);
    Serial.print(",");
    Serial.print(phiDot);
    Serial.print("\r\n");
    lastUpdateTime = time;
  }

  // Update prev time
  prevTime = currTime;

  delay(5);
}


