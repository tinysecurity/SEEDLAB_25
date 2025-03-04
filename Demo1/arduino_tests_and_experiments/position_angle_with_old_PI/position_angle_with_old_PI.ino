#ifndef MACRO_DEFAULTS
#define MACRO_DEFAULTS
#define LEFT 0
#define RIGHT 1
#define WHEEL_RADIUS 0.0762 // NOTE: we need the wheel radius to calculate linear vel
#define WIDTH_OF_WHEELBASE 0.3683 // NOTE: we need wheelbase width to calculate angle
#endif



#include <Wire.h>
#include "encoder.hpp"
#include "position.hpp"
#include "pid_movement.hpp"

const uint8_t i2cAddress = 8;
volatile uint8_t offset;
volatile uint8_t msgLength = 0;
volatile uint8_t instruction[32] = {0};

// Initialize desired position and angle
float desiredDistance = 0;
float desiredAngle = 0;

// Initialize current values
int currDistance = 0; // in meters
int currAngleDegrees = 0;

// Do once
bool forward1 = 0;
bool turn1 = 0;

void setup() {
  Serial.begin(9600);

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

  // ---------- I2C ----------
  Wire.begin(i2cAddress);
  Wire.onReceive(receive);
}

void loop() {
  currTime = (float)millis()/1000;
  currDistance = position.x;
  if(currTime >= 5){
    if(!forward1){
      desiredDistance = 1;
      desiredAngle = 180;
      findRads();
      forward1 = 1;
    }
  }
  PIDposControl(RIGHT);
  PIDposControl(LEFT);
  updatePosition(position);
  prevTime = currTime;
  delay(5);
}

void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();

  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
    //reply = (instruction[0]) + 100; //the reply that is sent to the Pi is the length of the original message
  }
}



// Takes an input of the total distance(meters) to move and which angle(degrees) to move at.
// Increments the desired position to the new desired position
void findRads(){
  float rho = desiredDistance / WHEEL_RADIUS;
  float phiRads = desiredAngle*PI/180;
  float radsL =  -(rho - phiRads) / 2;
  float radsR = (rho + phiRads) / 2;
  desiredPos[LEFT] = desiredPos[LEFT] + radsL;
  desiredPos[RIGHT] = desiredPos[RIGHT] + radsR;
}
