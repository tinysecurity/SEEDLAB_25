#ifndef PID_MOVEMENT_HPP
#define PID_MOVEMENT_HPP

#include <Arduino.h>
#include "encoder.hpp" // able to read from encoders directly
// but only if you use the second overload and not the first

#ifndef MACRO_DEFAULTS
#define MACRO_DEFAULTS
#define LEFT 0
#define RIGHT 1
#define WHEEL_RADIUS 0.0762 // NOTE: we need the wheel radius to calculate linear vel 
#define WIDTH_OF_WHEELBASE 0.3683 // NOTE: we need wheelbase width to calculate angle
#endif

float rad2AngVel(float, float, float, float);
float rad2LinVel(float, float, float, float);
void PIDposControl(bool);

const uint8_t motorDirection[2] = {7,8};
const uint8_t motor[2] = {9,10};
const uint8_t motorEnable = 4;

// Initialize time tracking variables
unsigned long desiredTs = 5;

// Initialize previous and current time for velocity calculation
float prevTime = 0;
float currTime = 0;

// Initialize Feedback parameters
float KpSpeed[2] = {2.9, 2.9};
float KpPos[2] = {24.659, 25.590};
float KiPos[2] = {106.106, 115.247};
float posError[2] = {0,0};
float speedError[2] = {0,0};
float voltage[2] = {0,0};
float desiredSpeed[2] = {0,0};
float actualSpeed[2] = {0,0};
float desiredPos[2] = {0,0};
float actualPos[2] = {0,0};
float prevPos[2] = {0, 0};
float integralError[2] = {0,0};

// Initialize PWM Variables
float batteryVoltage = 8.5;
float maxSpeed = TWO_PI;
unsigned int PWM = 0;

float rad2AngVel(float rads1, float time1, float rads2, float time2){
  return (rads2 - rads1) / (time2 - time1);
}

float rad2LinVel(float rads1, float time1, float rads2, float time2){
  return rad2AngVel(rads1,time1,rads2,time2)*WHEEL_RADIUS;
}

void PIDposControl(bool motorChoice){
  // Find actual values
  actualPos[motorChoice] = countsToRadians(readEncoder(motorChoice));
  actualSpeed[motorChoice] = rad2AngVel(prevPos[motorChoice], prevTime, actualPos[motorChoice], currTime);

  // proportional error
  posError[motorChoice] = desiredPos[motorChoice] - actualPos[motorChoice];
  // integral error
  integralError[motorChoice] = integralError[motorChoice] + posError[motorChoice]*((float)desiredTs /1000);
  // find desired speed
  desiredSpeed[motorChoice] = KpPos[motorChoice] * posError[motorChoice] + KiPos[motorChoice] * integralError[motorChoice];
  
  // Wind up control (keeps the motors at a maximum speed)
  if(abs(desiredSpeed[motorChoice]) >= maxSpeed){
    // keep the voltage at or bellow the battery voltage while keeping the sign
    desiredSpeed[motorChoice] = maxSpeed * (desiredSpeed[motorChoice]/abs(desiredSpeed[motorChoice]));
    // Undo the integration calculation
    integralError[motorChoice] = integralError[motorChoice] - posError[motorChoice]*((float)desiredTs /1000);
  }

  // speed controller
  speedError[motorChoice] = desiredSpeed[motorChoice] - actualSpeed[motorChoice];
  
  // voltage output
  voltage[motorChoice] = KpSpeed[motorChoice]*speedError[motorChoice];

  // go backwards if negative voltage
  if(voltage[motorChoice] >= 0){
    digitalWrite(motorDirection[motorChoice], HIGH);
  }
  else {
    digitalWrite(motorDirection[motorChoice], LOW);
  }

  // calculate voltage PWM output
  PWM = 255*abs(voltage[motorChoice])/batteryVoltage;
  analogWrite(motor[motorChoice], min(PWM,255));

  // update postition
  prevPos[motorChoice] = actualPos[motorChoice];
}

#endif