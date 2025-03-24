#ifndef PI_RHO_PHI_HPP
#define PI_RHO_PHI_HPP

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
void PiRhoPhi(float, float);
bool inTolerance();

// Initialize Motor Pin assignments
const uint8_t motorDirection[2] = {7,8};
const uint8_t motor[2] = {9,10};
const uint8_t motorEnable = 4;

// Initialize physical values
float voltage[2] = {0,0};
float actualPos[2] = {0,0};
float prevPos[2] = {0, 0};
float actualSpeed[2] = {0,0};



// Initialize time tracking variables
unsigned long desiredTs = 1; // old value 5, maybe match with delay used?
float prevTime = 0;
float currTime = 0;
float time = 0;
int timeOffSet = 0;

// Initialize control values RHO
float vBar = 0;
float rho = 0;
float rhoSet = 0;
float alpha = .95;
float rhoDesired = 0;
float rhoDot = 0;
float rhoDotError = 0;
float rhoDotDesired = 0;
float KpRhoDot = 28;
float KpRho = 16.36;
float KiRho = 36.85;
float rhoErrorP = 0;
float rhoErrorI = 0;


// Initialize control values PHI 
float vDelta = 0;
float phi = 0;
float phiSet = 0;
float phiDesired = 0;
float phiDot = 0;
float phiDotError = 0;
float phiDotDesired = 0;
float KpPhiDot = 20;
float KpPhi = 27.52;
float KiPhi = 139.17;
float phiErrorP = 0;
float phiErrorI = 0;


// Initialize PWM Variables
float batteryVoltage = 8.5;
unsigned int PWM = 0;

float rad2AngVel(float rads1, float time1, float rads2, float time2){
  return (rads2 - rads1) / (time2 - time1);
}

float rad2LinVel(float rads1, float time1, float rads2, float time2){
  return rad2AngVel(rads1,time1,rads2,time2)*WHEEL_RADIUS;
}


// --------------- PI Controller for Rho and Phi ----------
// Inputs are desired distance (in feet) and desired angle (in degrees)
//  Time tracking and delay are handled in this function
void PiRhoPhi(float rhoSetInches, float phiSetDegrees){
  
  // --------------- Convert to m and rads --------------------
  phiSet = phiSetDegrees * 1.045 * (PI/180); // 1.0625 (for half speed) 1.045 (for full speed) to account for undershoot
  rhoSet = rhoSetInches * 0.0254 * 1.017;

  // ---------------- Current time --------------------------
  currTime = (float)(millis() - timeOffSet)/1000;
  time = millis();
  
  // ----------------- PI Controller ----------------------------
  // Get current speed and position for each wheel
  for(int motorChoice = 0; motorChoice < 2; motorChoice++){
    // Read position and angular velocity
    actualPos[motorChoice] = countsToRadians(readEncoder(motorChoice));
    actualSpeed[motorChoice] = rad2AngVel(prevPos[motorChoice], prevTime, actualPos[motorChoice], currTime);
  }

  // Calculate rhoDot and phiDot
  rhoDot = WHEEL_RADIUS * ((actualSpeed[RIGHT] + (-actualSpeed[LEFT])) / 2);
  phiDot = WHEEL_RADIUS * ((actualSpeed[RIGHT] - (-actualSpeed[LEFT])) / WIDTH_OF_WHEELBASE);

  // Calculate rho and Phi
  rho = WHEEL_RADIUS * ((actualPos[RIGHT] + (-actualPos[LEFT])) / 2);
  phi = WHEEL_RADIUS * ((actualPos[RIGHT] - (-actualPos[LEFT])) / WIDTH_OF_WHEELBASE);


  // ------------ Rho ----------------------
  
  // Discrete time filter to limit acceleration
  rhoDesired = alpha*rhoDesired + (1-alpha)*rhoSet;

  // proportional error rho
  rhoErrorP = rhoDesired - rho;
  
  // Integral Error
  rhoErrorI = rhoErrorI + rhoErrorP*((float)desiredTs /1000);

  // find desired speed rho
  rhoDotDesired = KpRho * rhoErrorP + KiRho * rhoErrorI;

  // ------------ Phi ----------------------
  // No folter for phi
  phiDesired = phiSet;

  // proportional error phi
  phiErrorP = phiDesired - phi;

  // Integral Error
  phiErrorI = phiErrorI + phiErrorP*((float)desiredTs /1000);

  // find desired speed phi
  phiDotDesired = KpPhi * phiErrorP + KiPhi * phiErrorI;

  //----------------- Inner Loop Control ---------------------------
  rhoDotError = rhoDotDesired - rhoDot;
  phiDotError = phiDotDesired - phiDot;

  // ---------------- Physical Motor Control ----------------------
  // Calculate Voltage for each given Vbar and Vdelta
  vBar = KpRhoDot * rhoDotError;
  vDelta = KpPhiDot * phiDotError;
  
  // Wind up control rho (keeps the motors at a maximum voltage)
  if(abs(vBar) >= batteryVoltage){
    // keep the speed at or bellow the max voltage while keeping the sign
    vBar = batteryVoltage * (vBar/abs(vBar));
    // Undo the integration calculation
    rhoErrorI = rhoErrorI - rhoErrorP*((float)desiredTs /1000);
  }
  
  // Wind up control phi(keeps the motors at a maximum speed)
  // NOTE: Try testing with batteryVoltage instead of bV/2. Might be faster rotation. Originally changed to bV/2 when attempting arc movement
  if(abs(vDelta) >= batteryVoltage){
    // keep the speed at or bellow the max voltage while keeping the sign
    vDelta = batteryVoltage * (vDelta/abs(vDelta));
    // Undo the integration calculation
    phiErrorI = phiErrorI - phiErrorP*((float)desiredTs /1000);
  }

  voltage[RIGHT] = (vBar + vDelta) / 2;
  voltage[LEFT] = -((vBar - vDelta) / 2);
  
  
  
  // Push voltage to motor with correct sign
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
  }
  // ------------------ Update Values --------------------------------
  // update postition
  prevPos[RIGHT] = actualPos[RIGHT];
  prevPos[LEFT] = actualPos[LEFT];

  // Update prev time
  prevTime = currTime;

  delay(desiredTs);
}

bool inTolerance(){
  float deltaPhi = 1;
  float deltaRho = .1;
  if(((phi < phiSet + deltaPhi) && (phi > phiSet - deltaPhi)) && ((rho < rhoSet + deltaRho) && (rho < rhoSet - deltaRho))){
    return true;
  }
  else{
    return false;
  }
}

#endif