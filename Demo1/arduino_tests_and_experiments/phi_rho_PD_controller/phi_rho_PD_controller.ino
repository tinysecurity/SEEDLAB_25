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

// Initialize control values RHO
float vBar = 0;
float rho = 0;
float rhoPrev = 0;
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
float phiPrev = 0;
float phiDesired = 0;
float phiDot = 0;
float phiDotError = 0;
float phiDotDesired = 0;
float KpPhiDot = 20;
float KpPhi = 27.52;
float KiPhi = 139.17;
float phiErrorP = 0;
float phiErrorI = 0;


// Step input parameters
float pathTime = 10;
float pathRadius = 1;

// Initialize time tracking variables
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
  
  //Desired Values
  rhoDesired = 3;
  phiDesired = PI;
}

void loop() {
  // ---------------- Current time --------------------------
  currTime = (float)(millis()-timeOffSet)/1000;
  time = millis();
  
  // ---------------- Step Loop ------------------------------
  // Step loop
  // if((currTime <= 2 || currTime >= 12)){
  //   // Step loop down time
  //   rhoDesired = 0;
  //   phiDesired = 0;
  // }else if (currTime > 2 && currTime < 12){
  //   // Step loop up time
  //   rhoDesired = 1;
  //   phiDesired = 0;
  // }else if (currTime >= 12){
  //   //Go back to staart of step loop
  //   currTime = 0;
  //   timeOffSet = millis();
  //   Serial.println("TIME,vBar,vDelta,rhoDot,phiDot");
  // }else{
  //   // how did you get here
  //   currTime = 0;
  //   timeOffSet = millis();
  // }
  
  // ----------------- PD Controller ----------------------------
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
  // proportional error rho
  rhoErrorP = rhoDesired - rho;
  
  // Integral Error
  rhoErrorI = rhoErrorI + rhoErrorP*((float)desiredTs /1000);

  // find desired speed rho
  rhoDotDesired = KpRho * rhoErrorP + KiRho * rhoErrorI;

  // ------------ Phi ----------------------
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
  if(abs(vBar) >= 2*batteryVoltage){
    // keep the speed at or bellow the max voltage while keeping the sign
    vBar = 2*batteryVoltage * (vBar/abs(vBar));
    // Undo the integration calculation
    rhoErrorI = rhoErrorI - rhoErrorP*((float)desiredTs /1000);
  }
  
  // Wind up control phi(keeps the motors at a maximum speed)
  if(abs(vDelta) >= 2*batteryVoltage){
    // keep the speed at or bellow the max voltage while keeping the sign
    vDelta = 2*batteryVoltage * (vDelta/abs(vDelta));
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
  // Update rho and phi
  rhoPrev = rho;
  phiPrev = phi;

  // update postition
  prevPos[RIGHT] = actualPos[RIGHT];
  prevPos[LEFT] = actualPos[LEFT];

  // Update prev time
  prevTime = currTime;

  //----------------- Print relevant info ---------------------------
  if((time - lastUpdateTime) >= updateInterval){
    Serial.print(currTime);
    Serial.print(",");
    Serial.print(vBar);
    Serial.print(",");
    Serial.print(vDelta);
    Serial.print(",");
    Serial.print(rhoDotDesired);
    Serial.print(",");
    Serial.print(phiDotDesired);
    Serial.print(",");
    Serial.print(rhoDot);
    Serial.print(",");
    Serial.print(phiDot);
    Serial.print("\r\n");
    lastUpdateTime = time;
  }
  delay(1);
}


