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

// Initialize current values
float v_bar = 0;
float v_delta = 0;
float rho_dot = 0;
float phi_dot = 0;
long int timeOffSet = 0;
long int time = 0;
bool stepTest = 0;

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
  Serial.println("TIME,V_BAR,V_DELTA,RHO_DOT,PHI_DOT");
  
}

void loop() {
  currTime = (float)(millis()-timeOffSet)/1000;
  time = millis();
  // Step loop
  if(currTime > 2 && currTime < 6){
    // Step loop up time
    if(stepTest){
      v_bar = 12;
      v_delta = 0;
    }
    else{
      v_bar = 0;
      v_delta = 12;
    }
  }else if (currTime >= 7){
    //Go back to start of step loop
    currTime = 0;
    timeOffSet = millis();
    Serial.println("TIME,V_BAR,V_DELTA,RHO_DOT,PHI_DOT");
    stepTest = !stepTest;
  }else{
    // Step input down time
    v_bar = 0;
    v_delta = 0;
  }

  voltage[RIGHT] = (v_bar + v_delta) / 2;
  voltage[LEFT] = -(v_bar - v_delta) / 2;
  
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

  // Calculate rho_dot and phi_dot
  rho_dot = WHEEL_RADIUS * ((actualSpeed[RIGHT] + (-actualSpeed[LEFT])) / 2);
  phi_dot = WHEEL_RADIUS * ((actualSpeed[RIGHT] - (-actualSpeed[LEFT])) / WIDTH_OF_WHEELBASE);

  
  // Print relevant info
  if((time - lastUpdateTime) >= updateInterval){
    Serial.print(currTime);
    Serial.print(",");
    Serial.print(v_bar);
    Serial.print(",");
    Serial.print(v_delta);
    Serial.print(",");
    Serial.print(rho_dot);
    Serial.print(",");
    Serial.print(phi_dot);
    Serial.print("\r\n");
    lastUpdateTime = time;
  }

  // Update prev time
  prevTime = currTime;

  delay(5);
}


