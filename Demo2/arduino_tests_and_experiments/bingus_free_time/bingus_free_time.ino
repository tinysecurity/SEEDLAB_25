#ifndef MACRO_DEFAULTS
#define MACRO_DEFAULTS
#define LEFT 0
#define RIGHT 1
#define WHEEL_RADIUS 0.0762 // NOTE: we need the wheel radius to calculate linear vel
#define WIDTH_OF_WHEELBASE 0.3683 // NOTE: we need wheelbase width to calculate angle
#endif



#include <Wire.h>
#include "encoder.hpp"
#include "pi_rho_phi.hpp"

const uint8_t i2cAddress = 8;
volatile uint8_t offset;
volatile uint8_t msgLength = 0;
volatile uint8_t instruction[32] = {0};

// Initialize desired position and angle
float desiredDistance = 0;
float desiredAngle = 0;

// Set up state machine states
typedef enum {
  NORMAL, READ_INST, TURN, DRIVE, REPORT, RESET
} bingusState_t;

static bingusState_t bingusState = NORMAL;

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
  while(currTime <= 2){
    PiRhoPhi(desiredDistance, desiredAngle);
  }
  Serial.println("Done Waiting");
  desiredAngle = desiredAngle + 45;
  while(currTime <= 4){
    PiRhoPhi(desiredDistance, desiredAngle);
  }
  Serial.println("Done Turning");
  desiredDistance = desiredDistance + 24;
  while(currTime <= 6){
    PiRhoPhi(desiredDistance, desiredAngle);
  }
  Serial.println("Done Moving");
  while(currTime > 6){
    currTime = 0;
    timeOffSet = millis();
  }
}

bool inTolerance(){
  float deltaPhi = 10;
  float deltaRho = .1;
  if(((phi < phiSet + deltaPhi) && (phi > phiSet - deltaPhi)) && ((rho < rhoSet + deltaRho) && (rho < rhoSet - deltaRho))){
    return true;
  }
  else{
    return false;
  }
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
