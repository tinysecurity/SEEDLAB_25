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

// Initialize desired position, angle, if the marker is present, and arrow derection
float desiredDistance = 0;
float desiredAngle = 0;
bool markerPresent = false;
int arrowDirection = 0;

// Initialize variables to be read from I2C
bool markerFound;
float markerAngle;
float markerDistance;
bool arrowFound;
bool arrowDirection;

// Set up state machine states
typedef enum {
  BINGUS, READ_INST, LOOK, WAIT, TURN, DRIVE, ADJUST, ARROW, REPORT, RESET
} bingusState_t;

static bingusState_t bingusState = BINGUS;

// State control variables
bool atMarker;
int waitOffSet;
int waitTime; // In milliseconds

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
  
  switch(bingusState) {
    case READ_INST:
      // ------------ PUT CODE TO RECIEVE PI INSTRUCTIONS HERE --------
      // 

      // ------------------ Change State -----------------------------
      if(!markerFound) bingusState = LOOK;
      else if(markerFound && !atMarker) bingusState = TURN;
      else if(atMarker && arrowFound) bingusState = ARROW;
      else bingusState = READ_INST;
      break;
    case LOOK: // turn and look for the aruco marker
      desiredDistance = 0;
      desiredAngle = 10;
      bingusState = WAIT;
      waitOffSet = millis();
      break;
    case WAIT:
      if(time - waitOffSet >= 25) bingusState = REPORT;
      else bingusState = WAIT;
    case TURN:
      desiredDistance = 0;
      desiredAngle = markerAngle;
      if(inTolerance()) bingusState = DRIVE;
      break;
    case DRIVE:
      desiredDistance = markerDistance;
      desiredAngle = markerAngle;
      if(inTolerance()) bingusState = REPORT;
      break;
    case ARROW: // once in tolerance, turn the given arrow direction
      if(arrowDirection) desiredAngle = 90;
      else desired angle = -90;
      bingusState = REPORT;
      break; 
    case REPORT:
      // Report position to PI
      bingusState = RESET;
      break;
    case RESET:
      rho = 0;
      phi = 0;
      desiredDistance = 0;
      desiredAngle = 0;
      bingusState = READ_INST;
      break;
    case BINGUS:
    default:
      bingusState = READ_INST;
      break;
  }

  // ----------- Code that runs every loop -------------
  PiRhoPhi(desiredDistance, desiredAngle);
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
