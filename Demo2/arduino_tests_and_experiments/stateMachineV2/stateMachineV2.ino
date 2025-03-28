#ifndef MACRO_DEFAULTS
#define MACRO_DEFAULTS
#define LEFT 0
#define RIGHT 1
#define WHEEL_RADIUS 0.0762 // NOTE: we need the wheel radius to calculate linear vel
#define WIDTH_OF_WHEELBASE 0.3683 // NOTE: we need wheelbase width to calculate angle
#endif



#include <Wire.h>
#include "encoder.hpp"
#include "control_rho_phi.hpp"

const uint8_t i2cAddress = 8;
volatile uint8_t offset;
volatile uint8_t msgLength = 0;
volatile uint8_t instruction[32] = {0};

// Initialize desired position, angle
float desiredDistance = 0;
float desiredAngle = 0;


// Initialize variables to be read from I2C
bool markerFound;
float markerAngle;
float markerDistance;
bool arrowFound = true;
bool arrowDirection;

// Set up state machine states
typedef enum {
  BINGUS, READ_INST, LOOK, WAIT, TURN, DRIVE, ADJUST, ARROW, REPORT, RESET
} bingusState_t;

static bingusState_t bingusState = BINGUS;

// State control variables
bool atMarker;
bool adjusted;
long int waitOffSet;
long int waitTime = 3000; // In milliseconds

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
    case READ_INST: // -------------  Read data from Pi ----------------
      // ------------ PUT CODE TO RECIEVE PI INSTRUCTIONS HERE --------
      // 
      
      // ------------------ Change State -----------------------------
      if(!markerFound) bingusState = LOOK;
      else if(markerFound && !atMarker) bingusState = TURN;
      else if(atMarker && !adjusted) bingusState = ADJUST;
      else if(atMarker && adjusted && arrowFound) bingusState = ARROW;
      else bingusState = READ_INST;
      break;

    case LOOK: // ---------------- Turn and look for the aruco marker -----------
      desiredDistance = 0;
      desiredAngle = -15;
      controlRhoPhi(desiredDistance, desiredAngle);
      
      // ------------------ Change State -----------------------------
      if(inTolerance()){
        bingusState = WAIT;
        waitOffSet = millis();
      }
      break;

    case WAIT: // ----------------- Wait for a set amount of time --------------
      time = millis();
      
      // ------------------ Change State -----------------------------
      if(time - waitOffSet >= waitTime) {
        bingusState = REPORT;
      }
      else bingusState = WAIT;
      break;

    case TURN: // ---------- First step of moving toward the marker, turn the robot to the correct angle -----------
      desiredDistance = 0;
      desiredAngle = markerAngle;
      controlRhoPhi(desiredDistance, desiredAngle);
      
      // ------------------ Change State -----------------------------
      if(inTolerance()) bingusState = DRIVE;
      break;

    case DRIVE: // --------- Second Step of moving toward the marker, drive until within 1.5 feet of the marker --------
      desiredDistance = markerDistance - 18;
      desiredAngle = markerAngle;
      controlRhoPhi(desiredDistance, desiredAngle);
      
      // ------------------ Change State -----------------------------
      if(inTolerance()){
        atMarker = true;
        waitOffSet = millis();
        bingusState = WAIT;
      }
      break;

    case ADJUST: // ------- Correct the angle to look striaght at the marker -----------
      desiredDistance = 0;
      desiredAngle = markerAngle;
      controlRhoPhi(desiredDistance, desiredAngle);
      
      // ------------------ Change State -----------------------------
      if(inTolerance()){
        adjusted = true;
        waitOffSet = millis();
        bingusState = WAIT;
      }
      break;

    case ARROW: // ---------- Once in tolerance, turn the given arrow direction ------------
      if(arrowDirection) desiredAngle = 90;
      else desiredAngle = -90;
      desiredDistance = 0;
      controlRhoPhi(desiredDistance, desiredAngle);

      // ------------------ Change State -----------------------------
      if(inTolerance()){
        atMarker = false;
        adjusted = false;
        waitOffSet = millis();
        bingusState = WAIT;
      }
      break; 

    case REPORT: // Report data back to the Pi
      // ------ Add code to report data back to the pi-------

      // ------------------ Change State -----------------------------
      bingusState = RESET;
      break;

    case RESET: // Reset the control system
      resetControl();
      desiredDistance = 0;
      desiredAngle = 0;
      
      // ------------------ Change State -----------------------------
      bingusState = READ_INST;
      break;
      
    case BINGUS: // ------------ Default State -----------------------
    default:

      // ------------------ Change State -----------------------------
      bingusState = READ_INST;
      break;
  }

  // ----------- Code that runs every loop -------------
  controlRhoPhi(desiredDistance, desiredAngle);
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
