#ifndef MACRO_DEFAULTS
#define MACRO_DEFAULTS
#define LEFT 0
#define RIGHT 1
#define WHEEL_RADIUS 0.0762 // NOTE: we need the wheel radius to calculate linear vel 
#define WIDTH_OF_WHEELBASE 0.3683 // NOTE: we need wheelbase width to calculate angle
#define FLOAT_PRECISION 4
#define MAX_MESSAGE_LENGTH FLOAT_PRECISION*2+1
#define ADDRESS 8
#endif



#include <Wire.h>
#include "encoder.hpp"
#include "control_rho_phi.hpp"

// Initialize I2C struct
struct RecieveI2C {
  uint8_t offset;
  uint8_t instruction[MAX_MESSAGE_LENGTH];
  bool newData = false;
  float distance;
  float angle;
  bool marker;
  uint8_t color;
};
union Unsflinteger {
  unsigned long i;
  float f;
};

// Initialize I2C
RecieveI2C myi2c;

// Initialize desired position, angle
float desiredDistance = 0;
float desiredAngle = 0;


// Initialize variables to be read from I2C
bool markerFound;
float markerAngle;
float markerDistance;
bool arrowFound = false;
bool arrowDirection = LEFT;
int arrowCount = 0;
int wiggleCount = 0;
int wiggleDirection = LEFT;
uint8_t color;

// Set up state machine states
typedef enum {
  BINGUS, READ_INST, LOOK, WAIT, TURN, DRIVE, ADJUST, ARROW, WIGGLE, REPORT, RESET
} bingusState_t;

static bingusState_t bingusState = WAIT;

// State control variables
bool atMarker;
bool adjusted;
long int waitOffSet;
long int waitTime = 900; // In milliseconds

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
  Wire.begin(ADDRESS);
  Wire.onReceive(receive);
}



void loop() {
  
  switch(bingusState) {
    case READ_INST: // -------------  Read data from Pi ----------------
      // ------------ PUT CODE TO RECIEVE PI INSTRUCTIONS HERE --------
      if(myi2c.newData){
        markerDistance = myi2c.distance;
        markerAngle = myi2c.angle;
        markerFound = myi2c.marker;
        color = myi2c.color;
        switch(color){
          case 0:
          case 1:
            arrowFound = false;
            break;
          case 2:
            arrowFound = true;
            arrowDirection = LEFT;
            break;
          case 3:
            arrowFound = true;
            arrowDirection = RIGHT;
            break;
          default:
            arrowFound = false;
            break;
        }
        myi2c.newData = false;
      }

      // ------------------ Change State -----------------------------
      if(!markerFound && !arrowFound && arrowCount < 1 && !atMarker) bingusState = LOOK;
      else if(!markerFound && !arrowFound && arrowCount >= 1 && !atMarker) bingusState = WIGGLE;
      else if(markerFound && !atMarker) bingusState = TURN;
      else if(atMarker && !adjusted) bingusState = ADJUST;
      else if(atMarker && adjusted && arrowFound) bingusState = ARROW;
      else bingusState = READ_INST;
      break;

    case LOOK: // ---------------- Turn and look for the aruco marker -----------
      if(arrowDirection) desiredAngle = -45;
      else desiredAngle = 45;
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
        bingusState = REPORT; // CHANGE TO WAIT IF ERRORS
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
        bingusState = REPORT;
      }
      break;

    case ARROW: // ---------- Once in tolerance, turn the given arrow direction ------------
      if(arrowDirection) desiredAngle = -90;
      else desiredAngle = 90;
      desiredDistance = 0;
      controlRhoPhi(desiredDistance, desiredAngle);

      // ------------------ Change State -----------------------------
      if(inTolerance()){
        atMarker = false;
        adjusted = false;
        waitOffSet = millis();
        bingusState = WAIT;
        arrowCount = arrowCount + 1;
      }
      break; 

    case WIGGLE:
      switch(wiggleCount){
        case 0:
          wiggleDirection = arrowDirection;
          break;
        case 1:
        case 2:
          wiggleDirection = !arrowDirection;
          break;
        case 3:
          wiggleDirection = arrowDirection;
          break;
      }
      if(wiggleDirection) desiredAngle = -45;
      else desiredAngle = 45;
      controlRhoPhi(desiredDistance, desiredAngle);
      
      // ------------------ Change State -----------------------------
      if(inTolerance()){
        bingusState = WAIT;
        wiggleCount = wiggleCount + 1;
        if(wiggleCount > 3) wiggleCount = 0;
        waitOffSet = millis();
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

// Receive function for I2C communication
void receive(int howMany) {
  myi2c.offset = Wire.read(); // offset
  Wire.read(); // address+1, n/a as 1:1 transmitter:sink
  for (uint8_t i = 0; Wire.available(); i++) {
      myi2c.instruction[i] = Wire.read();
      //Serial.print(myi2c.instruction[i]);
  }
  myi2c.newData = true;
  // NOTE: hardcoding to 4 bytes for convenience
  Unsflinteger distanceConv;
  Unsflinteger angleConv;
  distanceConv.i = ((unsigned long)myi2c.instruction[1]<<24) + ((unsigned long)myi2c.instruction[2]<<16) + ((unsigned long)myi2c.instruction[3]<<8) + ((unsigned long)myi2c.instruction[4]);
  angleConv.i = ((unsigned long)myi2c.instruction[5]<<24) + ((unsigned long)myi2c.instruction[6]<<16) + ((unsigned long)myi2c.instruction[7]<<8) + ((unsigned long)myi2c.instruction[8]);
  //Serial.println(distanceConv.i);
  myi2c.distance = distanceConv.f;
  myi2c.angle = angleConv.f;
  //Serial.println(myi2c.distance);
  //Serial.println((unsigned long)((myi2c.instruction[1]<<24) + (myi2c.instruction[2]<<16) + (myi2c.instruction[3]<<8) + (myi2c.instruction[4])));
  myi2c.color = myi2c.instruction[0];
  if (myi2c.instruction[0] == 0) {
    myi2c.marker = false;
  } else {
    myi2c.marker = true;
  }
}
