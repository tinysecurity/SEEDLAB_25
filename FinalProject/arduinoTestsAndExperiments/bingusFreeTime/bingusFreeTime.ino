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

// Bingus free time variables
float bingusAngle= 0;
float bingusDistance = 0;

// Initialize variables to be read from I2C
bool markerFound;
float markerAngle;
float markerDistance;
bool arrowFound = false;
bool arrowDirection = LEFT;
uint8_t color;

// Set up state machine states
typedef enum {
  BINGUS, WAIT, TURN, DRIVE, REPORT, RESET
} bingusState_t;

static bingusState_t bingusState = WAIT;

// State control variables
bool atMarker;
bool adjusted;
long int waitOffSet;
long int waitTime = 1500; // In milliseconds

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

  // Random Seed
  randomSeed(millis());

}



void loop() {
  
  switch(bingusState) {
    case BINGUS: // -------------  Read data from Pi ----------------
      // ------------ PUT CODE TO RECIEVE PI INSTRUCTIONS HERE --------
      randomSeed(millis());
      bingusAngle = random(0, 720);
      bingusDistance = random(0,48);
      // ------------------ Change State -----------------------------
      bingusState = TURN;
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
      desiredAngle = bingusAngle;
      controlRhoPhi(desiredDistance, desiredAngle);
      
      // ------------------ Change State -----------------------------
      if(inTolerance()) bingusState = DRIVE;
      break;

    case DRIVE: // --------- Second Step of moving toward the marker, drive until within 1.5 feet of the marker --------
      desiredDistance = bingusDistance;
      desiredAngle = bingusAngle;
      controlRhoPhi(desiredDistance, desiredAngle);
      
      // ------------------ Change State -----------------------------
      if(inTolerance()){
        atMarker = true;
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
      bingusState = BINGUS;
      break;
      
    default:

      // ------------------ Change State -----------------------------
      bingusState = BINGUS;
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
