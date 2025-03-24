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
  switch(bingusState) {
    case READ_INST:
      // PUT CODE TO RECIEVE PI INSTRUCTIONS HERE
      desiredDistance = desiredDistance + 3;
      desiredAngle = desiredAngle + 90;
      bingusState = TURN;
      break;
    case LOOK: // turn and look for the aruco marker
        lookForMarker();
      break; 
    case TURN:
      // Turn first
      PiRhoPhi(0, desiredAngle);
      while(!inTolerance()){
        PiRhoPhi(0, desiredAngle);
      }
      Serial.println("Done Turning");
      bingusState = DRIVE;
      break;
    case DRIVE:
      // Drive second
      PiRhoPhi(desiredDistance, desiredAngle);
      while(!inTolerance()){
        PiRhoPhi(desiredDistance, desiredAngle);
      }
      Serial.println("Done Moving");
      bingusState = REPORT;
      break;
    case ARROW: // once in tolerance, turn the given arrow direction
        if(inTolerance() == true){
          if(arrowDirection == 0){ 
            PiRhoPhi(0, 90);  // turn right
          }
          else PiRhoPhi(0, -90);  // turn left
        }
      break; 
    case REPORT:
      // Report position to PI
      bingusState = RESET;
      break;
    case RESET:
      // rho = 0;
      // phi = 0;
      // desiredDistance = 0;
      // desiredAngle = 0;
      bingusState = READ_INST;
      break;
    case NORMAL:
      // How did you get here
      bingusState = READ_INST;
      break;
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

void lookForMarker(){
  while(markerPresent == false){
    // rotate in short bursts until the marker is now in view
    PiRhoPhi(0, 10);
    delay(100);

  }
}
