#include "encoder.hpp"
#include "position.hpp"
#include "pid_movement.hpp"

enum motorState_t: uint8_t {
  NorthEast = 0b00000000,
  NorthWest = 0b00000001,
  SouthWest = 0b00000011,
  SouthEast = 0b00000010
};

motorState_t motorState = NorthEast;

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
}

void loop() {
  motorState = motorState; // TODO: Integrate with pi

  PIDposControl(LEFT);
  PIDposControl(RIGHT);

  switch (motorState) {
    case NorthEast:
      desiredPos[LEFT] = 0;
      desiredPos[RIGHT] = 0;
      break;
    case NorthWest:
      desiredPos[LEFT] = 0;
      desiredPos[RIGHT] = PI;
      break;
    case SouthWest:
      desiredPos[LEFT] = PI;
      desiredPos[RIGHT] = PI;
      break;
    case SouthEast:
      desiredPos[LEFT] = PI;
      desiredPos[RIGHT] = 0;
      break;
    default:
      // YOU MESSED UP
      desiredPos[LEFT] = PI*1000;
      desiredPos[RIGHT] = PI*1000;
      break;
  }
}
