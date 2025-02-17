#include <Wire.h>
#include "encoder.hpp"
#include "position.hpp"
#include "pid_movement.hpp"

enum motorState_t: uint8_t {
  NorthEast = 0,
  NorthWest = 1,
  SouthWest = 3,
  SouthEast = 2
};

const uint8_t i2cAddress = 8;
motorState_t motorState = NorthEast;
volatile uint8_t offset;
volatile uint8_t msgLength = 0;
volatile uint8_t instruction[32] = {0};

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
  if (msgLength > 0) {
    motorState = instruction[0];
    msgLength = 0;
    Serial.print("We are in the ");
    switch (motorState) {
      case NorthEast: Serial.print("NE (0)"); break;
      case NorthWest: Serial.print("NW (1)"); break;
      case SouthWest: Serial.print("SW (3)"); break;
      case SouthEast: Serial.print("SE (2)"); break;
    }
    Serial.print(" quadrant :)\r\n");
  }

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

  currTime = (float)milis()/1000;
  PIDposControl(LEFT);
  PIDposControl(RIGHT);
  prevTime = currTime;
  delay(5);
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