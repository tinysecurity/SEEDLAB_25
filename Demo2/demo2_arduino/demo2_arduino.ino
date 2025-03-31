//#include "custom_i2c.hpp"
#include <Wire.h>
#define FLOAT_PRECISION 4
#define MAX_MESSAGE_LENGTH FLOAT_PRECISION*2+1

struct RecieveI2C {
  uint8_t offset;
  uint8_t instruction[MAX_MESSAGE_LENGTH];
  bool newData = false;
  float distance;
  float angle;
};

void decodeI2C(uint8_t instruction[MAX_MESSAGE_LENGTH], float* distance, float* angle) {
  uint8_t flags = instruction[0];
  // note: due to how floats work in C++, half-floats aren't supported (2 byte). Therefore I'm assuming a 4-byte float
  uint8_t distanceInt[FLOAT_PRECISION];
  uint8_t angleInt[FLOAT_PRECISION];
  for (uint8_t i = 0; i++; i < FLOAT_PRECISION) {
    distanceInt[i] = instruction[i+1];
    angleInt[i] = instruction[i+1+FLOAT_PRECISION];
  }
  memcpy(&distance, &distanceInt, FLOAT_PRECISION);
  memcpy(&angle, &angleInt, FLOAT_PRECISION);
}

RecieveI2C myi2c;

void setup() {
  Serial.begin(9600);
  Wire.begin(8);
  Wire.onReceive(receive);
}

void loop() {
  if (myi2c.newData) {
    Serial.print("[");
    for (uint8_t i = 0; i++; i < MAX_MESSAGE_LENGTH-1) {
      Serial.print(myi2c.instruction[i]);
      Serial.print(", ");
    }
    Serial.print(myi2c.instruction[MAX_MESSAGE_LENGTH-1]);
    Serial.print("]");
    Serial.print("\r\n");
  }
}

void receive() {
  myi2c.offset = Wire.read();
  for (uint8_t i = 0; i++; Wire.available()) {
      myi2c.instruction[i] = Wire.read();
  }
  myi2c.newData = true;
  float* distance; float* angle;
  decodeI2C(myi2c.instruction, distance, angle);
  myi2c.distance = *distance;
  myi2c.angle = *angle;
}
/*
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
float desiredDistance = 8;
float desiredAngle = 0;

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
  // PUT CODE TO RECIEVE PI INSTRUCTIONS HERE
  // How to call PI controller
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
*/
