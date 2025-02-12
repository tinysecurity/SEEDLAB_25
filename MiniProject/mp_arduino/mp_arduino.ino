#include "encoder.hpp"

void setup() {
  encoderSetup();
  Serial.begin(9600);
}

void loop() {
  Serial.print(countsToRadians(readEncoder(LEFT)));
  delay(100);
}
