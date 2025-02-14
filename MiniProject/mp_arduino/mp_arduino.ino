#include "encoder.hpp"
#include "position.hpp"
#include "pid_movement.hpp"

void setup() {
  encoderSetup();
  Serial.begin(9600);
}

void loop() {
  position = updatePosition(position);
  Serial.print(position.phi);
  delay(100);
}
