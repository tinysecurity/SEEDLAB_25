#include "encoder.hpp"
#include "position.hpp"

positionData_t position = {0, 0 ,0};

void setup() {
  encoderSetup();
  Serial.begin(9600);
}

void loop() {
  position = updatePosition(position);
  Serial.print(position.phi);
  delay(100);
}
