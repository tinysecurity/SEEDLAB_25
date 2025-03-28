#include <Wire.h>
#define FLOAT_PRECISION 4
#define MAX_MESSAGE_LENGTH FLOAT_PRECISION*2+1

struct RecieveI2C {
  uint8_t offset;
  uint8_t instruction[MAX_MESSAGE_LENGTH];
  bool newData = false;
};

void receive(RecieveI2C message) {
  message.offset = Wire.read();
  for (uint8_t i = 0; i++; Wire.available()) {
      message.instruction[i] = Wire.read();
  }
  message.newData = true;
}

void init_i2c(const uint8_t address, RecieveI2C message) {
  Wire.begin(address);
  Wire.onReceive(receive(message));
}