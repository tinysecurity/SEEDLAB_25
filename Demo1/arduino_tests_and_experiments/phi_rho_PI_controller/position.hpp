#ifndef POSITION_HPP
#define POSITION_HPP

#include <Arduino.h>
#include "encoder.hpp" // able to read from encoders directly
// but only if you use the second overload and not the first

#ifndef MACRO_DEFAULTS
#define MACRO_DEFAULTS
#define LEFT 0
#define RIGHT 1
#define WHEEL_RADIUS 0.0762 // NOTE: we need the wheel radius to calculate linear vel 
#define WIDTH_OF_WHEELBASE 0.3683 // NOTE: we need wheelbase width to calculate angle
#endif

struct positionData_t {
  float x;
  float y;
  float phi;
};

positionData_t updatePosition(positionData_t, float, float);
positionData_t updatePosition(positionData_t);

positionData_t position = {0, 0 ,0};
float previousDistance[] = {0, 0};

positionData_t updatePosition(positionData_t oldPosition, float deltaPosLeft, float deltaPosRight) {
  positionData_t newPosition;
  // using position-based equations from tutorial
  newPosition.x = oldPosition.x + cos(oldPosition.phi)*(deltaPosLeft+deltaPosRight)/2.0;
  newPosition.y = oldPosition.y + sin(oldPosition.phi)*(deltaPosLeft+deltaPosRight)/2.0;
  newPosition.phi = oldPosition.phi + (deltaPosLeft-deltaPosRight)/WIDTH_OF_WHEELBASE;
  return newPosition; // return struct to contain all 3 variables
}

positionData_t updatePosition(positionData_t oldPosition) {
  positionData_t newPosition;
  float deltaPosLeft = countsToDistance(readEncoder(LEFT)) - previousDistance[LEFT];
  float deltaPosRight = countsToDistance(readEncoder(RIGHT)) - previousDistance[RIGHT];
  // using position-based equations from tutorial
  newPosition.x = oldPosition.x + cos(oldPosition.phi)*(deltaPosLeft+deltaPosRight)/2.0;
  newPosition.y = oldPosition.y + sin(oldPosition.phi)*(deltaPosLeft+deltaPosRight)/2.0;
  newPosition.phi = oldPosition.phi + (deltaPosLeft-deltaPosRight)/WIDTH_OF_WHEELBASE;
  previousDistance[LEFT] = countsToDistance(readEncoder(LEFT));
  previousDistance[RIGHT] = countsToDistance(readEncoder(RIGHT));
  return newPosition; // return struct to contain all 3 variables
}

#endif