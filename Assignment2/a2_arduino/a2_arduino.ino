//#define A2_FADING
#define A2_LOCATION
//#define A2_STEPRESPONSE
//#define A2_STEPINPUT
// Liam Abell, Jake Stanley, Nolan Pratt | Control & Localization Subsystem
// EENG350
// This program spins the motor at different speeds using a PWM signal


/*
Parts of this code were modified from or inspired by
Encoder.h by Paul Stoffregen. Code used under MIT license:
https://opensource.org/license/mit
Repository: 
https://github.com/PaulStoffregen/Encoder/blob/master/Encoder.h
*/

#define LEFT 0
#define RIGHT 1
#define WHEEL_RADIUS 0.0762 // NOTE: we need the wheel radius to calculate linear vel
#define WIDTH_OF_WHEELBASE 0.3683 // NOTE: we need wheelbase width to calculate angle

const uint8_t encoderA[2] = {2,3};
const uint8_t motorEnable = 4;
const uint8_t encoderB[2] = {5,6};
const uint8_t motorDirection[2] = {7,8};
const uint8_t motor[2] = {9,10};

// NOTE: DO NOT read encoderPosition directly!
// Use readEncoder() instead
volatile uint8_t encoderState[2] = {0,0};
volatile long encoderPosition[2] = {0,0};

// Initialize time tracking variables
unsigned long time = 0;
unsigned long timeOffset = 0;
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 50;

// Initialize previous and current time for velocity calculation
float prevTime = 0;
float currTime = 0;

// Initialize previous and current radians for each motor for velocity calculation
float prevRads[2] = {0, 0};
float currRads[2] = {0, 0};

// Initialize Feedback parameters
float Kp[2] = {2.9, 2.9};
float error = 0;
float voltage = 0;
float desiredSpeed = 0;
float actualSpeed = 0;

// Initialize PWM Variables
float batteryVoltage = 7.8;
unsigned int PWM = 0;

// Initialize position parameters and datatypes
struct positionData_t {
  float x;
  float y;
  float phi;
};
struct positionData_t position = {0, 0 ,0};
float previousDistance[] = {0, 0};

void setup() {
  // ---------- SERIAL ----------
  Serial.begin(115200); 

  // ---------- ENCODERS ----------
  pinMode(encoderA[LEFT], INPUT);
  pinMode(encoderB[LEFT], INPUT);
  pinMode(encoderA[RIGHT], INPUT);
  pinMode(encoderB[RIGHT], INPUT);

  // grab starting state of encoders
  if (digitalRead(encoderA[LEFT])) { encoderState[LEFT] |= 4; }
  if (digitalRead(encoderB[LEFT])) { encoderState[LEFT] |= 8; }
  if (digitalRead(encoderA[RIGHT])) { encoderState[RIGHT] |= 4; }
  if (digitalRead(encoderB[RIGHT])) { encoderState[RIGHT] |= 8; }

  attachInterrupt(digitalPinToInterrupt(encoderA[LEFT]), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA[RIGHT]), updateRightEncoder, CHANGE);

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
  #ifdef A2_FADING // start fading code

  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(11, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(11, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  #endif // end fading code
  /* -------------------------------------------------------- */
  #ifdef A2_LOCATION // start location code

  time = millis();
  currTime = (float)(time)/1000;
  
  // Print relevant info
  if((time - lastUpdateTime) >= updateInterval){
    Serial.print(currTime);
    Serial.print(",");
    Serial.print(position.x);
    Serial.print(",");
    Serial.print(position.y);
    Serial.print(",");
    Serial.print(position.phi);
    Serial.print("\r\n");
  }

  // update location data 
  float posLeft = -countsToDistance(readEncoder(LEFT));
  float posRight = countsToDistance(readEncoder(RIGHT));
  float deltaL = posLeft-previousDistance[LEFT];
  float deltaR = posRight-previousDistance[RIGHT];

  // calculate position and update global vars
  struct positionData_t pastPosition =  position;
  position = updatePosition(pastPosition, deltaL, deltaR);

  previousDistance[LEFT] = posLeft;
  previousDistance[RIGHT] = posRight;
  delay(100);

  #endif // end location code
  /* -------------------------------------------------------- */
  #ifdef A2_STEPRESPONSE // start step response code

  unsigned int i = 0;
  int motorChoice = RIGHT;

  // starting @ 0V
  Serial.print("TIME,VOLTAGE,VELOCITY\r\n"); //  csv format
  timeOffset = millis();
  analogWrite(motor[motorChoice], voltage*255/7.8);
  for (i = 0; i < 20; i++) {
    currRads[motorChoice] = countsToRadians(readEncoder(motorChoice));
    time = millis();
    currTime = (float)(time - timeOffset)/1000;
    Serial.print(currTime);
    Serial.print(",");
    Serial.print(voltage);
    Serial.print(",");
    Serial.print(rad2AngVel(prevRads[motorChoice], prevTime, currRads[motorChoice], currTime));
    Serial.print("\r\n");
    prevTime = currTime;
    prevRads[motorChoice] = currRads[motorChoice];
    delay(50);
  }
  // run at @ 3V
  voltage = 3;
  analogWrite(motor[motorChoice], voltage*255/7.8);
  for (i = 0; i < 40; i++) {
    currRads[motorChoice] = countsToRadians(readEncoder(motorChoice));
    time = millis();
    currTime = (float)(time - timeOffset)/1000;
    Serial.print(currTime);
    Serial.print(",");
    Serial.print(voltage);
    Serial.print(",");
    Serial.print(rad2AngVel(prevRads[motorChoice], prevTime, currRads[motorChoice], currTime));
    Serial.print("\r\n");
    prevTime = currTime;
    prevRads[motorChoice] = currRads[motorChoice];
    delay(50);
  }
  // run @ 0V again
  voltage = 0;
  analogWrite(motor[motorChoice], voltage*255/7.8);
  for (i = 0; i < 20; i++) {
    currRads[motorChoice] = countsToRadians(readEncoder(motorChoice));
    time = millis();
    currTime = (float)(time - timeOffset)/1000;
    Serial.print(currTime);
    Serial.print(",");
    Serial.print(voltage);
    Serial.print(",");
    Serial.print(rad2AngVel(prevRads[motorChoice], prevTime, currRads[motorChoice], currTime));
    Serial.print("\r\n");
    prevTime = currTime;
    prevRads[motorChoice] = currRads[motorChoice];
    delay(50);
  }
  // Serial.print("Finished");
  // Serial.print("\r\n");

  #endif // end step response code
  /* -------------------------------------------------------- */
  #ifdef A2_STEPINPUT // start step input code

  int motorChoice = RIGHT;

  // update global vars
  currRads[motorChoice] = countsToRadians(readEncoder(motorChoice));
  time = millis();
  currTime = (float)(time - timeOffset)/1000;

  // Find actual speed
  actualSpeed = rad2AngVel(prevRads[motorChoice], prevTime, currRads[motorChoice], currTime);
  
  // Print relevant info
  if((time - lastUpdateTime) >= updateInterval){
    Serial.print(currTime);
    Serial.print(",");
    Serial.print(desiredSpeed);
    Serial.print(",");
    Serial.print(actualSpeed);
    Serial.print("\r\n");
    lastUpdateTime = currTime;
  }
  // Feedback Control Code
  error = desiredSpeed - actualSpeed;
  voltage = Kp[motorChoice]*error;

  // go backwards if negative voltage
  if(voltage >= 0){
    digitalWrite(motorDirection[motorChoice], HIGH);
  }
  else {
    digitalWrite(motorDirection[motorChoice], LOW);
  }

  // calculate voltage figures
  PWM = 255*abs(voltage)/batteryVoltage;
  analogWrite(motor[motorChoice], min(PWM,255));

  // update global vars
  prevTime = currTime;
  prevRads[motorChoice] = currRads[motorChoice];

  // Step input cycle
  if(currTime >= 2 && currTime <= 6){
    desiredSpeed = 3.14;
  }
  else {
    desiredSpeed = 0;
  }
  if (currTime >= 7) {
    timeOffset = millis();

  }  
  delay(5);


  #endif // end step input code
}

struct positionData_t updatePosition(struct positionData_t oldPosition, float deltaPosLeft, float deltaPosRight) {
  struct positionData_t newPosition;
  // using position-based equations from tutorial
  newPosition.x = oldPosition.x + cos(oldPosition.phi)*(deltaPosLeft+deltaPosRight)/2.0;
  newPosition.y = oldPosition.y + sin(oldPosition.phi)*(deltaPosLeft+deltaPosRight)/2.0;
  newPosition.phi = oldPosition.phi + (deltaPosLeft-deltaPosRight)/WIDTH_OF_WHEELBASE;
  return newPosition; // return struct to contain all 3 variables
}

void updateEncoder(bool leftOrRight) {
  // Current B | Current A | Old B | Old A
  // mask out unneeded bits
  uint8_t state = encoderState[leftOrRight] & 0b00001100;
  // shift by 2 to account for old states
  state >>= 2;
  // bit2 and bit3 become new encoder positions
  if (digitalRead(encoderA[leftOrRight])) { state |= 0b00000100; }
  if (digitalRead(encoderB[leftOrRight])) { state |= 0b00001000; }

  // switch states modified from Encoder.h, see above
  switch (state) {
    case 1:  case 7:
    case 8:  case 14:
      encoderPosition[leftOrRight]++;
      break;

    case 2:  case 4:
    case 11: case 13:
      encoderPosition[leftOrRight]--;
      break;

    case 3:  case 12:
      encoderPosition[leftOrRight] += 2;
      break;

    case 6:  case 9:
      encoderPosition[leftOrRight] -= 2;
      break;

    default:
      break;
  }
  encoderState[leftOrRight] = state;
}

long readEncoder(bool leftOrRight) {
  // disable interrupts to preserve read precision
  noInterrupts();
  // adjustment on read
  updateEncoder(leftOrRight);
  interrupts();
  return encoderPosition[leftOrRight];
}

float countsToRadians(double counts) {
  return TWO_PI*(float)counts/3200;
}

float countsToDistance(double counts) {
  return WHEEL_RADIUS*countsToRadians(counts);
}

float rad2AngVel(float rads1, float time1, float rads2, float time2){
  return (rads2 - rads1) / (time2 - time1);
}

float rad2LinVel(float rads1, float time1, float rads2, float time2){
  return rad2AngVel(rads1,time1,rads2,time2)*WHEEL_RADIUS;
}

// not possible to put parameters in interrupts,
// this is an easy workaround
void updateLeftEncoder() { updateEncoder(LEFT); }
void updateRightEncoder() { updateEncoder(RIGHT); }