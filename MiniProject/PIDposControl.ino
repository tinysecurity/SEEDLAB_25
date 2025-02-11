
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
unsigned long desiredTs = 5;

// Initialize previous and current time for velocity calculation
float prevTime = 0;
float currTime = 0;

// Initialize Feedback parameters
float KpSpeed[2] = {2.9, 2.9};
float KpPos[2] = {24.659, 25.590};
float KiPos[2] = {106.106, 115.247};
float posError[2] = {0,0};
float speedError[2] = {0,0};
float voltage[2] = {0,0};
float desiredSpeed[2] = {0,0};
float actualSpeed[2] = {0,0};
float desiredPos[2] = {0,0};
float actualPos[2] = {0,0};
float prevPos[2] = {0, 0};
float integralError[2] = {0,0};

// Initialize PWM Variables
float batteryVoltage = 7.8;
unsigned int PWM = 0;

// Iterators
int i;
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
  // put your main code here, to run repeatedly:
  
  // int motorChoice = RIGHT;

  
  // update global vars
  time = millis();
  currTime = (float)(time - timeOffset)/1000;

  
  
  // PI controller for position (for loop to control each motor)
  for(i = 0; i < 2; i++) {
    // Find actual values
    actualPos[i] = countsToRadians(readEncoder(i));
    actualSpeed[i] = rad2AngVel(prevPos[i], prevTime, actualPos[i], currTime);

    // proportional error
    posError[i] = desiredPos[i] - actualPos[i];
    // integral error
    integralError[i] = integralError[i] + posError[i]*((float)desiredTs /1000);
    // find desired speed
    desiredSpeed[i] = KpPos[i] * posError[i] + KiPos[i] * integralError[i];
    // speed controller
    speedError[i] = desiredSpeed[i] - actualSpeed[i];
    // voltage output
    voltage[i] = KpSpeed[i]*speedError[i];

    // go backwards if negative voltage
    if(voltage[i] >= 0){
      digitalWrite(motorDirection[i], HIGH);
    }
    else {
      digitalWrite(motorDirection[i], LOW);
    }

    // calculate voltage PWM output
    PWM = 255*abs(voltage[i])/batteryVoltage;
    analogWrite(motor[i], min(PWM,255));

    // update postition
    prevPos[i] = actualPos[i];

    // Step input cycle
    if(currTime >= 2 && currTime <= 6){
      desiredPos[i] = 3.14;
    }
    else {
      desiredPos[i] = 0;
    }
    if (currTime >= 7) {
      timeOffset = millis();

    }  
    
  }
  prevTime = currTime;
  
  // Print relevant info
  if((time - lastUpdateTime) >= updateInterval){
    Serial.print(currTime);
    Serial.print(",");
    Serial.print(desiredPos[LEFT]);
    Serial.print(",");
    Serial.print(actualPos[LEFT]);
    Serial.print(",");
    Serial.print(desiredPos[RIGHT]);
    Serial.print(",");
    Serial.print(actualPos[RIGHT]);
    Serial.print("\r\n");
    lastUpdateTime = currTime;
  }

  delay(5);
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
