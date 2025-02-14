/*
 Parts of this code were modified from or inspired b*y
 Encoder.h by Paul Stoffregen. Code used under MIT license:
 https://opensource.org/license/mit
 Repository:
 https://github.com/PaulStoffregen/Encoder/blob/master/Encoder.h
 */
#ifnef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

#ifnef MACRO_DEFAULTS
#define MACRO_DEFAULTS
#define LEFT 0
#define RIGHT 1
#define WHEEL_RADIUS 0.0762 // NOTE: we need the wheel radius to calculate linear vel
#define WIDTH_OF_WHEELBASE 0.3683 // NOTE: we need wheelbase width to calculate angle
#endif

void encoderSetup();
void updateEncoder();
long readEncoder(bool);
float countsToRadians(long);
float countsToDistance(long);
void updateLeftEncoder();
void updateRightEncoder();

const uint8_t encoderA[2] = {2,3};
const uint8_t encoderB[2] = {5,6};

// NOTE: DO NOT read encoderPosition directly!
// Use readEncoder() instead
volatile uint8_t encoderState[2] = {0,0};
volatile long encoderPosition[2] = {0,0};

void encoderSetup() {
	pinMode(encoderA[LEFT], INPUT);
	pinMode(encoderB[LEFT], INPUT);
	pinMode(encoderA[RIGHT], INPUT);
	pinMode(encoderB[RIGHT], INPUT);

	if (digitalRead(encoderA[LEFT])) { encoderState[LEFT] |= 4; }
	if (digitalRead(encoderB[LEFT])) { encoderState[LEFT] |= 8; }
	if (digitalRead(encoderA[RIGHT])) { encoderState[RIGHT] |= 4; }
	if (digitalRead(encoderB[RIGHT])) { encoderState[RIGHT] |= 8; }

	attachInterrupt(digitalPinToInterrupt(encoderA[LEFT]), updateLeftEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(encoderA[RIGHT]), updateRightEncoder, CHANGE);
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

float countsToRadians(long counts) {
	return TWO_PI*(float)counts/3200;
}

float countsToDistance(long counts) {
	return WHEEL_RADIUS*countsToRadians(counts);
}

// not possible to put parameters in interrupts,
// this is an easy workaround
void updateLeftEncoder() { updateEncoder(LEFT); }
void updateRightEncoder() { updateEncoder(RIGHT); }

#endif