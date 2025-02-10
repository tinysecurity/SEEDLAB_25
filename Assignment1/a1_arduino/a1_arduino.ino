//    code state macros
//    switching cables between states:
//    both switches and both states of the encoder use the interrupt pins
//    remember to switch them when switching states
//#define EXERCISE_1
#define EXERCISE_2
//    if EXERCISE_2 uncommented, uncomment one of the following lines with it
//    otherwise leave them commented
//#define EXERCISE_2_NO_INTERRUPT
#define EXERCISE_2_YES_INTERRUPT

// macros for exercise 1
#define LED1  4
#define LED2  5
#define LED3  6
#define LED4  7
#define SW1   2
#define SW2   3
#define STATE_DELAY_DELTA  50

// macros for exercise 2
#define ENC_A  2
#define ENC_B  3


//************************************************
#ifdef EXERCISE_1 // START EXERCISE 1

// not able to make a macro array
const unsigned int LEDLookup[] = {LED1, LED2, LED3, LED4};
int state = 0;
volatile unsigned int stateDelay = 1000;

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);

  attachInterrupt(digitalPinToInterrupt(SW1), button1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SW2), button2ISR, RISING);
}

void loop() {
  // state%4 returns 0,1,2,3,0,1,2,3
  // state<4 returns 1,1,1,1,0,0,0,0
  // > this works because 1=HIGH in arduino
  digitalWrite(LEDLookup[state%4], state<4);

  // state increments 0->7 each loop() cycle
  if (state < 7) {
    state++;
  } else {
    state = 0;
  }

  // our delay is variable each cycle
  delay(stateDelay);
}

void button1ISR() {
  // prevents int underflow
  if (stateDelay <= STATE_DELAY_DELTA) return;
  // set the new stateDelay to be "delta" smaller
  stateDelay = stateDelay - STATE_DELAY_DELTA;
}

void button2ISR() {
  // prevents int overflow in the rare case that you press
  // the button thoudands of times in a row
  if (stateDelay >= (65535-STATE_DELAY_DELTA)) return;
  // set the new stateDelay to be "delta" larger
  stateDelay = stateDelay + STATE_DELAY_DELTA;
}

#endif // END EXERCISE 1
//************************************************


//************************************************
#ifdef EXERCISE_2 // START EXERCISE 2

// uint8_t for easy bit manipulation
volatile uint8_t allStates = 0b00000011;
volatile long globalPosition = 0;

void setup() {
  Serial.begin(9600);

  // not INPUT_PULLUP because of the encoder i'm using
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  // interrupts only compile when you want them
  #ifdef EXERCISE_2_YES_INTERRUPT
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderChangeISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderChangeISR, CHANGE);
  #endif
}

void updateStates() {
  // a lot of explanation incoming.
  // this function essentially returns a truth table entry,
  // which i search through in the updatePosition() function.
  // the truth table goes OLD_A | OLD_B | NEW_A | NEW_B
  // therefore, the below line makes new values into old values
  allStates = allStates << 2;
  // masking out trash data
  allStates = allStates & 0b00001100;
  // adding NEW_A into the table index
  allStates = allStates | ((uint8_t)digitalRead(ENC_A) << 1);
  // adding NEW_B
  allStates = allStates | ((uint8_t)digitalRead(ENC_B));
}

void updatePosition() {
  // allStates is interpreted as an integer, which is 
  // essentially the index of the truth table entry we want
  // if you're unfamiliar, i'm using case fallthroughs (no break)
  switch (allStates) {
    case 0: case 3: case 5:
    case 6: case 9: case 10:
    case 12: case 15:
      break;
    case 1: case 7:
    case 8: case 14:
      globalPosition--;
      break;
    case 2: case 4:
    case 11: case 13:
      globalPosition++;
      break;
	}
}

void loop() {
  // non-interrupt logic only compiles when you need it
  #ifdef EXERCISE_2_NO_INTERRUPT
  // finds the current/previous state of the encoder and
  // then updates the position accordingly.
  updateStates();
  updatePosition();
  #endif
  Serial.println(globalPosition); 
}

// interrupt functions exist when needed
#ifdef EXERCISE_2_YES_INTERRUPT
void encoderChangeISR() {
  updateStates();
  updatePosition();
}
#endif

#endif // END EXERCISE 2
//************************************************