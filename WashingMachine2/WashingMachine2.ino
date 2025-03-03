// Arduino washing machine controller
//
// By Voara Andriamihaminjaka <voara@vk.com>

#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <OneButton.h>
#include <TimerFreeTone.h>
#include "pitches.h"

// GPIO assignment
// Inputs
#define ZCD_OUT 2     // ZeroCrossDetector input for TRIAC phase control : Interrupt pin
#define TACHO_PIN A0  // Not used, just for my reference on where it is connected

// Encoder pins
#define ENC_A 0
#define ENC_B 1
#define ENC_SW 3

// Outputs - AND USE AS STEPS AS WELL
#define MOTOR_DIRECTION 4   // LOW: CW | HIGH: CCW
#define SPRAY_WASH_MAIN 5   // Active High
#define SPRAY_WASH_PRE 6    // Active High
#define DRAIN_PUMP 7        // Active High
#define MOTOR_SPEED_CTRL 8  // Triac phase control
#define HEATER_CTRL 9       // Triac ON/OFF
#define BUZZER 10           // PWM Tone control

// Utility variables
#define CLOCKWISE LOW
#define COUNTER_CLOCKWISE HIGH
#define SPIN_SPEED 1500
#define AGITATE_SPEED 250

#define MOTOR_CCW_LOW 100
#define MOTOR_CW_LOW 101
#define MOTOR_CCW_HIGH 102
#define MOTOR_CW_HIGH 103
#define SOAK 200
#define END 0

#define DELAY_BETWEEN_SEQUENCE 3000

#define GATE_PULSE_LENGTH 5  // 20 microseconds
#define TACHOPULSES 56       // number of pulses per revolution - use drum rotation, not motor

// Copied from Saulius' work
// PID utilites
#define SAMPLE_RATE 1             // Variable that determines how fast our PID loop
#define MIN_OUTPUT_LIMIT 1700     // limit of PID output
#define MAX_OUTPUT_LIMIT 1840     // limit of PID output 8.64 ms
#define MIN_TRIGGERING_TIME 1700  // the shortest delay before triac fires ~ 1.28 ms
#define MAX_TRIGGERING_TIME 1840  // translates to 10 ms
#define RISE_TIME 100             // RPM rise time delay in microseconds (risetime x RPM)

// Uncomment this if you want to test the system manually
#define MANUAL_JOG

/* Washing machine internally have 5 main controls
    I. Adding water
      1. Main wash spray 
      2. Pre-wash spray
      (+ Pressure sensor input)

    II. Heating water
      3. Heater + NTC feedback

    III. Draining water
      3. Drain pump

    IV. Rotating the drum
      4. Motor CW/CCW/Speed control
    
    V. User interface
      5. Selecting program
      6. Display & Audio feedback

    All of which, works in a sequence, apart from the user interface which should display time
*/

// Motor control flag
bool startFlag = false;
bool runFlag = false;

uint16_t RPM;                      // real rpm variable
uint32_t lastRPMpulse;             // count the time between the actual tacho pulse and the last one
const uint8_t rpmCorrection = 86;  // here for some reason it is necessary so that the real rpm corresponds to the measured ones

unsigned long lastpiddelay = 0;

double Setpoint, Input, Output;        // define PID variables
double sKp = 0.1, sKi = 0.2, sKd = 0;  // PID tuning parameters for starting motor
double rKp = 0.25, rKi = 1, rKd = 0;   // PID tuning parameters for runnig motor

int firingTime = 1800;  // this should be the same as maxoutputlimit - 490 experimental 0
int count;
int initialPulse = 0;
int desiredRPM;
int lastRPM;
int minRPM;
int minRPMPulse;
int maxRPM;
int maxRPMPulse;
int tempcounter = 80;

bool direction = false;
bool sprayWash = false;
bool drainPump = false;
bool refreshDisplay = false;
bool manualRPM = true;
bool sequenceStarted = false;
long sequenceStartTime = 0;
uint16_t duration;
uint8_t sequence;
uint8_t programIndex = 0;


int encoderPosition = 0;

// notes in the melody:
int melody[] = {
  NOTE_D5, 0, NOTE_FS5, 0, NOTE_A5, 0, NOTE_D6
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  8, 8, 8, 8, 8, 8, 8
};

uint16_t program_01[] = { DRAIN_PUMP, 30, SPRAY_WASH_MAIN, 300, MOTOR_CCW_LOW, 8, DRAIN_PUMP, 10, SPRAY_WASH_PRE, 5, MOTOR_CW_LOW, 8, DRAIN_PUMP, 15, END, 0 };
uint16_t program_02[] = { DRAIN_PUMP, 10,
                          MOTOR_CCW_LOW, 10, SOAK, 30, MOTOR_CW_HIGH, 10, SOAK, 30,
                          MOTOR_CCW_HIGH, 10, SOAK, 30, MOTOR_CW_LOW, 10, SOAK, 30,
                          END, 0 };
uint16_t normalWash[] = { SPRAY_WASH_MAIN, 240,                                     // Water intake + Soap
                          MOTOR_CCW_LOW, 15, SOAK, 30, MOTOR_CW_LOW, 15, SOAK, 30,  // Agitate
                          MOTOR_CCW_LOW, 20, SOAK, 30, MOTOR_CW_LOW, 20, SOAK, 30,
                          SPRAY_WASH_MAIN, 60,
                          MOTOR_CCW_LOW, 15, SOAK, 30, MOTOR_CW_LOW, 15, SOAK, 30,
                          MOTOR_CCW_LOW, 20, SOAK, 30, MOTOR_CW_LOW, 20, SOAK, 30,
                          MOTOR_CCW_LOW, 15, SOAK, 30, MOTOR_CW_LOW, 15, SOAK, 30,
                          MOTOR_CCW_LOW, 20, SOAK, 30, MOTOR_CW_LOW, 20, SOAK, 30,
                          DRAIN_PUMP, 30, SPRAY_WASH_MAIN, 80, SOAK, 90,  // Rinse
                          MOTOR_CCW_LOW, 5, SOAK, 30, MOTOR_CW_LOW, 5, SOAK, 30,
                          MOTOR_CCW_LOW, 10, SOAK, 30, MOTOR_CW_LOW, 10, SOAK, 30,
                          MOTOR_CCW_LOW, 5, SOAK, 30, MOTOR_CW_LOW, 5, SOAK, 30,
                          MOTOR_CCW_LOW, 10, SOAK, 30, MOTOR_CW_LOW, 10, SOAK, 30,
                          DRAIN_PUMP, 30, SPRAY_WASH_MAIN, 100, SOAK, 150,
                          MOTOR_CCW_LOW, 5, SOAK, 30, MOTOR_CW_LOW, 5, SOAK, 30,
                          MOTOR_CCW_LOW, 5, SOAK, 30, MOTOR_CW_LOW, 5, SOAK, 30,
                          MOTOR_CCW_LOW, 5, SOAK, 30, MOTOR_CW_LOW, 5, SOAK, 30,
                          MOTOR_CCW_LOW, 5, SOAK, 30, MOTOR_CW_LOW, 5, SOAK, 30,
                          DRAIN_PUMP, 30, SPRAY_WASH_MAIN, 60, SOAK, 60,
                          MOTOR_CCW_LOW, 10, SOAK, 10, MOTOR_CW_LOW, 10, SOAK, 10,
                          MOTOR_CCW_LOW, 15, SOAK, 10, MOTOR_CW_LOW, 15, SOAK, 30,
                          DRAIN_PUMP, 60,
                          MOTOR_CW_HIGH, 30, SOAK, 10, DRAIN_PUMP, 10,  // SPIN
                          MOTOR_CCW_HIGH, 30, MOTOR_CW_HIGH, 30,
                          MOTOR_CCW_HIGH, 30, MOTOR_CW_HIGH, 30,
                          DRAIN_PUMP, 40,
                          END, 0 };

LiquidCrystal_I2C lcd(0x24, 16, 2);                            // set the LCD address to 0x27 for a 16 chars and 2 line display
PID myPID(&Input, &Output, &Setpoint, sKp, sKi, sKd, DIRECT);  // define PID variables and parameters
Encoder encoder(ENC_A, ENC_B);
OneButton button(ENC_SW, true);  // Handle the central button

long lastDisplayUpdate;

void setup() {
  delay(1000);
  hardwareSetup();
  pidSetup();
  ringtone();
  // syncTimer();

#ifdef MANUAL_JOG
  encoder.write(firingTime << 2);
  button.attachClick(handleClick);
  button.attachDoubleClick(handleDoubleClick);
  button.attachLongPressStart(handleLongPress);
#endif

  lastDisplayUpdate = millis();
}

/*******************************************************************************************************
****************************  Motor control section   **************************************************
********************************************************************************************************/
// ISR Vector called whenever the AC voltage crosses zero
// With our circuit, the crossing is not really zero but happens 2.54ms later (45.72° phase shift)
void zeroCrossInterrupt() {
  if (startFlag) {
    TCCR1B = 0x03;       // start timer with divide by 64 input
    TCNT1 = 0;           // reset timer - count from zero
    OCR1A = firingTime;  // set the compare register brightness desired.
  }
}

ISR(TIMER1_COMPA_vect) {  // comparator match
  //digitalWrite(MOTOR_SPEED_CTRL, HIGH);
  PORTB |= 1;                         // set TRIAC gate to high
  TCNT1 = 65536 - GATE_PULSE_LENGTH;  // trigger pulse width
}

ISR(TIMER1_OVF_vect) {  // timer1 overflow
  PORTB &= ~1;
  TCCR1B = 0x00;  // disable timer stops unintended triggers
}


// ISR Vector for tachometer on A0
ISR(PCINT1_vect) {
  if (PINC & 0b00000001) {  // digitalRead(A0)
    unsigned long time = micros() - lastRPMpulse;
    float time_in_sec = ((float)time + rpmCorrection) / 1000000;
    float prerpm = 60 / time_in_sec;
    RPM = prerpm / TACHOPULSES;
    lastRPMpulse = micros();
  }
}

void loop() {

#ifdef MANUAL_JOG

  if (encoderCheck()) {
    firingTime = encoderPosition;
    lcd.setCursor(0, 0);
    lcd.print("pulse: ");
    lcd.print(firingTime);
  }
  button.tick();

  if ((millis() - lastDisplayUpdate) >= 500) {
    lcd.setCursor(0, 1);
    lcd.print("RPM: ");
    lcd.print(RPM < 1000 ? RPM < 100 ? RPM < 10 ? "   " : "  " : " " : "");
    lcd.print(RPM);
    lcd.print(" RPM");
    lastDisplayUpdate = millis();
  }

#else
  sequenceHandler(program_01[programIndex], program_01[programIndex + 1]);
  // If a sequence requires a motor to spin, we launch the motorLoop
  if (startFlag) {
    motorLoop();
  }
#endif
}

void handleClick() {
  startFlag ^= 1;
  if(!startFlag){
    direction ^= 1;
    digitalWrite(MOTOR_DIRECTION, direction);
  }
}

void handleDoubleClick() {
  sprayWash ^= 1;
  digitalWrite(SPRAY_WASH_MAIN, sprayWash);
  digitalWrite(SPRAY_WASH_PREa, sprayWash);
}

void handleLongPress() {
  drainPump ^= 1;
  digitalWrite(DRAIN_PUMP, drainPump);
}

bool encoderCheck() {
  long newPosition = encoder.read() / 4;

  if (newPosition != encoderPosition) {  // Encoder is rotated

    encoderPosition = newPosition;
    return true;
  } else return false;
}

// Calls pinMode() for each GPIO and interrupt
void hardwareSetup() {
  DDRD |= 0b11110000;   // (1 << MOTOR_DIRECTION) | (1 << SPRAY_WASH_MAIN) | (1 << SPRAY_WASH_PRE) | (1 << DRAIN_PUMP)
  DDRD &= ~0b00000111;  //(1 << ZCD_OUT); // Pin 2 Input
  DDRC &= ~1;           // Pin A0 input
  DDRB |= 0b00000111;   // MOTOR_SPEED_CTRL | HEATER_CTRL | BUZZER

  // set up Timer1
  OCR1A = 100;  // initialize the comparator
  OCR1B = 100;
  TIMSK1 = 0x03;  // enable comparator B, comparator A and overflow interrupts
  TCCR1A = 0x00;  // timer control registers set for
  TCCR1B = 0x00;  // normal operation, timer disabled

  // Activate interrupt for PCINT8 (A0) for rpm counter
  PCICR |= 1 << PCIE1;  // Group 1
  PCMSK1 |= 1;          // Pin A0

  attachInterrupt(0, zeroCrossInterrupt, RISING);

  // LCD Init
  lcd.init();
  lcd.backlight();
}

// Initialize the PID Algorithm
void pidSetup() {
  Input = 400;     // asiign initial value for PID
  Setpoint = 400;  // asiign initial value for PID

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(MIN_OUTPUT_LIMIT, MAX_OUTPUT_LIMIT);
  myPID.SetSampleTime(SAMPLE_RATE);  // Sets the sample rate
}

void sequenceHandler(uint8_t _sequence, uint16_t _duration) {
  switch (_sequence) {
    case SPRAY_WASH_MAIN:
      // If it is not started yet
      if (!sequenceStarted) {
        startSequence(SPRAY_WASH_MAIN, HIGH);
        digitalWrite(SPRAY_WASH_PRE, HIGH);
      }
      // Sequence stop condition : Timeout
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        stopSequence(SPRAY_WASH_MAIN, LOW);
        digitalWrite(SPRAY_WASH_PRE, LOW);
      }
      break;
    case SPRAY_WASH_PRE:
      // If it is not started yet
      if (!sequenceStarted) {
        startSequence(SPRAY_WASH_PRE, HIGH);
      }
      // Sequence stop condition : Timeout
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        stopSequence(SPRAY_WASH_PRE, LOW);
      }
      break;
    case DRAIN_PUMP:
      // If it is not started yet
      if (!sequenceStarted) {
        startSequence(DRAIN_PUMP, HIGH);
      }
      // Seuqence stop condition : Timeout
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        stopSequence(DRAIN_PUMP, LOW);
      }
      break;
    case SOAK:
      // If it is not started yet
      if (!sequenceStarted) {
        startSequence();
      }
      // Seuqence stop condition : Timeout
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        stopSequence();
      }
      break;
    case MOTOR_CCW_LOW:
      // If it is not started yet
      if (!sequenceStarted) {
        startSequence(MOTOR_DIRECTION, HIGH);
        desiredRPM = SPIN_SPEED;
        startFlag = true;
        delay(300);  // Delay before puting relay ON

        // manualRPMSet(1840);
      }

      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        startFlag = false;
        manualRPM = false;
        delay(300);  // Delay before putting relay OFF
        stopSequence(MOTOR_DIRECTION, LOW);
      }
      break;
    case MOTOR_CW_LOW:
      // If it is not started yet
      if (!sequenceStarted) {
        startSequence(MOTOR_DIRECTION, LOW);
        startFlag = true;
        desiredRPM = SPIN_SPEED;
        delay(300);
        // manualRPMSet(1800);
      }

      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        startFlag = false;
        manualRPM = false;
        delay(300);
        stopSequence(MOTOR_DIRECTION, LOW);
      }
      break;
    case MOTOR_CCW_HIGH:
      // If it is not started yet
      if (!sequenceStarted) {
        startSequence(MOTOR_DIRECTION, HIGH);
        startFlag = true;
        delay(300);
        desiredRPM = SPIN_SPEED;
      }


      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        startFlag = false;
        runFlag = false;
        delay(300);
        stopSequence(MOTOR_DIRECTION, LOW);
      }
      break;
    case MOTOR_CW_HIGH:
      // If it is not started yet
      if (!sequenceStarted) {
        startSequence(MOTOR_DIRECTION, LOW);
        startFlag = true;
        delay(300);
        desiredRPM = SPIN_SPEED;
      }

      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        startFlag = false;
        runFlag = false;
        stopSequence();
      }
      break;
    case END:
      if (!sequenceStarted) {
        digitalWrite(BUZZER, HIGH);
        delay(1000);
        digitalWrite(BUZZER, LOW);
        delay(500);
        digitalWrite(BUZZER, HIGH);
        delay(1000);
        digitalWrite(BUZZER, LOW);
        sequenceStarted = true;
      }
      break;
  }
}

void motorLoop() {
  // Motor Soft Start
  if (!(runFlag || manualRPM)) {
    myPID.SetTunings(sKp, sKi, sKd);  // Set the PID gain constants and start
    int i = (desiredRPM - tempcounter);
    for (int j = 1; j <= i; j++) {
      Input = RPM;
      Setpoint = tempcounter;
      myPID.Compute();
      firingTime = map(Output, MIN_OUTPUT_LIMIT, MAX_OUTPUT_LIMIT, MAX_OUTPUT_LIMIT, MIN_OUTPUT_LIMIT);  // inverse the output
      firingTime = constrain(firingTime, MIN_TRIGGERING_TIME, MAX_TRIGGERING_TIME);                      // check that dimming is in 20-625 range
      tempcounter++;
      delayMicroseconds(RISE_TIME);
    }
    if (tempcounter >= desiredRPM) {
      lastpiddelay = millis();
      runFlag = true;
      tempcounter = 80;
    }
  }

  // normal motor running state
  if (runFlag && !manualRPM) {
    unsigned long pidDelay = millis();

    if ((pidDelay - lastpiddelay) > 1000) {  // delay to switch PID values. Prevents hard start
      myPID.SetTunings(rKp, rKi, rKd);       // Set the PID gain constants and start
      lastpiddelay = millis();
    }

    Input = RPM;
    Setpoint = desiredRPM;
    myPID.Compute();
    firingTime = map(Output, MIN_OUTPUT_LIMIT, MAX_OUTPUT_LIMIT, MAX_OUTPUT_LIMIT, MIN_OUTPUT_LIMIT);  // inverse the output
    firingTime = constrain(firingTime, MIN_TRIGGERING_TIME, MAX_TRIGGERING_TIME);                      // check that dimming is in 20-625 range
  }
}

void manualRPMSet(int _RPM) {
  manualRPM = true;
  firingTime = _RPM;
}


// Routine executed at the begining of each wash cycle
void startSequence(uint8_t _pin, bool _pin_state) {
  sequenceStarted = true;
  sequenceStartTime = millis();  // Save the time the sequence started
  digitalWrite(_pin, _pin_state);
}

void startSequence() {
  sequenceStarted = true;
  sequenceStartTime = millis();  // Save the time the sequence started
}

// Routine executed at the end of each wash cycle
void stopSequence(uint8_t _pin, bool _pin_state) {
  sequenceStarted = false;
  programIndex += 2;  // Go to the next sequence on the program
  digitalWrite(_pin, _pin_state);

#ifdef DELAY_BETWEEN_SEQUENCE
  delay(DELAY_BETWEEN_SEQUENCE);
#endif
}

// Routine executed at the end of each wash cycle
void stopSequence() {
  sequenceStarted = false;
  programIndex += 2;  // Go to the next sequence on the program

#ifdef DELAY_BETWEEN_SEQUENCE
  delay(DELAY_BETWEEN_SEQUENCE);
#endif
}

void ringtone() {
  for (int thisNote = 0; thisNote < 7; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 300 / noteDurations[thisNote];
    TimerFreeTone(BUZZER, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration;
    delay(pauseBetweenNotes);
    // stop the tone playing:
  }
}
