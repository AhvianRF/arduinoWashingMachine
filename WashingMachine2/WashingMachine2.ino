// Arduino washing machine controller
//
// By Voara Andriamihaminjaka <voara@vk.com>

#include <LiquidCrystal_I2C.h>
#include "pitches.h"

// notes in the melody:
int melody[] = {
  NOTE_D5, 0, NOTE_FS5, 0, NOTE_A5, 0, NOTE_D6
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  8, 8, 8, 8, 8, 8, 8
};

// GPIO assignment
// Inputs
#define ZCD_OUT 2  // AC Sensing for TRIAC phase control : Interrupt pin
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
#define CLOCKWISE true
#define COUNTER_CLOCKWISE false

#define MOTOR_CCW_LOW 100
#define MOTOR_CW_LOW 101
#define MOTOR_CCW_HIGH 102
#define MOTOR_CW_HIGH 103
#define END 0

#define ZCD_PHASE_SHIFT 2200  // microseconds
#define GATE_PULSE_LENGTH 250

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

bool zeroCross = false;
uint8_t zeroCrossCount = 0;
bool sprayWashMain = false;
bool sprayWashPre = false;
bool motorDirection = CLOCKWISE;

bool sequenceStarted = false;
long sequenceStartTime = 0;
uint16_t duration;
uint8_t sequence;
uint8_t programIndex = 0;

uint16_t program_01[] = { DRAIN_PUMP, 10, SPRAY_WASH_MAIN, 15, DRAIN_PUMP, 8, SPRAY_WASH_PRE, 10, DRAIN_PUMP, 20, END, 0 };

LiquidCrystal_I2C lcd(0x24, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  delay(1000);
  hardwareSetup();
  ringtone();

  attachInterrupt(0, zeroCrossInterrupt, FALLING);
}

void loop() {
  sequenceHandler(program_01[programIndex], program_01[programIndex + 1]);
}

// Calls pinMode() for each GPIO and interrupt
void hardwareSetup() {
  DDRD |= 0b11110000;  // (1 << MOTOR_DIRECTION) | (1 << SPRAY_WASH_MAIN) | (1 << SPRAY_WASH_PRE) | (1 << DRAIN_PUMP)
  DDRD &= ~(1 << ZCD_OUT);
  DDRB |= 0b00000111;  // MOTOR_SPEED_CTRL | HEATER_CTRL | BUZZER
}

void ringtone() {
  for (int thisNote = 0; thisNote < 7; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 600 / noteDurations[thisNote];
    tone(BUZZER, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BUZZER);
  }
}

void sequenceHandler(uint8_t _sequence, uint16_t _duration) {
  switch (_sequence) {
    case SPRAY_WASH_MAIN:
      // If it is not started yet
      if (!sequenceStarted) {
        sequenceStarted = true;
        sequenceStartTime = millis();  // Save the time the sequence started
        digitalWrite(SPRAY_WASH_MAIN, HIGH);
      }
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        digitalWrite(SPRAY_WASH_MAIN, LOW);
        sequenceStarted = false;
        programIndex += 2;  // Go to the next sequence on the program
      }
      break;
    case SPRAY_WASH_PRE:
      // If it is not started yet
      if (!sequenceStarted) {
        sequenceStarted = true;
        sequenceStartTime = millis();  // Save the time the sequence started
        digitalWrite(SPRAY_WASH_PRE, HIGH);
      }
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        digitalWrite(SPRAY_WASH_PRE, LOW);
        sequenceStarted = false;
        programIndex += 2;  // Go to the next sequence on the program
      }
      break;
    case DRAIN_PUMP:
      // If it is not started yet
      if (!sequenceStarted) {
        sequenceStarted = true;
        sequenceStartTime = millis();  // Save the time the sequence started
        digitalWrite(DRAIN_PUMP, HIGH);
      }
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        digitalWrite(DRAIN_PUMP, LOW);
        sequenceStarted = false;
        programIndex += 2;  // Go to the next sequence on the program
      }
      break;
    case MOTOR_CCW_LOW:
      // If it is not started yet
      if (!sequenceStarted) {
        sequenceStarted = true;
        sequenceStartTime = millis();  // Save the time the sequence started
        digitalWrite(MOTOR_DIRECTION, HIGH);
      }
      motorZVSControl(2);
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        digitalWrite(MOTOR_DIRECTION, LOW);
        sequenceStarted = false;
        programIndex += 2;  // Go to the next sequence on the program
      }
      break;
    case MOTOR_CW_LOW:
      // If it is not started yet
      if (!sequenceStarted) {
        sequenceStarted = true;
        sequenceStartTime = millis();  // Save the time the sequence started
        digitalWrite(MOTOR_DIRECTION, LOW);
      }
      motorZVSControl(2);
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        sequenceStarted = false;
        programIndex += 2;  // Go to the next sequence on the program
      }
      break;
    case MOTOR_CCW_HIGH:
      // If it is not started yet
      if (!sequenceStarted) {
        sequenceStarted = true;
        sequenceStartTime = millis();  // Save the time the sequence started
        digitalWrite(MOTOR_DIRECTION, HIGH);
      }
      motorZVSControl(10);
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        digitalWrite(MOTOR_DIRECTION, LOW);
        sequenceStarted = false;
        programIndex += 2;  // Go to the next sequence on the program
      }
      break;
    case MOTOR_CW_HIGH:
      // If it is not started yet
      if (!sequenceStarted) {
        sequenceStarted = true;
        sequenceStartTime = millis();  // Save the time the sequence started
        digitalWrite(MOTOR_DIRECTION, LOW);
      }
      motorZVSControl(10);
      if ((millis() - sequenceStartTime) >= _duration * 1000) {
        sequenceStarted = false;
        programIndex += 2;  // Go to the next sequence on the program
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


/*******************************************************************************************************
****************************  Motor control section   **************************************************
********************************************************************************************************/

// IRS Vector called whenever the AC voltage crosses zero
void zeroCrossInterrupt() {
  if (!zeroCross) {
    zeroCross = true;  // Triggers on rising edge only and if it was reset from loop (ie. Motor running)
    zeroCrossCount++;
    if (zeroCrossCount > 20) zeroCrossCount = 1;
  }
}


// Sends pulses to the motor Triac Gate and do zero volt switching
// from 1 to 10 (1 slow ... 10 max speed)
void motorZVSControl(uint8_t speed) {
  // Triggers only when zero cross is detected
  if (zeroCross) {
    if (zeroCrossCount <= (speed * 2)) {
      motorSendPulse();
    }
    zeroCross = false;
  }
}

// Regular TRIAC function controlling a firing angle
void motorPhaseControl(uint8_t angle) {
  if (zeroCross) {
    delayMicroseconds(angle * 10000 / 180);
    motorSendPulse();
    zeroCross = false;
  }
}

// Send a pulse to the motor speed control pin
void motorSendPulse() {
  /* without delay, it triggers 2.54ms late (45.72°)
    */
  delayMicroseconds(ZCD_PHASE_SHIFT);  // 2 ms delay
  //digitalWrite(MOTOR_SPEED_CTRL, HIGH);
  PORTB |= 1;
  delayMicroseconds(GATE_PULSE_LENGTH);
  //digitalWrite(MOTOR_SPEED_CTRL, LOW);
  PORTB &= ~1;
}
