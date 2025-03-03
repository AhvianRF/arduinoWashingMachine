#ifndef __PIN_DEFINITION_H__
    #define __PIN_DEFINITION_H__
    
    // GPIO assignment

    // ZCD : ZeroCrossDetector input for TRIAC phase control : Interrupt pin
    #define ZCD_DIRECTION_REG DDRD 
    #define ZCD_DATA_REG PORTD
    #define ZCD_PIN 2

    // TACHO
    #define TACHO_DIRECTION_REG DDRC
    #define TACHO_DATA_REG  PORTC
    #define TACHO_PIN 0

    // Encoder pins
    #define ENC_A 0
    #define ENC_B 1
    #define ENC_SW 3

    // Outputs - AND USE AS STEPS AS WELL
    
    // MOTOR DIRECTION    
    #define MOTOR_DIRECTION_DREG DDRD
    #define MOTOR_DIRECTION_DATA_REG PORTD
    #define MOTOR_DIRECTION_PIN 4
    #define MOTOR_DIRECTION 4   // LOW: CW | HIGH: CCW

    // TRIAC GATE
    #define MOTOR_SPEED_CTRL_DIRECTION_REG  DDRB
    #define MOTOR_SPEED_CTRL_DATA_REG   PORTB
    #define MOTOR_SPEED_CTRL_PIN 0
    #define MOTOR_SPEED_CTRL 8  // Triac phase control


    #define SPRAY_WASH_MAIN 5   // Active High
    #define SPRAY_WASH_PRE 6    // Active High
    #define DRAIN_PUMP 7        // Active High

    #define HEATER_CTRL 9       // Triac ON/OFF
    #define BUZZER 10           // PWM Tone control

#endif