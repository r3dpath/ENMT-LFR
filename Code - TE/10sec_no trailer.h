#ifndef LFR_H
#define LFR_H

#include <Arduino.h>

#define L_MOTOR PIN_PA1
#define R_MOTOR PIN_PA2
#define BUZZER_PIN PIN_PA3
#define LED_1 PIN_PF2
#define LED_2 PIN_PF3 
#define LED_3 PIN_PF4

#define IR_1 PIN_PD0 
#define IR_2 PIN_PD1 
#define IR_3 PIN_PD2 
#define IR_4 PIN_PD3 
#define IR_5 PIN_PD4

#define SENSOR_MIN_THRESHOLD 170
#define SENSOR_SIDE_THRESHOLD 180

// #define SENSOR_MIN_THRESHOLD 50
// #define SENSOR_SIDE_THRESHOLD 0

#define Kp 0.7  // Proportional constant - You might need to tune this
#define Ki 0  // Integral constant - Start with 0 and tune later if needed
#define Kd 5  // Derivative constant - You might need to tune this

#define BaseSpeed 65
#define MaxSpeed 65
#define BoostSpeed 8
#define LBias 1.1

typedef enum {
    LEFTMOTOR,
    RIGHTMOTOR
} motor_t;

void setup_PWM(void);
void motorUpdate(void);
void setMotor(uint8_t, uint8_t);
void sensorPrint(uint8_t*);
uint8_t sensorParse(uint8_t*);


#endif //LFR_H

//base = 40, Kp = 0.8, Ki = 0, Kd = 0.02
//

//---
// 11 sec w rubber bands

// #define baseSpeed 75
// #define maxSpeed 75

// #define Kp 1.2  // Proportional constant - You might need to tune this
// #define Ki 0  // Integral constant - Start with 0 and tune later if needed
// #define Kd 1  // Derivative constant - You might need to tune this

//w base + 30, max + 25
//---

/*
Solid 12s run with wire slowdown
#define Kp 1.2  // Proportional constant - You might need to tune this
#define Ki 0.001  // Integral constant - Start with 0 and tune later if needed
#define Kd 10  // Derivative constant - You might need to tune this

#define baseSpeed 60
#define maxSpeed 65
#define boostSpeed 0
*/