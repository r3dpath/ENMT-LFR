#ifndef LFR_H
#define LFR_H

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

typedef enum {
    LEFTMOTOR,
    RIGHTMOTOR
} motor_t;

void sensorRead(uint8_t *sensors);
void setMotor(uint8_t speedL, uint8_t speedR);

#endif //LFR_H