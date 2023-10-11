#include <Arduino.h>
#include <LFR.h>

void setMotor(uint8_t speedL, uint8_t speedR) {
    analogWrite(L_MOTOR, speedL);
    analogWrite(R_MOTOR, speedR);
}

void sensorRead(uint8_t *sensors) {
    sensors[0] = analogRead(IR_1);
    sensors[1] = analogRead(IR_2);
    sensors[2] = analogRead(IR_3);
    sensors[3] = analogRead(IR_4);
    sensors[4] = analogRead(IR_5);
}

void sensorParse()