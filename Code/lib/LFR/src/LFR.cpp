#include <Arduino.h>
#include <LFR.h>

float sensorDataU[12] = {0.0025, -0.0025, -0.0025, 0.0025, -0.155, 0.155, 0.135, -0.095, 2.25, -0.75, -1.25, 0.75};
float sensorDataL[12] = {0.0025, -0.0025, -0.0025, 0.0025, -0.105, 0.065, 0.085, -0.045, 0.95, 0.15, -0.15, 0.05};

static uint8_t sensorlast = 0;

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

float sensorParse(void) {
    uint8_t sensors[5] = {0, 0, 0, 0, 0};
    sensorRead(sensors);
    sensorPrint(sensors);
    for (uint8_t i = 0; i < 5; i++) {
        if (sensors[i] < SENSOR_THRESHOLD) {
            digitalWrite(LED_1, HIGH);
            if (i<2) {
                sensorlast = 0;
                return sensorMatMul(sensors, sensorDataL);
            }
            else {
                sensorlast = 40;
                return sensorMatMul(sensors+1, sensorDataU);
            }
        } 
    }
    return sensorlast;
}

void sensorPrint(uint8_t *sensors) {
  for (int i = 0; i < 5; i++) {
    Serial1.print(sensors[i]);
    Serial1.print(" ");
  }
  Serial1.println();
}

float sensorMatMul(uint8_t* sen, float* mult) {
    float R_1 = sen[0]*mult[0]+sen[1]*mult[1]+sen[2]*mult[2]+sen[3]*mult[3];
    float R_2 = sen[4]*mult[0]+sen[5]*mult[1]+sen[6]*mult[2]+sen[7]*mult[3];
    return -R_2/(2*R_1);
}