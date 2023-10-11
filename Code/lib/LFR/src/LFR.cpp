#include <Arduino.h>
#include <LFR.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

BLA::Matrix<3, 4> sensorDataU = {0.0025, -0.0025, -0.0025, 0.0025, -0.155, 0.155, 0.135, -0.095, 2.25, -0.75, -1.25, 0.75};
BLA::Matrix<3, 4> sensorDataL = {0.0025, -0.0025, -0.0025, 0.0025, -0.105, 0.065, 0.085, -0.045, 0.95, 0.15, -0.15, 0.05};

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
    for (uint8_t i = 0; i < 5; i++) {
        if (sensors[i] < SENSOR_THRESHOLD) {
            if (i<2) {
                sensorlast = 0;
                BLA::Matrix<4> mat = {sensors[0], sensors[1], sensors[2], sensors[3]};
                return sensorMatMul(mat, sensorDataL);
            }
            else {
                sensorlast = 40;
                BLA::Matrix<4> mat = {sensors[1], sensors[2], sensors[3], sensors[4]};
                return sensorMatMul(mat, sensorDataU);
            }
        } else {
            return sensorlast;
        }
    }

}

float sensorMatMul(BLA::Matrix<4> sensors, BLA::Matrix<3, 4> mult) {
    BLA::Matrix<3> R = mult * sensors;
    return -R(1)/(2*R(0));
}