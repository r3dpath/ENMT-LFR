#include <Arduino.h>
#include <LFR.h>

static uint8_t CURVE[61] = {179, 178, 178, 177, 177, 176, 175, 175, 174, 173, 172, 171, 170, 169,
                            168, 167, 165, 163, 161, 159, 156, 154, 150, 146, 142, 137, 131, 125,
                            118, 110, 102,  94,  86,  79,  73,  67,  62,  58,  54,  50,  48,  45,
                            43,  41,  39,  37,  36,  35,  34,  33,  32,  31,  30,  29,  29,  28,
                            27,  27,  26,  26,  25};

static uint8_t sensorlast = 0;

void setMotor(uint8_t speedL, uint8_t speedR) {
    analogWrite(L_MOTOR, speedL);
    analogWrite(R_MOTOR, speedR);
}

void sensorRead(uint8_t *sensors) {
    sensors[4] = analogRead(IR_1);
    sensors[3] = analogRead(IR_2);
    sensors[2] = analogRead(IR_3);
    sensors[1] = analogRead(IR_4);
    sensors[0] = analogRead(IR_5);
}

float sensorParse(void) {
    uint8_t sensors[5] = {0, 0, 0, 0, 0};
    sensorRead(sensors);
    sensorPrint(sensors);
    uint8_t minindex = 0;
    for (uint8_t i = 1; i<5; i++) {
        if (sensors[i] < sensors[minindex]) {
            minindex = i;
        }
    }
    if (sensors[minindex] < SENSOR_MIN_THRESHOLD) {
        if (minindex == 0) {
            sensorlast = 0;
            return sensorlast;
        } else if (minindex == 4) {
            sensorlast = 120;
            return sensorlast;
        } else if (sensors[minindex-1] < SENSOR_SIDE_THRESHOLD) {
            sensorlast = (20*minindex-(20-curveFit(sensors[minindex-1], sensors[minindex])));
            return sensorlast;
        } else if (sensors[minindex+1] < SENSOR_SIDE_THRESHOLD) {
            sensorlast = (20*minindex+curveFit(sensors[minindex+1], sensors[minindex]));
            return sensorlast;
        } else {
            sensorlast = 20*minindex;
            return sensorlast;
        }
    } else {
        return sensorlast;
    }
}

void sensorPrint(uint8_t *sensors) {
  for (int i = 0; i < 5; i++) {
    Serial1.print(sensors[i]);
    Serial1.print(" ");
  }
  Serial1.println();
}

float curveFit(uint8_t x1, uint8_t x2) {
    uint16_t lstsqrs=65000;
    uint8_t loc = 0;

    for (uint8_t i=0; i<50; i++) {
        uint16_t err =  abs(x1 - CURVE[i] + x2 - (-CURVE[i+11]+204));
        if (err < lstsqrs) {
            lstsqrs = err;
            loc = i;
        }
    }

    return (loc/5)+10;
}