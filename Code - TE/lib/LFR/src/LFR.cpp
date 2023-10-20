#include <Arduino.h>
#include <LFR.h>

const uint8_t CURVE[61] = {179, 178, 178, 177, 177, 176, 175, 175, 174, 173, 172, 171, 170, 169,
                            168, 167, 165, 163, 161, 159, 156, 154, 150, 146, 142, 137, 131, 125,
                            118, 110, 102,  94,  86,  79,  73,  67,  62,  58,  54,  50,  48,  45,
                            43,  41,  39,  37,  36,  35,  34,  33,  32,  31,  30,  29,  29,  28,
                            27,  27,  26,  26,  25};

static uint8_t sensorlast = 0;
static int16_t integral = 0;
static int16_t lastError = 0;

static void sensorRead(uint8_t*);
static uint8_t curveFit(uint8_t, uint8_t);
static int8_t PID(int8_t);

static uint8_t derivative = 0;

static uint8_t baseSpeed = BaseSpeed;
static uint8_t maxSpeed = MaxSpeed;
static uint8_t boostSpeed = BoostSpeed;

void setup_PWM(void) {
    PORTMUX_TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
    PORTA.DIRSET = PIN1_bm | PIN2_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm;
    TCA0.SINGLE.PER = 255;
}

void motorUpdate(void) {
    if (millis() > 2600) {
        baseSpeed = BaseSpeed - 10;
        maxSpeed = MaxSpeed - 10;
        boostSpeed = 0;
    }
    // if (millis() > 4000) {
    //     boostSpeed = 0;
    // }

    uint8_t sensors[5] = {0, 0, 0, 0, 0};
    sensorRead(sensors);
    uint8_t pos = sensorParse(sensors);
    Serial1.println(pos);
    uint8_t leftSpeed;
    uint8_t rightSpeed;
    int8_t error = 50-pos;
    int8_t turn = PID(error);
    if (sensors[2] < SENSOR_MIN_THRESHOLD) {
        leftSpeed = constrain((baseSpeed + boostSpeed) - turn, 0, (maxSpeed + boostSpeed));
        rightSpeed = constrain((baseSpeed + boostSpeed) + turn, 0, (maxSpeed + boostSpeed));
    } else {
        leftSpeed = constrain(baseSpeed - turn, 0, maxSpeed);
        rightSpeed = constrain(baseSpeed + turn, 0, maxSpeed);
    }

    setMotor(leftSpeed, rightSpeed);

    Serial1.print(String(leftSpeed) + " " + String(rightSpeed) + "\n");
}

void setMotor(uint8_t speedL, uint8_t speedR) {
    TCA0.SINGLE.CMP1 = speedL * LBias;
    analogWrite(R_MOTOR, speedR);
}

void sensorPrint(uint8_t *sensors) {
  for (int i = 0; i < 5; i++) {
    Serial1.print(sensors[i]);
    Serial1.print(" ");
  }
  Serial1.println();
}

uint8_t sensorParse(uint8_t *sensors) {
    sensorPrint(sensors);
    uint8_t minindex = 0;
    static uint8_t lastRead = 50;
    for (uint8_t i = 1; i<5; i++) {
        if (sensors[i] < sensors[minindex]) {
            minindex = i;
        }
    }
    //derivative = abs(lastRead-sensors[minindex]);
    lastRead = sensors[minindex];
    if (sensors[minindex] < SENSOR_MIN_THRESHOLD) {
        // if ((sensorlast == 0 && minindex > 2) || (sensorlast == 100 && minindex < 2)) {
        //     return sensorlast;
        // }
        if (minindex != 0) {
            if (sensors[minindex-1] < SENSOR_SIDE_THRESHOLD) {
                sensorlast = (10+(20*minindex)-(20-curveFit(sensors[minindex-1], sensors[minindex])));

                return sensorlast;
            }
        }
        if (minindex != 4) {
            if (sensors[minindex+1] < SENSOR_SIDE_THRESHOLD) {
                sensorlast = (10+20*minindex+curveFit(sensors[minindex], sensors[minindex+1]));
                return sensorlast;
            }
        }

        sensorlast = 10+20*minindex;
        return sensorlast;

    } else {
        if (sensorlast == 10) {
            sensorlast = 0;
            
            return sensorlast;
        } else if (sensorlast == 90) {
            sensorlast = 100;
            return sensorlast;
        }
        
        return sensorlast;
    }
}


static void sensorRead(uint8_t *sensors) {
    sensors[4] = analogRead(IR_2);
    sensors[3] = analogRead(IR_1);
    sensors[2] = analogRead(IR_3);
    sensors[1] = analogRead(IR_5);
    sensors[0] = analogRead(IR_4);
}

static uint8_t curveFit(uint8_t x1, uint8_t x2) {
    uint16_t lstsqrs=65000;
    uint8_t loc = 0;

    for (uint8_t i=0; i<50; i++) {
        uint16_t err =  abs(x1 - CURVE[i] + x2 - (-CURVE[i]+204));
        if (err < lstsqrs) {
            lstsqrs = err;
            loc = i;
        }
    }
    //Serial1.println((loc/5)+10);
    return (loc/5)+10;
}

static int8_t PID(int8_t error) {
    integral += error/20;
    derivative = error - lastError;
    //Serial1.println(integral*Ki);
    if(error == 0) {
        integral = 0;
    }
    int8_t turn = constrain(Kp * error + integral*Ki + derivative*Kd, -127, 128);
    //Serial1.println(derivative);
    // if (turn > 0) {
    //     turn -= Kd * derivative;
    //     if (turn < 0) {
    //         turn = 0;
    //     }
    // } else {
    //     turn += Kd * derivative;
    //     if (turn > 0) {
    //         turn = 0;
    //     }
    // }
    lastError = error;
    return turn;
}