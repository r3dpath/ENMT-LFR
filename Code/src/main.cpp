#include <Arduino.h>
#include "LFR.h"

#define WRITEFREQ 1 //Supports 1, 2, 8, 16, 32 and 64 kHz
#define ANALOGRES 8 //Supports 8 and 10 bit resolution

uint8_t sensors[5] = {0, 0, 0, 0, 0};

void setup_PWM() {
  PORTMUX_TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
  PORTA.DIRSET = PIN1_bm | PIN2_bm;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2EN_bm;
  TCA0.SINGLE.PER = 1024;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);
  pinMode(IR_5, INPUT);
  Serial1.begin(57600);
  // analogWriteFrequency(WRITEFREQ);
  analogReadResolution(ANALOGRES);
}

void loop() {
  while (1) {
    Serial1.println(sensorParse());
    delay(500);
  }
}

