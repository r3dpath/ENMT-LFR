#include <Arduino.h>

#define LED_PIN PIN_PF2
#define BUZZER_PIN PIN_PA3

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  tone(BUZZER_PIN, 1000);
  delay(1000);
  noTone(BUZZER_PIN);
  delay(1000);
}
