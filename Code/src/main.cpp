#include <Arduino.h>
#include "LFR.h"
#define BUZZER PIN_PA3

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_1, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(R_MOTOR, OUTPUT);
  pinMode(L_MOTOR, OUTPUT);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int speed = 0; speed <= 255; speed += 5) { // Change 5 to adjust step size
    analogWrite(L_MOTOR, speed);
    analogWrite(R_MOTOR, speed);
    delay(50); // Adjust delay for acceleration rate
  }

  delay(1000); // Run at full speed for 1 second

  // Decelerate motors
  for (int speed = 255; speed >= 0; speed -= 5) { // Change 5 to adjust step size
    analogWrite(L_MOTOR, speed);
    analogWrite(R_MOTOR, speed);
    delay(50); // Adjust delay for deceleration rate
  }

  delay(1000); // Pause for 1 second
  tone(BUZZER_PIN, 1000);
  delay(1000);
  noTone(BUZZER_PIN);
  delay(1000);
  digitalWrite(LED_1,HIGH); //Turns on LED
  delay(1000);
  digitalWrite(LED_1, LOW); //Turn off LED
  delay(1000);

}

