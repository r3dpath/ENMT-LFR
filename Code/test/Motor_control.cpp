// Motor Control

#include <avr/io.h>
#include <util/delay.h>

const int leftMotorPin = 1; // PA1
const int rightMotorPin = 2; // PA2

void setup() {
  // Set motor pins as outputs
  PORTA.DIRSET = PIN1_bm | PIN2_bm; // Set PA1 and PA2 as outputs
}

void loop() {
  // Move the left motor forward
  PORTA.OUTSET = PIN1_bm; // Turn on PA1
  PORTA.OUTCLR = PIN2_bm; // Turn off PA2

  // Wait for some time (adjust this according to your requirements)
  _delay_ms(1000);

  // Stop both motors
  PORTA.OUTCLR = PIN1_bm; // Turn off PA1
  PORTA.OUTCLR = PIN2_bm; // Turn off PA2

  // Wait for some time
  _delay_ms(1000);

  // Move the right motor forward
  PORTA.OUTCLR = PIN1_bm; // Turn off PA1
  PORTA.OUTSET = PIN2_bm; // Turn on PA2

  // Wait for some time
  _delay_ms(1000);

  // Stop both motors
  PORTA.OUTCLR = PIN1_bm; // Turn off PA1
  PORTA.OUTCLR = PIN2_bm; // Turn off PA2

  // Wait for some time
  _delay_ms(1000);
}
