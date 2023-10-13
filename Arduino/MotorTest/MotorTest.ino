#define L_MOTOR PIN_PA1
#define R_MOTOR PIN_PA2
#define LED_1 PIN_PF2

void setup() {
  pinMode(LED_1, OUTPUT);
  
  // Configure PORTMUX for PA1 and PA2
  PORTMUX_TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
  
  // Set the direction of PA1 and PA2 as output
  PORTA.DIRSET = PIN1_bm | PIN2_bm;
  
  // Configure TCA0 for PWM
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2EN_bm;
  TCA0.SINGLE.PER = 255;
  
  // Set the PWM values for PA1 and PA2
  //TCA0.SINGLE.CMP2 = 100;  // Duty cycle for PA2 (25%)
}

void writeMotor(uint8_t L, uint8_t R) {
  TCA0.SINGLE.CMP1 = L;
  analogWrite(R_MOTOR, R);
}


void loop() {
  writeMotor(80, 80);
  while (1) {
    digitalWrite(LED_1, HIGH);
    delay(1000);
    writeMotor(0, 0);
    digitalWrite(LED_1, LOW);
    delay(1000);
  }
}
