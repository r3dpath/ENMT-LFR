#define LED_1 PIN_PF2

#define L_MOTOR PIN_PA1
#define R_MOTOR PIN_PA2
#define BUZZER_PIN PIN_PA3
#define LED_1 PIN_PF2
#define LED_2 PIN_PF3 
#define LED_3 PIN_PF4

#define IR_1 PIN_PD0 
#define IR_2 PIN_PD1 
#define IR_3 PIN_PD2 
#define IR_4 PIN_PD3 
#define IR_5 PIN_PD4

#define SENSOR_THRESHOLD 50
#define WRITEFREQ 1 //Supports 1, 2, 8, 16, 32 and 64 kHz
#define ANALOGRES 8 //Supports 8 and 10 bit resolution

uint8_t sensors[5] = {0, 0, 0, 0, 0};

void setup_PWM() {
  PORTA.DIRSET = PIN1_bm | PIN2_bm;

  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm;
  TCA0.SINGLE.PER = 255;
}

void setMotor(uint8_t L, uint8_t R) {
  TCA0.SINGLE.CMP1 = L;
  //TCA0.SINGLE.CMP2 = R;
}

void setup() {
  pinMode(LED_1, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  //pinMode(R_MOTOR, OUTPUT);
  //pinMode(L_MOTOR, OUTPUT);
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);
  pinMode(IR_5, INPUT);
  Serial1.begin(57600);
  Serial1.print("Hi\n");
  //analogWriteFrequency(WRITEFREQ);
  analogReadResolution(ANALOGRES);
  //setup_PWM();
  setMotor(40, 40);
}

void sensorRead(uint8_t *sensors) {
    sensors[0] = analogRead(IR_1);
    sensors[1] = analogRead(IR_2);
    sensors[2] = analogRead(IR_3);
    sensors[3] = analogRead(IR_4);
    sensors[4] = analogRead(IR_5);
}

void printSensors() {
  sensorRead(sensors);
  for (int i = 0; i < 5; i++) {
    Serial1.print(sensors[i]);
    Serial1.print(" ");
  }
  Serial1.println();
}

void loop() {
    digitalWrite(LED_2, HIGH);
    printSensors();
    digitalWrite(LED_2, LOW);
    delay(500);
}
