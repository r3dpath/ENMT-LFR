
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
const int numSensors = 5;
const float weights[numSensors] = {-0.6, -0.2, 0, 0.2, 0.6};
int lastError = 0;
int integral = 0;
//int lastTurnDirection = 0;
// PID Constants
const float Kp = 10;  // Proportional constant - You might need to tune this
const float Ki = 0.01;  // Integral constant - Start with 0 and tune later if needed
const float Kd = 0.3;  // Derivative constant - You might need to tune this
int baseSpeed = 10;
//const float DEADBAND = 0.2;

void setup_PWM() {
  PORTA.DIRSET = PIN1_bm | PIN2_bm;

  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm;
  TCA0.SINGLE.PER = 255;
}

void setMotor(uint8_t L, uint8_t R) {
  TCA0.SINGLE.CMP1 = L;
  analogWrite(R_MOTOR, R);
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
    // Configure PORTMUX for PA1 and PA2
  PORTMUX_TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
  
  // Set the direction of PA1 and PA2 as output
  PORTA.DIRSET = PIN1_bm | PIN2_bm;
  
  // Configure TCA0 for PWM
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2EN_bm;
  TCA0.SINGLE.PER = 255;
  analogReadResolution(ANALOGRES);
}

void sensorRead(uint8_t *sensors) {
    sensors[0] = analogRead(IR_1);
    sensors[1] = analogRead(IR_2);
    sensors[2] = analogRead(IR_3);
    sensors[3] = analogRead(IR_4);
    sensors[4] = analogRead(IR_5);
}


float calculatePosition(uint8_t *sensors) {
    int weightedSum = 0;
    int total = 0;
    for (int i = 0; i < 5; i++) {
        weightedSum += (235 - sensors[i]) * weights[i];  // 255-sensors[i] to consider black as maximum value
        total += (235 - sensors[i]);
    }
    float position = (float)weightedSum / total;
        
    Serial1.print(position);
    return position;
}

void printSensors() {
  sensorRead(sensors);
  for (int i = 0; i < 5; i++) {
    Serial1.print(sensors[i]);
    Serial1.print(" ");
  }
  Serial1.println();
}

int computePID(float error) {
    integral += error;
    int derivative = error - lastError;
    int turn = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    return turn;
}

void driveMotors(int speed, int turnValue) {
    int leftSpeed = constrain(speed + turnValue, 0, 13);
    int rightSpeed = constrain(speed - turnValue, 0, 13);
    setMotor(rightSpeed, leftSpeed);
    Serial1.print(String(leftSpeed) + " " + String(rightSpeed) + "\n");
}

void loop() {
    // digitalWrite(LED_2, HIGH);U
    // printSensors();
    // digitalWrite(LED_2, LOW);
    // delay(500);
    sensorRead(sensors);
    printSensors();

    // Calculate line position error using weighted sum
    float error = calculatePosition(sensors);    
    int turnValue = computePID(error);  



    driveMotors(baseSpeed, turnValue);  // Assuming 127 as the base speed

    digitalWrite(LED_2, HIGH);
    printSensors();
    digitalWrite(LED_2, LOW);
    delay(10);  // Reduced delay for quicker response
}

