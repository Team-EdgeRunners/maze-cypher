/*
  ================================================================================
  MODULE: L298N Dual H-Bridge Motor Driver + 2x N20 Hall Sensor Encoder Motors
  PLATFORM: STM32 (e.g. STM32F103C8T6 Blue Pill)
  ENVIRONMENT: PlatformIO (VS Code Extension)
  ================================================================================
  
  [1] PLATFORMIO CONFIGURATION (platformio.ini):
  ---------------------------------------------
  Environment: [env:l298n_motors_test]
  Platform: ststm32
  Board: bluepill_f103c8
  Framework: arduino

  [2] HARDWARE WIRING:
  --------------------
  A. L298N Motor Driver Connections:
  +-------------------+----------------------------------+
  | L298N Pin         | STM32 Board Pin                  |
  +-------------------+----------------------------------+
  | ENA (PWM Motor A) | PA8  (TIM1_CH1 PWM Output)        |
  | IN1 (Motor A Dir1)| PB12                             |
  | IN2 (Motor A Dir2)| PB13                             |
  | ENB (PWM Motor B) | PA11 (TIM1_CH4 PWM Output)       |
  | IN3 (Motor B Dir1)| PB14                             |
  | IN4 (Motor B Dir2)| PB15                             |
  | VMS / 12V         | Motor Battery Power (6V - 12V)   |
  | GND               | Common GND (Must connect to STM32)|
  | 5V (from L298N)   | 5V In (If powering STM32 from it)|
  +-------------------+----------------------------------+

  B. N20 Motor Hall Sensor Encoder Connections:
  +-------------------+----------------------------------+
  | Encoder Pin       | STM32 Board Pin                  |
  +-------------------+----------------------------------+
  | Motor A Enc VCC   | 3.3V                             |
  | Motor A Enc GND   | GND                              |
  | Motor A Enc Out A | PB0 (External Interrupt EXTI0)   |
  | Motor A Enc Out B | PB1                              |
  |                   |                                  |
  | Motor B Enc VCC   | 3.3V                             |
  | Motor B Enc GND   | GND                              |
  | Motor B Enc Out A | PB10 (External Interrupt EXTI10) |
  | Motor B Enc Out B | PB11                             |
  +-------------------+----------------------------------+

  ================================================================================
*/

#include <Arduino.h>

// Motor A Pins (Left Motor)
#define ENA_PIN PA8
#define IN1_PIN PB12
#define IN2_PIN PB13

// Motor B Pins (Right Motor)
#define ENB_PIN PA11
#define IN3_PIN PB14
#define IN4_PIN PB15

// Encoder Pins
#define ENC_A_PHASE_A PB0   // Interrupt pin for Motor A
#define ENC_A_PHASE_B PB1
#define ENC_B_PHASE_A PB10  // Interrupt pin for Motor B
#define ENC_B_PHASE_B PB11

// Encoder pulse counters (volatile for ISR safe access)
volatile int32_t encoderCountA = 0;
volatile int32_t encoderCountB = 0;

// N20 Motor Specs (Typical: 7 Pulses Per Revolution per channel, Gear Ratio e.g. 30:1 or 50:1)
// Adjust PPR based on your specific N20 gear ratio (e.g. 7 * GearRatio)
const float ENCODER_PPR = 210.0f; // Example: 7 PPR * 30:1 gear ratio = 210 pulses/rev

// Interrupt Service Routine (ISR) for Motor A Encoder
void isr_encoderA() {
  if (digitalRead(ENC_A_PHASE_B) == HIGH) {
    encoderCountA++;
  } else {
    encoderCountA--;
  }
}

// Interrupt Service Routine (ISR) for Motor B Encoder
void isr_encoderB() {
  if (digitalRead(ENC_B_PHASE_B) == HIGH) {
    encoderCountB++;
  } else {
    encoderCountB--;
  }
}

// Motor Control Helper Function
// speed: -255 to +255 (Negative = Reverse, Positive = Forward, 0 = Stop)
void setMotorA(int speed) {
  if (speed > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, constrain(speed, 0, 255));
  } else if (speed < 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, constrain(-speed, 0, 255));
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 0);
  }
}

void setMotorB(int speed) {
  if (speed > 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, constrain(speed, 0, 255));
  } else if (speed < 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, constrain(-speed, 0, 255));
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, 0);
  }
}

void stopMotors() {
  setMotorA(0);
  setMotorB(0);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println(F("\n==========================================="));
  Serial.println(F(" STM32 + L298N + Dual N20 Encoder Test     "));
  Serial.println(F("==========================================="));

  // Configure Motor Output Pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Configure Encoder Input Pins
  pinMode(ENC_A_PHASE_A, INPUT_PULLUP);
  pinMode(ENC_A_PHASE_B, INPUT_PULLUP);
  pinMode(ENC_B_PHASE_A, INPUT_PULLUP);
  pinMode(ENC_B_PHASE_B, INPUT_PULLUP);

  // Attach Hardware Interrupts for Encoder Pulse Counting
  attachInterrupt(digitalPinToInterrupt(ENC_A_PHASE_A), isr_encoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PHASE_A), isr_encoderB, RISING);

  stopMotors();
  Serial.println(F("[SUCCESS] Motor driver and encoders initialized!"));
  Serial.println(F("Starting automated test sequence...\n"));
}

void loop() {
  static uint32_t lastTime = 0;
  static int32_t lastCountA = 0;
  static int32_t lastCountB = 0;

  // --- Phase 1: Forward Motion (Speed = 160 / ~60% PWM) ---
  Serial.println(F(">>> Phase 1: Moving Motors FORWARD (Speed: 160/255)..."));
  setMotorA(160);
  setMotorB(160);
  
  for (int i = 0; i < 20; i++) { // Run for 2 seconds (20 x 100ms)
    delay(100);
    uint32_t now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    int32_t currentCountA = encoderCountA;
    int32_t currentCountB = encoderCountB;

    float rpmA = ((currentCountA - lastCountA) / ENCODER_PPR) / (dt / 60.0f);
    float rpmB = ((currentCountB - lastCountB) / ENCODER_PPR) / (dt / 60.0f);

    lastCountA = currentCountA;
    lastCountB = currentCountB;

    Serial.print(F("Motor A (Left) Ticks: "));
    Serial.print(currentCountA);
    Serial.print(F(" | RPM: "));
    Serial.print(rpmA, 1);

    Serial.print(F("   ||   Motor B (Right) Ticks: "));
    Serial.print(currentCountB);
    Serial.print(F(" | RPM: "));
    Serial.println(rpmB, 1);
  }

  // --- Phase 2: Stop / Brake ---
  Serial.println(F("\n>>> Phase 2: STOPPING Motors (Brake for 1 sec)..."));
  stopMotors();
  delay(1000);

  // --- Phase 3: Reverse Motion (Speed = -160 / ~60% PWM) ---
  Serial.println(F("\n>>> Phase 3: Moving Motors REVERSE (Speed: -160/255)..."));
  setMotorA(-160);
  setMotorB(-160);
  
  for (int i = 0; i < 20; i++) {
    delay(100);
    int32_t currentCountA = encoderCountA;
    int32_t currentCountB = encoderCountB;

    Serial.print(F("Motor A Ticks: "));
    Serial.print(currentCountA);
    Serial.print(F(" | Motor B Ticks: "));
    Serial.println(currentCountB);
  }

  // --- Phase 4: Stop ---
  Serial.println(F("\n>>> Phase 4: STOPPING Motors (Brake for 2 sec). Repeating test...\n"));
  stopMotors();
  delay(2000);
}
