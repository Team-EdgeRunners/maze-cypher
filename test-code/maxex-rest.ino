#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// --- ToF Sensor Setup ---
#define XSHUT_1 10
#define XSHUT_2 11
#define XSHUT_3 12

// --- Motor Control Pins (L298N) ---
#define MOTOR_IN1 19
#define MOTOR_IN2 18
#define MOTOR_EN_A 21 // PWM pin for speed control (Motor A)
#define MOTOR_EN_B 20 // PWM pin for speed control (Motor B - Unused here)

// --- Encoder Pins ---
#define ENCODER_C1 26 // Hardware Interrupt pin
#define ENCODER_C2 27 // Direction pin

#define LED_PIN LED_BUILTIN 

// --- ROBOT SPECS (CHANGE WHEEL_DIAMETER_CM TO MATCH YOURS!) ---
const float WHEEL_DIAMETER_CM = 4.3;  
const float GEAR_RATIO = 100.0;       
const float ENCODER_BASE_TICKS = 7.0; 

// --- Calibration Math ---
const float TICKS_PER_WHEEL_REV = ENCODER_BASE_TICKS * GEAR_RATIO;
const float WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.14159;
const float TICKS_PER_CM = TICKS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_CM;

Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();

// BMI160 Address
const int BMI160_ADDR = 0x69; 

// --- Encoder Variable ---
volatile long encoderCount = 0; 

// --- The Hardware Interrupt Function ---
void updateEncoder() {
  if (digitalRead(ENCODER_C2) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void errorHalt(String message) {
  Serial.println("\n--- CRITICAL ERROR ---");
  Serial.println(message);
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// Custom, crash-proof function to initialize the BMI160
void initBMI160() {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x7E); 
  Wire.write(0x11); 
  Wire.endTransmission();
  delay(50);
  
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x7E); 
  Wire.write(0x15); 
  Wire.endTransmission();
  delay(50);
}

// --- Custom Function to Drive a Specific Distance ---
void driveDistance(float target_cm) {
  long targetTicks = target_cm * TICKS_PER_CM;
  
  noInterrupts();
  encoderCount = 0;
  interrupts();

  Serial.print("Driving ");
  Serial.print(target_cm);
  Serial.println(" cm...");

  // Turn the motor ON (Forward at safe ~6V speed)
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN_A, 128); 

  // Wait here until the encoder reaches the target
  while (abs(encoderCount) < targetTicks) {
    delay(1); 
  }

  // We hit the target! Slam the brakes.
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN_A, 0); 
  
  Serial.println("Destination Reached! Starting sensor stream...");
  delay(1000); // Pause for 1 second before streaming data
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  // PICO USB SAFETY DELAY
  for(int i = 0; i < 50; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(100);
  }
  
  digitalWrite(LED_PIN, HIGH);
  Serial.println("\n--- Starting Hardware ---");

  // Initialize I2C Bus on GP4 and GP5
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  // 1. Initialize ToF Sensors
  Serial.println("Booting ToF Sensors...");
  pinMode(XSHUT_1, OUTPUT); pinMode(XSHUT_2, OUTPUT); pinMode(XSHUT_3, OUTPUT);
  digitalWrite(XSHUT_1, LOW); digitalWrite(XSHUT_2, LOW); digitalWrite(XSHUT_3, LOW);
  delay(100);

  pinMode(XSHUT_1, INPUT); delay(100);
  if (!sensor1.begin(0x30, false, &Wire)) errorHalt("Failed Sensor 1");
  
  pinMode(XSHUT_2, INPUT); delay(100);
  if (!sensor2.begin(0x31, false, &Wire)) errorHalt("Failed Sensor 2");
  
  pinMode(XSHUT_3, INPUT); delay(100);
  if (!sensor3.begin(0x29, false, &Wire)) errorHalt("Failed Sensor 3");

  // 2. Initialize BMI160 (Raw I2C)
  Serial.println("Booting BMI160 IMU...");
  initBMI160();

  // 3. Initialize Motor & Encoder
  Serial.println("Booting Motor & Encoder...");
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN_A, OUTPUT);
  pinMode(MOTOR_EN_B, OUTPUT);
  
  pinMode(ENCODER_C1, INPUT_PULLUP);
  pinMode(ENCODER_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C1), updateEncoder, RISING);

  Serial.println("\n--- All Sensors Running! ---");

  // 4. Drive test: Move exactly 10cm before doing anything else
  driveDistance(10.0);
}

void loop() {
  // --- 1. Read ToF Sensors ---
  VL53L0X_RangingMeasurementData_t measure1, measure2, measure3;
  sensor1.rangingTest(&measure1, false);
  sensor2.rangingTest(&measure2, false);
  sensor3.rangingTest(&measure3, false);

  int t1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : 1000;
  int t2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : 1000;
  int t3 = (measure3.RangeStatus != 4) ? measure3.RangeMilliMeter : 1000;

  // --- 2. Read BMI160 (Gyro + Accel) ---
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x0C); 
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 12); 

  int16_t gx=0, gy=0, gz=0, ax=0, ay=0, az=0;
  if (Wire.available() == 12) {
    gx = Wire.read() | (Wire.read() << 8);
    gy = Wire.read() | (Wire.read() << 8);
    gz = Wire.read() | (Wire.read() << 8);
    ax = Wire.read() | (Wire.read() << 8);
    ay = Wire.read() | (Wire.read() << 8);
    az = Wire.read() | (Wire.read() << 8);
  }

  // --- 3. Read Encoder safely ---
  noInterrupts();
  long currentEncoderCount = encoderCount;
  interrupts();

  // --- 4. Print pure CSV data: T1,T2,T3,AX,AY,AZ,GX,GY,GZ,ENCODER ---
  Serial.print(t1); Serial.print(",");
  Serial.print(t2); Serial.print(",");
  Serial.print(t3); Serial.print(",");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(currentEncoderCount); 
  Serial.println(); 

  delay(50); // Keep it at 20 frames per second
}