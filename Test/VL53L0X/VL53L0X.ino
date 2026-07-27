/*
  ================================================================================
  SENSOR: VL53L0XV2 / VL53L0X Time-of-Flight (TOF) Distance Sensor
  PLATFORM: STM32 (e.g. STM32F103C8T6 Blue Pill, STM32F4, NUCLEO Series)
  ENVIRONMENT: PlatformIO (VS Code Extension) / Arduino IDE
  ================================================================================

  [1] PLATFORMIO CONFIGURATION (platformio.ini):
  ---------------------------------------------
  Environment: [env:vl53l0x_test]
  Platform: ststm32
  Board: bluepill_f103c8 (Or update platformio.ini to match your STM32 model)
  Framework: arduino

  Libraries (Automatic Download via platformio.ini):
  - adafruit/Adafruit VL53L0X @ ^1.2.4

  [2] HARDWARE WIRING:
  --------------------
  +------------------+----------------------------------+
  | VL53L0XV2 Pin    | STM32 Board Pin                  |
  +------------------+----------------------------------+
  | VCC              | 3.3V (or 5V if module has LDO)   |
  | GND              | GND                              |
  | SDA              | PB7 (I2C1_SDA)                   |
  | SCL              | PB6 (I2C1_SCL)                   |
  | XSHUT (Optional) | PA4 (Shutdown / Enable Control)  |
  +------------------+----------------------------------+
  * NOTE: Connect 4.7k ohm pull-up resistors from SDA -> 3.3V and SCL -> 3.3V
    if your TOF breakout board lacks built-in pull-ups.

  ================================================================================
*/

#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <Wire.h>

// STM32 I2C1 Default Pins
#define SDA_PIN PB7
#define SCL_PIN PB6
#define XSHUT_PIN PA4 // Optional hardware reset pin

// Create VL53L0X sensor instance
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void resetI2CBus();

void setup() {
  // Initialize Serial output (115200 Baud)
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait up to 3 sec for Serial Monitor

  Serial.println(F("\n==========================================="));
  Serial.println(F("   STM32 + VL53L0XV2 TOF Test (PlatformIO) "));
  Serial.println(F("==========================================="));

  // Pulse XSHUT pin to reset sensor hardware
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(20);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(50); // Allow 50ms for internal MCU boot

  // Reset I2C Bus to clear any stuck SDA low states
  resetI2CBus();

  // Configure STM32 I2C Pins
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  delay(50);

  Serial.println(F("Initializing VL53L0XV2 sensor over I2C..."));

  // Default I2C address for VL53L0X is 0x29
  if (!lox.begin(0x29)) {
    Serial.println(F("[ERROR] VL53L0X failed to initialize!"));
    Serial.println(F("Please verify:"));
    Serial.println(F("  1. Wiring: VCC=3.3V/5V, GND, SDA=PB7, SCL=PB6"));
    Serial.println(F("  2. Pull-up resistors (4.7k to 3.3V) on SDA/SCL lines"));
    Serial.println(F("  3. If XSHUT is wired to PA4, disconnect PA4 or wire XSHUT directly to 3.3V"));
    while (1) {
      delay(500);
    }
  }

  Serial.println(F("[SUCCESS] VL53L0XV2 Initialized successfully!"));
  Serial.println(F("Streaming distance measurements...\n"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  // Read distance measurement from sensor
  lox.rangingTest(&measure, false); // pass 'true' to output debug data

  if (measure.RangeStatus != 4) { // Status 4 means out of range / phase error
    uint16_t dist_mm = measure.RangeMilliMeter;
    float dist_cm = dist_mm / 10.0;

    Serial.print(F("Distance: "));
    Serial.print(dist_mm);
    Serial.print(F(" mm  |  "));
    Serial.print(dist_cm, 1);
    Serial.println(F(" cm"));
  } else {
    Serial.println(F("Status: Out of range / Signal too weak"));
  }

  delay(100); // 10Hz sampling rate
}

// Clears I2C bus lockups by sending 9 SCL clock pulses
void resetI2CBus() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
  }
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
}
