/*
  ================================================================================
  MODULE: BMI160 6-Axis / 6DOF Gyro & Accelerometer Module
  PLATFORM: STM32 (e.g. STM32F103C8T6 Blue Pill)
  ENVIRONMENT: PlatformIO (VS Code Extension) / Arduino IDE
  INTERFACE: I2C (IIC) Mode
  ================================================================================
  
  [1] PLATFORMIO CONFIGURATION (platformio.ini):
  ---------------------------------------------
  Environment: [env:bmi160_test]
  Platform: ststm32
  Board: bluepill_f103c8
  Framework: arduino

  Libraries (Auto-managed via platformio.ini):
  - emotitron/BMI160-Arduino @ ^1.2.0

  [2] HARDWARE WIRING (I2C Mode):
  -------------------------------
  +------------------+----------------------------------+
  | BMI160 Module Pin| STM32 Board Pin                  |
  +------------------+----------------------------------+
  | VCC              | 3.3V                             |
  | GND              | GND                              |
  | SDA              | PB7 (I2C1_SDA)                   |
  | SCL              | PB6 (I2C1_SCL)                   |
  | SA0 / SDO        | GND (Address = 0x68) or 3.3V(0x69)|
  | CS               | 3.3V (High for I2C mode)         |
  +------------------+----------------------------------+
  * NOTE: Pull-up resistors (4.7k to 3.3V) recommended on SDA & SCL lines.

  ================================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <BMI160Gen.h>

#define SDA_PIN PB7
#define SCL_PIN PB6
#define BMI160_I2C_ADDR 0x68 // Default I2C address (0x68 if SA0=GND, 0x69 if SA0=3.3V)

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println(F("\n==========================================="));
  Serial.println(F("     STM32 + BMI160 6-DOF IMU Sensor Test  "));
  Serial.println(F("==========================================="));

  // Configure STM32 I2C Pins
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  Serial.println(F("Initializing BMI160 sensor over I2C..."));

  // Initialize BMI160 in I2C mode
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, BMI160_I2C_ADDR)) {
    Serial.println(F("[ERROR] BMI160 sensor initialization failed!"));
    Serial.println(F("Please check:"));
    Serial.println(F("  1. Wiring: VCC=3.3V, GND, SDA=PB7, SCL=PB6, CS=3.3V"));
    Serial.println(F("  2. SA0 pin state (GND for 0x68, 3.3V for 0x69)"));
    Serial.println(F("  3. Run [env:i2c_scanner] to verify I2C address"));
    while (1) {
      delay(500);
    }
  }

  Serial.println(F("[SUCCESS] BMI160 Initialized successfully!"));
  Serial.println(F("Streaming Accelerometer (g) & Gyroscope (deg/s) data...\n"));
}

void loop() {
  int16_t gx_raw, gy_raw, gz_raw;
  int16_t ax_raw, ay_raw, az_raw;

  // Read 6-axis raw data
  BMI160.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // Convert raw readings to physical units (Default ranges: Accel +/-2g, Gyro +/-250 deg/s)
  float ax = ax_raw / 16384.0f;
  float ay = ay_raw / 16384.0f;
  float az = az_raw / 16384.0f;

  float gx = gx_raw / 131.0f;
  float gy = gy_raw / 131.0f;
  float gz = gz_raw / 131.0f;

  // Output formatting
  Serial.print(F("ACCEL [g] -> X: "));
  Serial.print(ax, 2);
  Serial.print(F(" | Y: "));
  Serial.print(ay, 2);
  Serial.print(F(" | Z: "));
  Serial.print(az, 2);

  Serial.print(F("   ||   GYRO [deg/s] -> X: "));
  Serial.print(gx, 2);
  Serial.print(F(" | Y: "));
  Serial.print(gy, 2);
  Serial.print(F(" | Z: "));
  Serial.println(gz, 2);

  delay(100); // 10Hz update rate
}
